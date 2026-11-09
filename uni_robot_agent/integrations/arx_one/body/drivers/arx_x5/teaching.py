"""Operator SOFT/hold transition. Never changes controller/CAN lifecycle."""

import math
import time

from ...core.contract import DeviceError, ErrorCode


def set_teaching(driver, device, procedure, params, cancel):
    if params != {'site_ready': True} or params['site_ready'] is not True:
        raise DeviceError(ErrorCode.INVALID_ARGUMENTS, 'supported arm/object and clear workspace must be confirmed')
    state = driver._states[device]
    if not state.command_lock.acquire(blocking=False):
        raise DeviceError(ErrorCode.RESOURCE_BUSY, 'arm command is in progress')
    sent = False
    try:
        side = driver.arms[device]['side']
        wanted = procedure == 'enter_soft_teach'
        mode = driver.transport.teach_state(side)
        if not mode.get('supported') or mode.get('active') is wanted:
            raise DeviceError(ErrorCode.PRECONDITION_FAILED, 'teaching interface unavailable or mode already changed')
        detail = driver.transport.diagnostics(side, refresh=True)
        if not detail.get('hardware_ready'):
            raise DeviceError(ErrorCode.PRECONDITION_FAILED, 'controller/CAN ownership, identity or feedback is not ready')
        previous = None
        first = time.monotonic()
        last_stamp = None
        samples = 0
        while time.monotonic() - first < 1.:
            if cancel.is_set():
                raise DeviceError(ErrorCode.PRECONDITION_FAILED, 'teaching cancelled before dispatch')
            joints, _ = driver._require_fresh(device, teaching=True)
            with state.lock:
                stamp, moving = state.messages, state.moving
                vel = list(state.joint_vel or [])
            if moving or len(vel) != 7 or any(not math.isfinite(v) or abs(v) > .03 for v in vel):
                raise DeviceError(ErrorCode.PRECONDITION_FAILED, 'arm and gripper must be stationary')
            if previous is not None and max(abs(a-b) for a,b in zip(previous, joints)) > .005:
                raise DeviceError(ErrorCode.PRECONDITION_FAILED, 'feedback is moving; hold the arm still')
            if stamp != last_stamp:
                samples += 1; last_stamp = stamp
            previous = joints
            cancel.wait(.025)
        if samples < 3 or cancel.is_set():
            raise DeviceError(ErrorCode.PRECONDITION_FAILED, 'insufficient fresh stationary feedback')
        # Clear cached pre-teaching targets. Native resume snapshots again at dispatch.
        state.last_command = state.gripper_target = None
        sent = True
        response = driver.transport.driver_command(side, procedure)
        if not response.success or list(response.values) != [1., float(wanted)]:
            raise RuntimeError('native teaching transition was not acknowledged')
        if driver.transport.teach_state(side).get('active') is not wanted:
            raise RuntimeError('native teaching latch did not match request')
        # ACK is only a software latch. Require newer, stationary feedback before
        # reporting completion; unknown preserves the Body quarantine on failure.
        with state.lock:
            boundary = state.messages
        deadline = time.monotonic() + 1.
        count = 0
        while time.monotonic() < deadline:
            joints, _ = driver._require_fresh(device, teaching=True)
            with state.lock:
                stamp = state.messages
                vel = list(state.joint_vel or [])
            if len(vel) != 7 or any(not math.isfinite(v) or abs(v) > .03 for v in vel):
                raise RuntimeError('post-transition feedback is moving')
            if max(abs(a-b) for a,b in zip(previous, joints)) > .02:
                raise RuntimeError('post-transition position changed; no automatic correction')
            if stamp > boundary:
                count += 1; boundary = stamp
            if count >= 3: break
            cancel.wait(.025)
        if count < 3 or cancel.is_set():
            raise RuntimeError('post-transition feedback/cancellation is unresolved')
        return {'completed': True, 'mode': 'soft_teach' if wanted else 'position_hold',
                'gripper_soft': wanted, 'motor_disable_confirmed': False,
                'completion_basis': 'native_mode_latch_and_fresh_feedback',
                'note': 'SOFT also releases the gripper; not an electrical disable or certified stop'}
    except DeviceError as exc:
        if sent:
            raise DeviceError(ErrorCode.OUTCOME_UNKNOWN, str(exc), execution_outcome='unknown') from exc
        raise
    except Exception as exc:
        raise DeviceError(ErrorCode.OUTCOME_UNKNOWN if sent else ErrorCode.PRECONDITION_FAILED,
            str(exc), execution_outcome='unknown' if sent else 'not_started') from exc
    finally:
        state.command_lock.release()
