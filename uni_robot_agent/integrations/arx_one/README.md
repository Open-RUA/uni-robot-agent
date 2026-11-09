# ARX ONE

A dual-arm ARX ONE (two ARX X5 arms, three RealSense D405 cameras) behind the shared gripper
and scene tools. It is also a reference for connecting another real robot: the layout below
separates what runs in the agent's processes from the device process that owns the hardware.

Starting a controller is a physical action: the vendor constructor can home an arm and close its
gripper. Clear the workspace first. [DEPLOYMENT.md](DEPLOYMENT.md) covers a new machine:
third-party binaries, configuration, calibration and perception services.

```bash
python -m uni_robot_agent.integrations.arx_one.operator_cli --config environment.json status
python -m uni_robot_agent.integrations.arx_one.operator_cli --config environment.json up \
  --runtime-root RELEASE --host-config host.json --yes
deploy/scripts/run_env.sh    # serve one physical episode
deploy/scripts/run_agent.sh  # run the agent against it
```

## Layout

| Path | Runs in | Responsibility |
|---|---|---|
| `__init__.py`, `config.py`, `session.py`, `prompt.py` | Agent | Registration, typed configuration, the prepared-episode session, robot context |
| `operator_cli.py` | Operator | `status`, `identity`, `up`, `down`, `home`, `gripper`, `calibrate-gripper`, `ack`; the model cannot call them |
| `motion/` | Agent | The real-robot motion program on the shared tools (one preflighted path per move, joint-holding gripper actions, a fresh reading per action) and its feedback. It relies on Body continuous motion, so it stays here until the shared protocol has a typed path command; without that capability the shared stepwise motion applies |
| `environment/` | Episode service | One physical episode over the Body RPC: admission, control lease, actions, observations, tracking recovery. It drives any Body that meets the arm and camera contracts in `body/core/` |
| `perception/` | Episode, offline and depth services | Snapshots, RGB-D geometry, camera extrinsics (`extrinsics.py`), depth completion (`completion/`) |
| `offline/` | Outside episodes | `fleet/`: site manifest (Bodies, robots, calibration bindings), the Fleet client and fleet-wide stop confirmation (`python -m ...offline.fleet demo`); `capture/`: read-only observation capture and archive inspection; `calibration/`: eye-to-hand and eye-in-hand calibration and verification. Nothing in an episode imports them |
| `body/` | Device process | Owns the arms and cameras: generic `core/` and `host/`, one driver and kit per device kind under `drivers/`; see [body/README.md](body/README.md) |
| `deploy/` | — | `configs/`: configuration templates; `scripts/`: launch scripts |

Dependencies point one way: `motion/` → `environment/` → `perception/` → `body/core`;
`offline/capture/` → `offline/fleet/` → `perception/`, `body/core`; `offline/calibration/` → `perception/`. Nothing under
`body/` imports agent-side code, so the Body runtime can be packaged and run on its own. Inside `body/`, `core/` and `host/`
never import a driver: drivers import only `core/`, and each driver's `kit.py` plugs into `host/`.

## Contracts

- **Motion.** A `move` is one preflighted path; Body runs it without intermediate stops and
  verifies arrival once. A path stopped by a limit runs its reachable prefix and is reported,
  never retried. Gripper-only actions hold every joint at its measured angle.
- **Outcomes.** Feedback separates goal status, execution status and confirmed stop. A physical
  action is never replayed after a lost acknowledgement; a failure needs a confirmed stop and a
  new capture before the next action. `finish` reports success, failure or give-up; physical
  task success is assessed outside the run.
- **Control lease.** An episode holds at most one lease at a time. It is released after
  `idle_timeout_s` without a physical action or scene capture; recovery then takes a new lease
  after a fresh capture. Lost control, an action timeout or an unconfirmed stop goes to the
  operator instead.
- **Observations.** One request time selects one RGB-D packet per camera within one and a half
  frame periods after it. A wrist camera's pose uses joints interpolated at its image's source
  time, never extrapolated; the reported arm state is a fresh reading taken after the images.
  Archived observations are immutable and checksummed; reopening one never makes it a fresh
  motion precondition.
- **Depth.** Measured depth is never altered. Completion adds model depth only at holes, and a
  geometry query names the exact depth artifact it uses; there is no fallback between sources.
- **Calibration.** Offline tools bind a record by name and SHA-256 in the Site, with no
  defaults or estimated fallbacks, and results stay `motion_ready=false`. Episodes read the
  transforms in `environment.json`, copied from validated records (see
  [DEPLOYMENT.md](DEPLOYMENT.md#5-calibration)).

## Adapting to another robot

1. Decide where the hardware is served. `environment/` speaks the Body 2.0 RPC (leases,
   operations, observation history). Running a Body for your robot keeps all of it; a robot with
   its own device service (ROS 2 control, a vendor RPC) instead needs `environment/control.py`
   and the observation code rewritten against that service, and can leave out `body/` and `offline/fleet/`.
2. For a Body, keep `body/core` and `body/host` unchanged. For each device kind add
   `body/drivers/<name>/` with `driver.py` (implements `Driver` in `body/core/driver.py`) and
   `kit.py` (validation, install rules and build inputs for the host); copy `body/drivers/fake/`,
   the smallest complete driver and kit. List the kits in `body/runtime.json` and select them in
   `host.json`; see [body/README.md](body/README.md#kits). Keep limits, tolerances and
   calibration in configuration.
3. Describe the robot in `environment.json`: any number of arms with their names, joint names
   and limits, and cameras attached to the world or to an arm. The adapter, the native path
   command, the operator commands and the prompt's robot line follow that configuration. Arms
   must report the Body signals `joint_position_rad`, `gripper_opening`, `eef_position_m` and
   `eef_rpy_rad`; edit `prompt.py` for robot-specific rules.
4. Declare only the capabilities the robot supports. Tool behaviour belongs in the shared tool
   families, not in the integration.
