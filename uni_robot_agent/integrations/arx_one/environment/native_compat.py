"""Identity, contract and readiness checks against the Body 2.0 service in body/."""
from ..body.core.arm import contract
from ..body.core.camera import contract as camera_contract


def verify(client, config, *, arm_names=None, verify_readiness=True):
    """Check the commissioned arms against their Body; returns each arm's motion limits."""
    arms = config.arms if arm_names is None else {name: config.arms[name] for name in arm_names}
    if not arms:
        raise ValueError("At least one commissioned arm must be selected")
    ids = [a.device_id for a in arms.values()]
    description = client.call("describe", {"device_ids": ids})
    if description.get("body_id") != config.arms_body.body_id:
        raise ValueError("Arm Body identity differs from the commissioned installation")
    devices = {d["device_id"]: d for d in description["devices"]}
    if set(devices) != set(ids):
        raise ValueError("Body arm identities differ from configuration")
    limits = {}
    for name, arm in arms.items():
        device = devices[arm.device_id]
        problems = contract.problems(device)
        if problems:
            raise ValueError(f"Body {name} does not meet the arm contract: " + "; ".join(problems))
        metadata = device["metadata"]
        for key, expected in arm.expected_metadata.items():
            if metadata.get(key) != expected:
                raise ValueError(f"Body {name} {key} differs from the commissioned one; "
                                 "compare with operator_cli identity")
        if metadata.get("joint_limits_rad") != {"lower": arm.joint_lower, "upper": arm.joint_upper}:
            raise ValueError(f"Body {name} joint limits differ from commissioned limits")
        if arm.kinematics and metadata.get("model_sha256") not in (None, arm.kinematics.sha256):
            raise ValueError(f"Body {name} controller model differs from the configured URDF")
        limits[name] = {key: metadata["motion_limits"][key] for key in contract.MOTION_LIMITS}
        limits[name]["continuous_motion"] = contract.supports_group_path(device)
    checked = client.call("check", {"device_ids": ids})["devices"]
    if set(checked) != set(ids) or (verify_readiness and any(x.get("readiness") != "satisfied" for x in checked.values())):
        raise ValueError("Body arms are not ready; commission controllers externally")
    return limits


def verify_cameras(client, config):
    """Check the commissioned cameras against their Body; returns the ``rgbd`` signals to read per camera."""
    if not config.cameras:
        return {}
    description = client.call("describe", {"device_ids": [c.device_id for c in config.cameras.values()]})
    if description.get("body_id") != config.cameras_body.body_id:
        raise ValueError("Camera Body identity differs from the commissioned installation")
    devices = {d["device_id"]: d for d in description["devices"]}
    signals = {}
    for name, camera in config.cameras.items():
        device = devices.get(camera.device_id)
        if device is None:
            raise ValueError(f"Camera Body has no device {camera.device_id}")
        problems = camera_contract.problems(device)
        if problems:
            raise ValueError(f"Camera {name} does not meet the camera contract: " + "; ".join(problems))
        for key, expected in camera.expected_metadata.items():
            if device["metadata"].get(key) != expected:
                raise ValueError(f"Camera {name} {key} differs from the commissioned one; "
                                 "compare with operator_cli identity")
        signals[name] = camera_contract.signals_to_read(device)
    return signals
