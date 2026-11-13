"""Lightweight host plugins. Import and validation never open CAN or USB."""
from pathlib import Path

from uni_robot_agent.integrations.piper_x.body.host.kit import BaseKit
from ..config import Installation


class PiperKit(BaseKit):
    def validate(self, host, config, bodies):
        super().validate(host, config, bodies)
        for body in bodies.values():
            args = body["driver_args"]
            if set(args) - {"deployment", "execute"} or "deployment" not in args:
                raise ValueError("Piper Body needs an explicit deployment manifest")
            if type(args.get("execute", False)) is not bool or self.name == "piper_cameras" and "execute" in args:
                raise ValueError("Only arm Bodies accept a boolean execute flag")
            deployment = Path(args["deployment"])
            if not deployment.is_absolute():
                raise ValueError("Resolved host driver_args.deployment must be absolute")
            installation = Installation(deployment)
            if installation.host.host_id != host["host_id"]:
                raise ValueError("Deployment refers to another host")
            ref = installation.robot.arms_body if self.name == "piper_arms" else installation.robot.cameras_body
            if installation.host.bodies[ref].body_id != body["body_id"]:
                raise ValueError("Body identity differs from robot configuration")
            if args.get("execute"):
                installation.require_motion()

    def resources(self, body):
        installation = Installation(body["driver_args"]["deployment"])
        if self.name == "piper_arms":
            return [("can", installation.host.arms[a.binding].resource()) for a in installation.robot.arms.values()]
        return [("camera", installation.host.cameras[c.binding].serial) for c in installation.robot.cameras.values()]


arm_kit = PiperKit(name="piper_arms", factory=__package__ + ".arms:ArmBody")
camera_kit = PiperKit(name="piper_cameras", factory=__package__ + ".cameras:CameraBody")


def host_config(installations, *, execute=False):
    """Compose several robot instances on one host without any shared robot state."""
    if not installations:
        raise ValueError("Select at least one deployment")
    host = installations[0].host
    if any(i.host != host for i in installations):
        raise ValueError("Co-located robots must reference the same host configuration")
    arm_plugin, camera_plugin = __name__ + ":arm_kit", __name__ + ":camera_kit"
    bodies = {}
    for installation in installations:
        for name, kit in ((installation.robot.arms_body, arm_plugin), (installation.robot.cameras_body, camera_plugin)):
            if name in bodies:
                raise ValueError("Repeated Body instance in deployments")
            body = host.bodies[name]
            bodies[name] = {"kit": kit, "body_id": body.body_id, "port": body.port,
                "driver_args": {"deployment": str(installation.path), **({"execute": execute} if kit == arm_plugin else {})}}
    return {"schema_version": 2, "host_id": host.host_id, "operator": host.operator,
            "state_dir": str(host.state_dir), "tokens": str(host.tokens_file), "listen": host.listen,
            "kits": {arm_plugin: {}, camera_plugin: {}}, "bodies": bodies}
