"""Read and localize immutable scenes without owning motion or lifecycle."""
from uni_robot_agent.context.prompt import PromptContribution
from uni_robot_agent.execution.scheduler import ToolExecutionPolicy
from uni_robot_agent.services.environment.contracts import CaptureRequest
from uni_robot_agent.tools.presentation import state_for_model
from uni_robot_agent.tools.scene.perception import locate, snapshot_images, snapshot_metadata
from uni_robot_agent.tools.toolkit import ToolOutput, ToolProvider, ToolSpec, bind_tools, schema

PIXEL = {"type": "array", "items": {"type": "integer", "minimum": 0}, "minItems": 2, "maxItems": 2}
NAMES = ("view_env_state", "capture_scene", "locate")


def tool_policies():
    return {
        "view_env_state": ToolExecutionPolicy(resource_key="episode", concurrency="parallel"),
        # A new snapshot is what a replan after a failed action starts from.
        "capture_scene": ToolExecutionPolicy(resource_key="episode", concurrency="exclusive",
                                             replan_observation=True),
        "locate": ToolExecutionPolicy(resource_key="perception", concurrency="parallel"),
    }


def tool_specs(cameras, *, segmentation_enabled=True, depth_cameras=None, calibrated_cameras=None,
               scene_geometry=False, device_episode=False):
    calibrated_cameras = list(cameras) if calibrated_cameras is None else list(calibrated_cameras)
    depth_cameras = list(cameras) if depth_cameras is None else list(depth_cameras)
    depth_cameras = [name for name in depth_cameras if name in calibrated_cameras]
    camera = {"type": "string", "enum": list(cameras)}
    views = {"type": "array", "items": camera, "minItems": 1,
             "maxItems": len(cameras), "uniqueItems": True}
    specs = [
        ToolSpec("view_env_state",
            "Read completed RGB/state records without depth capture or physics. Calibration may be unavailable; use the declared scene-capture tool for calibrated geometry. step=-1 selects the latest record; step=0 the initial record."
            if depth_cameras else "Read completed RGB/state records without advancing physics. Calibration may be unavailable; use capture_scene for calibrated geometry. step=-1 selects the latest record; step=0 the initial record.",
            schema({"step": {"type": "integer", "minimum": -1, "default": -1}, "cameras": views}), True),
        ToolSpec("capture_scene",
            "Freeze RGB, depth and calibration from one state without advancing action steps. Pass the returned immutable snapshot_id to the declared feature-localization tool; RGB is attached."
            if depth_cameras else "Freeze RGB and calibration from one state without advancing action steps. RGB is attached; use its immutable snapshot_id for localization. Calibration belongs to each returned image grid.",
            schema({"cameras": views}), True),
    ]
    methods, properties, descriptions = [], {}, []
    if depth_cameras:
        properties["points"] = {"type": "array", "items": PIXEL, "minItems": 1, "maxItems": 4}
    if segmentation_enabled:
        methods.append("segment")
        properties.update(prompt={"type": "string"}, point=PIXEL,
                          min_score={"type": "number", "minimum": 0, "maximum": 1, "default": .2})
        descriptions.append(
            "method=segment accepts exactly one text prompt or mask-refinement point=[row,col]. "
            "When found=true, inspect overlays, geometry_valid and geometry_quality; "
            "an unmatched result has found=false, reason and score, without geometry quality."
            if depth_cameras else
            "method=segment requires camera and exactly one text prompt or refinement point=[row,col]. "
            "Returns found, score, a 2D box and overlay; inspect identity in the overlay.")
        descriptions.append("A returned segment box uses [x_min,y_min,x_max,y_max] (xyxy): "
                            "x is column, y is row. Input points remain [row,col].")
    if depth_cameras:
        methods.append("pixels")
        descriptions.insert(0,
            "Locate a feature in a snapshot from the declared scene-capture tool. method=pixels accepts 1–4 interior points=[[row,col],...]. "
            "Coordinates start at top-left. Inspect geometry.valid and geometry.valid_count; "
            "geometry.points_xyz corresponds to geometry.input_pixels for valid entries only.")
        if not segmentation_enabled:
            descriptions.append("Segmentation is unavailable in this session.")
        if set(depth_cameras) != set(cameras):
            descriptions.append(f"Depth geometry is available only on cameras: {', '.join(depth_cameras)}.")
    if (not depth_cameras or scene_geometry) and len(calibrated_cameras) >= 2:
        methods.append("triangulate")
        properties["observations"] = {"type": "array", "minItems": 2, "maxItems": min(3, len(calibrated_cameras)),
            "items": schema({"camera": {"type": "string", "enum": calibrated_cameras},
                             "pixel": PIXEL}, ("camera", "pixel"))}
        descriptions.append("method=triangulate requires observations from 2-3 distinct cameras in this snapshot, "
                            "each {camera,pixel:[row,col]} identifying the SAME physical point. "
                            "Omit camera, points, prompt and point. Uses RGB pinhole calibration to return geometry.valid, "
                            "reasons, ray_angle_deg and reprojection_errors_px; point_xyz exists only when valid. "
                            "No automatic correspondence matching: low reprojection error does not prove identity. "
                            "Do not match independent mask centers. Use distinct viewpoints and undistorted images.")
    if scene_geometry and "triangulate" in methods:
        properties["correspondences"] = {"type": "array", "minItems": 1, "maxItems": 64,
                                          "items": properties["observations"]}
        properties["fit_table"] = {"type": "boolean", "default": False}
        descriptions.append("Alternatively supply correspondences=[observations,...] for multiple points; "
                            "fit_table=true fits at least three valid non-collinear world points. Do not also supply observations.")
    if device_episode:
        from dataclasses import replace
        specs = [replace(spec, description=(
            "Read completed immutable image/state records; step is the record index, not simulated physics."
            if spec.name == "view_env_state" else
            "Freeze current camera streams and calibration into an immutable snapshot. Camera timing and depth pairing are described in its metadata; capture does not calibrate devices.")) for spec in specs]
    if not calibrated_cameras:
        from dataclasses import replace
        specs = [replace(spec, description=(
            "Read completed RGB/state records without advancing physics; camera calibration is disabled. "
            "step=-1 selects the latest record; step=0 the initial record."
            if spec.name == "view_env_state" else
            "Freeze RGB into an immutable snapshot; camera intrinsics and extrinsics are unavailable. "
            "Use snapshot_id for 2D segmentation when exposed; capture does not recover calibration."))
            for spec in specs]
    if methods:
        description = (
            " ".join(descriptions) + " Geometry does not establish object identity or a safe grasp point."
            if depth_cameras else
            "Locate a feature in one immutable captured snapshot. Coordinates start at top-left. "
            + " ".join(descriptions) + " Geometry does not establish a safe grasp point.")
        required = ("method", "snapshot_id") if "triangulate" in methods else ("method", "snapshot_id", "camera")
        specs.append(ToolSpec("locate", description,
            schema({"method": {"type": "string", "enum": methods},
                    "snapshot_id": {"type": "string", "minLength": 1}, "camera": camera,
                    **properties}, required), True))
    return specs


class SceneTools(ToolProvider):
    provider_id = "scene"

    def __init__(self, episode, *, perception=None):
        self.episode = episode
        self.perception = perception

    def _specs(self):
        cameras = self.episode.description.environment.cameras
        return tool_specs([camera.name for camera in cameras], segmentation_enabled=self.perception is not None,
                          depth_cameras=[camera.name for camera in cameras if camera.has_depth],
                          calibrated_cameras=[camera.name for camera in cameras if camera.has_calibration],
                          scene_geometry=self.episode.description.environment.metadata.get("scene_geometry", False),
                          device_episode=self.episode.description.protocol_version == "devices-1")

    @property
    def prompt_contributions(self):
        specs = self._specs()
        methods = next((spec.input_schema["properties"]["method"]["enum"]
                        for spec in specs if spec.name == "locate"), [])
        method_description = ", ".join(methods)
        if "pixels" in methods and "triangulate" not in methods:
            method_description = "segment and pixels" if "segment" in methods else "pixels only; segmentation is unavailable"
        return (PromptContribution(system=(
            "Scene operation labels (not fixed callable names): " + ", ".join(spec.name for spec in specs) + ". "
            + (f"Available locate methods: {method_description}. " if methods else "Feature localization is unavailable. ")
            + "See the tool schema for arguments and result fields. "
            "Snapshots are immutable historical measurements, not promises that objects remain in those locations."
        ), source="tools/scene"),)

    def get_tools(self):
        return bind_tools(self._specs(), provider_id=self.provider_id,
                          handlers={name: getattr(self, name) for name in NAMES}, invoke=self.episode.invoke)

    def robot_reference(self):
        metadata = self.episode.description.environment.metadata
        ref = metadata.get("artifacts", {}).get("robot_calibration")
        if ref is None:
            return {}
        return {"robot_calibration_path": str(self.episode.artifact_mirror.local_path(ref)),
                "robot_calibration_sha256": ref["sha256"],
                "robot_manual": metadata["robot_manual"]}

    def view_env_state(self, step=-1, cameras=None):
        manifest = self.episode.env.record(step)
        images = snapshot_images(self.episode.env, manifest, cameras, mirror=self.episode.artifact_mirror)
        return ToolOutput(extra={"record_index": manifest.get("record_index", step),
            "state": state_for_model(manifest["state"]), "state_version": manifest["state_version"],
            **({"command_index": manifest["state"]["completed_commands"]} if manifest["state"].get("kind") == "devices"
               else {"step": manifest["state"]["native_steps"]}), "snapshot_id": manifest["snapshot_id"],
            "image_cameras": [image.name for image in images],
            **snapshot_metadata(manifest, [image.name for image in images], mirror=self.episode.artifact_mirror),
            **self.robot_reference()}, images=images)

    def capture_scene(self, cameras=None):
        manifest = self.episode.env.capture(CaptureRequest(cameras=cameras))
        images = snapshot_images(self.episode.env, manifest, cameras, mirror=self.episode.artifact_mirror)
        return ToolOutput(extra={"snapshot_id": manifest["snapshot_id"],
            "state_version": manifest["state_version"], "state": state_for_model(manifest["state"]),
            "cameras": list(manifest["cameras"]), "image_cameras": [image.name for image in images],
            **snapshot_metadata(manifest, [image.name for image in images], mirror=self.episode.artifact_mirror),
            **self.robot_reference()}, images=images)

    def locate(self, **arguments):
        return locate(self.episode, perception=self.perception, **arguments)
