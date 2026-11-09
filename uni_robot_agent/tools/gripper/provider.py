"""Model-facing arm-pose and scalar-gripper operations."""
from __future__ import annotations

from uni_robot_agent.context.prompt import PromptContribution
from uni_robot_agent.execution.scheduler import ToolExecutionPolicy
from uni_robot_agent.services.environment.contracts import EpisodeInfo, RequestRejected
from uni_robot_agent.tools.toolkit import ActionOutcome, ToolOutput, ToolProvider, ToolSpec, bind_tools, schema
from uni_robot_agent.tools.gripper.contracts import MOTION_PROFILES, GripperRequest, HoldRequest, MoveRequest, motion_profiles
from uni_robot_agent.tools.gripper.opening import native_opening, normalized_opening_tolerance
from uni_robot_agent.tools.gripper.motion import ClientPrimitiveBackend, MotionExecutor
from uni_robot_agent.tools.presentation import round_physical, state_for_model
from uni_robot_agent.tools.scene.perception import snapshot_images, snapshot_metadata

POSE6 = {"type": "array", "items": {"type": "number"}, "minItems": 6, "maxItems": 6,
         "description": "Absolute world grasp-site [x,y,z,roll,pitch,yaw]; metres and fixed-axis XYZ radians, R=Rz(yaw)@Ry(pitch)@Rx(roll)."}
OPENING = {"type": "number", "minimum": 0, "maximum": 1, "description": "Normalized opening: 0=closed, 1=open."}
NAMES = ("move", "grasp", "release", "hold")
# Motion stops after which the requested action did not finish as planned.
INCOMPLETE_STOPS = frozenset({"planning_rejected", "unsafe_motion", "tracking_interrupted", "native_end",
                              # Action-model runs (actor_act) that ended before their decided steps.
                              "partial_execution", "failure", "native_terminal", "error"})



def tool_policies():
    return {name: ToolExecutionPolicy(resource_key="episode", return_mode="background",
            concurrency="exclusive", mutates_world=True) for name in NAMES}


def tool_specs(arms, *, opening_models=None, profiles=MOTION_PROFILES, normalized_opening_tolerance=None):
    opening_models = opening_models or {}
    # Without a metric model, a declared tolerance checks the measured 0-1 opening instead.
    tracking = ("A calibrated explicit opening is tracked within the same max_steps budget;"
                if opening_models or normalized_opening_tolerance is None else
                f"An explicit opening is checked against the measured 0-1 opening within "
                f"{normalized_opening_tolerance:g} (opening_basis=measured_normalized_opening), within the same "
                "max_steps budget; a reading the environment does not mark as measured stays unknown;")
    if opening_models and set(opening_models) != set(arms):
        raise ValueError("Opening calibration must cover all tool arms")
    opening_key = "gripper_opening_m" if opening_models else "gripper"
    opening = ({"type": "number", "minimum": 0,
                "maximum": max(m["max_opening_m"] for m in opening_models.values()),
                "description": "Full inward pad gap in metres; 0 requests native preload closure. Omit to retain the drive target."}
               if opening_models else OPENING)
    arm = {"type": "string", "enum": list(arms)}

    def targets(properties=None, required=()):
        return {"type": "array", "minItems": 1, "maxItems": len(arms),
                "items": schema({"arm": arm, **(properties or {})}, ("arm", *required))}

    budget = {"type": "integer", "minimum": 1, "maximum": 24, "default": 24}
    position_tolerance = {"type": "number", "minimum": .001, "maximum": .1}
    orientation_tolerance = {"type": "number", "minimum": .02, "maximum": .5}
    profiles = "; ".join(
        f"{name}: {limits.max_step_m:g}m increments/{limits.position_tolerance_m:g}m position/{limits.orientation_tolerance_rad:g}rad orientation tolerance"
        for name, limits in profiles.items()
    )
    specs = [
        ToolSpec("move",
            "Move coordinated arms to absolute world grasp-site poses [x,y,z,roll,pitch,yaw], in metres and fixed-axis XYZ radians (R=Rz(yaw)@Ry(pitch)@Rx(roll)). All six values are required; both position and orientation are checked against the original goal and its tolerances. Copy the measured orientation to translate without reorienting, or the measured position to rotate in place. Omitted gripper preserves the previous command; a supplied opening applies during motion, not after arrival. "
            f"Profiles: {profiles}. Profiles add no offset. Tolerances define final arrival around the original goal. On supported backends, a rejected waypoint may trigger one search for a temporary execution goal within 80% of those tolerances. Native waypoint IK remains strict; arrival still uses measured error from the original goal. There is no automatic return to the exact center or coarse-to-fine phase. Inspect goal_recovery and per-arm execution_target_position/execution_target_quaternion when recovery occurs. Position and orientation interpolate together; one goal does not express 'move then rotate'. Use separate calls and inspect completed feedback before dependent actions. Rotation consumes displacement budget. These are target bounds, not collision guarantees. Unlisted arms hold; target list order is not a sequence. Inspect feedback.pose_reached, targets_reached, per-arm opening_reached, steps_executed and stop_reason. {tracking} after pose arrival the arm holds while the opening continues. opening_stalled/opening_unavailable are not target completion or contact evidence. Omitted opening retains the complete drive intent without requiring a held object to close to zero. Pose arrival is not object contact. Returns state, feedback, snapshot_id and final RGB images from the first scene camera and participating arms' wrists when available.",
            schema({"profile": {"type": "string", "enum": list(MOTION_PROFILES), "default": "standard"},
                    "targets": targets({"pose": POSE6, opening_key: opening}, ("pose",)),
                    "max_steps": budget, "position_tolerance_m": position_tolerance,
                    "orientation_tolerance_rad": orientation_tolerance}, ("targets",))),
        *[ToolSpec(name,
            f"{verb} selected grippers while holding fresh arm poses. Reports executed control steps and measured opening; object attachment or release must be verified from observations. Returns snapshot_id and final RGB images from the first scene camera and participating arms' wrists when available; inspect the object effect. Shared control-step budget 1..12; unlisted arms hold.",
            schema({"targets": targets({"opening_m": opening} if opening_models and name == "release" else {}), "steps": {"type": "integer", "minimum": 1, "maximum": 12, "default": 6}}, ("targets",)))
          for name, verb in (("grasp", "Close"), ("release", "Open"))],
        ToolSpec("hold",
            "Advance physics holding measured arm poses and existing gripper commands. steps=1..40; native termination stops early. Returns snapshot_id and the first scene camera RGB image when available. Inspect completed feedback and images; this consumes action steps.",
            schema({"steps": {"type": "integer", "minimum": 1, "maximum": 40, "default": 1}})),
    ]
    if opening_models:
        from dataclasses import replace
        notes = {
            "move": " gripper_opening_m is a full pad gap in metres; omit to retain the drive target.",
            "release": " Optional opening_m per target requests a partial opening in metres; omission opens fully.",
        }
        specs = [replace(spec, description=spec.description.replace("gripper_targets", "gripper_openings_m")
                         + notes.get(spec.name, "")) for spec in specs]
    return specs


class GripperTools(ToolProvider):
    provider_id = "gripper"

    def __init__(self, episode, *, names=NAMES):
        if not names or len(set(names)) != len(names) or not set(names) <= set(NAMES):
            raise ValueError("Select distinct supported gripper tools")
        self.names = tuple(names)
        self.episode = episode
        self.opening_models = episode.description.environment.metadata.get("gripper_opening_models", {})

    @property
    def prompt_contributions(self):
        names = self.names
        return (PromptContribution(system=(
            "Gripper operation labels (not fixed callable names): " + ", ".join(names) + ". "
            "Tool schemas define parameters, defaults, limits and returned evidence. "
            "Read each arm's control_frame and grasp_site_description in the environment description. "
            "call_ok reports call validity, not motion or object success. "
            "Direct feedback reports the whole call: steps_executed counts accepted control commands; executed_native_steps counts environment steps. "
            "Read state.native_steps, state.ended and remaining_steps for episode progress. "
            "Environment state uses arms[arm].control_pose, joints, gripper (normalized measured opening) and gripper_command (complete normalized drive intent, retained across accepted actions). "
            "When metric calibration is available, arm metadata.gripper_opening_m is the actual signed pad gap in metres; gripper_target_opening_m describes the retained intent, and gripper_applied_target_opening_m describes the native slew-limited target. "
            "To preserve grip, omit the gripper argument; copying measured opening changes the command. "
            "control_pose and feedback.actual_position measure the registered grasp site. State control_pose keeps its position and wxyz quaternion; move inputs and feedback.arms[arm].target_pose/actual_pose use six values in metres and XYZ radians. Grasp-site arrival does not establish object attachment. "
            "Direct action snapshot_id identifies the returned RGB observation. camera_calibration contains same-frame intrinsic_K and T_world_from_optical for calibrated cameras; disabled or unsupported cameras have no metric calibration. An artifact_error means observation delivery failed, not that acknowledged motion failed; do not repeat an action to recover its images."
        ), source="tools/gripper"),)

    def get_tools(self):
        return self._bind_tools()

    def get_declared_tools(self):
        return self._bind_tools()

    def _bind_tools(self):
        metadata = self.episode.description.environment.metadata
        specs = tool_specs([arm.name for arm in self.episode.description.environment.arms],
                           opening_models=self.opening_models,
                           profiles=motion_profiles(metadata.get("motion_profiles")),
                           normalized_opening_tolerance=normalized_opening_tolerance(metadata))
        return bind_tools([spec for spec in specs if spec.name in self.names], provider_id=self.provider_id,
                          handlers={name: getattr(self, name) for name in self.names}, invoke=self.invoke)

    def invoke(self, handler, **arguments):
        before = self.episode.state.state_version
        output = self.episode.invoke(handler, **arguments)
        if output.action is None and not output.success:
            # A program that failed after acknowledged steps did move, whatever its error says.
            uncertain = output.extra.get("execution_uncertain") is True
            if uncertain or self.episode.state.state_version != before:
                executed = "unknown" if uncertain else "some"
                output.action = ActionOutcome(complete=False, executed=executed, stopped=self._stopped(executed))
        return output

    def _stopped(self, executed):
        # A stepped simulation does not advance between requests, so a known result leaves it at rest.
        # Physical and device environments must confirm the stop themselves.
        description = self.episode.description
        simulated = (isinstance(description, EpisodeInfo)
                     and not description.environment.metadata.get("physical_hardware"))
        return True if simulated and executed != "unknown" else None

    def action_outcome(self, feedback, *, uncertain=False) -> ActionOutcome:
        """What the action did, for the executor's replan handling (ToolExecutionPolicy.replan_on_failure)."""
        reason = feedback.get("stop_reason")
        # unsafe_motion is detected on an accepted step: the world moved even when no step was counted.
        executed = ("unknown" if uncertain else "some" if reason == "unsafe_motion"
                    else "none" if feedback.get("no_execution") else "some")
        # A release or explicit opening that missed its target is incomplete; a grasp that stops
        # closing on an object is not.
        missed_opening = feedback.get("action") != "grasp" and any(
            arm.get("opening_reached") is False for arm in (feedback.get("arms") or {}).values())
        complete = (not uncertain and feedback.get("targets_reached") is not False
                    and reason not in INCOMPLETE_STOPS and not missed_opening)
        return ActionOutcome(complete=complete, executed=executed, stopped=self._stopped(executed))

    def action_output(self, reply, *, cameras=None):
        self.episode.publish(reply)
        image_error = None
        image_metadata = {}
        snapshot = reply.snapshot
        try:
            if cameras != [] and self.episode.description.environment.cameras:
                snapshot = self.episode.observation(cameras)
            images = [] if snapshot is None or cameras == [] else snapshot_images(
                self.episode.env, snapshot, cameras, mirror=self.episode.artifact_mirror
            )
            if images:
                image_metadata = snapshot_metadata(snapshot, [image.name for image in images],
                                                   mirror=self.episode.artifact_mirror)
        except Exception as error:  # noqa: BLE001 - image transport must not replay acknowledged physical work.
            images = []
            image_error = f"{type(error).__name__}: {error}"
        return ToolOutput(extra={"state": state_for_model(self.episode.state.model_dump(mode="json")),
            "remaining_steps": self.episode.remaining_steps, "feedback": round_physical(reply.feedback),
            **({"artifact_error": error} if (error := reply.artifact_error or image_error) else {}),
            "snapshot_id": None if snapshot is None else snapshot["snapshot_id"],
            **image_metadata, "image_cameras": [image.name for image in images]}, images=images,
            action=self.action_outcome(reply.feedback))

    def _run_motion_program(self, request, method, *, cameras=None, opening_feedback=None):
        """Run a model-level action by composing primitive service requests.

        ``EnvironmentClient.step`` is the canonical path: the tool program
        owns waypoint generation and stopping policy, while the service only
        executes one physical command at a time.
        """
        self.episode.require_active()
        start_state = self.episode.state.model_copy(deep=True)
        backend = ClientPrimitiveBackend(self.episode.env, start_state, on_reply=self.episode.accept_primitive)
        feedback = getattr(MotionExecutor(backend), method)(request)
        if opening_feedback:
            feedback["opening_requests"] = opening_feedback
        reply = backend.last_reply
        if reply is None:
            raise RequestRejected("The motion program did not execute a primitive step")
        # Several primitive receipts form one model tool result.  Publish only
        # the final receipt once and attach the aggregate tool feedback.
        reply = reply.model_copy(update={
            "state_before": start_state,
            "feedback": feedback,
        })
        return self.action_output(reply, cameras=cameras)

    def _metric_targets(self, targets, *, key="gripper_opening_m"):
        converted, feedback = [], {}
        for target in targets:
            row = dict(target)
            if self.opening_models and "gripper" in row:
                raise RequestRejected("Use gripper_opening_m in metres; native gripper is not a tool parameter")
            if key in row:
                model = self.opening_models.get(row["arm"])
                if model is None:
                    raise RequestRejected("This arm has no metric opening calibration")
                row["gripper"], feedback[row["arm"]] = native_opening(row.pop(key), model)
            converted.append(row)
        return converted, feedback

    def move(self, targets, profile="standard", max_steps=24,
                position_tolerance_m=None, orientation_tolerance_rad=None):
        targets, opening_feedback = self._metric_targets(targets)
        request = MoveRequest(profile=profile, targets=targets, max_steps=max_steps,
                              position_tolerance_m=position_tolerance_m,
                              orientation_tolerance_rad=orientation_tolerance_rad)
        return self._run_motion_program(request, "move",
                                        cameras=self._action_cameras([target.arm for target in request.targets]),
                                        opening_feedback=opening_feedback)

    def _action_cameras(self, arms):
        cameras = self.episode.description.environment.cameras
        scene = [camera.name for camera in cameras if camera.role == "scene"]
        wrists = [camera.name for camera in cameras
                  if camera.role == "wrist" and (len(self.episode.description.environment.arms) == 1
                                                 or any(arm in camera.name for arm in arms))]
        return scene[:1] + wrists or [camera.name for camera in cameras[:1]]

    def _gripper(self, targets, action, steps):
        if action == "grasp" and any(set(target) != {"arm"} for target in targets):
            raise RequestRejected("grasp takes arm names only and preserves the closing preload")
        targets, opening_feedback = self._metric_targets(targets, key="opening_m")
        request = GripperRequest(arms=[target["arm"] for target in targets], action=action, steps=steps,
                                 openings={target["arm"]: target["gripper"] for target in targets if "gripper" in target})
        return self._run_motion_program(request, "gripper", cameras=self._action_cameras(request.arms),
                                        opening_feedback=opening_feedback)

    def grasp(self, targets, steps=6):
        return self._gripper(targets, "grasp", steps)

    def release(self, targets, steps=6):
        return self._gripper(targets, "release", steps)

    def hold(self, steps=1):
        cameras = [camera.name for camera in self.episode.description.environment.cameras if camera.role == "scene"]
        request = HoldRequest(steps=steps)
        return self._run_motion_program(request, "hold", cameras=cameras[:1] or None)
