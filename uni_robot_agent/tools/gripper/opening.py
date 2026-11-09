"""Model-facing metric openings over an integration-declared drive calibration."""
import math

from uni_robot_agent.services.environment.contracts import RequestRejected


def native_opening(opening_m, model):
    if model["kind"] != "affine_pad_gap_v1":
        raise RequestRejected("Unsupported gripper opening calibration")
    if (isinstance(opening_m, bool) or not isinstance(opening_m, (float, int))
            or not math.isfinite(opening_m) or not 0 <= opening_m <= model["max_opening_m"]):
        raise RequestRejected(f"Opening must be finite and between 0 and {model['max_opening_m']} meters")
    if opening_m <= model["min_opening_m"]:
        command, effective = model["closed_command"], model["min_opening_m"]
    else:
        command = (opening_m - model["command_offset_m"]) / model["command_scale_m"]
        effective = opening_m
    if not 0 <= command <= 1:
        raise RequestRejected("Calibrated opening exceeds native command limits")
    return float(command), {"requested_opening_m": float(opening_m), "effective_target_opening_m": float(effective),
                            "opening_saturated": opening_m < model["min_opening_m"]}


class OpeningTracker:
    """Track a calibrated gap target independently of arm arrival and contact identity."""

    tolerance_m = .002
    progress_m = .0005
    window_steps = 8

    def __init__(self, command, model):
        target = model["command_offset_m"] + model["command_scale_m"] * command
        self.target_m = max(model["min_opening_m"], min(model["max_opening_m"], target))
        self.actual_m = None
        self.errors = []

    def update(self, arm, *, advanced=True):
        """Refresh the gap; rejected attempts must pass advanced=False."""
        value = arm.metadata.get("gripper_opening_m")
        self.actual_m = float(value) if value is not None and math.isfinite(value) else None
        if self.actual_m is None:
            self.errors.clear()
        elif advanced:
            self.errors.append(abs(self.actual_m - self.target_m))
            self.errors = self.errors[-self.window_steps:]

    @property
    def reached(self):
        return None if self.actual_m is None else abs(self.actual_m - self.target_m) <= self.tolerance_m

    @property
    def stalled(self):
        return (self.reached is False and len(self.errors) == self.window_steps
                and self.errors[0] - min(self.errors[1:]) <= self.progress_m)

    def feedback(self):
        status = ("unknown" if self.reached is None else "reached" if self.reached
                  else "stalled" if self.stalled else "pending")
        return {"target_opening_m": self.target_m, "actual_opening_m": self.actual_m,
                "opening_error_m": None if self.actual_m is None else abs(self.actual_m - self.target_m),
                "opening_tolerance_m": self.tolerance_m, "opening_reached": self.reached,
                "opening_status": status}


class NormalizedOpeningTracker:
    """Track a 0-1 opening target against the measured 0-1 opening when no metric model exists.

    An environment enables it by declaring ``normalized_opening_tolerance``. Only a reading the
    environment marks with arm metadata ``gripper_opening_measured: true`` counts; any other
    reading stays unknown rather than being judged from the command.
    """

    progress = .01
    window_steps = OpeningTracker.window_steps

    def __init__(self, command, tolerance):
        self.target = float(command)
        self.tolerance = float(tolerance)
        self.actual = None
        self.errors = []

    def update(self, arm, *, advanced=True):
        """Refresh the opening; rejected attempts must pass advanced=False."""
        measured = arm.metadata.get("gripper_opening_measured") is True and math.isfinite(arm.gripper)
        self.actual = float(arm.gripper) if measured else None
        if self.actual is None:
            self.errors.clear()
        elif advanced:
            self.errors.append(abs(self.actual - self.target))
            self.errors = self.errors[-self.window_steps:]

    @property
    def reached(self):
        return None if self.actual is None else abs(self.actual - self.target) <= self.tolerance

    @property
    def stalled(self):
        return (self.reached is False and len(self.errors) == self.window_steps
                and self.errors[0] - min(self.errors[1:]) <= self.progress)

    def feedback(self):
        status = ("unknown" if self.reached is None else "reached" if self.reached
                  else "stalled" if self.stalled else "pending")
        return {"target_opening": self.target, "actual_opening": self.actual,
                "opening_error": None if self.actual is None else round(abs(self.actual - self.target), 4),
                "opening_tolerance": self.tolerance, "opening_basis": "measured_normalized_opening",
                "opening_reached": self.reached, "opening_status": status}


def normalized_opening_tolerance(metadata):
    """The declared tolerance for 0-1 opening checks, or None when the environment declares none."""
    tolerance = metadata.get("normalized_opening_tolerance")
    if tolerance is None:
        return None
    if isinstance(tolerance, bool) or not isinstance(tolerance, (int, float)) or not 0 < tolerance <= .5:
        raise ValueError("normalized_opening_tolerance must be in (0, 0.5]")
    return float(tolerance)


def opening_tracker(command, arm, metadata):
    """A metric tracker when the arm has a calibration model, else a normalized one when declared."""
    models = metadata.get("gripper_opening_models", {})
    if arm in models:
        return OpeningTracker(command, models[arm])
    tolerance = normalized_opening_tolerance(metadata)
    return None if tolerance is None else NormalizedOpeningTracker(command, tolerance)
