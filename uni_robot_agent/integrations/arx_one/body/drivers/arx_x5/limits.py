"""Manufacturer references and bounds of version-locked controller profiles.

The V2 documentation is a reference, not permission to exceed a different
SDK's clipping loop. Changing the runtime bounds requires an audited binary
and a matching controller identity; operator speed settings live separately.
"""

import math
from dataclasses import dataclass

SDK_REVISION = "c78328785ea23a81d908e1dcc551eca992f2f1e9"
VENDOR_SDK_SHA256 = "fb4e905d523f16d331b46089f942d5cff47f53a85acf774519dfa1d17909234a"
SDK_SHA256 = "3742728fa1ef56eef5eaf5b3a552e0b8e3c6944ba1cd1b937fdb88e9fddc2265"
SDK_VARIANT = "rua_arx_arm64_j4_90_v1"
AMD64_VENDOR_SDK_SHA256 = "d07d53d14da17f0a1db2753e3339f9e4c055dad72df613c811b9692c4bede85c"
AMD64_J4_90_SDK_SHA256 = "ec4a7e7f2528797c6e9fcf1b30e773165be81b8d8130ed28c00510fd651e67a7"
AMD64_J4_90_VARIANT = "rua_arx_amd64_j4_90_v1"
MODEL_SHA256 = "7c5381e59b6c00d19feaca88526c9b89aa46bbd34bef7b0f7a26f8858f822368"
DOCUMENTED_JOINT_LOWER_DEG = (-150, 0, 0, -90, -90, -120)
DOCUMENTED_JOINT_UPPER_DEG = (180, 210, 180, 90, 90, 120)

# ControllerBase constructor + stateEndControl/statePositionControl, audited
# without constructing a controller (construction initializes CAN and homes).
# The latest ROS2 library changes J1 from [-3.14, 2.618] to [-2.618, 3.14].
ORIGINAL_JOINT_LOWER = (-2.618, -0.1, -0.1, -1.29, -1.4835298641951802, -1.7453292519943295)
ORIGINAL_JOINT_UPPER = (3.14, 3.6, 3.0, 1.29, 1.4835298641951802, 1.7453292519943295)
# RUA variant (2026-09-12): J4 widened to the documented +-90 deg in both native clipping loops;
# J4 now matches the documented endpoint. Path truncation retains its own margin.
JOINT_LOWER = ORIGINAL_JOINT_LOWER[:3] + (-math.radians(90),) + ORIGINAL_JOINT_LOWER[4:]
JOINT_UPPER = ORIGINAL_JOINT_UPPER[:3] + (math.radians(90),) + ORIGINAL_JOINT_UPPER[4:]
EEF_LOWER = (0, -0.53, -0.53, -1.7453292519943295, -1.4835298641951802, -1.4835298641951802)
EEF_UPPER = (0.53, 0.53, 0.53, 1.7453292519943295, 1.4835298641951802, 1.4835298641951802)


@dataclass(frozen=True)
class ControllerProfile:
    name: str
    target: str
    end_type: int
    sdk_sha256: str
    vendor_sha256: str
    model_sha256: str
    sdk_filename: str
    model_filename: str
    variant: str
    joint_lower: tuple
    joint_upper: tuple
    gripper_closed_raw: float
    gripper_open_raw: float


PROFILES = {
    # Separate opt-in release: the original AMD64 profiles remain rollback targets.
    "x5_2025_amd64_j4_90": ControllerProfile(
        "x5_2025_amd64_j4_90", "linux-amd64", 2,
        AMD64_J4_90_SDK_SHA256, AMD64_VENDOR_SDK_SHA256,
        MODEL_SHA256, "libarx_x5_src.so", "x5_2025.urdf", AMD64_J4_90_VARIANT,
        JOINT_LOWER, JOINT_UPPER, 0.0, -3.4,
    ),
    "x5_2025_arm64_j4_90": ControllerProfile(
        "x5_2025_arm64_j4_90", "linux-arm64", 2, SDK_SHA256,
        VENDOR_SDK_SHA256, MODEL_SHA256, "libarx_x5_src-arm64.so",
        "x5_2025.urdf", SDK_VARIANT, JOINT_LOWER, JOINT_UPPER, 0.0, -3.4,
    ),
    # The same X5 2025/type=2 robot on the x86_64 Ubuntu host.  The vendor
    # AMD64 library is the unmodified locked artifact; keep its identity
    # separate from the ARM64 J4-patched build above.
    "x5_2025_amd64_type2": ControllerProfile(
        "x5_2025_amd64_type2", "linux-amd64", 2,
        "d07d53d14da17f0a1db2753e3339f9e4c055dad72df613c811b9692c4bede85c",
        "d07d53d14da17f0a1db2753e3339f9e4c055dad72df613c811b9692c4bede85c",
        MODEL_SHA256, "libarx_x5_src.so", "x5_2025.urdf", "official_amd64_type2",
        ORIGINAL_JOINT_LOWER, ORIGINAL_JOINT_UPPER, 0.0, -3.4,
    ),
    # Unmodified official AMD64 SDK, audited at the same pinned revision.
    # For a controller built with gripper end type 0 and x5.urdf.
    "x5_amd64_type0": ControllerProfile(
        "x5_amd64_type0", "linux-amd64", 0,
        "d07d53d14da17f0a1db2753e3339f9e4c055dad72df613c811b9692c4bede85c",
        "d07d53d14da17f0a1db2753e3339f9e4c055dad72df613c811b9692c4bede85c",
        "379cb7ce4718e5f4df1909287741aa46e827b3e69e77f541b1feb3a97d545370",
        "libarx_x5_src.so", "x5.urdf", "official_amd64_type0",
        ORIGINAL_JOINT_LOWER, ORIGINAL_JOINT_UPPER, 5.0, 0.0,
    ),
}


def controller_profile(value=None):
    if isinstance(value, ControllerProfile):
        if PROFILES.get(value.name) != value:
            raise ValueError("unverified controller profile")
        return value
    if isinstance(value, dict):
        value = value.get("profile")
    if value is None:
        raise ValueError("an explicit controller profile is required; there is no default")
    name = value
    if name not in PROFILES:
        raise ValueError(f"unknown controller profile: {name}")
    return PROFILES[name]


def identity(profile=None):
    """The facts the environment pins for this arm: controller profile, SDK and model, enforced limits.

    Documentation links and documented ranges are not identity; they stay out so that editing them
    never changes what an installation must match. The enforced joint limits are the contract's own
    ``joint_limits_rad`` field, pinned through ``joint_lower``/``joint_upper``, so they are not repeated.
    """
    profile = controller_profile(profile)
    return {
        "profile": profile.name,
        "target": profile.target,
        "arm_end_type": profile.end_type,
        "model_sha256": profile.model_sha256,
        "sdk_revision": SDK_REVISION,
        "sdk_sha256": profile.sdk_sha256,
        "sdk_variant": profile.variant,
        "unmodified_vendor_sdk_sha256": profile.vendor_sha256,
        "original_joint_limits_rad": {
            "lower": list(ORIGINAL_JOINT_LOWER),
            "upper": list(ORIGINAL_JOINT_UPPER),
        },
        "eef_limits": {
            "frame": "controller_eef_native",
            "order": "x_m,y_m,z_m,roll_rad,pitch_rad,yaw_rad",
            "lower": list(EEF_LOWER), "upper": list(EEF_UPPER),
        },
    }


def joint_limit_details(index, value, profile=None):
    profile = controller_profile(profile)
    return {
        "joint": index + 1,
        "target_rad": value,
        "lower_rad": profile.joint_lower[index],
        "upper_rad": profile.joint_upper[index],
        "limit_source": "rua_sdk_j4_limit_patch" if index == 3 and profile.variant in (SDK_VARIANT, AMD64_J4_90_VARIANT) else "installed_sdk_command_clipping",
        "sdk_variant": profile.variant,
        "sdk_revision": SDK_REVISION,
        "documented_lower_rad": math.radians(DOCUMENTED_JOINT_LOWER_DEG[index]),
        "documented_upper_rad": math.radians(DOCUMENTED_JOINT_UPPER_DEG[index]),
    }


def controller_identity_matches(sdk_sha256, model_sha256, profile=None):
    """An old controller must not accept targets validated against a new SDK."""
    profile = controller_profile(profile)
    return sdk_sha256 == profile.sdk_sha256 and model_sha256 == profile.model_sha256
