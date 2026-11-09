"""Host kit for RealSense D405 cameras: identity and backend checks, native capture files and build inputs."""

from __future__ import annotations

from ...host.kit import BaseKit, Build


class RealSenseKit(BaseKit):
    def validate(self, host, config, bodies):
        super().validate(host, config, bodies)
        from .driver import validate_cameras

        for name, body in bodies.items():
            args = body["driver_args"]
            validate_cameras(args.get("cameras"))
            backend = args.get("backend", "native")
            if backend not in ("native", "pyrealsense2"):
                raise ValueError(f"{name}: unsupported camera backend")
            if backend == "native" and args.get("fps", 30) != 30:
                raise ValueError(f"{name}: the native D405 profile requires 30 FPS")

    def resources(self, body):
        cameras = body["driver_args"]["cameras"].values()
        return [("camera", spec["serial"]) for spec in cameras] + [("camera_sdk", spec["sdk_serial"]) for spec in cameras]

    def native_files(self, root):
        return {"camera_capture": root / "native/cameras/rua-camera-linux-capture",
                "rgbd_processing": root / "native/cameras/librua-rgbd-processing.so"}

    def probe(self, body):
        return list(body["driver_args"]["cameras"]), ["frame", "intrinsics"]


kit = RealSenseKit(
    name="realsense",
    factory=__package__ + ".driver:driver",
    build=Build(
        native_inputs=(("capture", "capture"), ("sdk_patch", "sdk_patch"), ("native", "native")),
        exclude=("rua-camera-linux-capture", "librua-rgbd-processing.so"),
        deb_depends=("libusb-1.0-0", "python3-pil"),
    ),
)
