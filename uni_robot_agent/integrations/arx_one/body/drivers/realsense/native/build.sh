#!/bin/bash
# RealSense native build, run by build_runtime.sh with BUILD SOURCE DEPS WORK DEST PKG TARGET VERSION set.
# Installs librealsense and compiles the capture helper and the RGB-D processing library against it.
set -euo pipefail
SDK="$DEPS/realsense"
mkdir -p "$DEST/native/cameras" "$DEST/native/realsense/lib" "$DEST/licenses/librealsense"
cp "$SDK/LICENSE" "$DEST/licenses/librealsense/"
cp -a "$SDK/lib"/librealsense2.so* "$DEST/native/realsense/lib/"
c++ -std=c++17 -O2 -Wall -Wextra -pthread -isystem "$SDK/include" \
 "$SOURCE/capture/linux-capture.cpp" -L"$SDK/lib" -Wl,-rpath,'$ORIGIN/../realsense/lib' -lrealsense2 \
 -o "$DEST/native/cameras/rua-camera-linux-capture"
c++ -std=c++17 -O2 -fPIC -shared -isystem "$SDK/include" \
 "$SOURCE/capture/rgbd-processing.cpp" -L"$SDK/lib" -Wl,-rpath,'$ORIGIN/../realsense/lib' -lrealsense2 \
 -o "$DEST/native/cameras/librua-rgbd-processing.so"
"$DEST/native/cameras/rua-camera-linux-capture" --self-test
