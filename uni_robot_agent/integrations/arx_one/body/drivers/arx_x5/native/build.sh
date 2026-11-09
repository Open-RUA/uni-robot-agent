#!/bin/bash
# ARX X5 native build, run by build_runtime.sh with BUILD SOURCE DEPS WORK DEST PKG TARGET VERSION set.
# Builds and tests the ROS 2 controller packages and installs slcand and the rua-can helper.
set -euo pipefail
set +u # ROS setup scripts read optional, unset environment variables.
source /opt/ros/jazzy/setup.bash
set -u
cd "$WORK"
colcon build --merge-install --base-paths "$SOURCE/ros/src" --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON -DRUA_DEPENDENCIES_ROOT="$DEPS"
colcon test --merge-install --base-paths "$SOURCE/ros/src" --packages-select arx_x5_controller
colcon test-result --verbose
mkdir -p "$DEST/native/ros" "$DEST/native/bin" "$DEST/licenses/arx" "$DEST/licenses/eigen" "$DEST/licenses/can-utils"
cp -a "$WORK/install/." "$DEST/native/ros/"
cp "$DEPS/can-utils/usr/bin/slcand" "$DEST/native/bin/"
chmod 0755 "$DEST/native/bin/slcand"
cp "$DEPS/arx/LICENSE" "$SOURCE/ros/NOTICE" "$DEST/licenses/arx/"
cp "$DEPS/eigen"/COPYING* "$DEST/licenses/eigen/"
cp "$DEPS/can-utils/usr/share/doc/can-utils/copyright" "$DEST/licenses/can-utils/"
mkdir -p "$PKG/usr/local/sbin"
cat > "$PKG/usr/local/sbin/rua-can" <<'CAN'
#!/bin/bash
set -euo pipefail
ROOT=$(realpath /opt/rua-hardware/current)
exec env -i PATH=/usr/sbin:/usr/bin:/sbin:/bin RUA_RUNTIME_ROOT="$ROOT" /usr/bin/python3 -I "$ROOT/runtime.pyz" can "$@"
CAN
chmod 0755 "$PKG/usr/local/sbin/rua-can"
