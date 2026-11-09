#!/bin/bash
# Linux build only; no device, bus or controller is started.
# Builds the .deb from the inputs host/build.py wrote: runs each kit's native/build.sh,
# then installs the runtime, the rua-host wrapper and the Body service unit.
set -euo pipefail
umask 022
INPUT=$(realpath "${1:?artifact input directory}")
[ "$(id -u)" != 0 ] || { echo 'Build as an ordinary user' >&2; exit 2; }
field() { /usr/bin/python3 -c 'import json,sys;v=json.load(open(sys.argv[1]))[sys.argv[2]];print(" ".join(v) if isinstance(v,list) else v)' "$INPUT/release.json" "$1"; }
TARGET=$(field target)
case "$TARGET:$(uname -m)" in
 linux-arm64:aarch64) DEB_ARCH=arm64 ;;
 linux-amd64:x86_64) DEB_ARCH=amd64 ;;
 *) echo "Release $TARGET does not match this build host" >&2; exit 2 ;;
esac
VERSION=$(field build_id)
PACKAGE=$(field package)
DESCRIPTION=$(field description)
KITS=$(field kits)
# Each entry may carry a version constraint, so join the list in Python rather than splitting words.
DEPENDS=$(/usr/bin/python3 -c 'import json,sys
# The Body framework itself needs numpy; kits declare everything else.
base = ["python3 (>= 3.12)", "python3 (<< 3.13)", "python3-numpy", "sudo"]
print(", ".join(base + json.load(open(sys.argv[1]))["deb_depends"]))' "$INPUT/release.json")
BUILD=$(mktemp -d /tmp/rua-build.XXXXXXXX)
trap 'rm -rf -- "$BUILD"' EXIT
mkdir -p "$BUILD/source" "$BUILD/work" "$BUILD/dependency-cache"
/usr/bin/python3 - "$INPUT" <<'VERIFY'
import hashlib,json,pathlib,sys
p=pathlib.Path(sys.argv[1]);m=json.loads((p/'release.json').read_text())
inputs=[('native-sources.tar.gz','native_sources_sha256'),('runtime.pyz','runtime_sha256'),('build_runtime.sh','build_script_sha256')]
# The dependency cache is optional; when release.json names one it must match.
if 'dependency_cache_sha256' in m: inputs.append(('dependency-cache.tar.gz','dependency_cache_sha256'))
for name,key in inputs:
 if key not in m or hashlib.sha256((p/name).read_bytes()).hexdigest()!=m[key]:raise SystemExit('Build input checksum mismatch: '+name)
VERIFY
/usr/bin/python3 - "$INPUT" "$BUILD" <<'UNPACK'
import sys,tarfile
from pathlib import Path
source,build=map(Path,sys.argv[1:])
for name,destination in [('native-sources.tar.gz','source'),('dependency-cache.tar.gz','dependency-cache')]:
 if (source/name).exists():
  with tarfile.open(source/name) as t:t.extractall(build/destination,filter='data')
UNPACK
if [ -f "$BUILD/source/dependencies.py" ]; then
 /usr/bin/python3 "$BUILD/source/dependencies.py" prepare --cache "$BUILD/dependency-cache" --output "$BUILD/dependencies" --target "$TARGET"
fi
PKG="$BUILD/package"
DEST="$PKG/opt/rua-hardware/releases/$VERSION"
mkdir -p "$DEST/native/bin" "$DEST/env" "$DEST/licenses" "$PKG/DEBIAN" "$PKG/usr/bin" "$PKG/usr/local/sbin" "$PKG/usr/lib/systemd/user"
install -m 0644 "$INPUT/runtime.pyz" "$INPUT/release.json" "$DEST/"
install -m 0644 "$BUILD/source/dependencies.lock.json" "$DEST/"
if [ -f "$BUILD/dependencies/dependencies.resolved.json" ]; then install -m 0644 "$BUILD/dependencies/dependencies.resolved.json" "$DEST/"; fi
for kit in $KITS; do
 SOURCE="$BUILD/source/$kit"
 if [ -f "$SOURCE/native/build.sh" ]; then
  mkdir -p "$BUILD/work/$kit"
  BUILD="$BUILD" SOURCE="$SOURCE" DEPS="$BUILD/dependencies" WORK="$BUILD/work/$kit" DEST="$DEST" PKG="$PKG" \
   TARGET="$TARGET" VERSION="$VERSION" bash "$SOURCE/native/build.sh"
 fi
 if [ -f "$SOURCE/native/env.sh" ]; then install -m 0644 "$SOURCE/native/env.sh" "$DEST/env/$kit.sh"; fi
done
# Relocatable installations contain files, never symlinks to temporary build sources.
if find "$DEST" -type l -lname "$BUILD*" | read -r _; then echo 'build path leaked into installed symlink' >&2; exit 3; fi
cat > "$PKG/usr/bin/rua-host" <<'WRAPPER'
#!/bin/bash
set -eo pipefail
export RUA_RUNTIME_ROOT=$(realpath /opt/rua-hardware/current)
export RUA_HOST_CONFIG=/etc/rua/host.json RUA_HOST_COMMAND=${1:-}
unset PYTHONHOME PYTHONPATH
# Each kit's env.sh prepares only the commands that need its native environment.
for env in "$RUA_RUNTIME_ROOT"/env/*.sh; do [ -e "$env" ] && source "$env"; done
export PYTHONDONTWRITEBYTECODE=1
exec /usr/bin/python3 "$RUA_RUNTIME_ROOT/runtime.pyz" "$@"
WRAPPER
cat > "$PKG/usr/lib/systemd/user/rua-body@.service" <<'UNIT'
[Unit]
Description=RUA installed Body %i
After=network.target
[Service]
Type=simple
ExecStart=/usr/bin/rua-host body --config /etc/rua/host.json --instance %i
WorkingDirectory=%h
Restart=no
TimeoutStopSec=15
KillMode=process
[Install]
WantedBy=default.target
UNIT
# dpkg owns the distribution archive; versioned runtime directories are
# installed by postinst. Upgrades retain files mapped by live processes.
mkdir -p "$PKG/usr/lib/rua-hardware"
tar -czf "$PKG/usr/lib/rua-hardware/release.tar.gz" -C "$PKG/opt/rua-hardware/releases" "$VERSION"
rm -rf -- "$PKG/opt"
cat > "$PKG/DEBIAN/control" <<CONTROL
Package: $PACKAGE
Version: $VERSION
Architecture: $DEB_ARCH
Maintainer: RUA maintainers
Depends: $DEPENDS
Description: $DESCRIPTION
 Bytecode Body runtime and the native components of kits: $KITS. No source checkout required.
CONTROL
cat > "$PKG/DEBIAN/postinst" <<POST
#!/bin/sh
set -e
umask 022
mkdir -p /opt/rua-hardware/releases
if [ ! -d /opt/rua-hardware/releases/$VERSION ]; then
 staged=\$(mktemp -d /opt/rua-hardware/releases/.install-XXXXXXXX)
 trap 'rm -rf -- "\$staged"' EXIT
 tar -xzf /usr/lib/rua-hardware/release.tar.gz -C "\$staged"
 mv "\$staged/$VERSION" /opt/rua-hardware/releases/$VERSION
fi
ln -sfn releases/$VERSION /opt/rua-hardware/current
# Installation never starts or restarts hardware.
POST
chmod 0755 "$PKG/usr/bin/rua-host" "$PKG/DEBIAN/postinst"
dpkg-deb --root-owner-group --build "$PKG" "$INPUT/${PACKAGE}_${VERSION}_${DEB_ARCH}.deb"
