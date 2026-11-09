#!/bin/bash
# Run the Body runtime from a user-owned release; arguments: ROOT HOST_CONFIG COMMAND...
set -eo pipefail
ROOT=$(realpath "$1")
CONFIG=$(realpath "$2")
shift 2
export RUA_RUNTIME_ROOT=$ROOT RUA_HOST_CONFIG=$CONFIG RUA_HOST_COMMAND=${1:-}
unset PYTHONHOME PYTHONPATH
# Each kit's env.sh prepares only the commands that need its native environment.
for env in "$ROOT"/env/*.sh; do [ -e "$env" ] && source "$env"; done
export PYTHONDONTWRITEBYTECODE=1
exec /usr/bin/python3 "$ROOT/runtime.pyz" "$@"
