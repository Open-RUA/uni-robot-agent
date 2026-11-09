# Sourced by rua-host (see host/kit.py): a no-op unless host.json configures the realsense kit, and never fails.
# The camera Body loads the installed librealsense.
rua_config=${RUA_HOST_CONFIG:-}
rua_previous=
for rua_argument in "$@"; do
 case "$rua_argument" in --config=*) rua_config=${rua_argument#--config=} ;; esac
 [ "$rua_previous" = --config ] && rua_config=$rua_argument
 rua_previous=$rua_argument
done
if [ "${RUA_HOST_COMMAND:-}" = body ] && /usr/bin/python3 -c 'import json,sys
sys.exit("realsense" not in json.load(open(sys.argv[1])).get("kits", {}))' "$rua_config" 2>/dev/null; then
 export LD_LIBRARY_PATH="$RUA_RUNTIME_ROOT/native/realsense/lib:${LD_LIBRARY_PATH:-}"
fi
unset rua_config rua_previous rua_argument
