# Sourced by rua-host (see host/kit.py): a no-op unless host.json configures the arx_x5 kit, and never fails.
# The X5 Body, controller and doctor commands run inside the installed ROS 2 environment.
rua_config=${RUA_HOST_CONFIG:-}
rua_previous=
for rua_argument in "$@"; do
 case "$rua_argument" in --config=*) rua_config=${rua_argument#--config=} ;; esac
 [ "$rua_previous" = --config ] && rua_config=$rua_argument
 rua_previous=$rua_argument
done
case "${RUA_HOST_COMMAND:-}" in
 body|controllers|doctor)
  rua_domain=$(/usr/bin/python3 -c 'import json,sys
print(json.load(open(sys.argv[1])).get("kits", {}).get("arx_x5", {}).get("ros_domain_id", ""))' "$rua_config" 2>/dev/null || true)
  if [ -n "$rua_domain" ]; then
   if [ -f /opt/ros/jazzy/setup.bash ]; then
    source /opt/ros/jazzy/setup.bash
    if [ -f "$RUA_RUNTIME_ROOT/native/ros/local_setup.bash" ]; then source "$RUA_RUNTIME_ROOT/native/ros/local_setup.bash"; fi
   fi
   export LD_LIBRARY_PATH="$RUA_RUNTIME_ROOT/native/ros/lib:${LD_LIBRARY_PATH:-}"
   export ROS_DOMAIN_ID=$rua_domain ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
  fi
  ;;
esac
unset rua_config rua_previous rua_argument rua_domain
