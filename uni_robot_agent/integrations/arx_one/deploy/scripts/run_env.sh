#!/bin/bash
# Serve one physical episode. Required: ARX_CONFIG ARX_RUN_ID ARX_INSTRUCTION ARX_OUTPUT_DIR.
# Optional: ARX_DEPTH_CONFIG, ARX_ENV_PORT.
set -euo pipefail
exec "${ARX_PYTHON:-python}" -m uni_robot_agent.integrations.arx_one.environment.server \
  --config "$ARX_CONFIG" --run-id "$ARX_RUN_ID" --instruction "$ARX_INSTRUCTION" \
  --output-dir "$ARX_OUTPUT_DIR" --port "${ARX_ENV_PORT:-18760}" \
  ${ARX_DEPTH_CONFIG:+--depth-config "$ARX_DEPTH_CONFIG"}
