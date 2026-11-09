#!/bin/bash
# Serve SAM 3 from its own environment. Required: ARX_SAM_PYTHON ARX_SAM_CONFIG.
set -euo pipefail
exec "$ARX_SAM_PYTHON" -m uni_robot_agent.services.perception.server --config "$ARX_SAM_CONFIG" \
  --host 127.0.0.1 --port "${ARX_SAM_PORT:-18761}"
