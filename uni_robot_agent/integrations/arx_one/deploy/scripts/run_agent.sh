#!/bin/bash
# Run the agent against a served episode, outside any repository, with its own Codex home.
# Required: ARX_RUN_DIR ARX_MODEL. Extra arguments are passed to the agent command.
set -euo pipefail
mkdir -p "$ARX_RUN_DIR/codex-home" "$ARX_RUN_DIR/workspace"
export CODEX_HOME="$ARX_RUN_DIR/codex-home"
cd "$ARX_RUN_DIR/workspace"
# ARX_ENTRY selects another launcher for the same command line.
exec "${ARX_PYTHON:-python}" ${ARX_ENTRY:--m uni_robot_agent.cli} run --integration arx_one --agent codex \
  --model "$ARX_MODEL" --env-url "http://127.0.0.1:${ARX_ENV_PORT:-18760}" \
  --sam3-url "http://127.0.0.1:${ARX_SAM_PORT:-18761}" \
  --episode-timeout-s 5400 --max-turns 300 --output-dir "$ARX_RUN_DIR/agent" "$@"
