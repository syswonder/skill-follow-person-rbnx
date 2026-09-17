#!/usr/bin/env bash
# One-shot supervised follow launch: start the MiniCPM-RobotTrack inference
# server in the background, then boot the ranger follow stack in the foreground.
#
# Usage (inside the robonix container, from the repo root):
#   sudo docker exec -it robonix_ros_dev bash -lc 'cd /root/<mount>/ && bash scripts/start_follow.sh'
#
# SAFETY:  this boots the full follow stack (chassis + camera + robottrack) and
# thereby claims can_ranger and /cmd_vel.  The navigation stack MUST be stopped
# first.  The robottrack primitive defaults to mode: dry-run — it predicts and
# logs but publishes NO velocity until a human flips mode to live.

set -euo pipefail

readonly SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
readonly REPO_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"

# Start the inference server if :5801 is not already answering.
if ! curl -sf --max-time 2 "http://127.0.0.1:5801/health" >/dev/null 2>&1 \
   && ! curl -sf --max-time 2 "http://127.0.0.1:5801" >/dev/null 2>&1; then
  echo "starting MiniCPM-RobotTrack inference server (background)" >&2
  bash "${SCRIPT_DIR}/start_robottrack_inference_server.sh" \
    >"${REPO_ROOT}/logs/inference.log" 2>&1 &
  echo "$!" >"${REPO_ROOT}/.run/inference.pid"
else
  echo "inference server already responding on :5801" >&2
fi

exec bash "${REPO_ROOT}/start.sh"
