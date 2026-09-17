#!/usr/bin/env bash
set -uo pipefail

# Stop the standalone Ranger follow stack.  Minimal: the follow stack carries no
# UI dashboard or semantic-router leases (unlike robot-unitree-go2/stop.sh), so
# shutdown is just `rbnx shutdown -f <manifest>`.

DEPLOY_DIR="$(cd "$(dirname "$0")" && pwd)"
MANIFEST="${ROBONIX_MANIFEST:-$DEPLOY_DIR/robonix_manifest.yaml}"
export ROBONIX_DEPLOY_DIR="$DEPLOY_DIR"
export PATH="$HOME/.local/bin:$HOME/.cargo/bin:$PATH"

if command -v rbnx >/dev/null 2>&1 && [[ -f "$DEPLOY_DIR/rbnx-boot/state.json" ]]; then
  rbnx shutdown -f "$MANIFEST" "$@"
else
  echo "no Robonix boot state; nothing to stop"
fi
