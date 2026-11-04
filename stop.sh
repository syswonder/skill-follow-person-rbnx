#!/usr/bin/env bash
set -uo pipefail

# Stop the Ranger follow stack: `rbnx shutdown -f <manifest>`.
#
# This does NOT stop the two sidecars the follow stack depends on -- the
# MiniCPM-RobotTrack inference server and the follow-distance intent router.
# Use scripts/shutdown-follow.sh for a complete teardown that also releases
# can_ranger and the host ports when `rbnx shutdown` leaves orphans behind.

DEPLOY_DIR="$(cd "$(dirname "$0")" && pwd)"
MANIFEST="${ROBONIX_MANIFEST:-$DEPLOY_DIR/robonix_manifest.yaml}"
export ROBONIX_DEPLOY_DIR="$DEPLOY_DIR"
export PATH="$HOME/.local/bin:$HOME/.cargo/bin:$PATH"

if command -v rbnx >/dev/null 2>&1 && [[ -f "$DEPLOY_DIR/rbnx-boot/state.json" ]]; then
  rbnx shutdown -f "$MANIFEST" "$@"
else
  echo "no Robonix boot state; nothing to stop"
fi
