#!/usr/bin/env bash
# Flip the RobotTrack command mode between dry-run and live.
#
# This is the one switch that decides whether the stack may publish velocity, so
# it validates instead of trusting the edit: it refuses to leave the manifest in
# a state that does not parse or fails the offline consistency gate, and it
# refuses to go live unless the whole velocity chain is present.
#
# Usage:  bash scripts/set_follow_mode.sh dry-run | live
#
# The change takes effect on the NEXT stack start; it does not touch a running
# stack.
set -euo pipefail

readonly SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
readonly REPO_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
readonly MANIFEST="${ROBONIX_MANIFEST:-${REPO_ROOT}/robonix_manifest.yaml}"
readonly MODE_LINE='      mode: '

die() { printf '%s\n' "$*" >&2; exit 2; }

[[ "$#" -eq 1 ]] || die "usage: $0 dry-run|live"
case "$1" in
  dry-run|live) MODE="$1" ;;
  *) die "mode must be exactly 'dry-run' or 'live', got '$1'" ;;
esac

[[ -f "$MANIFEST" && ! -L "$MANIFEST" ]] || die "manifest missing or a symlink: $MANIFEST"

current=$(grep -c "^${MODE_LINE}" "$MANIFEST" || true)
[[ "$current" -eq 1 ]] || die "expected exactly one '${MODE_LINE}' line, found ${current}"

if grep -qx "${MODE_LINE}${MODE}" "$MANIFEST"; then
  echo "already in ${MODE} mode; nothing to do"
  exit 0
fi

cp -a "$MANIFEST" "${MANIFEST}.bak.$(date +%Y%m%dT%H%M%S)"
sed -i "s|^${MODE_LINE}.*$|${MODE_LINE}${MODE}|" "$MANIFEST"
echo "manifest mode -> ${MODE}"

# Validate with the same gate CI would use.  A repair is better than a boot that
# only fails once the chassis is already claimed.
if ! PYTHONDONTWRITEBYTECODE=1 \
     python3 "${SCRIPT_DIR}/validate_manifest_consistency.py" >/tmp/set_follow_mode.check 2>&1; then
  cat /tmp/set_follow_mode.check >&2
  latest=$(ls -t "${MANIFEST}".bak.* | head -1)
  cp -a "$latest" "$MANIFEST"
  die "consistency check failed in ${MODE} mode; manifest restored from ${latest}"
fi
grep -E '^\[ OK \]' /tmp/set_follow_mode.check | sed 's/^/  /'

if [[ "$MODE" == "live" ]]; then
  cat >&2 <<'EOF'

  ============================================================
   LIVE MODE ARMED.  Before restarting the stack, confirm:
     * the navigation stack is stopped (both own can_ranger)
     * the test area is clear and the remote controller is in hand
     * the person being followed knows the robot will move
   The chassis guard starts with live mode and is the only
   writer of /cmd_vel.
  ============================================================
EOF
fi
