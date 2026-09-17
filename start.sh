#!/usr/bin/env bash
set -euo pipefail

# Start the standalone Ranger follow stack (chassis + camera + robottrack) in
# the robonix container.  Mirrors robot-agilex-ranger_mini_v3/start.sh, minus
# the Piper arm and the nav2/mapping services this stack does not carry.
#
# SAFETY:  the follow stack owns the can_ranger bus and /cmd_vel.  It MUST NOT
# run at the same time as the navigation stack.  Confirm the navigation stack
# is stopped (`rbnx shutdown -f .../robonix_manifest.no-arm.yaml` in that repo)
# before starting this one.

DEPLOY_DIR="$(cd "$(dirname "$0")" && pwd)"
RUN_DIR="$DEPLOY_DIR/.run"
LOG_DIR="$DEPLOY_DIR/logs"
mkdir -p "$RUN_DIR" "$LOG_DIR"

# Machine-local overrides live outside the manifest and Git.
if [[ -f "$DEPLOY_DIR/.env" ]]; then
  set -a
  # shellcheck disable=SC1091
  source "$DEPLOY_DIR/.env"
  set +a
fi

export PATH="$HOME/.cargo/bin:$PATH"
export ROBONIX_DEPLOY_DIR="$DEPLOY_DIR"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_zenoh_cpp}"

set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
set -u

router_pid=""
MANIFEST="${ROBONIX_MANIFEST:-$DEPLOY_DIR/robonix_manifest.yaml}"
stack_started=0

can_ready() {
  local iface="$1"
  local bitrate="$2"
  local detail
  detail="$(ip -details link show "$iface" 2>/dev/null)" || return 1
  [[ "${detail%%$'\n'*}" == *"state UP"* ]] && [[ "$detail" == *"bitrate $bitrate"* ]]
}

prepare_ranger_can() {
  local iface="${RANGER_CAN_INTERFACE:-can_ranger}"
  local bitrate="${RANGER_CAN_BITRATE:-500000}"
  rg -q '^[[:space:]]*- name:[[:space:]]+ranger_chassis([[:space:]]|$)' "$MANIFEST" || return 0
  can_ready "$iface" "$bitrate" && return 0

  echo "configuring Ranger CAN $iface at $bitrate bps" >&2
  local -a elevate=()
  if [[ "$EUID" -ne 0 ]]; then
    elevate=(sudo)
  fi
  "${elevate[@]}" ip link set "$iface" down
  "${elevate[@]}" ip link set "$iface" type can bitrate "$bitrate"
  "${elevate[@]}" ip link set "$iface" up
  can_ready "$iface" "$bitrate" || {
    echo "Ranger CAN $iface is not UP at $bitrate bps" >&2
    exit 1
  }
}

cleanup() {
  local status=$?
  trap - EXIT INT TERM
  if [[ "$stack_started" == "1" && -f "$DEPLOY_DIR/rbnx-boot/state.json" ]]; then
    rbnx shutdown -f "$MANIFEST" || true
  fi
  if [[ -n "$router_pid" ]]; then
    kill -TERM "$router_pid" 2>/dev/null || true
    wait "$router_pid" 2>/dev/null || true
  fi
  exit "$status"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

prepare_ranger_can

if [[ "$RMW_IMPLEMENTATION" == "rmw_zenoh_cpp" ]]; then
  router_bin="/opt/ros/humble/lib/rmw_zenoh_cpp/rmw_zenohd"
  [[ -x "$router_bin" ]] || {
    echo "missing $router_bin; install ros-humble-rmw-zenoh-cpp" >&2
    exit 1
  }
  if ! python3 - <<'PY'
import socket
try:
    with socket.create_connection(("127.0.0.1", 7447), timeout=0.25):
        pass
except OSError:
    raise SystemExit(1)
PY
  then
    "$router_bin" >"$LOG_DIR/rmw_zenohd.log" 2>&1 &
    router_pid=$!
    echo "$router_pid" >"$RUN_DIR/rmw_zenohd.pid"
    for _ in $(seq 1 40); do
      if python3 - <<'PY'
import socket
try:
    with socket.create_connection(("127.0.0.1", 7447), timeout=0.25):
        pass
except OSError:
    raise SystemExit(1)
PY
      then
        break
      fi
      if ! kill -0 "$router_pid" 2>/dev/null; then
        tail -80 "$LOG_DIR/rmw_zenohd.log" >&2 || true
        exit 1
      fi
      sleep 0.25
    done
  fi
fi

stack_started=1
rbnx boot --no-update-check -f "$MANIFEST" "$@"
