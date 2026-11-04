#!/usr/bin/env bash
# Adaptive container launcher for Ranger Mini v3 (nav + follow).
# Runs on the HOST, replaces the hand-typed `docker run`.
#
# Two hardware needs the container has:
#   * camera + CAN are USB char devices  ->  --device-cgroup-rule "c <usb_major>:* rmw"
#   * inference GPU needs the nvidia runtime.  `--gpus all` is NOT usable on
#     this host any more: it dies in the prestart hook with
#     "Auto-detected mode as 'csv' / invoking the NVIDIA Container Runtime
#     Hook directly ... not supported".  The working equivalent, verified with
#     torch.cuda.is_available(), is:
#         --runtime nvidia -e NVIDIA_VISIBLE_DEVICES=all \
#         -e NVIDIA_DRIVER_CAPABILITIES=all
#     (the runtime mounts the real driver libs and opens nvhost/nvmap/
#     nvidia-caps, which a plain `-v /dev:/dev` does NOT grant — cuInit fails
#     "Operation not permitted" / "file too short" without it).
#
# The USB major is DETECTED instead of hardcoded. Gotcha: /proc/devices has TWO
# usb-ish lines — 180 "usb" (legacy printers/mice) and 189 "usb_device" (usbfs =
# /dev/bus/usb/*/*, what the camera + CAN use). Match "usb_device", never "usb".
#
# Usage:  run_container.sh nav | follow
# Idempotent: recreates the named container (docker update cannot change the
# device cgroup, so this is stop/rm/run). The image carries env/entrypoint/cmd/
# workdir; the container holds no state.
#
# `--init` is NOT optional here.  The image's PID 1 is a bare keep-alive
# (`tail -f /dev/null`), which never calls wait().  Every process the stack
# kills -- and shutdown-follow.sh kills hundreds -- is therefore re-parented to
# PID 1 and stays forever as <defunct>: a `pgrep -f` in one of the start scripts
# then reports a dead stack as alive, and /proc slowly fills with thousands of
# entries.  tini as PID 1 reaps them.  Recreating the container (which this
# script does unconditionally) is also what clears an existing zombie backlog,
# because the whole PID namespace goes away with it.
set -euo pipefail

STACK="${1:-}"
case "$STACK" in
  nav)    CONTAINER=robonix_ros_dev
          DEPLOY=/mnt/ssd/wxf/jiaofu/robot-agilex-ranger_mini_v3
          IMAGE="${ROBONIX_NAV_IMAGE:-robonix:humble-orin-deps-slim}" ;;
  follow) CONTAINER=robonix_follow_dev
          DEPLOY=/mnt/ssd/wxf/jiaofu/skill-follow-person-rbnx
          IMAGE="${ROBONIX_FOLLOW_IMAGE:-robonix:humble-orin-follow}" ;;
  *) echo "usage: $0 nav|follow" >&2; exit 2 ;;
esac
ROBONIX=/mnt/ssd/wxf/jiaofu/robonix
ORBBEC_VID="${ORBBEC_VID:-2bc5}"   # Orbbec Gemini 336L
CAN_VID="${CAN_VID:-1d50}"         # candleLight USB-CAN adapter (gs_usb)

# Re-exec under sudo if not root (matches setup-hw.sh's behaviour; no docker group).
if [[ "$(id -u)" -ne 0 ]]; then
    exec sudo -E bash "$0" "$@"
fi

# ── 1. usbfs major (detected, not hardcoded) ──────────────────────────────────
usb_major=$(awk '$2=="usb_device"{print $1; exit}' /proc/devices 2>/dev/null || true)
[[ -n "$usb_major" ]] || usb_major=189   # stable kernel USB_MAJOR fallback

# Cross-check against a real device node when one is present (source of truth).
usb_node=$(ls -l /dev/bus/usb/*/* 2>/dev/null | awk 'NR==1{sub(/,.*/,"",$5); print $5}' || true)
if [[ -n "$usb_node" && "$usb_node" != "$usb_major" ]]; then
    echo "[warn] usb: /proc/devices=$usb_major vs node=$usb_node; using $usb_node" >&2
    usb_major="$usb_node"
fi

# ── 2. present hardware (informational gate, not a hard block) ────────────────
camera=$(lsusb 2>/dev/null | grep -iE "ID ${ORBBEC_VID}:" | head -1 || true)
can=$(lsusb 2>/dev/null | grep -iE "ID ${CAN_VID}:" | head -1 || true)

echo "== adaptive container: $STACK =="
echo "  USB major       = $usb_major  (cgroup rule: c $usb_major:* rmw)"
echo "  GPU             = --runtime nvidia (NVIDIA_VISIBLE_DEVICES=all)"
[[ -n "$camera" ]] && echo "  Orbbec camera   = ${camera#*ID }" || echo "  [warn] Orbbec ($ORBBEC_VID) 未检测到"
[[ -n "$can" ]]    && echo "  USB-CAN adapter = ${can#*ID }"    || echo "  [warn] USB-CAN ($CAN_VID) 未检测到"

# ── 3. recreate the container ─────────────────────────────────────────────────
if docker ps -a --format '{{.Names}}' | grep -qx "$CONTAINER"; then
    docker stop "$CONTAINER" >/dev/null 2>&1 || true
    docker rm "$CONTAINER" >/dev/null 2>&1 || true
fi

docker run -d --name "$CONTAINER" \
    --init \
    --network host \
    -v "$DEPLOY:/root/deploy" \
    -v "$ROBONIX:/root/workspace" \
    -v /dev:/dev \
    --runtime nvidia \
    -e NVIDIA_VISIBLE_DEVICES=all \
    -e NVIDIA_DRIVER_CAPABILITIES=all \
    --device-cgroup-rule "c ${usb_major}:* rmw" \
    "$IMAGE"

docker inspect -f "  -> $CONTAINER started, DeviceCgroupRules={{.HostConfig.DeviceCgroupRules}}" "$CONTAINER"
