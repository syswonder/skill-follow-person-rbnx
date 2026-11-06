#!/usr/bin/env bash
# Feasibility probe: run the pinned MiniCPM-RobotTrack server with the
# in-container TensorRT 10.7 vision backends instead of torch, then ask it to
# evaluate one synthetic frame -- and the same frame through the torch
# backends, so the two can be compared.
#
# Runs INSIDE the follow container.  Touches no robot hardware: no ROS, no
# chassis, no /cmd_vel.  It starts the HTTP server on 127.0.0.1:5801, POSTs a
# generated JPEG to /eval_dual, prints what came back, and stops the server.
#
# Usage:  bash _probe_vision_backend.sh trt | torch
#
# Why the wheels are unpacked instead of pip-installed: the container is
# recreated from the image on every start (scripts/run_container.sh), so
# anything pip-installed lives only until the next `docker rm`.  A PYTHONPATH
# entry costs nothing and needs no rollback.

set -uo pipefail

MODE="${1:-trt}"

# Nothing is overridden here on purpose: the launcher decides the backends, so
# this exercises exactly what the deployment does.  `torch` is the documented
# rollback switch.
case "$MODE" in
  trt)   ;;
  torch) export ROBOTTRACK_VISION_BACKEND=torch ;;
  *)
    echo "usage: $0 trt|torch" >&2
    exit 2
    ;;
esac

LOG="/tmp/probe_server_${MODE}.log"
bash /root/deploy/scripts/start_robottrack_inference_server.sh >"$LOG" 2>&1 &
SERVER_PID=$!
trap 'kill "$SERVER_PID" 2>/dev/null; sleep 2' EXIT

echo "[probe] mode=$MODE  waiting for /health ..."
ready=no
for _ in $(seq 1 120); do
  if curl -sf -m 3 http://127.0.0.1:5801/health >/dev/null 2>&1; then
    ready=yes
    break
  fi
  sleep 5
done
if [[ "$ready" != yes ]]; then
  echo "[probe] server never became healthy (mode=$MODE)" >&2
  tail -25 "$LOG" >&2
  exit 1
fi

curl -s -m 5 http://127.0.0.1:5801/health
echo

for round in 1 2 3; do
  # A fresh frame each round: an identical frame would hit the server's feature
  # cache and hide the vision-backbone cost the backends are being compared on.
  ROUND="$round" python3 - <<'PY'
import cv2, numpy as np, os
shift = int(os.environ["ROUND"]) * 7
img = np.full((480, 640, 3), 44, np.uint8)
cv2.rectangle(img, (250 + shift, 150), (390 + shift, 470), (170, 150, 130), -1)
cv2.circle(img, (320 + shift, 105), 46, (205, 185, 165), -1)
cv2.line(img, (250 + shift, 470), (250 + shift, 478), (30, 30, 30), 3)
ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 92])
assert ok
open("/tmp/probe_frame.jpg", "wb").write(buf.tobytes())
PY
  response=$(curl -s -m 90 -X POST -F "image=@/tmp/probe_frame.jpg" \
    http://127.0.0.1:5801/eval_dual)
  echo "$response" >"/tmp/probe_response_${MODE}_${round}.json"
  echo "[probe] round $round (fresh frame):"
  MODE="$MODE" ROUND="$round" python3 - <<'PY'
import json, os
path = f"/tmp/probe_response_{os.environ['MODE']}_{os.environ['ROUND']}.json"
try:
    data = json.load(open(path))
except Exception as error:                       # noqa: BLE001
    print("  unparsable:", open(path).read()[:400], error)
    raise SystemExit(0)
print("  base_velocity:", data.get("base_velocity"))
for key in ("e2e_ms", "infer_time", "policy_forward_ms", "overall_latency",
            "noninfer_delay", "server_pre_infer_ms", "server_decode_ms",
            "server_route_total_ms", "history_pack_ms"):
    if key in data:
        value = data[key]
        print(f"  {key}: {round(value, 2) if isinstance(value, (int, float)) else value}")
waypoints = data.get("waypoints") or []
print("  waypoints[0:2]:", waypoints[:2])
PY
done

echo "[probe] backends actually installed (from the server's own banner):"
grep -E "\[server\] (DINO|SigLIP) backend|dino-trt|trt-direct" "$LOG" | tail -6 \
  || echo "  (no backend banner found)"
echo "[probe] server log tail:"
grep -viE "^\s*$" "$LOG" | tail -6
echo "[probe] done (mode=$MODE)"
