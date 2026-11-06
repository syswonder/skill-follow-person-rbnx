#!/usr/bin/env bash
# One-shot supervised follow launch: start the two rbnx-external sidecars (the
# MiniCPM-RobotTrack inference server and the follow-distance intent router that
# Pilot uses as its VLM), then boot the rbnx follow stack in the foreground.
#
# Usage (inside the follow container, from the repo root):
#   docker exec -it robonix_follow_dev bash -lc 'cd /root/deploy && bash scripts/start_follow.sh'
#
# SAFETY:  this boots the full follow stack (chassis + lidar + camera + mapping
# + Nav2 + robottrack) and thereby claims can_ranger.  The navigation stack MUST
# be stopped first.  The robottrack primitive defaults to mode: dry-run -- it
# predicts and logs but publishes NO velocity until a human flips mode to live.
#
# The socket names below are loopback-only by construction: the router refuses
# any non-loopback bind, and the inference server binds 127.0.0.1.

set -euo pipefail

readonly SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
readonly REPO_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
readonly RUN_DIR="${REPO_ROOT}/.run"
readonly LOG_DIR="${REPO_ROOT}/logs"

mkdir -p "${RUN_DIR}" "${LOG_DIR}"

# Machine-local overrides (VLM_BASE_URL, LIDAR_IP, Tencent credentials, ...).
if [[ -f "${REPO_ROOT}/.env" ]]; then
  set -a
  # shellcheck disable=SC1091
  source "${REPO_ROOT}/.env"
  set +a
fi

readonly INTENT_HOST="${SEMANTIC_INTENT_HOST:-127.0.0.1}"
readonly INTENT_PORT="${SEMANTIC_INTENT_PORT:-18080}"

# The router's mode MUST follow the stack's mode.  A live stack with a preview
# router would recognize "靠近一点", then refuse to call the capability, so the
# voice command would silently do nothing.  Deriving it from the manifest means
# scripts/set_follow_mode.sh is the single switch for both.
readonly MANIFEST="${ROBONIX_MANIFEST:-${REPO_ROOT}/robonix_manifest.yaml}"
# dry-run -> preview (recognize but never call); live -> live (may call).
manifest_mode="$(sed -n 's/^      mode: *\(dry-run\|live\)$/\1/p' "$MANIFEST" | head -1)"
case "${manifest_mode:-dry-run}" in
  live) INTENT_MODE=live ;;
  *)    INTENT_MODE=preview ;;
esac
readonly INTENT_MODE
if [[ -n "${SEMANTIC_INTENT_EXECUTION_MODE:-}" && \
      "${SEMANTIC_INTENT_EXECUTION_MODE}" != "${INTENT_MODE}" ]]; then
  echo "NOTE: .env 的 SEMANTIC_INTENT_EXECUTION_MODE=${SEMANTIC_INTENT_EXECUTION_MODE}" >&2
  echo "      已忽略；路由器改跟栈模式 (${INTENT_MODE})，避免'听懂但不执行'。" >&2
fi

# --- 视觉后端 ---------------------------------------------------------------
#
# 推理服务默认跑本机 TensorRT 10.7 的 FP16 engine（DINOv3 + SigLIP），torch 是
# Go2 参考启动器自带的退路，一条环境变量就能切回去。详见
# scripts/start_robottrack_inference_server.sh 和 docs/specified-person-follow.md
# Step 15。
#
# engine 不在起栈时加载 —— server 的视觉缓存是第一帧才建的（_ensure_vision_cache
# 是懒加载），所以"起栈成功"并不等于"引擎能用"。等第一帧才发现失败时，人已经
# 站在车边上了。这里把 engine 的存在性提到起栈之前查。
readonly VISION_BACKEND="${ROBOTTRACK_VISION_BACKEND:-trt_direct}"
readonly TRT_ROOT="${ROBOTTRACK_TRT_ROOT:-/root/tensorrt10.7/TensorRT-10.7.0.23}"
readonly UPSTREAM_ROOT="${ROBOTTRACK_UPSTREAM_ROOT:-${REPO_ROOT}/upstream/MiniCPM-Robot}"
readonly TRT_ARTIFACT_DIR="${UPSTREAM_ROOT}/MiniCPM-RobotTrack/realworld/trt_artifacts"
readonly TRT_ENGINES=(
  "dino_patch_target_fp16.engine"
  "siglip_pooled_target_maxn_fp16.engine"
)

probe_http() {  # $1 = url
  curl -sf --max-time 2 "$1" >/dev/null 2>&1
}

wait_http() {  # $1 = url, $2 = seconds, $3 = label
  local deadline=$(( SECONDS + $2 ))
  while (( SECONDS < deadline )); do
    probe_http "$1" && return 0
    sleep 0.25
  done
  echo "$3 did not answer $1 within $2 s" >&2
  return 1
}

# --- 0. Pre-flight ----------------------------------------------------------
#
# The stack binds host-network ports and owns can_ranger.  A leftover process
# from a previous run, or the navigation stack (which shares the same host
# network), therefore makes `rbnx boot` die halfway with a port conflict --
# after the chassis may already have been claimed.  Refuse up front instead.
#
# The port table is also the only way to see the navigation stack from in here:
# the container has its own PID namespace, so nav's processes are invisible to
# pgrep, but `--network host` means its ports are not.  Both stacks bind 50051,
# so a busy 50051 is a nav stack or a stale follow stack -- never a false alarm.
readonly RBNX_PORTS=(50051 50061 50071 50081 50091 50093)
readonly CAN_IFACE="${RANGER_CAN_INTERFACE:-can_ranger}"

# Match the port anywhere in the row rather than at a fixed field index: `ss`
# emits a netid column for `-tu` that it does not for `-t`, so the address is
# field 5 here and field 4 there.
port_busy() {  # $1 = port; true when something is listening on it
    ss -tulnH 2>/dev/null | awk -v p=":$1" '
        { for (i = 1; i <= NF; i++) if ($i ~ (p "$")) { found = 1; exit } }
        END { exit !found }'
}

busy_ports=()
for port in "${RBNX_PORTS[@]}"; do
    if port_busy "$port"; then
        busy_ports+=("$port")
    fi
done
if (( ${#busy_ports[@]} )); then
    echo "拒绝启动：这些端口已被占用 -> ${busy_ports[*]}" >&2
    echo "  跟随栈上一轮没停干净  ->  bash scripts/shutdown-follow.sh" >&2
    echo "  导航栈还在跑（共用 host 网络，两栈都要 can_ranger，不能同开）" >&2
    echo "                        ->  先在导航侧停掉" >&2
    ss -tulnp 2>/dev/null | grep -E ":($(IFS='|'; printf '%s' "${busy_ports[*]}"))\b" >&2 || true
    exit 1
fi

# Zombies are the fingerprint of a container whose PID 1 does not call wait().
# They cannot be reaped from here (only the parent can) -- but they matter,
# because a `pgrep -f rbnx` then reports a long-dead stack as still running.
# Recreating the container is what clears them, so point at that, not at pkill.
stale="$(ps -eo stat= --no-headers 2>/dev/null | grep -c '^Z' || true)"
if [[ "${stale:-0}" -gt 0 ]]; then
    echo "注意：容器里有 ${stale} 个僵尸进程（<defunct>）。" >&2
    echo "      它们不占端口、不占 CPU，但会让 pgrep 把死掉的栈误判成活的。" >&2
    echo "      清掉的办法是重建容器（会换掉 PID 1，新容器带 --init 不再产生僵尸）：" >&2
    echo "        bash scripts/run_container.sh follow" >&2
fi

# can_ranger must exist before anything else; start.sh brings it UP, but it
# cannot conjure an interface that the USB-CAN adapter never created.
if ! ip link show "$CAN_IFACE" >/dev/null 2>&1; then
    echo "拒绝启动：找不到 CAN 接口 $CAN_IFACE（USB-CAN 适配器没插好或驱动没加载）" >&2
    exit 1
fi

# 视觉后端选了 TensorRT 就必须现在确认 engine 和 10.7 都在。DinoTrtDirect 在
# 第一帧才构造，缺文件时会以"第一帧 500"的形式出现，而不是起栈失败。
if [[ "${VISION_BACKEND}" != "torch" ]]; then
    missing=()
    for engine in "${TRT_ENGINES[@]}"; do
        [[ -s "${TRT_ARTIFACT_DIR}/${engine}" ]] || missing+=("${engine}")
    done
    if (( ${#missing[@]} )); then
        echo "拒绝启动：视觉后端选了 ${VISION_BACKEND}，但 engine 不在 ${TRT_ARTIFACT_DIR}" >&2
        printf '  缺: %s\n' "${missing[@]}" >&2
        echo "  回退到 torch 后端（慢约 40%，行为不变，不用改文件）：" >&2
        echo "    ROBOTTRACK_VISION_BACKEND=torch bash scripts/start_follow.sh" >&2
        exit 1
    fi
    if [[ ! -d "${TRT_ROOT}/lib" ]]; then
        echo "拒绝启动：视觉后端选了 ${VISION_BACKEND}，但容器里的 TensorRT 10.7 树不在" >&2
        echo "  （${TRT_ROOT}）。镜像被换过？重建容器前先确认它还在。" >&2
        echo "    ROBOTTRACK_VISION_BACKEND=torch bash scripts/start_follow.sh" >&2
        exit 1
    fi
else
    echo "注意：视觉后端被显式切成 torch（ROBOTTRACK_VISION_BACKEND=torch），不校验 engine。" >&2
fi

# --- 1. MiniCPM-RobotTrack inference server (background, 127.0.0.1:5801) -----
if probe_http "http://127.0.0.1:5801/health" || probe_http "http://127.0.0.1:5801"; then
  echo "inference server already responding on :5801" >&2
else
  echo "starting MiniCPM-RobotTrack inference server (background, vision=${VISION_BACKEND})" >&2
  bash "${SCRIPT_DIR}/start_robottrack_inference_server.sh" \
    >"${LOG_DIR}/inference.log" 2>&1 &
  inference_pid=$!
  echo "$inference_pid" >"${RUN_DIR}/inference.pid"

  # Wait only for the failures that happen NOW: a missing TensorRT tree or a
  # wheel that will not unpack makes the launcher exit within a second or two.
  # Engine loading and model loading are deliberately NOT waited for -- the
  # server warms up in parallel with the rbnx boot, and follow.sh budgets the
  # whole start at a fixed wall-clock.  Hence the short bound: it is 15 tries of
  # 0.5 s, not a health wait, and follow.sh's budget accounts for the extra 8 s.
  #
  # Note the engine load itself would not show up here even with a long wait:
  # the vision cache is built lazily on the FIRST frame, so a broken TensorRT
  # install surfaces as a failed first inference, not as a dead server.  The
  # engine pre-flight above is what covers that case.
  #
  # Without this the stack boots with a dead inference server: the follow bridge
  # then gets no plan at all, and in dry-run nothing says so.
  for _ in $(seq 1 15); do
    kill -0 "$inference_pid" 2>/dev/null || break
    probe_http "http://127.0.0.1:5801/health" && break
    sleep 0.5
  done
  if ! kill -0 "$inference_pid" 2>/dev/null; then
    echo "拒绝启动：推理服务起不来（vision=${VISION_BACKEND}）。日志尾部：" >&2
    tail -25 "${LOG_DIR}/inference.log" >&2 || true
    echo "  确认是后端的问题就切回 torch 再起：" >&2
    echo "    ROBOTTRACK_VISION_BACKEND=torch bash scripts/start_follow.sh" >&2
    exit 1
  fi
fi

# --- 2. Follow-distance intent router (background, 127.0.0.1:18080) ---------
# Pilot's vlm.upstream points here (see VLM_BASE_URL).  Without it, voice
# distance commands ("靠近一点") cannot reach the follow_distance capability.
if probe_http "http://${INTENT_HOST}:${INTENT_PORT}/v1/models"; then
  echo "intent router already responding on :${INTENT_PORT}" >&2
else
  echo "starting follow-distance intent router (background, mode=${INTENT_MODE})" >&2
  SEMANTIC_INTENT_HOST="${INTENT_HOST}" \
  SEMANTIC_INTENT_PORT="${INTENT_PORT}" \
  SEMANTIC_INTENT_EXECUTION_MODE="${INTENT_MODE}" \
    nohup bash "${REPO_ROOT}/packages/semantic_intent_router/scripts/start.sh" \
    >"${LOG_DIR}/semantic_intent_router.log" 2>&1 &
  echo "$!" >"${RUN_DIR}/semantic_intent_router.pid"
  wait_http "http://${INTENT_HOST}:${INTENT_PORT}/v1/models" 10 \
    "intent router" || {
      tail -40 "${LOG_DIR}/semantic_intent_router.log" >&2 || true
      exit 1
    }
fi

# Pilot will call whatever VLM_BASE_URL points at, so it must point here or the
# voice path silently does nothing.
if [[ "${VLM_BASE_URL:-}" != "http://${INTENT_HOST}:${INTENT_PORT}/v1" ]]; then
  echo "WARNING: VLM_BASE_URL is '${VLM_BASE_URL:-<unset>}' but the intent" >&2
  echo "         router is on http://${INTENT_HOST}:${INTENT_PORT}/v1." >&2
  echo "         Set VLM_BASE_URL in .env or voice commands will not reach it." >&2
fi

# --- 3. The rbnx stack itself (foreground) ---------------------------------
exec bash "${REPO_ROOT}/start.sh"
