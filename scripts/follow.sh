#!/usr/bin/env bash
# Host-side entry point for the follow deployment.
#
#   bash scripts/follow.sh status     看一眼现在什么状态
#   bash scripts/follow.sh check      静态四项检查（起栈后、动之前跑）
#   bash scripts/follow.sh watch      开实时仪表盘（Ctrl-C 退出）
#   bash scripts/follow.sh shell      进容器交互终端（ROS 已激活）
#   bash scripts/follow.sh start      起跟随栈
#   bash scripts/follow.sh stop       停跟随栈
#   bash scripts/follow.sh live       切到"挂挡"（会提示重起）
#   bash scripts/follow.sh dry-run    切回"空挡"
#
# SAFETY: the container name is hard-coded to robonix_follow_dev.  Nothing in
# this script can touch robonix_ros_dev, the navigation container.
set -uo pipefail

readonly FOLLOW_CONTAINER=robonix_follow_dev
readonly NAV_CONTAINER=robonix_ros_dev
readonly REPO_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
readonly ROS_ENV='source /opt/ros/humble/setup.bash; export RMW_IMPLEMENTATION=rmw_zenoh_cpp'

ok()   { printf '\033[32m[ OK ]\033[0m %s\n' "$*"; }
warn() { printf '\033[33m[WARN]\033[0m %s\n' "$*"; }
bad()  { printf '\033[31m[FAIL]\033[0m %s\n' "$*"; }

# Every action here needs docker, which needs sudo.  Fail early with an
# actionable message instead of letting each sudo call fail on its own.
require_sudo() {
  sudo -n true 2>/dev/null && return 0
  if [[ -t 0 ]]; then
    echo "需要 sudo 权限访问 docker，请输入密码：" >&2
    sudo -v || { bad "无法获得 sudo 权限"; exit 1; }
    return 0
  fi
  bad "需要 sudo 权限，但当前环境无法输入密码"
  echo "     请先在终端里执行一次:  sudo -v" >&2
  echo "     （输入密码后 5 分钟内本脚本可直接使用）" >&2
  exit 1
}

# All docker calls go through this so the container name can never be mistyped.
in_follow() { sudo docker exec "$FOLLOW_CONTAINER" bash -lc "$1"; }

need_container() {
  sudo docker ps --format '{{.Names}}' | grep -qx "$FOLLOW_CONTAINER" || {
    bad "跟随容器 $FOLLOW_CONTAINER 没在运行"
    echo "     先执行:  bash scripts/run_container.sh follow" >&2
    exit 1
  }
}

# 永远只允许关自己的容器；导航容器必须原样运行。
assert_nav_untouched() {
  if ! sudo docker ps --format '{{.Names}}' | grep -qx "$NAV_CONTAINER"; then
    warn "导航容器 $NAV_CONTAINER 不在了 —— 这不是本脚本造成的，请检查"
  fi
}

# 视觉塔实际跑的是 TensorRT 还是 torch，只能问 server 自己 —— 它装上引擎时会打
# 横幅。启动器那句 `backends=trt_direct/trt_direct` 只说明**请求**了什么，而
# 引擎是第一帧才加载的：横幅还没出现，就说明这次启动还没真正用上引擎。
#
# 计数即结论：2 = 两个塔都是 TRT FP16；1 = 只有一半；0 = torch 或还没热身。
# torch 是受支持的退路（ROBOTTRACK_VISION_BACKEND=torch），不是故障。
vision_trt_towers() {
  in_follow 'grep -a -c "\[server\] \(DINO\|SigLIP\) backend = TRT FP16" /root/deploy/logs/inference.log 2>/dev/null || true' \
    | tr -d '[:space:]'
}

vision_backend_detail() {
  in_follow 'grep -a -oE "\[server\] (DINO|SigLIP) backend = .*" /root/deploy/logs/inference.log 2>/dev/null | tail -2' \
    | sed 's/^/     /'
}

cmd_status() {
  need_container
  echo "== 容器 =="
  sudo docker ps --format '  {{.Names}}\t{{.Status}}' | grep -E "$FOLLOW_CONTAINER|$NAV_CONTAINER"
  echo
  echo "== 模式 =="
  echo -n "  manifest: "; grep '^      mode:' "$REPO_ROOT/robonix_manifest.yaml" | tr -d ' '
  local live; live=$(in_follow "$ROS_ENV; timeout 8 ros2 topic info /cmd_vel 2>/dev/null" | tr '\n' ' ')
  echo "  /cmd_vel: $live"
  echo "== 视觉后端 =="
  local towers; towers="$(vision_trt_towers)"
  case "${towers:-0}" in
    2) echo "  DINO + SigLIP 都是 TensorRT FP16" ;;
    0) echo "  torch（或引擎还没加载 —— 第一帧才加载）" ;;
    *) echo "  只有 ${towers}/2 个塔是 TensorRT" ;;
  esac
  vision_backend_detail
  cat <<'EOF'

  判读:
    Publisher count: 0  -> dry-run，车不会动（安全）
    Publisher count: 1  -> live，车可以动（只有 chassis guard 在发）
    Publisher count: >1 -> 危险，有多个程序在指挥车，立刻 stop
EOF
  echo "== 底盘控制权 =="
  local ours=0 theirs=0
  for pid in $(pgrep -f 'ranger_base_node' 2>/dev/null); do
    case "$(cat /proc/$pid/cgroup 2>/dev/null | head -1)" in
      *7e99f182*) ours=$((ours+1)) ;;
      *af51b263*) theirs=$((theirs+1)) ;;
    esac
  done
  echo "  跟随栈的底盘进程: $ours"
  echo "  导航栈的底盘进程: $theirs  (必须是 0 —— 两个栈不能同时开)"
  [[ "$theirs" -eq 0 ]] || bad "导航底盘还在跑！先停导航"
}

cmd_check() {
  need_container
  echo "== 跟随栈静态检查（动之前必须全过）=="
  echo
  echo "-- 1/4 推理服务 --"
  local health
  health=$(in_follow 'curl -s --noproxy "*" --max-time 5 http://127.0.0.1:5801/health 2>/dev/null')
  if grep -q '"model_loaded": *true' <<<"$health" && grep -q '"ok": *true' <<<"$health"; then
    ok "推理服务已加载模型"
  else
    bad "推理服务异常: ${health:0:120}"
    echo "     查: sudo docker exec $FOLLOW_CONTAINER bash -lc 'tail -40 /root/deploy/logs/inference.log'"
  fi
  # /health 只证明模型在，不证明视觉引擎能用：引擎挂在第一帧的懒加载上。
  local towers; towers="$(vision_trt_towers)"
  case "${towers:-0}" in
    2) ok "视觉后端: DINO + SigLIP 都是 TensorRT FP16" ;;
    0) warn "视觉后端: 没看到 TRT 横幅 —— 要么跑的是 torch，要么引擎还没加载（跑过一帧再看）"
       echo "     查: sudo docker exec $FOLLOW_CONTAINER bash -lc 'grep -a backend /root/deploy/logs/inference.log | tail -3'" ;;
    *) warn "视觉后端: 只有 ${towers}/2 个塔是 TensorRT（混着跑：性能与上面那张表不一致）" ;;
  esac
  vision_backend_detail

  echo "-- 2/4 意图路由（Pilot 的 VLM）--"
  local models
  models=$(in_follow 'curl -s --noproxy "*" --max-time 5 http://127.0.0.1:18080/v1/models 2>/dev/null')
  if grep -q 'ranger-follow-distance-router' <<<"$models"; then
    ok "意图路由在线"
  else
    bad "意图路由无响应（语音调距会失效，但不影响跟随本身）"
  fi

  echo "-- 3/4 相机三路 --"
  for t in /camera_435i/color/image_raw /camera_435i/depth/image_raw /camera_435i/color/camera_info; do
    local hz
    hz=$(in_follow "$ROS_ENV; timeout 8 ros2 topic hz $t 2>/dev/null" | grep -m1 'average rate' | awk '{print $3}')
    if [[ -n "$hz" ]]; then ok "$t  ${hz} Hz"; else bad "$t  无数据"; fi
  done

  echo "-- 4/4 底盘唯一控制者 --"
  local info pubs
  info=$(in_follow "$ROS_ENV; timeout 8 ros2 topic info /cmd_vel 2>/dev/null")
  pubs=$(sed -n 's/.*Publisher count: \([0-9]*\).*/\1/p' <<<"$info")
  case "$pubs" in
    0) ok "/cmd_vel 发布者 0 个 —— dry-run，车不会动" ;;
    1) ok "/cmd_vel 发布者 1 个 —— live，车可以动" ;;
    *) bad "/cmd_vel 发布者 ${pubs} 个！立刻执行 bash scripts/follow.sh stop" ;;
  esac
  assert_nav_untouched
}

cmd_watch() {
  need_container
  echo "开仪表盘（Ctrl-C 退出）..." >&2
  sudo docker exec -it "$FOLLOW_CONTAINER" bash -lc \
    "cd /root/deploy && $ROS_ENV && python3 scripts/watch_follow_chain.py 2>/dev/null"
}

cmd_shell() {
  need_container
  sudo docker exec -it "$FOLLOW_CONTAINER" bash -lc "cd /root/deploy && $ROS_ENV && exec bash"
}

cmd_start() {
  need_container
  # Detect an already-running stack by the atlas PORT, and only by the port.
  #
  # Two earlier attempts were wrong:
  #   * `ros2 topic list` needs >5 s on a cold zenoh daemon and returns nothing,
  #     so it reported "not running" and the next boot collided on 50051;
  #   * `pgrep -f robonix-atlas` matches ZOMBIE processes.  The container's
  #     PID 1 is `tail -f /dev/null`, which never reaps children, so every
  #     process this stack has ever killed lingers as <defunct> and pgrep keeps
  #     reporting it as alive.  A zombie holds no port, so the port is the
  #     truth and the process table is not.
  local busy=""
  busy=$(ss -tuln 2>/dev/null | grep -c ':50051' || true)
  if [[ "${busy:-0}" -gt 0 ]]; then
    warn "跟随栈已经在运行（atlas 活着 / 50051 被占）。不需要重复启动。"
    cat <<'EOF'
     想接着用:      bash scripts/follow.sh status  然后  check
     想重起:        bash scripts/follow.sh stop    然后  start
EOF
    return 0
  fi
  echo "启动跟随栈（约 80 秒）..."
  # The host shell's environment does NOT cross `docker exec` -- verified: an
  # exported variable arrives EMPTY in the container.  So the documented
  # one-line rollback
  #     ROBOTTRACK_VISION_BACKEND=torch bash scripts/follow.sh start
  # would silently do nothing and the stack would still come up on TensorRT.
  # Forward the three vision-backend switches explicitly.  An unset host
  # variable forwards as empty, which the launcher's ${VAR:-trt_direct} reads as
  # "unset" -- so this changes nothing unless you set one on purpose.
  #
  # Putting the switch in .env works too and is the durable way to do it:
  # start_follow.sh sources .env inside the container.
  local -a backend_env=()
  local backend_var
  for backend_var in ROBOTTRACK_VISION_BACKEND ROBOTTRACK_DINO_BACKEND ROBOTTRACK_SIGLIP_BACKEND; do
    backend_env+=(-e "${backend_var}=${!backend_var:-}")
  done
  sudo docker exec -d "${backend_env[@]}" "$FOLLOW_CONTAINER" bash -lc \
    'cd /root/deploy && bash scripts/start_follow.sh > logs/start_follow.log 2>&1'
  # The budget matches start_follow.sh: its pre-flight (~5 s) plus the up-to-8 s
  # it waits for the inference server not to have died on the spot, then the
  # rbnx boot.  Wait less and the [ OK ] count below reads low on a healthy
  # stack, which is indistinguishable from a real failure.
  sleep 85
  # start_follow.sh has its own pre-flight (ports / CAN / zombies / TRT engines)
  # and refuses before booting anything; it also refuses, slightly later, if the
  # inference server dies on the spot.  Surface either refusal here instead of
  # letting the [ OK ] filter below print nothing and look like a silent hang.
  if in_follow 'tr "\r" "\n" < /root/deploy/logs/start_follow.log' | grep -q '拒绝启动'; then
    bad "起栈被拒绝（预检或推理服务启动失败），rbnx 未启动："
    in_follow 'tr "\r" "\n" < /root/deploy/logs/start_follow.log' | \
      sed -n '/拒绝启动/,/^$/p' | sed 's/^/     /'
    return 1
  fi
  in_follow 'tr "\r" "\n" < /root/deploy/logs/start_follow.log | grep -E "OK|FAIL" | tail -18'
  echo
  in_follow 'tr "\r" "\n" < /root/deploy/logs/start_follow.log | grep -c "\[ OK \]"' | \
    xargs -I{} sh -c 'echo "  OK 条数: {} （应为 16）"'
  echo "  核对基线: 先 bash scripts/follow.sh status  再 bash scripts/follow.sh check"
}

cmd_stop() {
  need_container
  echo "停跟随栈..."
  in_follow 'cd /root/deploy && bash scripts/shutdown-follow.sh' | tail -8
  assert_nav_untouched
  refresh_container
}

# `stop` leaves the container RESTARTED, not merely quiet.
#
# The reason is the teardown itself: it kills hundreds of processes, several of
# which were already orphaned and re-parented to PID 1.  Zombies can only be
# reaped by their parent, so a restart is the one cheap action that guarantees a
# clean process table -- without it, `pgrep` keeps reporting a dead stack as
# alive and the leftovers pile up run after run.
#
# `docker restart` is preferred over recreating: it cannot lose the container if
# the image or a device rule has a problem.  Recreating is reserved for the one
# thing a restart cannot change -- a PID 1 that does not reap (`--init` missing),
# which is fixed at `docker run` time and nowhere else.  The container holds no
# state, so neither path can lose anything but time.
refresh_container() {
  local init out
  init=$(sudo docker inspect "$FOLLOW_CONTAINER" --format '{{.HostConfig.Init}}' 2>/dev/null || echo "")

  if [[ "$init" != "true" ]]; then
    warn "容器 PID 1 不回收子进程（Init=$init）—— 重启改不了 PID 1，只能重建"
    if ! out=$(bash "$REPO_ROOT/scripts/run_container.sh" follow 2>&1); then
      bad "重建容器失败，请手动执行：bash scripts/run_container.sh follow"
      printf '%s\n' "$out" | tail -5 >&2
      return 1
    fi
  else
    echo "重启容器（给下一轮一个干净的进程表）…"
    # `docker restart` also starts a container that is already stopped, which is
    # the state left behind if an earlier teardown took the keep-alive with it.
    if ! sudo docker restart "$FOLLOW_CONTAINER" >/dev/null 2>&1; then
      warn "docker restart 未成功，改用 docker start 兜底"
      if ! sudo docker start "$FOLLOW_CONTAINER" >/dev/null 2>&1; then
        bad "容器启不来，请手动执行：bash scripts/run_container.sh follow"
        return 1
      fi
    fi
  fi

  local status zombies
  status=$(sudo docker inspect "$FOLLOW_CONTAINER" --format '{{.State.Status}}' 2>/dev/null || echo "?")
  zombies=$(in_follow 'ps -eo stat= --no-headers 2>/dev/null | grep -c "^Z" || true' 2>/dev/null | tr -d '[:space:]')
  [[ "$zombies" =~ ^[0-9]+$ ]] || zombies=0
  if [[ "$status" == "running" && "$zombies" -eq 0 ]]; then
    ok "容器已重启：running，Init=true，僵尸 0"
  else
    warn "容器状态异常：status=$status，僵尸=$zombies"
    return 1
  fi
}

cmd_mode() {
  local mode="$1"
  bash "$REPO_ROOT/scripts/set_follow_mode.sh" "$mode"
  if [[ "$mode" == "live" ]]; then
    cat <<EOF

  下一步（改模式必须重起才生效）:
    bash scripts/follow.sh stop
    bash scripts/follow.sh start
    bash scripts/follow.sh check
EOF
  fi
}

case "${1:-}" in
  "")     ;;  # fall through to usage
  *)      require_sudo ;;
esac

case "${1:-}" in
  status)  cmd_status ;;
  check)   cmd_check ;;
  watch)   cmd_watch ;;
  shell)   cmd_shell ;;
  start)   cmd_start ;;
  stop)    cmd_stop ;;
  live)    cmd_mode live ;;
  dry-run) cmd_mode dry-run ;;
  *)
    cat <<'EOF'
用法: bash scripts/follow.sh <命令>

  status    看一眼现在什么状态（容器 / 模式 / 底盘控制权）
  check     静态四项检查（起栈后、动之前跑）
  watch     开实时仪表盘（Ctrl-C 退出）
  shell     进容器交互终端（ROS 已激活）
  start     起跟随栈（约 80 秒）
  stop      停跟随栈
  live      切到"挂挡"（会提示重起）
  dry-run   切回"空挡"
EOF
    exit 2 ;;
esac
