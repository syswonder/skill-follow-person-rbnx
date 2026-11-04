#!/usr/bin/env bash
# Clean shutdown for the Ranger follow stack, symmetric to scripts/start_follow.sh.
#
# Run INSIDE the follow container (like start_follow.sh):
#   docker exec robonix_follow_dev bash -lc 'cd /root/deploy && bash scripts/shutdown-follow.sh'
#
# Unlike the repo-root stop.sh (which only does `rbnx shutdown`), this also stops
# the two things rbnx does NOT own, then force-kills any orphans `rbnx shutdown`
# leaves behind, so can_ranger + all host-network ports are actually released.
#
# It is safe to pkill by name here because it runs INSIDE the follow container:
# the navigation stack lives in a different container / PID namespace and cannot
# be hit by these patterns.

set -uo pipefail

DEPLOY_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RUN_DIR="$DEPLOY_DIR/.run"
MANIFEST="${ROBONIX_MANIFEST:-$DEPLOY_DIR/robonix_manifest.yaml}"
export ROBONIX_DEPLOY_DIR="$DEPLOY_DIR"
export PATH="$HOME/.local/bin:$HOME/.cargo/bin:$PATH"

ok()   { printf '\033[32m[ OK ]\033[0m %s\n' "$*"; }
warn() { printf '\033[33m[WARN]\033[0m %s\n' "$*"; }

# Match the port anywhere in the row rather than at a fixed field index: `ss`
# emits a netid column for `-tu` that it does not for `-t`, so the address is
# field 5 here and field 4 there.
port_busy() {  # $1 = port; true when something is listening on it
    ss -tulnH 2>/dev/null | awk -v p=":$1" '
        { for (i = 1; i <= NF; i++) if ($i ~ (p "$")) { found = 1; exit } }
        END { exit !found }'
}

# A busy port has two very different meanings: a leftover of OUR teardown (a real
# failure -- the next `start` collides) or the navigation stack, which shares the
# host network and legitimately owns the same control-plane ports.
#
# They are distinguishable from in here.  `ss -p` resolves a socket to a PID by
# looking that PID up in /proc, and processes in another PID namespace are not in
# our /proc.  So a bound port we can attribute to a PID is ours; one we cannot is
# not ours to complain about.
port_held_by_us() {  # $1 = port
    ss -tulnpH 2>/dev/null | awk -v p=":$1" '
        { for (i = 1; i <= NF; i++) if ($i ~ (p "$")) {
              if ($0 ~ /users:/) found = 1
              exit } }
        END { exit !found }'
}

kill_pidfile() {  # $1 = pidfile path, $2 = human label
    local f="$1" label="$2" pid
    if [[ -f "$f" ]]; then
        pid="$(cat "$f" 2>/dev/null || true)"
        if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
            kill -TERM "$pid" 2>/dev/null || true
            sleep 1
            kill -KILL "$pid" 2>/dev/null || true
            ok "killed $label (pid $pid)"
        fi
        rm -f "$f"
    fi
}

echo "== follow stack shutdown =="

# 1. Inference server + follow-distance intent router — both rbnx-external,
#    backgrounded by start_follow.sh.  kill_pidfile hits the background shell;
#    the pkill in step 4 catches the children if they outlived it.
kill_pidfile "$RUN_DIR/inference.pid" "inference server"
kill_pidfile "$RUN_DIR/semantic_intent_router.pid" "follow-distance intent router"

# 2. Clean rbnx teardown of the follow stack (chassis + camera + robottrack).
if command -v rbnx >/dev/null 2>&1 && [[ -f "$DEPLOY_DIR/rbnx-boot/state.json" ]]; then
    if rbnx shutdown -f "$MANIFEST" 2>&1 | grep -q "torn down"; then
        ok "rbnx shutdown completed"
    else
        warn "rbnx shutdown did not report clean teardown; force-killing below"
    fi
else
    warn "no rbnx boot state; skipping rbnx shutdown"
fi

# 3. rmw_zenohd router — started by start.sh, not owned by rbnx.
kill_pidfile "$RUN_DIR/rmw_zenohd.pid" "rmw_zenohd"

# 4. Sweep the container's own PID namespace.
#
#    This used to be an enumerated pkill list.  That list went stale the moment
#    the stack grew (robot_description, mid360_lidar/imu, mapping, nav2, pilot,
#    vitals, speech were all added later), and a stale list leaves orphans
#    holding ports -- 50071 stayed bound and the next `start` failed on a port
#    conflict.  Enumerating process names is a losing game; sweep instead.
#
#    SAFE: this runs INSIDE the follow container, which has its own PID
#    namespace.  Nothing here can reach the navigation container.  This script's
#    own process tree is skipped so we do not kill ourselves mid-teardown.
#
#    THE CONTAINER'S KEEP-ALIVE IS A CHILD OF PID 1, NOT PID 1 ITSELF.
#    With `--init` (see run_container.sh) PID 1 is tini, and the image's command
#    -- `tail -f /dev/null`, the one process that keeps the container alive --
#    runs as tini's child.  Killing it does not "clean up", it ends the
#    container: the next `follow.sh start` finds nothing to exec into and the
#    operator has to recreate it by hand.  So skip it too.
#
#    Identify it as PID 1's OLDEST child.  The keep-alive starts at container
#    boot; every stack process starts later, and anything our own teardown
#    re-parents onto PID 1 is younger still.  Age is therefore the one thing
#    that separates them.  Without `--init` PID 1 IS the keep-alive, this lookup
#    finds nothing, and the plain PID-1 skip below already covers it.
self=$$
parent=$PPID
keepalive="$(ps -eo pid=,ppid=,etimes= --no-headers 2>/dev/null | awk '
    $2 == 1 && $3 + 0 > best { best = $3 + 0; found = $1 }
    END { if (found != "") print found }')"
swept=0
for proc in /proc/[0-9]*; do
    pid="${proc#/proc/}"
    case "$pid" in
        ''|*[!0-9]*) continue ;;
    esac
    [[ "$pid" -eq 1 || "$pid" -eq "$self" || "$pid" -eq "$parent" ]] && continue
    [[ -n "$keepalive" && "$pid" -eq "$keepalive" ]] && continue
    # Skip our own ancestors (bash -> script), whatever their PIDs are.
    [[ -r "/proc/$pid/stat" ]] || continue
    if kill -9 "$pid" 2>/dev/null; then
        swept=$((swept + 1))
    fi
done
ok "swept ${swept} process(es) from the container PID namespace" \
   "(container keep-alive pid ${keepalive:-PID 1} preserved)"
sleep 2

# 5. Clean stale boot state so the next start.sh boot is fresh.
rm -f "$DEPLOY_DIR/rbnx-boot/state.json"

# 6. Verify the release instead of asserting it.
#
#    This used to print "can_ranger + ports released" unconditionally, which is
#    exactly the kind of claim that hides a half-dead stack: the next `start`
#    then dies on a port conflict with nothing pointing at the cause.  The
#    sweeps above are best-effort, so check the result and say so when
#    something is still holding on.
PORT_LIST=(50051 50061 50071 50081 50091 50093 7447)
still_bound=()   # ours, left behind -- a real failure
foreign=()       # someone else's, normally the navigation stack
for port in "${PORT_LIST[@]}"; do
    if port_held_by_us "$port"; then
        still_bound+=("$port")
    elif port_busy "$port"; then
        foreign+=("$port")
    fi
done
if (( ${#foreign[@]} )); then
    echo "  端口 ${foreign[*]} 被本栈以外的进程占用（导航栈共用了 host 网络，属正常）"
fi
if (( ${#still_bound[@]} )); then
    warn "本栈端口未释放: ${still_bound[*]} —— 下次 start 会因端口冲突失败"
    ss -tulnp 2>/dev/null | grep -E ":($(IFS='|'; printf '%s' "${still_bound[*]}"))\b" >&2 || true
else
    ok "ports released: ${PORT_LIST[*]}"
fi

if ip link show "${RANGER_CAN_INTERFACE:-can_ranger}" >/dev/null 2>&1; then
    ok "can_ranger 接口保留（底盘控制权已由 rbnx shutdown 释放）"
else
    warn "can_ranger 不存在了 —— 检查 USB-CAN 适配器"
fi

# Zombies are not ours to clean here: only the parent can reap, and the parent
# is PID 1.  A container with a bare keep-alive as PID 1 accumulates one per
# process this script kills, forever -- which is why pgrep is useless for
# liveness checks in here.  Recreating the container (now with --init) clears
# the backlog and stops new ones forming.
zombies="$(ps -eo stat= --no-headers 2>/dev/null | grep -c '^Z' || true)"
if [[ "${zombies:-0}" -gt 0 ]]; then
    warn "仍有 ${zombies} 个僵尸进程（PID 1 不回收，容器内无解）"
    echo "      通过 follow.sh stop 调用时，宿主机侧会自动重建容器清除；" >&2
    echo "      直接调用本脚本时，请手动执行： bash scripts/run_container.sh follow" >&2
fi

echo
if (( ${#still_bound[@]} )); then
    warn "follow stack shutdown INCOMPLETE — 见上面的未释放端口"
    exit 1
fi
ok "follow stack shutdown complete — ports released"
