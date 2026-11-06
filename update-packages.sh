#!/usr/bin/env bash
# update-packages.sh — 按部署 manifest 里声明的 url/branch，把**所有** url: 包
# （原语 / 服务 / 技能）更新到上游最新；已经是最新的直接跳过。
#
# 背景：manifest 里用 `url:` 引入的包只在第一次 rbnx build/boot 时被
#   git clone --depth 1 --branch <branch> 到 <manifest目录>/rbnx-boot/cache/<仓库名>/
# 之后一直复用那份 checkout，**不会自动更新**。rbnx boot / rbnx build 默认只打印
# 一行"落后 N 个 commit"的非阻塞提示，而本部署的 start*.sh / rebuild*.sh 都带了
# --no-update-check，连提示都被跳过。
#
# 更新语义与官方 `rbnx update` 一致：
#   git fetch --quiet --depth 200 origin <branch>  →  git merge --ff-only FETCH_HEAD
# 只做 fast-forward，绝不 reset、绝不覆盖本地改动：
#   - 本地有内容改动的文件会被列出来；git 只在"不会丢掉这些改动"时才前进，
#     若上游也改了同一文件，ff 直接中止（实验验证过），本地改动保持原样。
#   - cache 是从别的机器搬来的，大量文件只有可执行位差异，检查/合并都用
#     core.fileMode=false 忽略掉，避免把权限位噪声误判成冲突。
#   - remote 与 manifest 的 url 不一致、历史分叉，都只报告不处理。
#
# 本脚本不修改任何 robonix 源码，也不改 manifest；只操作 cache 里的 git checkout。
#
# 用法：
#   bash update-packages.sh           # 检查全部 url: 包，有更新就拉，没有就跳过
#   bash update-packages.sh --check   # 只检查（fetch + 报告差异），不做任何改动
#   bash update-packages.sh -l        # 列出 manifest 里的条目和当前 checkout 状态（离线）
#
# 可选：
#   -f, --manifest <path>   指定 manifest。默认顺序：$ROBONIX_MANIFEST（本部署的
#                           start*.sh / build.sh 用它选部署）→ ./robonix_manifest.yaml
#                           → ./robonix_manifest.no-arm.yaml
#       --clone             cache 里还不存在的包，直接按 manifest 的 url/branch 克隆
#       --force             忽略"checkout 的 origin 与 manifest 不一致"的检查（其他保护不变，
#                           仍然只做 ff，不 reset、不覆盖本地改动）
#   -h, --help              显示本帮助
#
# 环境变量：
#   FETCH_TIMEOUT_S=<秒>    单个包 fetch 的超时，默认 60。网络慢导致 fetch 超时时脚本
#                           会跳过该包（不判失败），调大这个值重跑即可。
set -euo pipefail

# ── 输出 ─────────────────────────────────────────────────────────────────────
C_OK=$'\033[32m'; C_WARN=$'\033[33m'; C_ERR=$'\033[31m'; C_DIM=$'\033[2m'; C_OFF=$'\033[0m'
if [ ! -t 1 ]; then C_OK=; C_WARN=; C_ERR=; C_DIM=; C_OFF=; fi
info() { printf '%s\n' "$*"; }
dim()  { printf '%s%s%s\n' "$C_DIM" "$*" "$C_OFF"; }
ok()   { printf '%s[ok]%s %s\n' "$C_OK" "$C_OFF" "$*"; }
warn() { printf '%s[warn]%s %s\n' "$C_WARN" "$C_OFF" "$*" >&2; }
err()  { printf '%s[err]%s %s\n' "$C_ERR" "$C_OFF" "$*" >&2; }

# 帮助文本 = 文件顶部的连续注释块（跳过 shebang 行），改注释不用同步改代码。
usage() { awk 'NR>1 && /^#/ {sub(/^# ?/, ""); print; next} NR>1 {exit}' "$0"; exit "${1:-0}"; }

# ── 参数 ─────────────────────────────────────────────────────────────────────
MANIFEST=""
LIST_ONLY=0
CHECK_ONLY=0
DO_CLONE=0
FORCE=0
FETCH_TIMEOUT_S="${FETCH_TIMEOUT_S:-60}"   # 单个包 fetch 的超时（秒），可用环境变量覆盖

while [ $# -gt 0 ]; do
  case "$1" in
    -f|--manifest) MANIFEST="${2:-}"; [ -n "$MANIFEST" ] || { err "--manifest 需要参数"; exit 2; }; shift 2 ;;
    -l|--list)     LIST_ONLY=1; shift ;;
    -n|--dry-run|--check) CHECK_ONLY=1; shift ;;
    --clone)       DO_CLONE=1; shift ;;
    --force)       FORCE=1; shift ;;
    -h|--help)     usage 0 ;;
    -*)            err "未知选项: $1"; usage 2 ;;
    *)             err "本脚本自动处理 manifest 里的全部 url: 包，不需要传名称（收到: $1）"
                   dim "  只想看差异：bash $0 --check    只看条目清单：bash $0 -l" >&2
                   exit 2 ;;
  esac
done

# manifest 默认：先跟随本部署脚本用的 $ROBONIX_MANIFEST（start*.sh / build.sh 都用它，
# start-no-arm.sh 会把它设成 no-arm 变体），其次 ./robonix_manifest.yaml，再次 no-arm。
if [ -z "$MANIFEST" ]; then
  if [ -n "${ROBONIX_MANIFEST:-}" ] && [ -f "$ROBONIX_MANIFEST" ]; then
    MANIFEST="$ROBONIX_MANIFEST"
  else
    for c in robonix_manifest.yaml robonix_manifest.no-arm.yaml; do
      [ -f "$c" ] && { MANIFEST="$c"; break; }
    done
  fi
fi
[ -n "$MANIFEST" ] && [ -f "$MANIFEST" ] || { err "找不到 manifest（用 -f 指定）"; exit 1; }
MANIFEST="$(cd "$(dirname "$MANIFEST")" && pwd)/$(basename "$MANIFEST")"
MANIFEST_DIR="$(dirname "$MANIFEST")"
CACHE_ROOT="$MANIFEST_DIR/rbnx-boot/cache"

command -v git >/dev/null || { err "缺少 git"; exit 1; }
command -v python3 >/dev/null || { err "缺少 python3"; exit 1; }

# ── 解析 manifest ────────────────────────────────────────────────────────────
# 输出用 US(0x1f) 分隔：section name kind(url|path|none) url branch path。
# 不能用 tab —— bash 的 read 在 IFS 含空白字符时会折叠连续分隔符，空字段会错位。
parse_manifest() {
  python3 - "$MANIFEST" <<'PY'
import sys
try:
    import yaml
except ImportError:
    sys.stderr.write("需要 python3 的 yaml 模块（pip install pyyaml）\n")
    sys.exit(3)
SEP = "\x1f"
with open(sys.argv[1], encoding="utf-8") as f:
    root = yaml.safe_load(f) or {}
for section in ("primitive", "service", "skill"):
    entries = root.get(section) or []
    if not isinstance(entries, list):
        continue
    for e in entries:
        if not isinstance(e, dict):
            continue
        url, path = e.get("url"), e.get("path")
        kind = "url" if url else ("path" if path else "none")
        print(SEP.join([
            section,
            str(e.get("name") or "(unnamed)"),
            kind,
            str(url or ""),
            str(e.get("branch") or ""),
            str(path or ""),
        ]))
PY
}

PARSED="$(parse_manifest)" || { err "解析 $MANIFEST 失败（见上方错误）"; exit 1; }

SECTIONS=(); PKGNAMES=(); KINDS=(); URLS=(); BRANCHES=(); PATHS=(); COUNT=0
while IFS=$'\x1f' read -r s n k u b p; do
  [ -n "$s" ] || continue
  SECTIONS[$COUNT]="$s"; PKGNAMES[$COUNT]="$n"; KINDS[$COUNT]="$k"
  URLS[$COUNT]="$u"; BRANCHES[$COUNT]="$b"; PATHS[$COUNT]="$p"
  COUNT=$((COUNT + 1))
done <<<"$PARSED"

[ "$COUNT" -gt 0 ] || { err "$MANIFEST 里没有 primitive/service/skill 条目"; exit 1; }

# rbnx 的 cache 目录名规则：取 url 最后一段，去掉结尾的 '/' 和 '.git'
# （对应 tools/rbnx/src/manifest.rs 的 deploy_repo_dir_name）。
repo_dir_name() {
  local u="${1%/}"
  u="${u%.git}"
  printf '%s' "${u##*/}"
}

entry_dir()  { printf '%s' "$CACHE_ROOT/$(repo_dir_name "${URLS[$1]}")"; }

# 所有针对某个 checkout 的 git 调用都走这个包装。
# 为什么：在容器里以 root 跑时，cache 目录的属主是宿主用户（uid 1000，这些目录是
# 从别的机器搬过来的），git 的 safe.directory 保护会判定 "dubious ownership" 并
# 拒绝一切操作（fetch 直接 128 退出）。这里只对"本脚本正要操作的那一个目录"加一次
# 例外，等价于 git 自己提示的 `git config --global --add safe.directory <dir>`，
# 但不动任何全局配置、也不留副作用。
git_in() {
  local _dir="$1"; shift
  git -c safe.directory="$_dir" -C "$_dir" "$@"
}

entry_head() {
  local d; d="$(entry_dir "$1")"
  [ -d "$d/.git" ] && git_in "$d" log -1 --format='%h %cd' --date=short 2>/dev/null || true
}

# ── -l：列出 manifest 全部条目（离线，不 fetch） ─────────────────────────────
print_names() {
  local i wsec=0 wname=0
  for i in $(seq 0 $((COUNT - 1))); do
    [ ${#SECTIONS[$i]} -gt $wsec ] && wsec=${#SECTIONS[$i]}
    [ ${#PKGNAMES[$i]} -gt $wname ] && wname=${#PKGNAMES[$i]}
  done
  info "manifest: $MANIFEST"
  info "cache   : $CACHE_ROOT"
  info ""
  printf '%-*s  %-*s  %-6s  %-14s  %s\n' "$wsec" "SECTION" "$wname" "NAME" "KIND" "BRANCH" "当前 checkout"
  for i in $(seq 0 $((COUNT - 1))); do
    if [ "${KINDS[$i]}" = "url" ]; then
      local d state; d="$(entry_dir "$i")"
      if   [ -d "$d/.git" ]; then state="$(entry_head "$i")"
      elif [ -d "$d" ];       then state="cache 目录存在但不是 git checkout"
      else                         state="未 clone（需先 rbnx build）"
      fi
      printf '%-*s  %-*s  %-6s  %-14s  %s\n' \
        "$wsec" "${SECTIONS[$i]}" "$wname" "${PKGNAMES[$i]}" "url" "${BRANCHES[$i]:-<默认分支>}" "$state"
    else
      printf '%-*s  %-*s  %-6s  %-14s  %s\n' \
        "$wsec" "${SECTIONS[$i]}" "$wname" "${PKGNAMES[$i]}" "${KINDS[$i]}" "-" "${PATHS[$i]:-无 url 也无 path}"
    fi
  done
  info ""
  dim "KIND=path 的条目是本地包（如 ./skills/guided_tour、\${ROBONIX_SOURCE_PATH}/services/speech），"
  dim "manifest 里没有 url/branch，本脚本不处理（它们随部署仓库或 robonix 源码树走）。"
}

if [ "$LIST_ONLY" = 1 ]; then print_names; exit 0; fi

# ── 收集要处理的 url: 条目 ───────────────────────────────────────────────────
TARGETS=(); LOCAL_NAMES=()
for i in $(seq 0 $((COUNT - 1))); do
  if [ "${KINDS[$i]}" = "url" ]; then TARGETS+=("$i"); else LOCAL_NAMES+=("${SECTIONS[$i]}/${PKGNAMES[$i]}"); fi
done

if [ ${#TARGETS[@]} -eq 0 ]; then
  warn "$MANIFEST 里没有任何 url: 包，无需更新"
  exit 0
fi

info "manifest: $MANIFEST"
info "url 包  : ${#TARGETS[@]} 个${LOCAL_NAMES[*]:+（另有本地路径包 ${#LOCAL_NAMES[@]} 个，未处理）}"
[ "$CHECK_ONLY" = 1 ] && dim "模式：--check（只报告差异，不改动）"
[ "$DO_CLONE" = 1 ]   && dim "模式：--clone 已开启（缺失的包会按 manifest 直接克隆）"

# ── 逐个检查并更新 ───────────────────────────────────────────────────────────
UPDATED=(); NOCHANGE=(); SKIPPED=(); FAILED=(); DRYRUN=(); NET_FAILED=0

update_one() {
  local idx="$1"
  local section="${SECTIONS[$idx]}" name="${PKGNAMES[$idx]}"
  local url="${URLS[$idx]}" branch="${BRANCHES[$idx]}"

  printf '\n== %s/%s ==\n' "$section" "$name"

  local dir; dir="$CACHE_ROOT/$(repo_dir_name "$url")"
  dim "  url   : $url"
  dim "  branch: ${branch:-<默认分支>}"
  dim "  dir   : $dir"

  if [ ! -d "$dir" ]; then
    if [ "$DO_CLONE" = 1 ]; then
      info "  cache 里没有，按 manifest 克隆："
      local -a cl=(clone --depth 1)
      [ -n "$branch" ] && cl+=(--branch "$branch")
      if git "${cl[@]}" "$url" "$dir"; then
        ok "  已克隆到最新（$dir），接下来需要 rbnx build"
        UPDATED+=("$section/$name|$dir|(未 clone)|(新 clone)")
      else
        err "  git clone 失败"
        FAILED+=("$section/$name  clone 失败")
      fi
    else
      warn "  cache 里还没有这个包，跳过（先跑 rbnx build --no-update-check -f $MANIFEST，或加 --clone）"
      SKIPPED+=("$section/$name  未 clone")
    fi
    return 0
  fi

  if [ ! -d "$dir/.git" ]; then
    warn "  不是 git checkout，跳过"
    SKIPPED+=("$section/$name  非 git checkout")
    return 0
  fi

  local origin; origin="$(git_in "$dir" config --get remote.origin.url 2>/dev/null || true)"
  if [ -n "$origin" ] && [ "$origin" != "$url" ] && [ "$FORCE" != 1 ]; then
    warn "  checkout 的 remote.origin.url 与 manifest 不一致，跳过（确认无误可加 --force）："
    warn "    checkout: $origin"
    warn "    manifest: $url"
    SKIPPED+=("$section/$name  origin 不匹配")
    return 0
  fi

  # 本地改动检查：只看**内容**改动。cache 是从别的机器搬过来的，大量文件只有
  # 可执行位差异（100644→100755），用 core.fileMode=false 忽略掉，否则任何一次
  # 上游改动都会被误判成冲突。
  local content_mods total_mods content_n mode_n untracked_n
  content_mods="$(git_in "$dir" -c core.fileMode=false diff --name-only 2>/dev/null || true)"
  total_mods="$(git_in "$dir" diff --name-only 2>/dev/null | grep -c . || true)"
  content_n="$(printf '%s' "$content_mods" | grep -c . || true)"
  mode_n=$(( total_mods - content_n ))
  untracked_n="$(git_in "$dir" ls-files --others --exclude-standard 2>/dev/null | grep -c . || true)"
  if [ "$content_n" -gt 0 ]; then
    warn "  checkout 里有本地内容改动（$content_n 个文件），继续尝试 ff："
    printf '%s\n' "$content_mods" | sed 's/^/      /' >&2
    dim "  git 只在能保住这些改动时才会前进；若上游也改了同一文件，ff 会中止，本地改动不会被覆盖。"
  fi
  [ "$mode_n" -gt 0 ] && dim "  （另有 $mode_n 个文件仅可执行位差异，已忽略）"
  [ "$untracked_n" -gt 0 ] && dim "  （另有 $untracked_n 个未跟踪文件，不会被本次更新触碰）"

  # 跟踪引用：manifest 的 branch 优先；没有就用 checkout 当前所在分支。
  local ref="$branch"
  if [ -z "$ref" ]; then
    ref="$(git_in "$dir" rev-parse --abbrev-ref HEAD 2>/dev/null || true)"
    [ "$ref" = "HEAD" ] && ref=""
  fi
  if [ -z "$ref" ]; then
    warn "  manifest 未指定 branch，且 checkout 处于 detached HEAD，无法确定跟踪引用，跳过"
    SKIPPED+=("$section/$name  无法确定引用")
    return 0
  fi

  # fetch 加超时：远端不可达时不要挂死（rbnx 自己也用 timeout 兜底）。
  # 非超时类失败（curl 16 HTTP2 framing、连接被重置等）重试一次 —— 这类多半是瞬时故障。
  local -a fetch_cmd=(git -c safe.directory="$dir" -C "$dir" fetch --quiet --depth 200 origin "$ref")
  if command -v timeout >/dev/null; then fetch_cmd=(timeout "$FETCH_TIMEOUT_S" "${fetch_cmd[@]}"); fi
  local fetch_rc=0 fetch_out="" attempt err_rc=0 err_out=""
  for attempt in 1 2; do
    fetch_rc=0
    fetch_out="$("${fetch_cmd[@]}" 2>&1)" || fetch_rc=$?
    [ "$fetch_rc" -eq 0 ] && break
    # 记住第一次"有信息量"的失败（非超时）：超时只是没等到，128 之类才说得清原因。
    if [ "$fetch_rc" -ne 124 ]; then err_rc="$fetch_rc"; err_out="$fetch_out"; fi
    [ "$fetch_rc" -eq 124 ] && break          # 超时重试只会再等一遍，交给下面的分支处理
    [ "$attempt" -eq 1 ] && { dim "  第 1 次 fetch 失败（退出码 $fetch_rc），2 秒后重试…"; sleep 2; }
  done
  if [ "$fetch_rc" -ne 0 ]; then
    if [ "$err_rc" -ne 0 ]; then
      err "  git fetch origin $ref 失败（git 退出码 $err_rc）："
      printf '%s\n' "$err_out" | sed 's/^/      /' >&2
      NET_FAILED=1
      FAILED+=("$section/$name  fetch 失败")
    else
      # 两次都只是超时：是网络慢，不是这个包的问题，跳过而不是判失败
      warn "  fetch 超过 ${FETCH_TIMEOUT_S}s 未完成，跳过（可用 FETCH_TIMEOUT_S=300 加大超时再跑）"
      SKIPPED+=("$section/$name  fetch 超时")
    fi
    return 0
  fi

  local local_sha remote_sha
  local_sha="$(git_in "$dir" rev-parse --short HEAD 2>/dev/null || true)"
  if [ -z "$local_sha" ]; then
    err "  取不到本地 HEAD（仓库状态异常）"
    FAILED+=("$section/$name  本地 HEAD 不可用")
    return 0
  fi
  if ! remote_sha="$(git_in "$dir" rev-parse --short FETCH_HEAD 2>/dev/null)"; then
    err "  取不到远端 tip（FETCH_HEAD 不可用）"
    FAILED+=("$section/$name  远端 sha 不可用")
    return 0
  fi

  if [ "$local_sha" = "$remote_sha" ]; then
    ok "  已是最新（$local_sha），跳过"
    NOCHANGE+=("$section/$name")
    return 0
  fi

  local behind rdate rsubj
  behind="$(git_in "$dir" rev-list --count HEAD..FETCH_HEAD 2>/dev/null || echo '?')"
  rdate="$(git_in "$dir" log -1 --format=%cd --date=short FETCH_HEAD 2>/dev/null || true)"
  rsubj="$(git_in "$dir" log -1 --format=%s FETCH_HEAD 2>/dev/null || true)"
  info "  本地 $local_sha  →  远端 $remote_sha   落后 ${behind} 个 commit"
  info "  远端最新：$rdate  $rsubj"

  if [ "$CHECK_ONLY" = 1 ]; then
    dim "  [--check] 未做改动"
    DRYRUN+=("$section/$name")
    return 0
  fi

  # 同样用 core.fileMode=false 合并：忽略权限位噪声，但内容冲突仍会让 git 中止
  # （git 的 ff 只在"不会覆盖本地改动"时才前进，所以本地改动是安全的）。
  # 注意 `|| merge_rc=$?`：直接 `merge_rc=$?` 会让失败的赋值撞上 set -e 把脚本杀掉。
  local merge_out merge_rc=0
  merge_out="$(git_in "$dir" -c core.fileMode=false merge --ff-only FETCH_HEAD 2>&1)" || merge_rc=$?
  if [ "$merge_rc" -eq 0 ]; then
    ok "  已更新：$local_sha → $(git_in "$dir" rev-parse --short HEAD)"
    UPDATED+=("$section/$name|$dir|$local_sha|$remote_sha")
  else
    err "  无法 fast-forward，未做任何改动。git 的原因："
    printf '%s\n' "$merge_out" | sed 's/^/      /' >&2
    dim "  若因本地改动冲突：先处理(提交/另存)那些文件再跑一次。"
    dim "  若因历史分叉：放弃本地历史重新克隆 → rbnx clean -f $MANIFEST --cache 之后再 rbnx build"
    FAILED+=("$section/$name  ff 失败（冲突或分叉）")
  fi
}

for idx in "${TARGETS[@]}"; do
  update_one "$idx"
done

# ── 汇总 ─────────────────────────────────────────────────────────────────────
printf '\n──────── 汇总 ────────\n'
if [ ${#UPDATED[@]} -gt 0 ]; then
  printf '%s已更新 %d 个：%s\n' "$C_OK" "${#UPDATED[@]}" "$C_OFF"
  for x in "${UPDATED[@]}"; do info "  ${x%%|*}"; done
fi
[ ${#NOCHANGE[@]} -gt 0 ] && info "已是最新 ${#NOCHANGE[@]} 个：${NOCHANGE[*]}"
[ ${#DRYRUN[@]}   -gt 0 ] && info "有更新待拉取（--check 未改动）${#DRYRUN[@]} 个：${DRYRUN[*]}"
if [ ${#SKIPPED[@]} -gt 0 ]; then
  info "跳过 ${#SKIPPED[@]} 个："
  for x in "${SKIPPED[@]}"; do info "  $x"; done
fi
if [ ${#FAILED[@]} -gt 0 ]; then
  printf '%s失败 %d 个：%s\n' "$C_ERR" "${#FAILED[@]}" "$C_OFF"
  for x in "${FAILED[@]}"; do info "  $x"; done
fi
[ ${#LOCAL_NAMES[@]} -gt 0 ] && dim "未处理（本地路径包，无 url/branch）${#LOCAL_NAMES[@]} 个：${LOCAL_NAMES[*]}"

if [ "$NET_FAILED" = 1 ]; then
  cat <<'EOF'

关于上面 fetch 失败的说明（这是提示，不是 git 的报错，也不是本脚本的判断）：
  每个失败包下面【缩进的那几行】才是 git 的原话，本脚本只是原样转发。
  git 退出码 128 通常意味着"远端不可达或不可访问"，可能是网络不通、认证问题、
  或 url 本身不对；它不代表 manifest 里的 branch 名一定有问题。对策：
    * 网络抖动：直接重跑一次。本脚本对非超时失败已自动重试过一次。
    * fetch 一直卡住不报错：那走的是"超时"分支（跳过、不判失败），加大
      FETCH_TIMEOUT_S（如 300）再跑。
  网络出口（代理等）由你自己在环境里准备好，本脚本不设、不改、不读任何代理配置；
  你 export 了什么，它调用的 git 就原样继承什么。
EOF
fi

if [ ${#UPDATED[@]} -gt 0 ]; then
  cat <<EOF

更新只把源码拉到最新，rbnx-build/ 里的旧产物和 .rbnx-built 标记都还在，
rbnx boot 不会因为源码变了就自动重建。必须显式重建后才会生效：

  # 只重建刚更新的包
EOF
  for x in "${UPDATED[@]}"; do
    IFS='|' read -r _label dir _old _new <<<"$x"
    printf '  rbnx build --no-update-check -p %s\n' "$dir"
  done
  cat <<EOF

  # 或整体重建
  rbnx build --no-update-check -f $MANIFEST

重建后按你的流程重启栈（rbnx boot / start-*.sh）才生效。
EOF
fi

[ ${#FAILED[@]} -eq 0 ] || exit 1
exit 0
