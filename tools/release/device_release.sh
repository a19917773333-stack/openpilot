#!/usr/bin/env bash
# Build from a clean source checkout; never synchronize files into the active runtime.
set -Eeuo pipefail

SRC=/data/build
STAGE=/data/relstage
REPO_URL=https://github.com/onemiless/openpilot.git
SRC_BRANCH="${SRC_BRANCH:-dev-sp-chipmunk}"
RELEASE_BRANCH="${RELEASE_BRANCH:-lean-tesla-release-c3}"
SRC_REF="origin/$SRC_BRANCH"
CURRENT_STAGE=""
export PATH="/usr/local/venv/bin:$PATH"


die() {
  echo "$*" >&2
  exit 1
}

cleanup() {
  # 收尾必须保证 comma 在运行（幂等）
  sudo systemctl start comma
}

on_err() {
  # 统一失败话术 + 非零拒发：非预期失败一律报「标题 失败，拒绝发布」。
  # 命令替换/子 shell 内的失败只静默向上传播退出码——话术在最外层报一次，
  # 否则 [ -z "$(… | grep …)" ] 这类正常空输出路径会刷假报错。
  if [ "${BASH_SUBSHELL:-0}" -gt 0 ]; then
    exit 1
  fi
  die "${CURRENT_STAGE:-发布管线} 失败，拒绝发布"
}

run_stage() {
  # run_stage "标题" 命令 [参数...] —— 统一 T=$SECONDS 标题、失败话术、非零拒发
  CURRENT_STAGE="$1"
  echo "[-] $1 T=$SECONDS"
  shift
  "$@"
  CURRENT_STAGE=""
}

wake_cpu_cores() {
  # 3 个构建阶段共用：把 4~7 号核上线再吃编译负载
  local n
  for n in 4 5 6 7; do
    if [ "$(cat /sys/devices/system/cpu/cpu$n/online 2>/dev/null)" = "0" ]; then
      echo 1 | sudo tee /sys/devices/system/cpu/cpu$n/online >/dev/null
    fi
  done
}

git_fetch_retry() {
  # git_fetch_retry <git 参数...> —— 设备到 GitHub 的 TLS 偶发握手失败
  # （fake-IP 代理链路），重试 3 次；三次都失败返回非零。
  local i
  for i in 1 2 3; do
    git "$@" && return 0
    sleep 5
  done
  return 1
}

check_hardware_profile() {
  local profile
  profile="${SUNNYPILOT_HARDWARE_PROFILE:-$(cat "${SUNNYPILOT_HARDWARE_PROFILE_FILE:-/data/hardware_profile}" 2>/dev/null || true)}"
  profile="${profile#"${profile%%[![:space:]]*}"}"
  profile="${profile%"${profile##*[![:space:]]}"}"
  if [ "$profile" = "c3xl" ]; then
    export C3XL_IFE_ROAD_SIZE="${C3XL_IFE_ROAD_SIZE:-1344x760}"
  fi
  case "$RELEASE_BRANCH:${profile:-standard}" in
    *-c3:c3|*-c3:c3xl) ;;
    *-c3:*) die "C3 release requires hardware_profile=c3 or c3xl" ;;
    *:standard) ;;
    *) die "C4 release requires hardware_profile=standard" ;;
  esac
}

trap cleanup EXIT
trap on_err ERR

check_prereqs() {
  check_hardware_profile
  sudo systemctl is-active --quiet comma || die "comma 未运行，拒绝发布"
}

ensure_repo() {
  local dir="$1" url="$2"
  if [ ! -d "$dir/.git" ]; then
    mkdir -p "$dir"
    git init -q "$dir"
  fi
  if git -C "$dir" remote get-url origin >/dev/null 2>&1; then
    git -C "$dir" remote set-url origin "$url"
  else
    git -C "$dir" remote add origin "$url"
  fi
}

checkout_repo() {
  local dir="$1" url="$2" sha="$3"
  ensure_repo "$dir" "$url"
  git_fetch_retry -C "$dir" fetch -4 -q --depth=1 origin "$sha" || die "$dir fetch 失败"
  git -C "$dir" checkout -q --force --detach "$sha" || die "$dir checkout 失败"
  git -C "$dir" clean -ffdxq || die "$dir clean 失败"
}

sync_sources() {
  ensure_repo "$SRC" "$REPO_URL"
  cd "$SRC"
  git_fetch_retry fetch -4 -q --depth=1 origin "$SRC_BRANCH:refs/remotes/origin/$SRC_BRANCH" || die "源分支 fetch 失败"
  local remote_sha local_sha
  remote_sha=$(git ls-remote origin "refs/heads/$SRC_BRANCH" | awk '{print $1}') \
    || die "远端源分支校验失败"
  local_sha=$(git rev-parse "$SRC_REF")
  [ "$remote_sha" = "$local_sha" ] || die "源分支已变化，拒绝发布"
  checkout_repo "$SRC" "$REPO_URL" "$local_sha"
}

materialize_gitlinks() {
  local submodules key path name
  submodules=$(git -C "$SRC" config -f .gitmodules --get-regexp '\.path$')
  [ -n "$submodules" ] || die "没有子模块，拒绝发布"
  while read -r key path; do
    name=${key#submodule.}
    name=${name%.path}
    checkout_repo "$SRC/$path" "$(git -C "$SRC" config -f .gitmodules "submodule.$name.url")" "$(git -C "$SRC" rev-parse "HEAD:$path")"
  done <<< "$submodules"
}

rebuild_native() {
  cd "$SRC"
  # launch_env.sh is also sourced by launchers without nounset enabled.
  set +u
  source "$SRC/launch_env.sh"
  set -u
  unset SKIP_CAPNP_REGEN SKIP_TINYGRAD_COMPILE
  wake_cpu_cores
  PYTHONPATH="$SRC:$SRC/openpilot${PYTHONPATH:+:$PYTHONPATH}" scons -j1 --minimal cache_size_limit=8e9 \
    openpilot/selfdrive/locationd openpilot/sunnypilot/selfdrive/locationd || die "locationd 构建失败"
  PYTHONPATH="$SRC:$SRC/openpilot${PYTHONPATH:+:$PYTHONPATH}" scons -j8 --minimal cache_size_limit=8e9 \
    || die "全量构建失败"
}

check_artifacts() {
  python3 - "$SRC" <<'ARTIFACT_PY'
import sys
from pathlib import Path
root = Path(sys.argv[1])
sys.path.insert(0, str(root/'tools/release'))
import release_lib
for relative in release_lib.ARTIFACT_PATHS:
  path = root/relative
  ok, reason = release_lib.validate_artifact(path, require_elf='.bin' not in path.name)
  if not ok or path.stat().st_size == 0:
    raise RuntimeError(f"{relative}: {reason}")
from openpilot.common.file_chunker import open_file_chunked
with open_file_chunked(str(root/'openpilot/selfdrive/modeld/models/driving_tinygrad.pkl')) as model:
  if not model.read(1):
    raise RuntimeError('empty driving model')
  while model.read(1024 * 1024):
    pass
release_lib.stamp_schemas(root/'openpilot/cereal/gen/cpp', [root/p for p in release_lib.load_schema_registry(root)])
ARTIFACT_PY
}


check_schema_stamp() {
  python3 "$SRC/tools/release/release_lib.py" check-schema-stamp --root "$SRC" "$SRC/openpilot/cereal/gen/cpp"
}

check_params_keys() {
  /usr/local/venv/bin/python "$SRC/tools/release/release_lib.py" check-params-keys "$SRC"
}

assemble_stage() {
  sudo rm -rf "$STAGE"
  mkdir -p "$STAGE"
  rsync -a "$SRC/" "$STAGE/" \
    --exclude=.git \
    --exclude=.sconsign.dblite \
    --exclude=.github \
    --exclude=.claude \
    --exclude=__pycache__ \
    --exclude='*.pyc' \
    --exclude='*.o' \
    --exclude='*.a' \
    --exclude='*.os' \
    --exclude=SConstruct \
    --exclude=SConscript \
    --exclude=site_scons \
    --exclude='tools/release' \
    --exclude='/release/' \
    --exclude='*.onnx' \
    --exclude='*.onnx.data' \
    --exclude=node_modules \
    --exclude='openpilot/selfdrive/modeld/models/*.onnx*'

  cd "$STAGE"

  # rsync exclude 覆盖不了的收尾剥离
  find . -name '.git' -exec rm -rf {} + 2>/dev/null || true        # submodule 指针文件
  find . -name '*.onnx*' -delete 2>/dev/null || true
  rm -rf tinygrad_repo/examples tinygrad_repo/test tinygrad_repo/docs \
         tinygrad_repo/scripts tinygrad_repo/.github
  rm -rf .sconsign.dblite
  find openpilot/third_party \( -iname '*x86*' -o -iname '*darwin*' \) -exec rm -rf {} + 2>/dev/null || true
}

stamp_tinygrad_pin_stage() {
  # pin 取自 lean-master 的 gitlink（ls-tree）——设备树可能是扁平消费者，
  # tinygrad_repo 无 .git，rev-parse 会穿透父仓库返回 release commit（垃圾 pin）
  python3 - "$SRC" "$STAGE" "refs/remotes/origin/$SRC_BRANCH" <<'PYEOF'
import sys
from pathlib import Path
sys.path.insert(0, str(Path(sys.argv[1])/"tools/release"))
import release_lib

sha = release_lib.stamp_tinygrad_pin(Path(sys.argv[2]), Path(sys.argv[1]), treeish=sys.argv[3])
print(f"[ok] tinygrad pin stamped: {sha[:7]}")
PYEOF
}

check_flat_tree() {
  # PC 路径守卫全量扫描：扁平树里所有 ELF 都不得含 /.comma 标记
  # （09-11 交叉构建事故的防线；设备本树构建的结构性产物，此处为回归防线）；
  # 且 stage 树必须能自行解析 tinygrad pin（发布门禁）。判据在
  # release_lib.check_flat_tree，壳只留调用。
  /usr/local/venv/bin/python "$SRC/tools/release/release_lib.py" check-flat-tree .
  echo "PC-path guard: all ELFs clean"
  echo "release gate: tinygrad pin resolvable in stage tree"
}

commit_release() {
  DATETIME=$(date '+%Y-%m-%dT%H:%M:%S')
  SRC_SHA=$(git -C "$SRC" rev-parse "$SRC_REF")
  git init -q -b "$RELEASE_BRANCH"
  git config user.name onemiless
  git config user.email onemiless@users.noreply.github.com
  # .overlay_init 是 updater 的运行时 overlay 标记（updated.py 管理、随更新周期
  # 创建/删除）——track 进 release commit 会让运行时删除变成"树脏"，每次重启后
  # 的发布都卡在前提检查。结构白名单仍保留该条目（防镜像删除误伤），但 git 不跟踪。
  git add -f . ':(exclude).overlay_init'
  git -c core.compression=0 -c gc.auto=0 commit -m "openpilot v$VERSION lean release (device-built, flat)

date: $DATETIME
source commit: $SRC_BRANCH@$SRC_SHA
built on: comma device"
}

run_stage "前提检查" check_prereqs
run_stage "停止 comma，隔离构建" sudo systemctl stop comma
run_stage "同步 $SRC_BRANCH 到独立构建树" sync_sources
run_stage "检出源提交的全部子模块" materialize_gitlinks
run_stage "全量构建（EKF 先串行）" rebuild_native
run_stage "验证构建产物" check_artifacts
run_stage "capnp schema/gen 一致性门禁" check_schema_stamp
run_stage "params 键表门禁" check_params_keys
run_stage "组装扁平树 stage" assemble_stage
run_stage "stamp tinygrad pin" stamp_tinygrad_pin_stage
run_stage "stage 发布门禁" check_flat_tree
run_stage "touch prebuilt" touch prebuilt
VERSION=$(grep -oE '[0-9]+\.[0-9]+\.[0-9]+' openpilot/sunnypilot/common/version.h | head -1)
run_stage "组发布 commit" commit_release

echo "[ok] 扁平树 stage 就绪: $STAGE（分支 $RELEASE_BRANCH, 单 commit）"
