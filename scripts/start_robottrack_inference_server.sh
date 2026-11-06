#!/usr/bin/env bash
# Start the pinned MiniCPM-RobotTrack HTTP inference server (:5801) that the
# follow bridge calls.  The model tree lives OUTSIDE the container under
# <repo>/upstream/MiniCPM-Robot and is mounted in at runtime; this script only
# pins and launches the server — it performs no download.

set -euo pipefail

readonly SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
readonly REPO_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"

# Model tree (checkpoint + DINOv3 + SigLIP) lives OUTSIDE the container, mounted
# under <repo>/upstream/MiniCPM-Robot (see .gitignore).  Resolve relative to the
# repo root so the script works no matter where the repo is mounted.
readonly UPSTREAM_ROOT="${ROBOTTRACK_UPSTREAM_ROOT:-${REPO_ROOT}/upstream/MiniCPM-Robot}"
readonly TRACK_ROOT="${UPSTREAM_ROOT}/MiniCPM-RobotTrack"
readonly SERVER_SCRIPT="${TRACK_ROOT}/realworld/sample/http_minicpm_robot_track_server.py"
readonly MODEL_DIR="${ROBOTTRACK_MODEL_DIR:-${TRACK_ROOT}/minicpm_robot_track/checkpoints/MiniCPM-RobotTrack}"
readonly DINO_DIR="${TRACK_ROOT}/minicpm_robot_track/backbones/dino_local_hf"
readonly SIGLIP_DIR="${TRACK_ROOT}/minicpm_robot_track/backbones/siglip-so400m-patch14-384"

# NOTE(已确认): 两个 venv(/root/minicpm-infer-venv、/root/minicpm-venv)因
# libcudnn.so.9 缺失导致 torch import 失败,已删除。系统 python3(/usr/bin/python3)
# 验证可用:torch 2.5.0a0+872d972e41.nv24.08、transformers、cv2 4.11.0、flask、
# Pillow、numpy。默认用系统 python3;如需覆盖用 ROBOTTRACK_PYTHON。
readonly PYTHON_BIN="${ROBOTTRACK_PYTHON:-/usr/bin/python3}"

# ── 视觉后端(DINOv3 + SigLIP)────────────────────────────────────────────────
# 部署文档 §9.3 / 上游 GO2_DEPLOYMENT.md §6 的目标是**本机用 FP16 TensorRT
# engine**;torch 是 Go2 参考启动器自带的退路。本机 JetPack 6.2.2 装的是
# TensorRT 10.3,它的 ONNX importer 拒收 DINOv3 rope 里的 `If` 节点
# (IIfConditionalOutputLayer inputs must have the same shape [2] vs [1]),
# 10.7 才是能建出 engine 的版本,所以 10.7 单独放在
# /root/tensorrt10.7/TensorRT-10.7.0.23(不覆盖系统 10.3,可回滚),
# 两份 engine 都是用它在本机建的。
#
# 10.7 的 python 绑定当初装在一个 venv 里,venv 已删;绑定本身还在镜像里,
# 所以这里解包到临时目录、用 PYTHONPATH 引进来 —— 容器每次启动都从镜像重建
# (scripts/run_container.sh),没有需要卸载的东西。
#
# 回退到 torch 后端(不用改文件):
#   ROBOTTRACK_VISION_BACKEND=torch bash scripts/start_follow.sh
readonly VISION_BACKEND="${ROBOTTRACK_VISION_BACKEND:-trt_direct}"
readonly DINO_BACKEND="${ROBOTTRACK_DINO_BACKEND:-${VISION_BACKEND}}"
readonly SIGLIP_BACKEND="${ROBOTTRACK_SIGLIP_BACKEND:-${VISION_BACKEND}}"
readonly TRT_ROOT="${ROBOTTRACK_TRT_ROOT:-/root/tensorrt10.7/TensorRT-10.7.0.23}"
# 本机的 SigLIP engine 是 24x24 pooled 导出 = 576 token,不是 server 默认的
# 729;传错会喂给 policy 一个池化粒度不同的特征网格。
readonly SIGLIP_TRT_TOKENS="${ROBOTTRACK_SIGLIP_TRT_OUTPUT_TOKENS:-576}"
readonly DINO_TRT_TOKENS="${ROBOTTRACK_DINO_TRT_OUTPUT_TOKENS:-576}"

for required in "${PYTHON_BIN}" "${SERVER_SCRIPT}" "${MODEL_DIR}" "${DINO_DIR}" "${SIGLIP_DIR}"; do
  if [[ ! -e "${required}" ]]; then
    echo "RobotTrack inference server not started: missing ${required}" >&2
    exit 2
  fi
done

# Only needed when a TensorRT backend is selected; unpacking is ~1s and the
# directory is reused if it is already there.
trt_python_dir=""
if [[ "${DINO_BACKEND}" == trt_direct || "${SIGLIP_BACKEND}" == trt_direct ]]; then
  if [[ ! -d "${TRT_ROOT}/lib" || ! -d "${TRT_ROOT}/python" ]]; then
    echo "RobotTrack inference server not started: the in-container TensorRT 10.7 tree is missing (${TRT_ROOT})." >&2
    echo "  Restore it in the image, or start with the torch backends:" >&2
    echo "    ROBOTTRACK_VISION_BACKEND=torch bash $(basename "${BASH_SOURCE[0]}")" >&2
    exit 2
  fi
  cp_tag="$("${PYTHON_BIN}" -c 'import sys; print(f"cp{sys.version_info[0]}{sys.version_info[1]}")')"
  trt_python_dir="${ROBOTTRACK_TRT_PY_DIR:-${TMPDIR:-/tmp}/robottrack-trt107-${cp_tag}}"
  if [[ ! -f "${trt_python_dir}/tensorrt/__init__.py" ]]; then
    rm -rf "${trt_python_dir}"
    mkdir -p "${trt_python_dir}"
    for wheel in tensorrt tensorrt_dispatch tensorrt_lean; do
      if ! "${PYTHON_BIN}" -m zipfile -e \
        "${TRT_ROOT}/python/${wheel}-10.7.0-${cp_tag}-none-linux_aarch64.whl" \
        "${trt_python_dir}"; then
        echo "RobotTrack inference server not started: cannot unpack ${wheel} for ${cp_tag}." >&2
        exit 2
      fi
    done
  fi
fi

export PYTHONPATH="${TRACK_ROOT}:${TRACK_ROOT}/realworld/sample${PYTHONPATH:+:${PYTHONPATH}}"
export MINICPM_ROBOT_TRACK_ROOT="${TRACK_ROOT}"
export DINOV3_MODEL_PATH="${DINO_DIR}"
export SIGLIP_MODEL_PATH="${SIGLIP_DIR}"
# CUDA runtime/cudnn lib path. torch 2.5.0a0 (cu12.6) needs libcudnn.so.9 from
# the pip nvidia/cudnn package (the system /lib/libcudnn.so.9 has SONAME .8, a
# known trap) and libcublas/libcudart/... from the CUDA 12.6 runtime. `--gpus
# all` only mounts the DRIVER lib (libcuda.so.1); these live in the image.
export LD_LIBRARY_PATH="/usr/local/lib/python3.10/dist-packages/nvidia/cudnn/lib:/usr/local/cuda-12.6/targets/aarch64-linux/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
# In front of the two above, so `libnvinfer.so.10` resolves to the 10.7 build
# rather than the system 10.3 (same SONAME).  The directory holds only
# libnvinfer*/libnvonnxparser*, no CUDA or cuDNN, so torch is unaffected.
if [[ -n "${trt_python_dir}" ]]; then
  export PYTHONPATH="${trt_python_dir}${PYTHONPATH:+:${PYTHONPATH}}"
  export LD_LIBRARY_PATH="${TRT_ROOT}/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
fi
export TRANSFORMERS_OFFLINE=1
export HF_HUB_OFFLINE=1
export TOKENIZERS_PARALLELISM=false
export NO_PROXY="127.0.0.1,localhost${NO_PROXY:+,${NO_PROXY}}"
export no_proxy="127.0.0.1,localhost${no_proxy:+,${no_proxy}}"

readonly HOST="${ROBOTTRACK_SERVER_HOST:-127.0.0.1}"
readonly PORT="${ROBOTTRACK_SERVER_PORT:-5801}"
readonly DEVICE="${ROBOTTRACK_DEVICE:-cuda:0}"
readonly INSTRUCTION="${ROBOTTRACK_INSTRUCTION:-Follow the person ahead}"
readonly VELOCITY_CONTROLLER="${ROBOTTRACK_VELOCITY_CONTROLLER:-lookahead}"

backend_args=(--dino_backend "${DINO_BACKEND}" --siglip_backend "${SIGLIP_BACKEND}")
if [[ "${DINO_BACKEND}" == trt_direct ]]; then
  backend_args+=(--dino_trt_output_tokens "${DINO_TRT_TOKENS}")
fi
if [[ "${SIGLIP_BACKEND}" == trt_direct ]]; then
  backend_args+=(--siglip_trt_output_tokens "${SIGLIP_TRT_TOKENS}")
fi

echo "Starting MiniCPM-RobotTrack inference: host=${HOST}:${PORT} device=${DEVICE} backends=${DINO_BACKEND}/${SIGLIP_BACKEND} controller=${VELOCITY_CONTROLLER}" >&2
exec "${PYTHON_BIN}" -u "${SERVER_SCRIPT}" \
  --host "${HOST}" \
  --port "${PORT}" \
  --device "${DEVICE}" \
  --model_dir "${MODEL_DIR}" \
  --instruction "${INSTRUCTION}" \
  --vision_amp "${ROBOTTRACK_VISION_AMP:-bf16}" \
  --planner_amp "${ROBOTTRACK_PLANNER_AMP:-none}" \
  "${backend_args[@]}" \
  --response-mode control \
  --timing-mode fast \
  --overlay-mode async \
  --velocity-controller "${VELOCITY_CONTROLLER}" \
  --controller-lookahead-idx "${ROBOTTRACK_CONTROLLER_LOOKAHEAD_IDX:-2}" \
  --controller-feedforward-points "${ROBOTTRACK_CONTROLLER_FEEDFORWARD_POINTS:-3}" \
  --controller-ff-blend "${ROBOTTRACK_CONTROLLER_FF_BLEND:-0.65}" \
  "$@"
