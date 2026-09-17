#!/usr/bin/env bash
# Start the pinned MiniCPM-RobotTrack HTTP inference server (:5801) that the
# follow bridge calls.  Model tree is already present in the robonix container
# at /root/MiniCPM-Robot — this script only pins and launches the server; it
# performs no download.

set -euo pipefail

readonly SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
readonly REPO_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"

# Model tree (checkpoint + DINOv3 + SigLIP) lives inside the container.
readonly UPSTREAM_ROOT="${ROBOTTRACK_UPSTREAM_ROOT:-/root/MiniCPM-Robot}"
readonly TRACK_ROOT="${UPSTREAM_ROOT}/MiniCPM-RobotTrack"
readonly SERVER_SCRIPT="${TRACK_ROOT}/realworld/sample/http_minicpm_robot_track_server.py"
readonly MODEL_DIR="${ROBOTTRACK_MODEL_DIR:-${TRACK_ROOT}/minicpm_robot_track/checkpoints/MiniCPM-RobotTrack}"
readonly DINO_DIR="${TRACK_ROOT}/minicpm_robot_track/backbones/dino_local_hf"
readonly SIGLIP_DIR="${TRACK_ROOT}/minicpm_robot_track/backbones/siglip-so400m-patch14-384"

# NOTE(启动阶段): the venv torch backend must be confirmed before first run.
# Both /root/minicpm-infer-venv and /root/minicpm-venv import torch but fail at
# torch/__init__.py:361 (CUDA runtime lookup); the container system python3 has
# torch 2.13.0+cu130.  Override ROBOTTRACK_PYTHON to whichever env actually
# loads torch + transformers + cv2 on this host.
readonly PYTHON_BIN="${ROBOTTRACK_PYTHON:-/root/minicpm-infer-venv/bin/python}"

for required in "${PYTHON_BIN}" "${SERVER_SCRIPT}" "${MODEL_DIR}" "${DINO_DIR}" "${SIGLIP_DIR}"; do
  if [[ ! -e "${required}" ]]; then
    echo "RobotTrack inference server not started: missing ${required}" >&2
    exit 2
  fi
done

export PYTHONPATH="${TRACK_ROOT}:${TRACK_ROOT}/realworld/sample${PYTHONPATH:+:${PYTHONPATH}}"
export MINICPM_ROBOT_TRACK_ROOT="${TRACK_ROOT}"
export DINOV3_MODEL_PATH="${DINO_DIR}"
export SIGLIP_MODEL_PATH="${SIGLIP_DIR}"
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

echo "Starting MiniCPM-RobotTrack inference: host=${HOST}:${PORT} device=${DEVICE} controller=${VELOCITY_CONTROLLER}" >&2
exec "${PYTHON_BIN}" -u "${SERVER_SCRIPT}" \
  --host "${HOST}" \
  --port "${PORT}" \
  --device "${DEVICE}" \
  --model_dir "${MODEL_DIR}" \
  --instruction "${INSTRUCTION}" \
  --vision_amp "${ROBOTTRACK_VISION_AMP:-bf16}" \
  --planner_amp "${ROBOTTRACK_PLANNER_AMP:-none}" \
  --dino_backend torch \
  --siglip_backend torch \
  --response-mode control \
  --timing-mode fast \
  --overlay-mode async \
  --velocity-controller "${VELOCITY_CONTROLLER}" \
  --controller-lookahead-idx "${ROBOTTRACK_CONTROLLER_LOOKAHEAD_IDX:-2}" \
  --controller-feedforward-points "${ROBOTTRACK_CONTROLLER_FEEDFORWARD_POINTS:-3}" \
  --controller-ff-blend "${ROBOTTRACK_CONTROLLER_FF_BLEND:-0.65}" \
  "$@"
