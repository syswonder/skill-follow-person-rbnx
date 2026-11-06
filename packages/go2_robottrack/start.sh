#!/usr/bin/env bash
set -euo pipefail

ROOT="$(CDPATH= cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROS_DISTRO="${ROS_DISTRO:-humble}"
ROS_SETUP="/opt/ros/${ROS_DISTRO}/setup.bash"
ROBONIX_IDL_SETUP="${ROOT}/rbnx-build/codegen/ros2_idl/install/setup.bash"
PACKAGE_SETUP="${ROOT}/.build/ros/install/setup.bash"
PROTO_STUB="${ROOT}/rbnx-build/codegen/proto_gen/atlas_pb2.py"
MCP_STUB="${ROOT}/rbnx-build/codegen/robonix_mcp_types/go2_robottrack_control_mcp.py"

for required in "${ROS_SETUP}" "${ROBONIX_IDL_SETUP}" "${PACKAGE_SETUP}" "${PROTO_STUB}" "${MCP_STUB}"; do
  if [[ ! -r "${required}" ]]; then
    echo "Missing built runtime artifact: ${required}" >&2
    echo "Run this package's build command before start." >&2
    exit 1
  fi
done
if ! command -v rbnx >/dev/null 2>&1; then
  echo "rbnx is not available." >&2
  exit 1
fi
ROBONIX_API_ROOT="$(rbnx path robonix-api)"
if [[ ! -d "${ROBONIX_API_ROOT}/robonix_api" ]]; then
  echo "rbnx returned an invalid robonix-api path: ${ROBONIX_API_ROOT}" >&2
  exit 1
fi

set +u
# shellcheck disable=SC1090
source "${ROS_SETUP}"
# shellcheck disable=SC1090
source "${ROBONIX_IDL_SETUP}"
# shellcheck disable=SC1090
source "${PACKAGE_SETUP}"
set -u

export PYTHONPATH="${ROBONIX_API_ROOT}:${ROOT}:${ROOT}/rbnx-build/codegen/proto_gen:${ROOT}/rbnx-build/codegen/robonix_mcp_types:${PYTHONPATH:-}"

# The specified-person lock imports torch (through person_reid.py) inside this
# process, and this image ships torch against a CUDA/cuDNN runtime that is not
# on the default loader path.  The deployment supplies the directory list as
# ROBONIX_TORCH_LIBRARY_PATH.  Unset means the ordinary follow profile: no
# torch import is attempted unless the lock is enabled, so nothing changes.
if [[ -n "${ROBONIX_TORCH_LIBRARY_PATH:-}" ]]; then
  export LD_LIBRARY_PATH="${ROBONIX_TORCH_LIBRARY_PATH}${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
fi

exec python3 -m go2_robottrack.provider
