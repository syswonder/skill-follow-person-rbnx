#!/usr/bin/env bash
# Offline validation gate for the follow deployment.  No ROS graph, no camera,
# no network, no robot access -- safe to run while the navigation stack is live.
#
# Mirrors the reference's scripts/validate_offline.sh.
set -euo pipefail

ROOT="$(CDPATH= cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"

echo "=== follow bridge (go2_robottrack) ==="
bash "${ROOT}/packages/go2_robottrack/tests/run_offline_tests.sh"

echo
echo "=== follow-distance intent router ==="
bash "${ROOT}/packages/semantic_intent_router/tests/run_offline_tests.sh"

echo
echo "=== identity gate: vx only with the enrolled person in frame ==="
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH="${ROOT}/packages/go2_robottrack" \
  python3 "${ROOT}/scripts/_diag_no_identity_no_vx.py"

echo
echo "=== manifest consistency ==="
PYTHONDONTWRITEBYTECODE=1 python3 "${ROOT}/scripts/validate_manifest_consistency.py"

echo
echo "offline validation OK"
