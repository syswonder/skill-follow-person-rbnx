#!/usr/bin/env bash
# Start the loopback follow-distance intent endpoint that Robonix Pilot uses as
# its ``vlm.upstream``.  Pure Python standard library: no Robonix, ROS, camera
# or model dependency, so it can start before the rest of the stack.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
HOST="${SEMANTIC_INTENT_HOST:-127.0.0.1}"
PORT="${SEMANTIC_INTENT_PORT:-18080}"
# preview (default) recognizes utterances but emits no capability leaf, which is
# what a dry-run stack wants.  live is set by the launcher once the follow stack
# is actually allowed to command motion.
MODE="${SEMANTIC_INTENT_EXECUTION_MODE:-preview}"

case "$MODE" in
  preview|live)
    ;;
  *)
    echo "SEMANTIC_INTENT_EXECUTION_MODE must be preview or live" >&2
    exit 64
    ;;
esac

export PYTHONPATH="$ROOT${PYTHONPATH:+:$PYTHONPATH}"
exec python3 -m semantic_intent_router.server \
  --host "$HOST" \
  --port "$PORT" \
  --execution-mode "$MODE"
