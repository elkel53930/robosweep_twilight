#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

LOG_DIR="$SCRIPT_DIR/logs"
mkdir -p "$LOG_DIR"

TS="$(date +"%Y%m%d_%H%M%S")"
LOG_FILE="$LOG_DIR/solve_maze_threaded_${TS}.log"

cmd=("$SCRIPT_DIR/rpi/camera_py/venv/bin/python" "$SCRIPT_DIR/solve_maze_threaded.py" "$@")
cmd_str="$(printf '%q ' "${cmd[@]}")"

script -q -c "$cmd_str" "$LOG_FILE"

echo "Saved log to: $LOG_FILE"
