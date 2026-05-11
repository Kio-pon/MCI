#!/bin/bash
# Kill blocking serial sessions and run the PSO tuner from the project env.

set -e

PORT="${1:-/dev/ttyUSB0}"
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
ENV_PYTHON="$SCRIPT_DIR/env/bin/python"
TUNER_SCRIPT="$SCRIPT_DIR/pso_tuner.py"

if [[ ! -x "$ENV_PYTHON" ]]; then
  echo "Error: project env python not found at $ENV_PYTHON"
  exit 1
fi

if [[ ! -f "$TUNER_SCRIPT" ]]; then
  echo "Error: tuner script not found at $TUNER_SCRIPT"
  exit 1
fi

echo "Checking for processes on $PORT..."
sudo fuser -k "$PORT" 2>/dev/null || true
sudo pkill -f "screen $PORT" 2>/dev/null || true
sudo pkill -f "minicom .*${PORT##*/}" 2>/dev/null || true
sudo pkill -f "picocom .*${PORT##*/}" 2>/dev/null || true
sudo pkill -f "python .*pso_tuner.py" 2>/dev/null || true
sudo pkill -f "python .*sensorplot.py" 2>/dev/null || true

echo "Setting permissions on $PORT..."
sudo chmod 666 "$PORT" || true

echo "Starting PSO tuner..."
exec "$ENV_PYTHON" "$TUNER_SCRIPT" --port "$PORT"
