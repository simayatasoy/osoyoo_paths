#!/usr/bin/env bash
# Copy the run logs from the robot to this laptop:
#   ./fetch_logs.sh                  (from pi@192.168.3.7)
#   ./fetch_logs.sh pi@192.168.3.4   (from another robot)
# They land in logs/<robot address>/ here. Logs already copied are skipped;
# nothing is deleted on the robot.
set -euo pipefail
HOST="${1:-pi@192.168.3.7}"
DEST="$(cd "$(dirname "$0")" && pwd)/logs/${HOST#*@}"
mkdir -p "$DEST"
rsync -av "$HOST:~/osoyoo_paths/logs/" "$DEST/"
echo
echo "logs in $DEST (newest first):"
ls -1t "$DEST" | head -10
