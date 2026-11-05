#!/usr/bin/env bash
# Copy this folder to the robot:   ./deploy.sh                (pi@192.168.3.7)
#                                  ./deploy.sh pi@192.168.3.4  (another robot)
# Files deleted here are also deleted on the robot, except its logs/ folder.
set -euo pipefail
HOST="${1:-pi@192.168.3.7}"
cd "$(dirname "$0")"
rsync -av --delete --exclude __pycache__ --exclude logs --exclude .git ./ "$HOST:~/osoyoo_paths/"
echo "copied to $HOST:~/osoyoo_paths"
