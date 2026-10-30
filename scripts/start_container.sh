#!/usr/bin/env bash
# Start the ROS 2 Humble desktop container with this project and data mounted from macOS.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
DATA="${ROVER_DATA_DIR:-$ROOT/runtime_data}"
NAME="${ROVER_CONTAINER_NAME:-surry-hills-ros2}"
LOCAL_IMAGE="ros2-desktop-ai-robot-book-humble:mine"
REMOTE_IMAGE="airobotbook/ros2-desktop-ai-robot-book-humble:latest"
mkdir -p "$DATA/rosbags" "$DATA/lerobot_dataset" "$DATA/checkpoints"
if ! docker info >/dev/null 2>&1; then
  echo "Docker Desktop is not running. Start Docker Desktop, then rerun this script." >&2
  exit 1
fi
if docker image inspect "$LOCAL_IMAGE" >/dev/null 2>&1; then
  IMAGE="$LOCAL_IMAGE"
  PLATFORM=()
else
  IMAGE="${ROS2_DOCKER_IMAGE:-$REMOTE_IMAGE}"
  PLATFORM=(--platform "${DOCKER_PLATFORM:-linux/amd64}")
  docker pull "${PLATFORM[@]}" "$IMAGE"
fi
if docker container inspect "$NAME" >/dev/null 2>&1; then
  echo "Container '$NAME' already exists. Stop/remove it with: docker rm -f '$NAME'" >&2
  exit 1
fi
# Keep the usual port when available; otherwise pick a free nearby host port.
NOVNC_PORT="${NOVNC_PORT:-$(python3 - <<'PY'
import socket
for port in range(6080, 6101):
    with socket.socket() as sock:
        try:
            sock.bind(("127.0.0.1", port))
        except OSError:
            continue
        print(port)
        break
else:
    raise SystemExit("No free localhost port in range 6080-6100; set NOVNC_PORT manually.")
PY
)}"
cat <<EOF
Starting $IMAGE
Project: $ROOT -> /workspace/surry_hills
Data:    $DATA -> /home/ubuntu/dataset
Desktop: http://localhost:$NOVNC_PORT (VNC password: ubuntu)
Stop the container with Ctrl-C. Dataset/checkpoints remain in runtime_data/.
EOF
docker run --rm -it "${PLATFORM[@]}" --name "$NAME" \
  --shm-size=1g -p "$NOVNC_PORT:80" \
  -e RESOLUTION="${RESOLUTION:-1600x900}" \
  --mount "type=bind,source=$ROOT,target=/workspace/surry_hills" \
  --mount "type=bind,source=$DATA,target=/home/ubuntu/dataset" \
  "$IMAGE"
