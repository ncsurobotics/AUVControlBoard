#!/usr/bin/env bash
################################################################################
# Run example/gate_pose.py inside a ROS2 Jazzy docker container (ROS is not
# installed on the host).
#
# Usage:
#   ./gate_pose_docker.sh                 # control board on /dev/ttyACM0
#   ./gate_pose_docker.sh -p /dev/ttyACM1 -v somevehicle
#
# All arguments are passed through to launch.py.
#
# First run builds a small image (ros:jazzy + pyserial). The ros:jazzy base
# is multi-arch (amd64 + arm64), so building on a Jetson (Orin Nano etc.)
# produces a native arm64 image automatically — no cross-build needed.
# DDS discovery with the ZED publisher works via host networking (+ host IPC
# for shared-memory transport if the publisher runs on the same machine).
################################################################################
set -e

IMAGE=auvcb-gate-pose:jazzy
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
IFACE_DIR="$(dirname "$SCRIPT_DIR")"

# Default serial port; honor -p if given so the device gets mapped in
PORT="/dev/ttyACM0"
args=("$@")
for ((i = 0; i < ${#args[@]}; i++)); do
    if [[ "${args[$i]}" == "-p" && $((i + 1)) -lt ${#args[@]} ]]; then
        PORT="${args[$((i + 1))]}"
    fi
done

# Build image on first use
if ! docker image inspect "$IMAGE" >/dev/null 2>&1; then
    echo "Building $IMAGE (first run only)..."
    docker build -t "$IMAGE" - <<'EOF'
FROM ros:jazzy
RUN apt-get update && \
    apt-get install -y --no-install-recommends python3-serial && \
    rm -rf /var/lib/apt/lists/*
EOF
fi

DEVICE_ARGS=()
if [[ -e "$PORT" ]]; then
    DEVICE_ARGS=(--device "$PORT")
else
    echo "Warning: $PORT not found; running without serial device (simulator use only)."
fi

# ros:jazzy entrypoint sources /opt/ros/jazzy/setup.bash automatically
exec docker run --rm -it \
    --net=host \
    --ipc=host \
    "${DEVICE_ARGS[@]}" \
    -v "$IFACE_DIR":/iface \
    -w /iface \
    "$IMAGE" \
    python3 launch.py "$@" example/gate_pose.py
