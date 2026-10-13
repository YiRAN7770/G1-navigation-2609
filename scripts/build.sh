#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${ROOT}"

if ! command -v colcon >/dev/null 2>&1; then
  echo "colcon is required in the ROS 2 Humble environment" >&2
  exit 1
fi

set +u
source /opt/ros/humble/setup.bash
set -u
if [[ "${RUN_ROSDEP:-0}" == "1" ]] && command -v rosdep >/dev/null 2>&1; then
  rosdep install --from-paths ros2_ws/src --ignore-src --rosdistro humble -y
fi

LIVOX_PREFIX="${ROOT}/.deps/livox_sdk2"
cmake -S ros2_ws/src/utilities/livox_ros_driver2/Livox-SDK2 \
  -B .deps/livox_sdk2-build -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="${LIVOX_PREFIX}"
cmake --build .deps/livox_sdk2-build --parallel
cmake --install .deps/livox_sdk2-build
export LIVOX_SDK2_ROOT="${LIVOX_PREFIX}"

colcon --log-base ros2_ws/log build --base-paths ros2_ws/src --build-base ros2_ws/build \
  --install-base ros2_ws/install \
  --symlink-install
