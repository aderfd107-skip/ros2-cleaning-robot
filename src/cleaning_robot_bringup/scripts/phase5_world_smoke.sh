#!/usr/bin/env bash

set -euo pipefail

duration="${1:-12}"
export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/ros_logs}"
mkdir -p "${ROS_LOG_DIR}"

worlds=(
  empty_room
  single_obstacle
  multi_obstacle
  narrow_passage
  multi_room
)

for world in "${worlds[@]}"; do
  echo "==> smoke launch: ${world}"
  set +e
  timeout --signal=INT --kill-after=5 "${duration}s" \
    ros2 launch cleaning_robot_simulation sim.launch.py \
    world_name:="${world}" \
    spawn_delay:=0.0
  status=$?
  set -e

  if [[ "${status}" -ne 0 && "${status}" -ne 124 ]]; then
    echo "Launch failed for ${world} with exit code ${status}" >&2
    exit "${status}"
  fi
done

echo "All world smoke launches completed."
