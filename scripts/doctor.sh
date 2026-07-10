#!/usr/bin/env bash

# Read-only ROS graph checks for the RM65 vision-grasping pipeline.
set -u

check() {
  local kind="$1"
  local name="$2"

  if ros2 "$kind" list 2>/dev/null | grep -Fxq "$name"; then
    printf '[OK]      %-8s %s\n' "$kind" "$name"
  else
    printf '[MISSING] %-8s %s\n' "$kind" "$name"
  fi
}

if ! command -v ros2 >/dev/null 2>&1; then
  echo '[ERROR] ros2 is not available. Source ROS 2 and the workspace first.'
  exit 1
fi

echo 'RM65 vision-grasping graph check'
check topic /joint_states
check topic /yolo/detections_3d
check topic /grasp_target_pose
check service /compute_ik
check action /rm_group_controller/follow_joint_trajectory

echo
echo 'TF check: camera_link -> base_link'
timeout 3 ros2 run tf2_ros tf2_echo base_link camera_link 2>/dev/null | sed -n '1,12p'
