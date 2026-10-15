#!/usr/bin/env bash
# Record ROS 2 bags (MCAP).
#   ./scripts/record_bag.sh            full recording for field tests (incl. point clouds)
#   ./scripts/record_bag.sh blackbox   ring buffer: 60 s splits, keeps the last 10 minutes
# Save the blackbox after an incident:  cp -r ~/rosbags/blackbox ~/rosbags/incident_$(date +%F_%H%M%S)
set -euo pipefail
MODE="${1:-full}"
OUT_ROOT="${ROSBAG_DIR:-$HOME/rosbags}"
mkdir -p "$OUT_ROOT"

CORE_TOPICS=(/tf /tf_static /odom/wheel /odometry/local /odometry/global /odometry/gnss
  /imu/data /gnss/fix /gnss/heading /gnss/vel /cmd_vel_mux /cmd_vel_safe /cmd_vel_nav
  /cmd_vel_teleop /blade/cmd /safety/state /base/estop /base/status /battery/state
  /mission/state /mower/blade_request /perception/nearest_person /diagnostics_agg
  /plan /kiss_icp/odometry)

if [[ "$MODE" == "blackbox" ]]; then
  DIR="$OUT_ROOT/blackbox"
  rm -rf "$DIR"
  # prune old splits in the background, keep the newest 10 (≈ 10 minutes)
  ( while sleep 30; do
      ls -1t "$DIR"/*.mcap 2>/dev/null | tail -n +11 | xargs -r rm -f
    done ) &
  PRUNER=$!
  trap 'kill $PRUNER 2>/dev/null || true' EXIT
  ros2 bag record -s mcap --max-bag-duration 60 -o "$DIR" "${CORE_TOPICS[@]}"
else
  DIR="$OUT_ROOT/$(date +%Y-%m-%d_%H-%M-%S)"
  ros2 bag record -s mcap --compression-mode file --compression-format zstd \
    --max-bag-duration 300 -o "$DIR" "${CORE_TOPICS[@]}" /points_obstacles \
    /camera/rgb/image_rect_color/compressed /mission/coverage_path
fi
