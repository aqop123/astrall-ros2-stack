#!/usr/bin/env bash
# 录制 FAST-LIO2 前端实机测试 rosbag（sqlite3 格式）。
# 用法：
#   bash scripts/slam_tests/record_test_bag.sh [输出目录] [录制秒数]
#   bash scripts/slam_tests/record_test_bag.sh /data/slam_test/static_01 60
# 不传秒数则持续录制，Ctrl+C 正常结束并落盘。
set -euo pipefail

source /opt/ros/humble/setup.bash
if [ -f "$(dirname "$0")/../../install/setup.bash" ]; then
  source "$(dirname "$0")/../../install/setup.bash"
fi

OUT=${1:-"$HOME/slam_bags/$(date +%Y%m%d_%H%M%S)"}
DURATION=${2:-0}
mkdir -p "$(dirname "$OUT")"

# 诊断话题 lio_diagnostics 是本次新增；若节点尚未启动，ros2 bag 会提示该话题不存在，
# 需先启动 bringup 再用 ros2 topic list 确认。
TOPICS=(
  /imu/data
  /rslidar_points
  /rslidar_rear_points
  /merged_points
  /fastlio2/lio_odom
  /fastlio2/body_cloud
  /fastlio2/world_cloud
  /fastlio2/lio_path
  /fastlio2/lio_diagnostics
  /tf
)

echo "Recording to: $OUT"
echo "Topics: ${TOPICS[*]}"
if [ "$DURATION" -gt 0 ]; then
  timeout --signal=INT "$DURATION" ros2 bag record -s sqlite3 -o "$OUT" "${TOPICS[@]}"
else
  ros2 bag record -s sqlite3 -o "$OUT" "${TOPICS[@]}"
fi
echo "Bag saved to: $OUT"
