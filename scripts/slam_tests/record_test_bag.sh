#!/usr/bin/env bash
# 录制 FAST-LIO2 前端实机测试 rosbag（sqlite3 格式）。
# 用法：
#   bash scripts/slam_tests/record_test_bag.sh [输出目录] [录制秒数]
#   bash scripts/slam_tests/record_test_bag.sh --light [输出目录] [录制秒数]
#   bash scripts/slam_tests/record_test_bag.sh /data/slam_test/static_01 60
# --light 只录前端诊断必需话题（不含 body_cloud/world_cloud/lio_path），推荐测试时使用。
# 不传秒数则持续录制，Ctrl+C 正常结束并落盘。
set -eo pipefail

source /opt/ros/humble/setup.bash
if [ -f "$(dirname "$0")/../../install/setup.bash" ]; then
  source "$(dirname "$0")/../../install/setup.bash"
fi

LIGHT=0
POSITIONAL=()
for arg in "$@"; do
  case "$arg" in
    --light) LIGHT=1 ;;
    *) POSITIONAL+=("$arg") ;;
  esac
done
set -- "${POSITIONAL[@]}"

OUT=${1:-"$HOME/slam_bags/$(date +%Y%m%d_%H%M%S)"}
DURATION=${2:-0}
mkdir -p "$(dirname "$OUT")"

# 诊断话题 lio_diagnostics 是本次新增；若节点尚未启动，ros2 bag 会提示该话题不存在，
# 需先启动 bringup 再用 ros2 topic list 确认。
if [ "$LIGHT" -eq 1 ]; then
  TOPICS=(
    /imu/data
    /rslidar_points
    /rslidar_rear_points
    /merged_points
    /fastlio2/lio_odom
    /fastlio2/lio_diagnostics
    /tf
  )
else
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
fi

echo "Mode: $([ "$LIGHT" -eq 1 ] && echo light || echo full)"
echo "Recording to: $OUT"
echo "Topics: ${TOPICS[*]}"
if [ "$DURATION" -gt 0 ]; then
  timeout --foreground --preserve-status --signal=INT --kill-after=5s "$DURATION" \
    ros2 bag record -s sqlite3 -o "$OUT" "${TOPICS[@]}"
else
  ros2 bag record -s sqlite3 -o "$OUT" "${TOPICS[@]}"
fi
echo "Bag saved to: $OUT"
