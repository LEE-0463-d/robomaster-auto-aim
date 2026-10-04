#!/bin/bash
# 启动游戏（如未运行）并常驻运行自瞄程序。
# 用法：GAME_DIR=/path/to/homework2026 bash run.sh
#       游戏目录通过环境变量 GAME_DIR 指定；若游戏已在运行可省略。
set -u

source /opt/ros/humble/setup.bash

# 游戏未运行则启动（必须带 ROS 环境，否则不发布图像）
if ! pgrep -f homework2026.x86_64 > /dev/null; then
  game_dir="${GAME_DIR:-}"
  if [ -n "$game_dir" ] && [ -x "$game_dir/homework2026.sh" ]; then
    echo "启动游戏：$game_dir"
    (cd "$game_dir" && nohup ./homework2026.sh \
        > /tmp/homework2026_launch.log 2>&1 &)
    sleep 5
  else
    echo "游戏未运行。请先自行启动，或用 GAME_DIR 指定游戏目录：" >&2
    echo "  GAME_DIR=/path/to/homework2026 bash run.sh" >&2
    exit 1
  fi
else
  echo "游戏已在运行。"
fi

# 首次运行时编译
cd "$(dirname "$0")" || exit 1
if [ ! -x build/auto_aim ]; then
  echo "首次编译..."
  cmake -B build -DCMAKE_BUILD_TYPE=Release > /dev/null \
    && cmake --build build -j"$(nproc)" > /dev/null || exit 1
fi

# 常驻循环：一局结束后自动重启等待下一局
echo "自瞄常驻已启动：请在游戏中点「开始游戏」（本窗口保持开启，Ctrl+C 退出）"
while true; do
  ./build/auto_aim
  sleep 2 
done
