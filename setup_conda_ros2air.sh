#!/usr/bin/env bash
# 一键配置 conda 环境 ros2air（Python 3.10 + numpy<2 + ultralytics + opencv），用于运行 drone_detector 等节点
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [[ -n "$CONDA_EXE" ]]; then
  CONDA_ROOT="${CONDA_EXE%/bin/conda}"
fi
if [[ -z "$CONDA_ROOT" || ! -d "$CONDA_ROOT" ]]; then
  if [[ -d "$HOME/anaconda3" ]]; then
    CONDA_ROOT="$HOME/anaconda3"
  elif [[ -d "$HOME/miniconda3" ]]; then
    CONDA_ROOT="$HOME/miniconda3"
  else
    echo "未找到 conda 安装路径，请设置 CONDA_ROOT 或安装 Anaconda/Miniconda"
    exit 1
  fi
fi

echo "使用 conda: $CONDA_ROOT"
source "$CONDA_ROOT/etc/profile.d/conda.sh"

if conda env list | grep -q '^ros2air '; then
  echo "环境 ros2air 已存在，仅安装/更新依赖..."
  conda activate ros2air
else
  echo "创建 conda 环境 ros2air (Python 3.10)..."
  conda create -n ros2air python=3.10 -y
  conda activate ros2air
fi

echo "安装 pip 依赖 (numpy<2, ultralytics, opencv-python-headless, pytz)..."
pip install "numpy<2" ultralytics opencv-python-headless pytz

echo "配置完成。使用方式："
echo "  conda activate ros2air"
echo "  source /opt/ros/humble/setup.bash"
echo "  source $SCRIPT_DIR/install/setup.bash"
echo "  # 必须在 ros2air 激活状态下重新编译，这样 ros2 run 才会用 conda 的 Python："
echo "  cd $SCRIPT_DIR && colcon build --packages-select drone_detector && source install/setup.bash"
echo "  ros2 run drone_detector drone_detector_node"
