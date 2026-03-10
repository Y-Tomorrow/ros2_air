#!/bin/bash

# 配置路径
PX4_DIR=~/PX4-Autopilot

PRE_CMD="conda deactivate"

echo "正在开启独立窗口并退出 Conda 环境..."

# 1. 启动 MicroXRCEAgent
gnome-terminal --window --title="Agent" -- bash -c "MicroXRCEAgent udp4 -p 8888; exec bash"

# 2. 启动 QGroundControl（根据自己情况修改路径及命名）
gnome-terminal --window --title="QGC" -- bash -c "cd $PX4_DIR && ./QGroundControl-x86_64.AppImage; exec bash"

# 3. 启动 PX4 SITL & Gazebo
# gnome-terminal --window --title="PX4" -- bash -c "cd $PX4_DIR && make px4_sitl gz_x500; exec bash"

echo "完成。请检查新开启的窗口。"
