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
gnome-terminal --window --title="GAZEBO" -- bash -c "cd $PX4_DIR && ./simulation-gazebo.py --source px4; exec bash"

# 4. 加入带相机无人机
gnome-terminal --window --title="无人机2" -- bash -c "export PX4_GZ_STANDALONE=1;
export PX4_GZ_MODEL=x500_depth;
export PX4_GZ_MODEL_POSE="0,2,0,0,0,0";
PX4_SYS_AUTOSTART=4001 ~/PX4-Autopilot/build/px4_sitl_default/bin/px4 -i 1;exec bash"

# 5. 桥接
gnome-terminal --window --title="无人机2" -- bash -c "export GZ_VERSION=garden;
ros2 run ros_gz_bridge parameter_bridge "/image_raw@sensor_msgs/msg/Image[gz.msgs.Image";"


echo "完成。请检查新开启的窗口。"
