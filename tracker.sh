# 1. 启动识别节点
gnome-terminal --window --title="Detector" -- bash -c "
source ~/anaconda3/etc/profile.d/conda.sh;
conda activate ros2air;
source /opt/ros/humble/setup.bash;
source ~/ros2_air/install/setup.bash;
ros2 run drone_detector drone_detector_node; exec bash"

# 2. 启动追踪节点
gnome-terminal --window --title="Tracker" -- bash -c "
source ~/anaconda3/etc/profile.d/conda.sh;
conda activate ros2air;
source /opt/ros/humble/setup.bash;
source ~/ros2_air/install/setup.bash;
ros2 launch drone_tracker drone_tracker.launch.py vehicle_id:=2; exec bash"