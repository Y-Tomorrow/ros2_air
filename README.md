# 一、gazebo仿真环境搭建与px4的编译


1、克隆px4源码，并进入文件夹
```bash
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
# 备份
zip -r PX4-Autopilot.zip PX4-Autopilot/
# 安装相关依赖：
cd PX4-Autopilot
bash ./PX4-Autopilot/Tools/setup/ubuntu.sh
```    
2、gazebo

1、添加软件源：
```bash
sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
```
2、添加密钥
```bash
wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
```
3、更新软件包并安装gazebo garden
```bash
sudo apt update
sudo apt install gz-garden
```

3、用gazebo garden编译
```bash
make px4_sitl gz_x500
```

# 二、安装QGoundControl地面站

1、安装相关的依赖项（可能需要重启来启用用户权限的更改）
```bash
sudo usermod -a -G dialout $USER
sudo apt-get remove modemmanager -y
sudo apt install gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl -y
sudo apt install libfuse2 -y
sudo apt install libxcb-xinerama0 libxkbcommon-x11-0 libxcb-cursor-dev -y
```
2、下载QGroundControl.AppImage

点击链接前往下载：https://qgroundcontrol.com/

3、授权并运行（QGC会自动与PX4连接）
```bash
chmod +x ./QGroundControl.AppImage
./QGroundControl.AppImage
```


# 三、安装ROS2
1、推荐鱼香ros一键安装（ros2 humble）
```bash
wget http://fishros.com/install -O fishros && . fishros
```
2.出现问题可以这样卸载
```bash
sudo apt remove ros-humble-*
sudo apt autoremove
```
# 四、安装 Micro XRCE-DDS Agent

1、下载源码
```bash
git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
```
2、编译
```bash
cd Micro-XRCE-DDS-Agent
mkdir build
cd build
cmake ..
make
```

3、安装
```bash
sudo make install
sudo ldconfig /usr/local/lib/
```

4、启动
```bash
MicroXRCEAgent udp4 -p 8888
```
# 五、轨迹跟踪仿真步骤

1、创建工作区
```bash
mkdir -p ~/ros2_ws/src
```
2、下载源码
```bash
cd ~/ros2_ws/src
git clone https://github.com/PX4/px4_msgs.git -b release/1.14
git clone https://github.com/PX4/px4_ros_com.git -b release/v1.14
```

准备工作完毕，接下来尝试运行官方代码

1、构建项目
```bash
cd ~/ros2_ws
colcon build
```
2、更新环境
```bash
cd ~/ros2_ws
source install/setup.bash
```

3、运行官方offboard案例
```bash
ros2 run px4_ros_com offboard_control
```

# 六、总结

如果上面的配置步骤全部成功，那么就可以实现任意轨迹的仿真了，下面是总结的步骤

1、创建功能包
```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_cmake your_bagname  
```    
（your_bagname替换为你想创建的功能包，例如 offboard_control）

2、编写轨迹代码，保存到~/ros2_ws/src/your_bagname/src/your_codename.cpp，这里提供一段圆形轨迹代码供参考，轨迹代码名字为circle1.cpp，每次编写完记得保存

    src/offboard_control/src/circle1.cpp

3、编写cmakelists.txt文件

cmakelist文件是在创建好的功能包里面的，双击打开，复制下面内容将其替换掉，然后保存

    src/offboard_control/CMakeLists.txt

4、编译
```bash
    cd ~/ros2_ws
    colcon build --packages-select your_bagname
```
（如果是上面的圆形轨迹案例记得把这里的your_bagname改为offboard_control）

5、运行
```bash
    source install/setup.bash
    ros2 run your_bagname your_codename
    ros2 run offboard_control circle1
```    

start.sh脚本
```bash
    chmod +x start.sh
    ./start.sh
```
start.sh里执行了

启动px4及其gazebo仿真：make px4_sitl gz_x500

启动通信：MicroXRCEAgent udp4 -p 8888

启动qgc地面站：./QGroundControl.AppImage  

启动gazebo环境和加入第一辆无人机

# 多机
指定gazebo
```bash    
    GZ_VERSION=garden make px4_sitl gz_x500
```
清理进程
```bash
    pkill -9 px4; pkill -9 gz; pkill -9 ruby
```
### 下载一个Gazebo的启动脚本(用于创建可以自由添加无人机的gazebo环境)

我的版本：
```bash
wget https://raw.githubusercontent.com/Y-Tomorrow/ros2_air/main/simulation-gazebo.py    
```
原版：
```bash
wget https://raw.githubusercontent.com/PX4/PX4-gazebo-models/main/simulation-gazebo
```

开启仿真
```bash
./simulation-gazebo.py --source px4  # 用px4自带模型
./simulation-gazebo.py --source download  # 用下载模型
./simulation-gazebo.py --no-drone  # 不加载无人机
```
添加第一辆无人机
```bash
export PX4_GZ_STANDALONE=1
export PX4_GZ_MODEL=x500
# 启动
PX4_SYS_AUTOSTART=4001 ~/PX4-Autopilot/build/px4_sitl_default/bin/px4 -i 0
```
添加第二辆无人机（需要修改）
```bash
export PX4_GZ_STANDALONE=1
export PX4_GZ_MODEL=x500
# 设置不同的位置，防止重叠
export PX4_GZ_MODEL_POSE="0,2,0,0,0,0"
# 启动 (注意 -i 1)
PX4_SYS_AUTOSTART=4001 ~/PX4-Autopilot/build/px4_sitl_default/bin/px4 -i 1
```
在对应终端输入commander takeoff检查无人机起飞状况

同时控制两辆无人机
```bash
cd ~/ros2_air
colcon build --packages-select offboard_control
source install/setup.bash
ros2 run offboard_control circle2_drones
```
# 带相机的无人机

修改/PX4-Autopilot/Tools/simulation/gz/models/x500_depth下的model.sdf

    <?xml version="1.0" encoding="UTF-8"?>
    <sdf version='1.9'>
    <model name='x500-Depth'>
        <include merge='true'>
        <uri>x500</uri>
        </include>

        <include>
        <uri>model://OakD-Lite</uri>
        <pose>.12 .03 .242 0 0 0</pose>
        <name>OakD-Lite</name>
        </include>

        <joint name="CameraJoint" type="fixed">
        <parent>base_link</parent>
        <child>OakD-Lite::camera_link</child>
        <pose relative_to="base_link">.12 .03 .242 0 0 0</pose>
        </joint>
    </model>
    </sdf>

复制~/.simulation-gazebo/models/OakD-Lite到/PX4-Autopilot/Tools/simulation/gz/models

在~/PX4-Autopilot/Tools/simulation/gz/models/OakD-Lite/model.sdf中的IMX214相机中加入图像话题

    <always_on>1</always_on>
    <update_rate>30</update_rate>
    <visualize>true</visualize>
    <topic>image_raw</topic>

启动带深度相机无人机（目录/PX4-Autopilot/Tools/simulation/gz/models/x500_dept）
```bash
export PX4_GZ_STANDALONE=1
export PX4_GZ_MODEL=x500_depth
# 设置不同的位置，防止重叠
export PX4_GZ_MODEL_POSE="0,2,0,0,0,0"
# 启动 (注意 -i 1)
PX4_SYS_AUTOSTART=4001 ~/PX4-Autopilot/build/px4_sitl_default/bin/px4 -i 1
```

安装ros-humble-ros-gzgarden将ROS2与Gazebo进行桥接
```bash
sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt-get update
sudo apt-get install gz-garden
sudo apt-get install ros-humble-ros-gzgarden

# 检查
ros2 pkg list | grep ros_gz
# 看到
# ros_gz
# ros_gz_bridge
# ros_gz_image
# ros_gz_interfaces
# ros_gz_sim
# ros_gz_sim_demos
# 就说明安装好了
```
桥接
```bash
# 1. 确保环境变量正确
export GZ_VERSION=garden

# 2. 执行桥接
ros2 run ros_gz_bridge parameter_bridge "/image_raw@sensor_msgs/msg/Image[gz.msgs.Image"
```

# YOLOv8 无人机识别

使用 YOLOv8 对画面中的无人机进行检测。模型文件放在 `model/best_v8.pt`。

### 1）一键创建/配置 conda 环境（推荐）

在主目录下执行：

```bash
cd ~/ros2_air
chmod +x setup_conda_ros2air.sh
./setup_conda_ros2air.sh
```

### 2）编译功能包

```bash
conda activate ros2air
source /opt/ros/humble/setup.bash
cd ~/ros2_air
colcon build --packages-select drone_detector
source install/setup.bash
```

## 3）运行识别节点

```bash
conda activate ros2air
source /opt/ros/humble/setup.bash
source ~/ros2_air/install/setup.bash
ros2 run drone_detector drone_detector_node
```

# 无人机追踪
```bash
ros2 launch drone_tracker drone_tracker.launch.py vehicle_id:=2
```

# 姿态控制
```bash
ros2 launch offboard_control attitude_control.launch.py
ros2 run offboard_control attitude_control_keyboard
```

---
**参考来源：**  
https://blog.csdn.net/m0_70327996/article/details/147143081

https://zhuanlan.zhihu.com/p/1904187016243045984

https://zhuanlan.zhihu.com/p/644405465