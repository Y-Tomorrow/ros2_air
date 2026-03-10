# 一、gazebo仿真环境搭建与px4的编译


1、克隆px4源码，并进入文件夹

    git clone https://github.com/PX4/PX4-Autopilot.git --recursive
    备份
    zip -r PX4-Autopilot.zip PX4-Autopilot/
    安装相关依赖：
    cd PX4-Autopilot
    bash ./PX4-Autopilot/Tools/setup/ubuntu.sh
    
2、gazebo

    1、添加软件源：

    sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

    2、添加密钥

    wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -

    3、更新软件包并安装gazebo garden

    sudo apt update
    sudo apt install gz-garden


3、用gazebo garden编译

    make px4_sitl gz_x500


# 二、安装QGoundControl地面站

1、安装相关的依赖项（可能需要重启来启用用户权限的更改）

    sudo usermod -a -G dialout $USER
    sudo apt-get remove modemmanager -y
    sudo apt install gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl -y
    sudo apt install libfuse2 -y
    sudo apt install libxcb-xinerama0 libxkbcommon-x11-0 libxcb-cursor-dev -y

2、下载QGroundControl.AppImage

点击链接前往下载：https://qgroundcontrol.com/

3、授权并运行（QGC会自动与PX4连接）

    chmod +x ./QGroundControl.AppImage
    ./QGroundControl.AppImage



# 三、安装ROS2
1、推荐鱼香ros一键安装（ros2 humble）

    wget http://fishros.com/install -O fishros && . fishros

2.出现问题可以这样卸载

    sudo apt remove ros-humble-*
    sudo apt autoremove

# 四、安装 Micro XRCE-DDS Agent

1、下载源码

    git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git

2、编译

    cd Micro-XRCE-DDS-Agent
    mkdir build
    cd build
    cmake ..
    make


3、安装

    sudo make install
    sudo ldconfig /usr/local/lib/


4、启动

    MicroXRCEAgent udp4 -p 8888

# 五、轨迹跟踪仿真步骤

1、创建工作区

    mkdir -p ~/ros2_ws/src

2、下载源码

    cd ~/ros2_ws/src
    git clone https://github.com/PX4/px4_msgs.git -b release/1.14
    git clone https://github.com/PX4/px4_ros_com.git -b release/v1.14


准备工作完毕，接下来尝试运行官方代码

1、构建项目

    cd ~/ros2_ws
    colcon build

2、更新环境

    cd ~/ros2_ws
    source install/setup.bash


3、运行官方offboard案例

    ros2 run px4_ros_com offboard_control


# 六、总结

如果上面的配置步骤全部成功，那么就可以实现任意轨迹的仿真了，下面是总结的步骤

1、创建功能包

    cd ~/ros2_ws/src
    ros2 pkg create --build-type ament_cmake your_bagname  
    
（your_bagname替换为你想创建的功能包，比如你想跑圆形轨迹可以命名为circle）

2、编写轨迹代码，保存到~/ros2_ws/src/your_bagname/src/your_codename.cpp，这里提供一段圆形轨迹代码供参考，轨迹代码名字为circle1.cpp，每次编写完记得保存

    src/circle/src/circle1.cpp

3、编写cmakelists.txt文件

cmakelist文件是在创建好的功能包里面的，双击打开，复制下面内容将其替换掉，然后保存

    src/circle/CMakeLists.txt

4、编译

    cd ~/ros2_ws
    colcon build --packages-select your_bagname
（如果是上面的圆形轨迹案例记得把这里的your_bagname改为circle）

5、运行

    source install/setup.bash
    ros2 run your_bagname your_codename
    
（如果是上面的圆形轨迹案例记得把这里的your_bagname改为circle，your_codename改为circle1）

    chmod +x start.sh
    ./start.sh

start.sh里执行了

启动px4及其gazebo仿真：make px4_sitl gz_x500

启动通信：MicroXRCEAgent udp4 -p 8888

启动qgc地面站：./QGroundControl.AppImage  



# 多机
指定gazebo
    
    GZ_VERSION=garden make px4_sitl gz_x500

清理进程

    pkill -9 px4; pkill -9 gz; pkill -9 ruby

下载一个Gazebo的启动脚本

    我的版本：
    wget https://github.com/Y-Tomorrow/ros2_air/blob/main/simulation-gazebo.py
    原版：
    wget https://raw.githubusercontent.com/PX4/PX4-gazebo-models/main/simulation-gazebo


开启仿真

    ./simulation-gazebo.py --source px4  # 用px4自带模型
    ./simulation-gazebo.py --source download  # 用下载模型
    ./simulation-gazebo.py --no-drone  # 不加载无人机

添加第一辆无人机

    export PX4_GZ_STANDALONE=1
    export PX4_GZ_MODEL=x500
    # 启动
    PX4_SYS_AUTOSTART=4001 ~/PX4-Autopilot/build/px4_sitl_default/bin/px4 -i 0

添加第二辆无人机（需要修改）

    export PX4_GZ_STANDALONE=1
    export PX4_GZ_MODEL=x500
    # 设置不同的位置，防止重叠
    export PX4_GZ_MODEL_POSE="0,2,0,0,0,0"
    # 启动 (注意 -i 1)
    PX4_SYS_AUTOSTART=4001 ~/PX4-Autopilot/build/px4_sitl_default/bin/px4 -i 1

在对应终端输入commander takeoff检查无人机起飞状况

同时控制两辆无人机

    cd ~/ros2_air
    colcon build --packages-select circle
    source install/setup.bash
    ros2 run circle circle2_drones

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

启动带深度相机无人机（目录/PX4-Autopilot/Tools/simulation/gz/models/x500_dept）

    export PX4_GZ_STANDALONE=1
    export PX4_GZ_MODEL=x500_depth
    # 设置不同的位置，防止重叠
    export PX4_GZ_MODEL_POSE="0,2,0,0,0,0"
    # 启动 (注意 -i 1)
    PX4_SYS_AUTOSTART=4001 ~/PX4-Autopilot/build/px4_sitl_default/bin/px4 -i 1

---
**参考来源：**  
https://blog.csdn.net/m0_70327996/article/details/147143081

https://zhuanlan.zhihu.com/p/1904187016243045984