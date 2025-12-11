# TB3-Gazebo-Nav2-explore-lite
TB3 + Gazebo + Nav2 + explore_lite 仿真与自主探索

## 仿真环境前置要求
### 按照教程安装vln_gazebo_simulator
参考：https://github.com/Tipriest/vln_gazebo_simulator

## 安装步骤
### 安装Nav2和TB3
``` bash
sudo apt install ros-humble-navigation2
sudo apt install ros-humble-nav2-bringup
sudo apt install ros-humble-slam-toolbox
sudo apt install ros-humble-turtlebot3-gazebo
```
### Clone ExploreLite
打开/Documents/vln_gazebo_simulator/src

``` bash
cd ~/Documents/vln_gazebo_simulator/src
git clone https://github.com/robo-friends/m-explore-ros2.git
```

### 编译
``` bash
cd ..
colcon build --symlink-install
```

### 

### 运行简单场景仿真
``` bash
source /opt/ros/humble/setup.bash
source ~/Workspace/explore_ws/install/setup.bash
export TURTLEBOT3_MODEL=waffle
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/${ROS_DISTRO}/share/turtlebot3_gazebo/models
# （可选）简单场景
ros2 launch nav2_bringup tb3_simulation_launch.py slam:=True
# （可选）房屋场景
ros2 launch tb3_simulation tb3_simulation_house_launch.py slam:=True


# habitatsim场景

# 导航
ros2 launch nav2_bringup navigation_launch.py slam:=True params_file:=/home/chengsn/Workspace/explore_ws/src/tb3_simulation/config/nav2_params_house.yaml
# 建图场景
ros2 launch slam_toolbox online_sync_launch.py use_sim_time:=True
# （可选）键盘控制节点

# 探索节点
ros2 launch explore_lite explore.launch.py
```
