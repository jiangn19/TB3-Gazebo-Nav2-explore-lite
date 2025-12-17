# TB3-Gazebo-Nav2-explore-lite
TB3 + Gazebo + Nav2 + explore_lite 仿真与自主探索

## 安装配置

### 仿真环境前置要求：按照教程安装vln_gazebo_simulator
```bash
参考：https://github.com/Tipriest/vln_gazebo_simulator
参考放置目录: ~/Documents/vln_gazebo_simulator #或你的工作空间路径
```
### 安装Nav2和TB3
``` bash
sudo apt install ros-humble-navigation2
sudo apt install ros-humble-nav2-bringup
sudo apt install ros-humble-slam-toolbox
sudo apt install ros-humble-turtlebot3-gazebo
```
### 下载explore源码
``` bash
cd ~/Documents/vln_gazebo_simulator/src #或你的工作空间路径
git clone https://github.com/jiangn19/TB3-Gazebo-Nav2-explore-lite
# git submodule update --init --recursive
```
### 编译
``` bash
cd ..
source /opt/ros/humble/setup.bash 
colcon build --symlink-install
# colcon build --symlink-install --packages-select explore_lite
```
### 配置路径依赖
``` bash
chmod +x ~/Documents/vln_gazebo_simulator/src/TB3-Gazebo-Nav2-explore-lite/setup_explore.bash #或你的工作空间路径
```
在bashrc中添加下面一行：
``` bash
source ~/Documents/vln_gazebo_simulator/src/TB3-Gazebo-Nav2-explore-lite/setup_explore.bash  #或你的工作空间路径
```
``` bash
source ~/.bashrc
turtle3init
```
### 测试
``` bash
# tb3功能性测试
# ros2 launch nav2_bringup tb3_simulation_launch.py slam:=True
# ros2 launch tb3_simulation tb3_simulation_house_launch.py slam:=True
```
## 参数
Nav2与explorelite默认参数位于config文件夹中
启动节点时，使用'params_file:='指定加载的参数位置


## 运行

### 运行仿真场景
``` bash
turtle3init
# 启动00829场景 tb3 + gazebo场景仿真
ros2 launch turtlebot3_gazebo turtlebot3_00829.launch.py
```

### 探索阶段
### [探索阶段]启动slam_toolbox建图
``` bash
turtle3init
```
```bash
# 启动slam-toolbox在线建图（使用仿真时间）
ros2 launch slam_toolbox online_async_launch.py \
  use_sim_time:=True \
  slam_params_file:=/home/{$USER_NAME}/Documents/vln_gazebo_simulator/src/TB3-Gazebo-Nav2-explore-lite/config/mapping/mapper_params_online_async.yaml # 或你自己的param路径
```
``` bash
# chengsn
ros2 launch slam_toolbox online_async_launch.py \
  use_sim_time:=True \
  params_file:=/home/chengsn/Workspace/VLN_ws/vln_gazebo_simulator/src/TB3-Gazebo-Nav2-explore-lite/config/mapping/mapper_params_online_async.yaml
```

### [探索阶段]启动nav2导航: 不包含amcl和nav2_map_server
### map到odom转换由slam提供
``` bash
turtle3init
```
``` bash
# default
ros2 launch nav2_bringup navigation_launch.py \
  use_sim_time:=True \
  params_file:=/home/{$USER_NAME}/Documents/vln_gazebo_simulator/src/TB3-Gazebo-Nav2-explore-lite/config/nav2/nav2_params.yaml # 或你自己的param路径
```
``` bash
# jiangn19
ros2 launch nav2_bringup navigation_launch.py \
  use_sim_time:=True \
  params_file:=/home/nanyuanchaliang/Documents/vln_gazebo_simulator/nav2_params.yaml
```
``` bash
# chengsn
ros2 launch nav2_bringup navigation_launch.py \
  use_sim_time:=True \
  params_file:=/home/chengsn/Workspace/VLN_ws/vln_gazebo_simulator/src/TB3-Gazebo-Nav2-explore-lite/config/nav2/nav2_params.yaml
```
``` bash
# sudongxu
ros2 launch nav2_bringup navigation_launch_without_smooth.py \
  use_sim_time:=True \
  params_file:=/home/sudongxu/Documents/vln_gazebo_simulator/src/TB3-Gazebo-Nav2-explore-lite/config/nav2/nav2_params_pnc_dwb2.yaml  # 不包含平滑节点，以符合机器人底盘的要求
```

### [探索阶段]启动explore_lite自主探索
```bash
turtle3init
```
``` bash
# 使用默认params启动:位于explore文件夹
ros2 launch explore_lite explore.launch.py use_sim_time:=True
```
``` bash
# 使用自定义params文件启动
ros2 launch explore_lite explore.launch.py \
  use_sim_time:=True \
  params_file:=/home/{$USER_NAME}/Documents/vln_gazebo_simulator/src/TB3-Gazebo-Nav2-explore-lite/config/explore/params_costmap.yaml
```
``` bash
# jiangn19
ros2 launch explore_lite explore.launch.py \
  use_sim_time:=True \
  params_file:=/home/nanyuanchaliang/Documents/vln_gazebo_simulator/params_costmap.yaml
```
``` bash
# chengsn
ros2 launch explore_lite explore.launch.py \
  use_sim_time:=True \
  params_file:=/home/chengsn/Workspace/VLN_ws/vln_gazebo_simulator/src/TB3-Gazebo-Nav2-explore-lite/config/explore/params_costmap.yaml
```

### [探索阶段]保存建好的2D地图
```bash
turtle3init
```
```bash
ros2 run nav2_map_server map_saver_cli -f ~/Workspace/VLN_ws/vln_gazebo_simulator/src/TB3-Gazebo-Nav2-explore-lite/map/map_00829
```

### [导航阶段]启动nav2导航
``` bash
[TODO]状态机实现模式切换
```
``` bash
[TODO]nav2_params导航参数调节
```
``` bash
turtle3init
```
``` bash
# default
ros2 launch nav2_bringup bringup_launch.py \
  use_sim_time:=True \
  autostart:=False \
  map:=/home/{$USER_NAME}/Documents/vln_gazebo_simulator/src/TB3-Gazebo-Nav2-explore-lite/map/map_00829.yaml \
  params_file:=/home/{$USER_NAME}/Documents/vln_gazebo_simulator/src/TB3-Gazebo-Nav2-explore-lite/config/nav2/nav2_params.yam  # 或你自己的param路径
```

``` bash
# sudongxu
ros2 launch nav2_bringup bringup_launch_without_smooth.py \
  use_sim_time:=True \
  autostart:=False \
  map:=/home/sudongxu/Documents/vln_gazebo_simulator/src/TB3-Gazebo-Nav2-explore-lite/map/map1216.yaml \
  params_file:=/home/sudongxu/Documents/vln_gazebo_simulator/src/TB3-Gazebo-Nav2-explore-lite/config/nav2/nav2_params_pnc_dwb2.yaml  # 不包含平滑节点，以符合机器人底盘的要求
```

``` bash
# chengsn
ros2 launch nav2_bringup navigation_launch.py \
  use_sim_time:=True \
  autostart:=False \
  map:=/home/chengsn/Workspace/VLN_ws/vln_gazebo_simulator/src/TB3-Gazebo-Nav2-explore-lite/maps/site_00829.yaml \
  params_file:=/home/chengsn/Workspace/VLN_ws/vln_gazebo_simulator/src/TB3-Gazebo-Nav2-explore-lite/config/nav2/nav2_params.yaml
```

``` bash
LaunchArgument参数解释
- slam：Whether run a SLAM，默认为False
   设置slam=True， 引入 slam_launch.py（map_saver + slam_toolbox 等）
   否则，设置slam=False， 引入 localization_launch.py（map_server + amcl）
   无论上述选择如何，都引入 navigation_launch.py（规划/控制/BT等）
- autostart：Automatically startup the nav2 stack，默认为True
- map：Full path to map yaml file to load
```

``` bash
而后通过以下指令开启rviz，上面已将autostart设置为False，需要在rviz中设置2D Pose Estimate 与 Nav2 Goal 。
```
[TODO]否则，需要手动发送2D Pose Estimate， 利用状态机自动发送。
``` bash
ros2 launch nav2_bringup rviz_launch.py
```


### [导航阶段]仅启动导航节点
``` bash
ros2 launch nav2_bringup navigation_launch_without_smooth.py \
  use_sim_time:=True \
  params_file:=/home/sudongxu/Documents/vln_gazebo_simulator/src/TB3-Gazebo-Nav2-explore-lite/config/nav2/nav2_params_pnc_dwb2.yaml  # 不包含平滑节点，以符合机器人底盘的要求
```

### [导航阶段]启动语义障碍投影节点
``` bash
turtle3init
```
#### 从yaml文件加载keepout mask
```bash
ros2 launch nav2_costmap_filters_demo bbox_costmap_filter.launch.py use_sim_time:=True params_file:=src/TB3-Gazebo-Nav2-explore-lite/src/nav2_costmap_filters_demo/params/keepout_params.yaml bboxes:=config/nav2/keepout_bboxes.yaml
```
```bash
# chengsn
ros2 launch nav2_costmap_filters_demo bbox_costmap_filter.launch.py use_sim_time:=True params_file:=src/TB3-Gazebo-Nav2-explore-lite/src/nav2_costmap_filters_demo/params/keepout_params.yaml bboxes:=src/TB3-Gazebo-Nav2-explore-lite/config/nav2/keepout_bboxes.yaml
```
