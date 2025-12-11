import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition


def generate_launch_description():
    ld = LaunchDescription()
    
    # 默认params文件路径
    default_config = os.path.join(
        get_package_share_directory("explore_lite"), "config", "params.yaml"
    )
    
    # 默认rviz配置文件路径
    default_rviz_config = os.path.join(
        get_package_share_directory("explore_lite"), "rviz", "explore.rviz"
    )
    
    # 声明参数，允许用户通过命令行覆盖
    params_file = LaunchConfiguration("params_file")
    rviz_config_file = LaunchConfiguration("rviz_config_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    namespace = LaunchConfiguration("namespace")
    use_rviz = LaunchConfiguration("use_rviz")

    declare_params_file_argument = DeclareLaunchArgument(
        "params_file",
        default_value=default_config,
        description="Path to explore_lite params file",
    )
    declare_rviz_config_file_argument = DeclareLaunchArgument(
        "rviz_config_file",
        default_value=default_rviz_config,
        description="Path to RViz config file",
    )
    declare_use_sim_time_argument = DeclareLaunchArgument(
        "use_sim_time", default_value="true", description="Use simulation/Gazebo clock"
    )
    declare_namespace_argument = DeclareLaunchArgument(
        "namespace",
        default_value="",
        description="Namespace for the explore node",
    )
    declare_use_rviz_argument = DeclareLaunchArgument(
        "use_rviz",
        default_value="true",
        description="Launch RViz visualization",
    )

    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    # In case of the transforms (tf), currently, there doesn't seem to be a better alternative
    # https://github.com/ros/geometry2/issues/32
    # https://github.com/ros/robot_state_publisher/pull/30
    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    explore_node = Node(
        package="explore_lite",
        name="explore_node",
        namespace=namespace,
        executable="explore",
        parameters=[params_file, {"use_sim_time": use_sim_time}],
        output="screen",
        remappings=remappings,
    )
    
    # RViz启动命令，使用配置文件
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_file],
        output="screen",
        condition=IfCondition(use_rviz),
    )

    ld.add_action(declare_params_file_argument)
    ld.add_action(declare_rviz_config_file_argument)
    ld.add_action(declare_use_sim_time_argument)
    ld.add_action(declare_namespace_argument)
    ld.add_action(declare_use_rviz_argument)
    ld.add_action(explore_node)
    ld.add_action(rviz_node)
    return ld


