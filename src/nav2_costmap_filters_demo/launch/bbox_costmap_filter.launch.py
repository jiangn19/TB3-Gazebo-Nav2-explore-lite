#!/usr/bin/env python3
"""
Launch file that starts the bbox mask publisher and the costmap_filter_info_server.

Usage example:
ros2 launch nav2_costmap_filters_demo bbox_costmap_filter.launch.py \
  params_file:=src/nav2_costmap_filters_demo/params/keepout_params.yaml \
  bboxes:=src/nav2_costmap_filters_demo/params/keepout_bboxes.yaml

This keeps the rest of the system (costmap_filter_info_server + lifecycle manager)
unchanged. The bbox mask is published on the topic configured in the bbox yaml
(`topic`, default `/keepout_filter_mask`) and `costmap_filter_info_server` uses
its configured `mask_topic` to read it.
"""
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, LoadComposableNodes
from launch_ros.actions import PushRosNamespace
from launch_ros.descriptions import ComposableNode
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    pkg_dir = get_package_share_directory('nav2_costmap_filters_demo')

    lifecycle_nodes = ['costmap_filter_info_server']

    # Launch arguments
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    bboxes_file = LaunchConfiguration('bboxes')
    use_composition = LaunchConfiguration('use_composition')
    container_name = LaunchConfiguration('container_name')
    container_name_full = (namespace, '/', container_name)

    declare_namespace_cmd = DeclareLaunchArgument('namespace', default_value='', description='Top-level namespace')
    declare_use_sim_time_cmd = DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation clock')
    declare_autostart_cmd = DeclareLaunchArgument('autostart', default_value='true', description='Autostart')
    declare_params_file_cmd = DeclareLaunchArgument('params_file', description='Full path to the ROS2 parameters file to use')
    declare_bboxes_cmd = DeclareLaunchArgument('bboxes', description='Full path to bbox YAML file to load')
    declare_use_composition_cmd = DeclareLaunchArgument('use_composition', default_value='False', description='Use composition')
    declare_container_name_cmd = DeclareLaunchArgument('container_name', default_value='nav2_container', description='Container name')

    # rewrite params (pass use_sim_time through)
    param_substitutions = {
        'use_sim_time': use_sim_time,
    }

    configured_params = RewrittenYaml(source_file=params_file, root_key=namespace, param_rewrites=param_substitutions, convert_types=True)

    # Use installed script from lib/<package>/ directory
    script_path = os.path.join(pkg_dir, '..', '..', 'lib', 'nav2_costmap_filters_demo', 'bbox_mask_server.py')
    
    exec_bbox = ExecuteProcess(
        cmd=['python3', script_path, bboxes_file, '--ros-args', '-p', ['use_sim_time:=', use_sim_time]],
        output='screen'
    )

    # costmap_filter_info_server node (same as upstream)
    load_nodes = GroupAction(
        condition=IfCondition(PythonExpression(['not ', use_composition])),
        actions=[
            Node(
                package='nav2_map_server',
                executable='costmap_filter_info_server',
                name='costmap_filter_info_server',
                namespace=namespace,
                output='screen',
                emulate_tty=True,
                parameters=[configured_params]
            ),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_costmap_filters',
                namespace=namespace,
                output='screen',
                emulate_tty=True,
                parameters=[{'use_sim_time': use_sim_time}, {'autostart': autostart}, {'node_names': lifecycle_nodes}]
            )
        ]
    )

    ld = LaunchDescription()

    # add arguments
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_bboxes_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_container_name_cmd)

    ld.add_action(exec_bbox)
    ld.add_action(load_nodes)

    return ld
