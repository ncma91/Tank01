"""Nav2 servers for Tank01 (own launch to control the velocity output topic).

Nav2's final velocity (after the velocity smoother) is published on /cmd_vel_nav and
goes through twist_mux and the safety supervisor - never directly to the base.
"""

import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

sys.path.insert(0, os.path.dirname(__file__))
from profile_utils import param_files  # noqa: E402


def launch_setup(context):
    params = param_files(LaunchConfiguration('profile').perform(context), 'nav2_params.yaml')
    sim_time = {'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'),
                                               value_type=bool)}
    common = dict(output='screen', parameters=[*params, sim_time])
    lifecycle_nodes = ['controller_server', 'smoother_server', 'planner_server',
                       'behavior_server', 'bt_navigator', 'waypoint_follower',
                       'velocity_smoother']
    return [
        Node(package='nav2_controller', executable='controller_server',
             remappings=[('cmd_vel', 'cmd_vel_raw')], **common),
        Node(package='nav2_smoother', executable='smoother_server', name='smoother_server',
             **common),
        Node(package='nav2_planner', executable='planner_server', name='planner_server',
             **common),
        Node(package='nav2_behaviors', executable='behavior_server', name='behavior_server',
             remappings=[('cmd_vel', 'cmd_vel_raw')], **common),
        Node(package='nav2_bt_navigator', executable='bt_navigator', name='bt_navigator',
             **common),
        Node(package='nav2_waypoint_follower', executable='waypoint_follower',
             name='waypoint_follower', **common),
        Node(package='nav2_velocity_smoother', executable='velocity_smoother',
             name='velocity_smoother',
             remappings=[('cmd_vel', 'cmd_vel_raw'), ('cmd_vel_smoothed', 'cmd_vel_nav')],
             **common),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_navigation', output='screen',
             parameters=[sim_time, {'autostart': True, 'node_names': lifecycle_nodes,
                                    'bond_timeout': 4.0}]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('profile', default_value=''),
        OpaqueFunction(function=launch_setup),
    ])
