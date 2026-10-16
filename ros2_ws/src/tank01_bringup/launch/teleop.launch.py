"""Gamepad teleoperation → /cmd_vel_teleop (highest twist_mux priority)."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory('tank01_bringup'), 'config', 'teleop.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('profile', default_value=''),
        Node(package='joy', executable='joy_node', name='joy_node', parameters=[cfg]),
        Node(package='teleop_twist_joy', executable='teleop_node',
             name='teleop_twist_joy_node', parameters=[cfg],
             remappings=[('cmd_vel', 'cmd_vel_teleop')]),
    ])
