"""Dual EKF + navsat_transform (docs/07_lokalisierung_navigation.md)."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    ekf = os.path.join(get_package_share_directory('tank01_bringup'), 'config', 'ekf.yaml')
    sim_time = {'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'),
                                               value_type=bool)}
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(package='robot_localization', executable='ekf_node', name='ekf_local',
             parameters=[ekf, sim_time],
             remappings=[('odometry/filtered', '/odometry/local')]),
        Node(package='robot_localization', executable='ekf_node', name='ekf_global',
             parameters=[ekf, sim_time],
             remappings=[('odometry/filtered', '/odometry/global')]),
        Node(package='robot_localization', executable='navsat_transform_node',
             name='navsat_transform', parameters=[ekf, sim_time],
             remappings=[('imu/data', '/gnss/heading'),
                         ('gps/fix', '/gnss/fix'),
                         ('odometry/filtered', '/odometry/global'),
                         ('odometry/gps', '/odometry/gnss'),
                         ('gps/filtered', '/gnss/filtered')]),
    ])
