"""Dual EKF + navsat_transform (docs/07_lokalisierung_navigation.md)."""

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
    ekf = param_files(LaunchConfiguration('profile').perform(context), 'ekf.yaml')
    sim_time = {'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'),
                                               value_type=bool)}
    return [
        Node(package='robot_localization', executable='ekf_node', name='ekf_local',
             parameters=[*ekf, sim_time],
             remappings=[('odometry/filtered', '/odometry/local')]),
        Node(package='robot_localization', executable='ekf_node', name='ekf_global',
             parameters=[*ekf, sim_time],
             remappings=[('odometry/filtered', '/odometry/global')]),
        Node(package='robot_localization', executable='navsat_transform_node',
             name='navsat_transform', parameters=[*ekf, sim_time],
             remappings=[('imu/data', '/gnss/heading'),
                         ('gps/fix', '/gnss/fix'),
                         ('odometry/filtered', '/odometry/global'),
                         ('odometry/gps', '/odometry/gnss'),
                         ('gps/filtered', '/gnss/filtered')]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('profile', default_value=''),
        OpaqueFunction(function=launch_setup),
    ])
