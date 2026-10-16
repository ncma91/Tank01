"""Start the mission node for one field.

ros2 launch tank01_mower mission.launch.py field:=/path/feld.geojson start:=true [profile:=prototyp]
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def launch_setup(context):
    cfg = os.path.join(get_package_share_directory('tank01_bringup'), 'config')
    files = [os.path.join(cfg, 'mower.yaml')]
    profile = LaunchConfiguration('profile').perform(context)
    override = os.path.join(cfg, 'profiles', profile, 'mower.yaml')
    if profile and os.path.exists(override):
        files.append(override)
    return [Node(
        package='tank01_mower', executable='mission_node', name='mission_node',
        output='screen',
        parameters=[
            *files,
            {'field_file': LaunchConfiguration('field'),
             'field_name': LaunchConfiguration('field_name'),
             'start': ParameterValue(LaunchConfiguration('start'), value_type=bool),
             'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'),
                                            value_type=bool)},
        ],
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('field', description='GeoJSON file with the field'),
        DeclareLaunchArgument('field_name', default_value=''),
        DeclareLaunchArgument('start', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('profile', default_value=''),
        OpaqueFunction(function=launch_setup),
    ])
