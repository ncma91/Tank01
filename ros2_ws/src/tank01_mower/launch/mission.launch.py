"""Start the mission node for one field.

ros2 launch tank01_mower mission.launch.py field:=/path/feld.geojson start:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bringup = get_package_share_directory('tank01_bringup')
    return LaunchDescription([
        DeclareLaunchArgument('field', description='GeoJSON file with the field'),
        DeclareLaunchArgument('field_name', default_value=''),
        DeclareLaunchArgument('start', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(
            package='tank01_mower', executable='mission_node', name='mission_node',
            output='screen',
            parameters=[
                os.path.join(bringup, 'config', 'mower.yaml'),
                {'field_file': LaunchConfiguration('field'),
                 'field_name': LaunchConfiguration('field_name'),
                 'start': ParameterValue(LaunchConfiguration('start'), value_type=bool),
                 'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'),
                                                value_type=bool)},
            ],
        ),
    ])
