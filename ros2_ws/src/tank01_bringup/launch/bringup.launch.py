"""Tank01 full system bringup.

ros2 launch tank01_bringup bringup.launch.py [use_sensors:=true] [use_nav:=true]
    [use_teleop:=true] [fake_hardware:=false] [geofence:=<geojson>]
    [require_perception:=true] [use_sim_time:=false]
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bringup = get_package_share_directory('tank01_bringup')
    description = get_package_share_directory('tank01_description')
    cfg = os.path.join(bringup, 'config')
    launch_dir = os.path.join(bringup, 'launch')

    use_sim_time = LaunchConfiguration('use_sim_time')
    sim_time = {'use_sim_time': ParameterValue(use_sim_time, value_type=bool)}

    def include(path, condition=None, **args):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(path),
            launch_arguments={'use_sim_time': use_sim_time, **args}.items(),
            condition=condition)

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('use_sensors', default_value='true'),
        DeclareLaunchArgument('use_localization', default_value='true'),
        DeclareLaunchArgument('use_nav', default_value='true'),
        DeclareLaunchArgument('use_teleop', default_value='true'),
        DeclareLaunchArgument('use_foxglove', default_value='true'),
        DeclareLaunchArgument('fake_hardware', default_value='false',
                              description='simulate the CAN base controller'),
        DeclareLaunchArgument('require_perception', default_value='true',
                              description='safety needs /perception/nearest_person'),
        DeclareLaunchArgument(
            'geofence', default_value=os.path.join(bringup, 'missions',
                                                   'beispiel_wiese.geojson'),
            description='GeoJSON with a "geofence" (or "field") polygon'),

        include(os.path.join(description, 'launch', 'description.launch.py')),

        # --- Level 1/2: base driver + safety supervisor ------------------------
        Node(package='tank01_base', executable='base_node', name='base_node',
             output='screen',
             parameters=[os.path.join(cfg, 'base.yaml'), sim_time,
                         {'use_fake_hardware': ParameterValue(
                             LaunchConfiguration('fake_hardware'), value_type=bool)}]),
        Node(package='tank01_safety', executable='safety_supervisor',
             name='safety_supervisor', output='screen',
             parameters=[os.path.join(cfg, 'safety.yaml'), sim_time,
                         {'geofence_file': LaunchConfiguration('geofence'),
                          'require_perception': ParameterValue(
                              LaunchConfiguration('require_perception'), value_type=bool)}]),
        Node(package='twist_mux', executable='twist_mux', name='twist_mux',
             parameters=[os.path.join(cfg, 'twist_mux.yaml'), sim_time],
             remappings=[('cmd_vel_out', 'cmd_vel_mux')]),
        Node(package='diagnostic_aggregator', executable='aggregator_node',
             name='diagnostic_aggregator',
             parameters=[os.path.join(cfg, 'diagnostics.yaml')],
             remappings=[('/diagnostics', '/base/status')]),

        # --- Sensors, localisation, navigation ---------------------------------
        include(os.path.join(launch_dir, 'sensors.launch.py'),
                IfCondition(LaunchConfiguration('use_sensors'))),
        include(os.path.join(launch_dir, 'localization.launch.py'),
                IfCondition(LaunchConfiguration('use_localization'))),
        include(os.path.join(launch_dir, 'navigation.launch.py'),
                IfCondition(LaunchConfiguration('use_nav'))),
        include(os.path.join(launch_dir, 'teleop.launch.py'),
                IfCondition(LaunchConfiguration('use_teleop'))),

        Node(package='foxglove_bridge', executable='foxglove_bridge',
             condition=IfCondition(LaunchConfiguration('use_foxglove')),
             parameters=[{'port': 8765, 'address': '0.0.0.0'}]),
    ])
