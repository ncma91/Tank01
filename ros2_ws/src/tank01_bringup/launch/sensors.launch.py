"""Sensor drivers. Disable individual sensors with use_<sensor>:=false.

Topic contract (consumed by localisation, Nav2 and safety):
  /gnss/fix, /gnss/heading, /gnss/vel   u-blox ZED-F9P moving base
  /rtcm                                 NTRIP corrections
  /imu/data                             Xsens MTi-630
  /points                               3D LiDAR (raw)
  /points_obstacles                     LiDAR without ground points (Patchwork++)
  /kiss_icp/odometry                    LiDAR odometry
  /camera/...                           ZED stereo camera
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction, IncludeLaunchDescription,
                            LogInfo)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory('tank01_bringup'), 'config')
    secrets = os.path.join(cfg, 'secrets.yaml')
    ntrip_params = [os.path.join(cfg, 'gnss.yaml')] + ([secrets] if os.path.exists(secrets)
                                                       else [])

    def flag(name):
        return IfCondition(LaunchConfiguration(name))

    def node(package, **kw):
        # Missing drivers are reported instead of aborting the whole bringup.
        if _has(package):
            return Node(package=package, **kw)
        return LogInfo(msg=f'[tank01] package {package} not installed - skipped')

    actions = [
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('use_gnss', default_value='true'),
        DeclareLaunchArgument('use_ntrip', default_value='true'),
        DeclareLaunchArgument('use_imu', default_value='true'),
        DeclareLaunchArgument('use_lidar', default_value='true'),
        DeclareLaunchArgument('use_camera', default_value='true'),
        DeclareLaunchArgument('lidar_hostname', default_value='192.168.10.20'),
        DeclareLaunchArgument('camera_model', default_value='zedx'),

        # GNSS rover + moving base heading
        node('ublox_gps', executable='ublox_gps_node', name='ublox_gps_node',
             condition=flag('use_gnss'), output='screen',
             parameters=[os.path.join(cfg, 'gnss.yaml')],
             remappings=[('fix', '/gnss/fix'), ('fix_velocity', '/gnss/vel'),
                         ('navheading', '/gnss/heading')]),
        node('ntrip_client', executable='ntrip_ros.py', name='ntrip_client',
             condition=flag('use_ntrip'), parameters=ntrip_params,
             remappings=[('fix', '/gnss/fix_nmea_source'), ('rtcm', '/rtcm')]),

        # IMU
        node('xsens_mti_driver', executable='xsens_mti_node', name='xsens_driver',
             condition=flag('use_imu'), parameters=[os.path.join(cfg, 'imu.yaml')],
             remappings=[('imu/data', '/imu/data')]),

        # LiDAR (Ouster). For Livox Mid-360 use livox_ros_driver2 and the same remaps.
        GroupAction(condition=flag('use_lidar'), actions=[
            SetRemap('/ouster/points', '/points'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(
                    get_package_share_directory('ouster_ros'), 'launch', 'driver.launch.py')),
                launch_arguments={'sensor_hostname': LaunchConfiguration('lidar_hostname'),
                                  'viz': 'false'}.items()),
        ]) if _has('ouster_ros') else LogInfo(msg='[tank01] ouster_ros not installed'),

        # Ground segmentation → obstacle points for the costmaps
        node('patchworkpp', executable='patchworkpp_node', name='ground_segmentation',
             condition=flag('use_lidar'),
             parameters=[{'sensor_height': 1.15, 'base_frame': 'base_link'}],
             remappings=[('pointcloud_topic', '/points'),
                         ('/patchworkpp/nonground', '/points_obstacles')]),

        # LiDAR odometry
        node('kiss_icp', executable='kiss_icp_node', name='kiss_icp_node',
             condition=flag('use_lidar'), parameters=[os.path.join(cfg, 'kiss_icp.yaml')],
             remappings=[('pointcloud_topic', '/points'),
                         ('/kiss/odometry', '/kiss_icp/odometry')]),
    ]

    if _has('zed_wrapper'):
        actions.append(GroupAction(condition=flag('use_camera'), actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(
                    get_package_share_directory('zed_wrapper'), 'launch',
                    'zed_camera.launch.py')),
                launch_arguments={'camera_model': LaunchConfiguration('camera_model'),
                                  'camera_name': 'camera',
                                  'publish_tf': 'false'}.items()),
        ]))
    return LaunchDescription(actions)


def _has(pkg: str) -> bool:
    try:
        get_package_share_directory(pkg)
        return True
    except Exception:  # PackageNotFoundError
        return False
