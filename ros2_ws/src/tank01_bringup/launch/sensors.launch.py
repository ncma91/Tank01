"""Sensor drivers. Disable individual sensors with use_<sensor>:=false.

Driver selection (default "auto" = taken from the profile, see profile_utils.py):
  gnss_driver:=ublox|um982     lidar_driver:=ouster|unitree     imu_driver:=xsens|lidar

Topic contract (consumed by localisation, Nav2 and safety):
  /gnss/fix, /gnss/heading             RTK position + dual-antenna heading
  /rtcm                                NTRIP corrections
  /imu/data                            IMU with roll/pitch orientation
  /points                              3D LiDAR (raw)
  /points_obstacles                    LiDAR without ground points (Patchwork++)
  /kiss_icp/odometry                   LiDAR odometry
  /camera/...                          stereo camera (full design)
"""

import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction, IncludeLaunchDescription,
                            LogInfo, OpaqueFunction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap

sys.path.insert(0, os.path.dirname(__file__))
from profile_utils import CONFIG_DIR, param_files, profile_default  # noqa: E402


def _has(pkg: str) -> bool:
    try:
        get_package_share_directory(pkg)
        return True
    except Exception:  # PackageNotFoundError
        return False


def launch_setup(context):
    profile = LaunchConfiguration('profile').perform(context)
    gnss = profile_default(profile, 'gnss_driver',
                           LaunchConfiguration('gnss_driver').perform(context))
    lidar = profile_default(profile, 'lidar_driver',
                            LaunchConfiguration('lidar_driver').perform(context))
    imu = profile_default(profile, 'imu_driver',
                          LaunchConfiguration('imu_driver').perform(context))
    secrets = os.path.join(CONFIG_DIR, 'secrets.yaml')
    ntrip_params = param_files(profile, 'gnss.yaml') + (
        [secrets] if os.path.exists(secrets) else [])

    def flag(name):
        return IfCondition(LaunchConfiguration(name))

    def node(package, **kw):
        # Missing drivers are reported instead of aborting the whole bringup.
        if _has(package):
            return Node(package=package, **kw)
        return LogInfo(msg=f'[tank01] package {package} not installed - skipped')

    actions = [LogInfo(msg=f'[tank01] sensors: profile="{profile}" gnss={gnss} '
                           f'lidar={lidar} imu={imu}')]

    # ---------------------------------------------------------------- GNSS
    if gnss == 'ublox':
        actions.append(node(
            'ublox_gps', executable='ublox_gps_node', name='ublox_gps_node',
            condition=flag('use_gnss'), output='screen',
            parameters=param_files(profile, 'gnss.yaml'),
            remappings=[('fix', '/gnss/fix'), ('fix_velocity', '/gnss/vel'),
                        ('navheading', '/gnss/heading')]))
    elif gnss == 'um982':
        actions.append(node(
            'tank01_gnss', executable='um982_node', name='um982_node',
            condition=flag('use_gnss'), output='screen',
            parameters=param_files(profile, 'um982.yaml'),
            remappings=[('gnss/fix', '/gnss/fix'), ('gnss/heading', '/gnss/heading'),
                        ('rtcm', '/rtcm')]))
    actions.append(node(
        'ntrip_client', executable='ntrip_ros.py', name='ntrip_client',
        condition=flag('use_ntrip'), parameters=ntrip_params,
        remappings=[('fix', '/gnss/fix'), ('rtcm', '/rtcm')]))

    # ---------------------------------------------------------------- LiDAR
    if lidar == 'ouster':
        if _has('ouster_ros'):
            actions.append(GroupAction(condition=flag('use_lidar'), actions=[
                SetRemap('/ouster/points', '/points'),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(os.path.join(
                        get_package_share_directory('ouster_ros'), 'launch',
                        'driver.launch.py')),
                    launch_arguments={'sensor_hostname': LaunchConfiguration('lidar_hostname'),
                                      'viz': 'false'}.items()),
            ]))
        else:
            actions.append(LogInfo(msg='[tank01] ouster_ros not installed'))
        sensor_height = 1.15
    elif lidar == 'unitree':
        actions.append(node(
            'unitree_lidar_ros2', executable='unitree_lidar_ros2_node',
            name='unitree_lidar_ros2_node', condition=flag('use_lidar'), output='screen',
            parameters=param_files(profile, 'unitree_l2.yaml'),
            remappings=[('unilidar/cloud', '/points'), ('unilidar/imu', '/lidar/imu_raw')]))
        sensor_height = 1.08
    else:
        raise ValueError(f'unknown lidar_driver {lidar!r}')

    # Ground segmentation → obstacle points for the costmaps
    actions.append(node(
        'patchworkpp', executable='patchworkpp_node', name='ground_segmentation',
        condition=flag('use_lidar'),
        parameters=[{'sensor_height': sensor_height, 'base_frame': 'base_link'}],
        remappings=[('pointcloud_topic', '/points'),
                    ('/patchworkpp/nonground', '/points_obstacles')]))
    # LiDAR odometry
    actions.append(node(
        'kiss_icp', executable='kiss_icp_node', name='kiss_icp_node',
        condition=flag('use_lidar'), parameters=param_files(profile, 'kiss_icp.yaml'),
        remappings=[('pointcloud_topic', '/points'),
                    ('/kiss/odometry', '/kiss_icp/odometry')]))

    # ---------------------------------------------------------------- IMU
    if imu == 'xsens':
        actions.append(node(
            'xsens_mti_driver', executable='xsens_mti_node', name='xsens_driver',
            condition=flag('use_imu'), parameters=param_files(profile, 'imu.yaml'),
            remappings=[('imu/data', '/imu/data')]))
    elif imu == 'lidar':
        # 6-axis IMU inside the Unitree L2 → Madgwick filter gives roll/pitch orientation.
        actions.append(node(
            'imu_filter_madgwick', executable='imu_filter_madgwick_node',
            name='imu_filter_madgwick', condition=flag('use_imu'),
            parameters=param_files(profile, 'unitree_l2.yaml'),
            remappings=[('imu/data_raw', '/lidar/imu_raw'), ('imu/data', '/imu/data')]))
    else:
        raise ValueError(f'unknown imu_driver {imu!r}')

    # ---------------------------------------------------------------- camera (full design)
    if profile == '' and _has('zed_wrapper'):
        actions.append(GroupAction(condition=flag('use_camera'), actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(
                    get_package_share_directory('zed_wrapper'), 'launch',
                    'zed_camera.launch.py')),
                launch_arguments={'camera_model': LaunchConfiguration('camera_model'),
                                  'camera_name': 'camera',
                                  'publish_tf': 'false'}.items()),
        ]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('profile', default_value=''),
        DeclareLaunchArgument('gnss_driver', default_value='auto'),
        DeclareLaunchArgument('lidar_driver', default_value='auto'),
        DeclareLaunchArgument('imu_driver', default_value='auto'),
        DeclareLaunchArgument('use_gnss', default_value='true'),
        DeclareLaunchArgument('use_ntrip', default_value='true'),
        DeclareLaunchArgument('use_imu', default_value='true'),
        DeclareLaunchArgument('use_lidar', default_value='true'),
        DeclareLaunchArgument('use_camera', default_value='true'),
        DeclareLaunchArgument('lidar_hostname', default_value='192.168.10.20'),
        DeclareLaunchArgument('camera_model', default_value='zedx'),
        OpaqueFunction(function=launch_setup),
    ])
