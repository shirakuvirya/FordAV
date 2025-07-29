#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 1) Declare arguments
    calibration = LaunchConfiguration('calibration')
    max_range   = LaunchConfiguration('max_range')
    min_range   = LaunchConfiguration('min_range')
    model       = LaunchConfiguration('model')

    ford_demo_share = get_package_share_directory('ford_demo')
    fusion_share    = get_package_share_directory('fusion_description')

    declare_args = [
        DeclareLaunchArgument(
            'calibration',
            default_value=os.path.join(
                ford_demo_share, 'params', 'lidarIntrinsics.yaml'
            ),
            description='Calibration YAML for Velodyne sensors'
        ),
        DeclareLaunchArgument('max_range', default_value='130.0'),
        DeclareLaunchArgument('min_range', default_value='3.0'),
        DeclareLaunchArgument('model', default_value='32E'),
    ]

    # ────────────────────────────────────────────────────────────────
    # 2) URDF loader: read fusion.urdf and publish TF tree
    urdf_path = os.path.join(fusion_share, 'urdf', 'fusion.urdf')
    with open(urdf_path, 'r') as inf:
        robot_description_xml = inf.read()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_xml,
            'use_sim_time': True,  # Use simulation time if available
        }]
    )
    # ────────────────────────────────────────────────────────────────

    # 3) Velodyne transform nodes
    lidar_config = [
        ('red',    'lidar_red_scan',    'lidar_red_pointcloud'),
        ('yellow', 'lidar_yellow_scan', 'lidar_yellow_pointcloud'),
        ('blue',   'lidar_blue_scan',   'lidar_blue_pointcloud'),
        ('green',  'lidar_green_scan',  'lidar_green_pointcloud'),
    ]

    lidar_nodes = []
    for color, scan_topic, pc_topic in lidar_config:
        lidar_nodes.append(
            Node(
                package='velodyne_pointcloud',
                executable='velodyne_transform_node',
                name=f'velodyne_{color}_convert',
                output='screen',
                parameters=[{
                    'calibration':   calibration,
                    'max_range':     max_range,
                    'min_range':     min_range,
                    'fixed_frame':   f'lidar_{color}',
                    'target_frame':  'body',
                    'model':         model,
                    # tell the node to use ROS‑2’s sensor QoS (best_effort, small queue)
                    #'use_sensor_qos': True,
                }],
                remappings=[
                    ('velodyne_packets', scan_topic),
                    ('velodyne_points',  pc_topic),
                ],
            )
        )

    return LaunchDescription(
        declare_args
        + [robot_state_publisher]
        + lidar_nodes
    )
