from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():
    # Declare arguments
    calibration = LaunchConfiguration('calibration')
    manager = LaunchConfiguration('manager')
    max_range = LaunchConfiguration('max_range')
    min_range = LaunchConfiguration('min_range')
    model = LaunchConfiguration('model')

    declare_args = [
        DeclareLaunchArgument(
            'calibration',
            default_value=os.path.join(
                get_package_share_directory('ford_demo'),
                'params', 'lidarIntrinsics.yaml'
            ),
            description='Calibration file for Velodyne sensors'
        ),
        DeclareLaunchArgument('manager', default_value='velodyne_nodelet_manager'),
        DeclareLaunchArgument('max_range', default_value='130.0'),
        DeclareLaunchArgument('min_range', default_value='3.0'),
        DeclareLaunchArgument('model', default_value='32E')
    ]

    # In ROS 2, Nodelets are converted to regular nodes (usually within velodyne_pointcloud)
    # We'll directly call the 'velodyne_transform_node' executable for each sensor

    lidar_nodes = []
    lidar_config = [
        ('red', 'lidar_red_scan', 'lidar_red_pointcloud'),
        ('yellow', 'lidar_yellow_scan', 'lidar_yellow_pointcloud'),
        ('blue', 'lidar_blue_scan', 'lidar_blue_pointcloud'),
        ('green', 'lidar_green_scan', 'lidar_green_pointcloud'),
    ]

    for color, scan_topic, pointcloud_topic in lidar_config:
        lidar_nodes.append(
            Node(
                package='velodyne_pointcloud',
                executable='velodyne_transform_node',  # Nodelet replaced with executable
                name=f'velodyne_{color}_convert',
                parameters=[{
                    'calibration': calibration,
                    'max_range': max_range,
                    'min_range': min_range,
                    'fixed_frame': f'lidar_{color}',
                    'target_frame': f'lidar_{color}',
                    'model': model
                }],
                remappings=[
                    ('velodyne_packets', scan_topic),
                    ('velodyne_points', pointcloud_topic)
                ],
                output='screen'
            )
        )

    return LaunchDescription(declare_args + lidar_nodes)
