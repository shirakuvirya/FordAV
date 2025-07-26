from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Launch argument
    map_folder = LaunchConfiguration('map_folder')

    declare_args = [
        DeclareLaunchArgument(
            'map_folder',
            default_value='',
            description='Path to the point cloud map folder'
        )
    ]

    # Node: point_cloud_map_loader
    point_cloud_map_loader_node = Node(
        package='map_loader',
        executable='point_cloud_map_loader',
        name='point_cloud_map_loader',
        arguments=[map_folder],
        parameters=[{
            'pcd_topic': '/pointcloud_map',
            'pose_topic': '/pose_ground_truth',
            'neighbor_dist': 128.0,
            'publish_rate': 0.25
        }],
        output='screen'
    )

    return LaunchDescription(declare_args + [point_cloud_map_loader_node])
