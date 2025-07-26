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
            description='Path to the reflectivity map folder'
        )
    ]

    # Node: reflectivity_map_loader
    reflectivity_map_loader_node = Node(
        package='map_loader',
        executable='reflectivity_map_loader',
        name='reflectivity_map_loader',
        arguments=[map_folder],
        parameters=[{
            'pcd_topic': '/reflectivity_map',
            'pose_topic': '/pose_ground_truth',
            'neighbor_dist': 128.0,
            'publish_rate': 1.0
        }],
        output='screen'
    )

    return LaunchDescription(declare_args + [reflectivity_map_loader_node])
