from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Launch arguments
    map_dir = LaunchConfiguration('map_dir')
    calibration_dir = LaunchConfiguration('calibration_dir')
    reflectivity_dir = LaunchConfiguration('reflectivity_dir')
    point_cloud_dir = LaunchConfiguration('point_cloud_dir')
    model = LaunchConfiguration('model')
    rvizconfig = LaunchConfiguration('rvizconfig')

    # Paths to packages
    ford_demo_share = get_package_share_directory('ford_demo')
    fusion_description_share = get_package_share_directory('fusion_description')
    map_loader_share = get_package_share_directory('map_loader')

    # Launch arguments with defaults
    declare_args = [
        DeclareLaunchArgument('map_dir', default_value=''),
        DeclareLaunchArgument('calibration_dir', default_value=''),
        DeclareLaunchArgument('reflectivity_dir', default_value=os.path.join('', 'ground_reflectivity')),
        DeclareLaunchArgument('point_cloud_dir', default_value=os.path.join('', '3d_point_cloud')),
        DeclareLaunchArgument('model', default_value=os.path.join(fusion_description_share, 'urdf', 'fusion.urdf')),
        DeclareLaunchArgument('rvizconfig', default_value=os.path.join(ford_demo_share, 'rviz', 'demo.rviz'))
    ]

    # Reflectivity map loader
    reflectivity_map_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(map_loader_share, 'launch', 'reflectivity_map_demo.launch.py')
        ),
        launch_arguments={'map_folder': reflectivity_dir}.items()
    )

    # 3D point cloud map loader
    point_cloud_map_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(map_loader_share, 'launch', 'point_cloud_map_demo.launch.py')
        ),
        launch_arguments={'map_folder': point_cloud_dir}.items()
    )

    # Extrinsics broadcaster (shell script execution)
    extrinsics_broadcaster = ExecuteProcess(
        cmd=[
            os.path.join(ford_demo_share, 'scripts', 'extrinsics_broadcast.sh'),
            'publish',
            calibration_dir
        ],
        output='screen'
    )

    # RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', rvizconfig],
        output='screen'
    )

    return LaunchDescription(
        declare_args + [
            reflectivity_map_launch,
            point_cloud_map_launch,
            extrinsics_broadcaster,
            rviz_node
        ]
    )
