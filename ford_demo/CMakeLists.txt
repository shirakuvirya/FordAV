cmake_minimum_required(VERSION 3.5)
project(ford_demo)

find_package(ament_cmake REQUIRED)

# If you have C++ targets, uncomment and adjust:
# find_package(rclcpp REQUIRED)
# find_package(sensor_msgs REQUIRED)

# Add include directories (if needed)
# include_directories(include ${SDFormat_INCLUDE_DIRS})

install(DIRECTORY
  launch
  rviz
  params
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/lidar_merger.py
  scripts/lidar_merger_v2.py
  scripts/extrinsics_broadcast.sh 
  scripts/extrinsics_broadcaster.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
