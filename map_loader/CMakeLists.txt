cmake_minimum_required(VERSION 3.5)
project(map_loader)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS io)
find_package(pcl_conversions REQUIRED)
find_package(Eigen3 REQUIRED)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "-O2 -Wall ${CMAKE_CXX_FLAGS}")

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(reflectivity_map_loader src/reflectivity_map_loader.cpp)
ament_target_dependencies(reflectivity_map_loader rclcpp std_msgs sensor_msgs pcl_conversions)

add_executable(point_cloud_map_loader src/point_cloud_map_loader.cpp)
ament_target_dependencies(point_cloud_map_loader rclcpp std_msgs sensor_msgs pcl_conversions)

target_link_libraries(reflectivity_map_loader ${PCL_LIBRARIES})
target_link_libraries(point_cloud_map_loader ${PCL_LIBRARIES})

install(TARGETS
  reflectivity_map_loader
  point_cloud_map_loader
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
