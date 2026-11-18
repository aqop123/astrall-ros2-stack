cmake_minimum_required(VERSION 3.16)
project(dual_lidar_fusion)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(dual_lidar_fusion_node src/dual_lidar_fusion_node.cpp)
target_compile_options(dual_lidar_fusion_node PRIVATE -Wall)
ament_target_dependencies(dual_lidar_fusion_node rclcpp sensor_msgs message_filters)
target_link_libraries(dual_lidar_fusion_node Eigen3::Eigen)

install(TARGETS dual_lidar_fusion_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_package()
