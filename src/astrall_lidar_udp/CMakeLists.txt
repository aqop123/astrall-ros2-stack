cmake_minimum_required(VERSION 3.16)
project(astrall_lidar_udp)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/UdpPacket.msg"
  DEPENDENCIES std_msgs)

add_executable(lidar_udp_node src/lidar_udp_node.cpp)
ament_target_dependencies(lidar_udp_node rclcpp std_msgs)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(lidar_udp_node "${cpp_typesupport_target}")

install(TARGETS lidar_udp_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(rosidl_default_runtime)
ament_package()
