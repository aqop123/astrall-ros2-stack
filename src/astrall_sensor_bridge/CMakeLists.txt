cmake_minimum_required(VERSION 3.16)
project(astrall_sensor_bridge)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(FFMPEG REQUIRED IMPORTED_TARGET libavcodec libavutil libswscale)

if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(x86_64|amd64)$")
  set(SDK_ARCH x86-64)
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "^(aarch64|arm64)$")
  set(SDK_ARCH arm64)
else()
  message(FATAL_ERROR "Unsupported CPU architecture: ${CMAKE_SYSTEM_PROCESSOR}")
endif()

# SDK 随工作空间一起入库：优先使用 <workspace>/sdk，兼容旧的 <SDK根目录>/astrall_ws 布局
set(SDK_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../sdk")
if(NOT EXISTS "${SDK_ROOT}/include/interface.h")
  set(SDK_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../../sdk")
endif()
set(SDK_LIBRARY "${SDK_ROOT}/${SDK_ARCH}/libASTRALL_SDK.so")
if(NOT EXISTS "${SDK_LIBRARY}")
  message(FATAL_ERROR "Astrall SDK library not found: ${SDK_LIBRARY}")
endif()

add_executable(astrall_sensor_node src/astrall_sensor_node.cpp)
target_include_directories(astrall_sensor_node PRIVATE "${SDK_ROOT}/include")
ament_target_dependencies(astrall_sensor_node rclcpp sensor_msgs nav_msgs geometry_msgs)
target_link_libraries(astrall_sensor_node "${SDK_LIBRARY}" PkgConfig::FFMPEG)
set_target_properties(astrall_sensor_node PROPERTIES
  BUILD_RPATH "${SDK_ROOT}/${SDK_ARCH}"
  INSTALL_RPATH "${SDK_ROOT}/${SDK_ARCH}")

install(TARGETS astrall_sensor_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_package()
