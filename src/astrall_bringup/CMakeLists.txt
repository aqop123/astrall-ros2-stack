cmake_minimum_required(VERSION 3.16)
project(astrall_bringup)

find_package(ament_cmake REQUIRED)

# 配置与 RViz 布局随包一起安装到 share/astrall_bringup/
install(DIRECTORY config launch rviz DESTINATION share/${PROJECT_NAME})

ament_package()
