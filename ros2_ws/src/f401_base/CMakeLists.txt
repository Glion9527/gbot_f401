cmake_minimum_required(VERSION 3.20)
project(f401_base)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Threads REQUIRED)

# 仓库根目录：ros2_ws/src/f401_base/../../..
set(F401_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../../..)

# 直接复用 host/ 的真实驱动，不复制一份协议/串口代码。
add_executable(f401_base_node
  src/f401_base_node.cpp
  ${F401_ROOT}/host/src/drivers/serial_port.cpp
  ${F401_ROOT}/host/src/drivers/link_v2.cpp
  ${F401_ROOT}/host/src/drivers/serial_base.cpp
)

target_include_directories(f401_base_node PRIVATE
  ${F401_ROOT}/host/include
  ${F401_ROOT}/protocol
)

ament_target_dependencies(f401_base_node
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  std_srvs
  tf2_ros
)
target_link_libraries(f401_base_node Threads::Threads)

install(TARGETS f401_base_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
