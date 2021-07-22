cmake_minimum_required(VERSION 3.5)
project(rmf_massrobotics_bridge)
set(CMAKE_EXPORT_COMPILE_COMMANDS on)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include(GNUInstallDirs)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rmf_traffic REQUIRED)
find_package(rmf_traffic_ros2 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmf_visualization_schedule REQUIRED)

#===============================================================================
add_executable(websocket_bridge
  src/websocket_bridge/main.cpp
)

ament_target_dependencies(websocket_bridge
  rclcpp
  rmf_visualization_schedule
)

target_link_libraries(websocket_bridge
  ${websocketpp_LIBRARIES}
  ${OpenSSL_LIBRARIES}
  rmf_traffic::rmf_traffic
  rmf_traffic_ros2::rmf_traffic_ros2
)

target_include_directories(websocket_bridge
  PUBLIC
    ${websocketpp_INCLUDE_DIR}
    ${OpenSSL_INCLUDE_DIR}
)

#===============================================================================
install(
  TARGETS websocket_bridge
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

ament_package()
