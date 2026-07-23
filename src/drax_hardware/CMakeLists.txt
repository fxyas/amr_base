cmake_minimum_required(VERSION 3.8)
project(drax_hardware)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find ROS2 packages that we use
find_package(ament_cmake REQUIRED)

find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(diagnostic_updater REQUIRED)

# Build the hardware plugin
add_library(
  ${PROJECT_NAME}
  SHARED
  src/drax_hardware.cpp
  src/socket_can.cpp
  src/canopen_node.cpp
  src/motor.cpp

)

# Where header files are located
target_include_directories(
  ${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Link against ROS2 libraries
ament_target_dependencies(
  ${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  diagnostic_updater
)

# Export the plugin description
pluginlib_export_plugin_description_file(
  hardware_interface
  plugin.xml
)

# Install library
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)
# Install header files
install(
  DIRECTORY include/
  DESTINATION include
)

# Install plugin.xml
install(
  FILES plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(
    hardware_interface
    pluginlib
    rclcpp
    rclcpp_lifecycle
    diagnostic_updater
)

ament_export_include_directories(include)

ament_export_libraries(${PROJECT_NAME})

ament_package()
