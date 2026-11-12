cmake_minimum_required(VERSION 3.8)
project(gui)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(pluginlib REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Widgets Gui)

set(CMAKE_AUTOMOC ON)

add_library(gui SHARED
  src/main_panel.cpp
  include/gui/main_panel.hpp
)

target_include_directories(gui PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
)


ament_target_dependencies(gui
  rclcpp
  sensor_msgs
  cv_bridge
  rviz_common
  pluginlib
)

target_link_libraries(gui
  Qt5::Widgets
  ${OpenCV_LIBS}
)

pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

install(
  TARGETS gui
  EXPORT export_gui
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include 
)

ament_export_targets(export_gui HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp sensor_msgs cv_bridge rviz_common)

ament_package()



