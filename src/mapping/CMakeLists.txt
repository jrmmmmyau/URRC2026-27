cmake_minimum_required(VERSION 3.8)
project(mapping)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(mapping_node
  src/mapping.cpp
)

target_include_directories(mapping_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(mapping_node
  rclcpp
  sensor_msgs
  nav_msgs
  cv_bridge
)

target_link_libraries(mapping_node
  ${OpenCV_LIBRARIES}
)

install(TARGETS mapping_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()