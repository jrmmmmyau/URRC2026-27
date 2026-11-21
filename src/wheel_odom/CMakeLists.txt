cmake_minimum_required(VERSION 3.8)
project(wheel_localization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(wheel_localization_node src/wheel_localization_node.cpp)
target_compile_features(wheel_localization_node PUBLIC cxx_std_17)
ament_target_dependencies(wheel_localization_node
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs)

install(TARGETS wheel_localization_node
  DESTINATION lib/${PROJECT_NAME})

# uncomment once these folders exist
# install(DIRECTORY config launch
#   DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()