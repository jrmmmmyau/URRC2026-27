cmake_minimum_required(VERSION 3.16)
project(perception)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core)
find_package(apriltag_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TagObservation.msg"
  "msg/TagObservationArray.msg"
  DEPENDENCIES geometry_msgs std_msgs
)
ament_export_dependencies(rosidl_default_runtime)

add_executable(tag_observer src/tag_observer.cpp)
target_compile_features(tag_observer PRIVATE cxx_std_17)
ament_target_dependencies(tag_observer rclcpp apriltag_msgs geometry_msgs std_msgs tf2 tf2_ros)
rosidl_get_typesupport_target(tag_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(tag_observer "${tag_typesupport_target}")


if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(tag_observer PRIVATE -Wall -Wextra -Wpedantic)
endif()


install(TARGETS tag_observer DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  FILES README.md LICENSE
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
