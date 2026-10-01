cmake_minimum_required(VERSION 3.8)
project(navigation_server)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/PlanPath.srv"
  DEPENDENCIES geometry_msgs nav_msgs
)
ament_export_dependencies(rosidl_default_runtime)
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(navsrv src/navigation_server.cpp)
ament_target_dependencies(navsrv rclcpp geometry_msgs nav_msgs)
target_link_libraries(navsrv "${cpp_typesupport_target}")

add_executable(map_publisher src/map_publisher.cpp)
ament_target_dependencies(map_publisher rclcpp nav_msgs)

add_executable(plan_client src/plan_client.cpp)
ament_target_dependencies(plan_client rclcpp geometry_msgs nav_msgs)
target_link_libraries(plan_client "${cpp_typesupport_target}")

install(TARGETS
  navsrv
  map_publisher
  plan_client
  DESTINATION lib/${PROJECT_NAME})

ament_package()
