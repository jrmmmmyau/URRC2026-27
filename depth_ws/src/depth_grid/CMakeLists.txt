cmake_minimum_required(VERSION 3.8)
project(depth_grid)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(depth_grid_node src/depth_grid_node.cpp)

ament_target_dependencies(depth_grid_node
    rclcpp
    sensor_msgs
    nav_msgs
    cv_bridge
)

target_include_directories(depth_grid_node PRIVATE
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(depth_grid_node
    ${OpenCV_LIBRARIES}
)

install(TARGETS
    depth_grid_node
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()
