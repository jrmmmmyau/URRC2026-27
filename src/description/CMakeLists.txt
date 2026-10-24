cmake_minimum_required(VERSION 3.8)
project(description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)

# Install organized Gazebo worlds and their shared / per-variant assets.
install(
  DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../../map/worlds
  DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING
    PATTERN "*.sdf"
)

install(
  DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../../map/assets
  DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING
    PATTERN "*.png"
    PATTERN "*.obj"
)

ament_package()
