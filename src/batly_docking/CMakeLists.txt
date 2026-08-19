cmake_minimum_required(VERSION 3.8)
project(batly_docking)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ── Find dependencies ─────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# ── Build executable ──────────────────────────────
add_executable(detected_dock_pose_publisher
  src/detected_dock_pose_publisher.cpp
)

# Link libraries (if any)
ament_target_dependencies(detected_dock_pose_publisher
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# Install executable
install(TARGETS detected_dock_pose_publisher
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# ── Testing / linting ────────────────────────────
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
