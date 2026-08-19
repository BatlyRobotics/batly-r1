cmake_minimum_required(VERSION 3.5)
project(batly_params)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config rviz maps
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
