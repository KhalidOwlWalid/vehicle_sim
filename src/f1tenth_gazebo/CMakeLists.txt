
cmake_minimum_required(VERSION 3.5)

project(f1tenth_gazebo)

# Skip if Gazebo not present
find_package(gazebo QUIET)
if(NOT gazebo_FOUND)
  message(WARNING "Gazebo not found, proceeding without that simulator.")
  return()
endif()

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  DIRECTORY
    launch
    models
    worlds
  DESTINATION
    share/${PROJECT_NAME}/
)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/${PROJECT_NAME}.sh.in")

ament_package()