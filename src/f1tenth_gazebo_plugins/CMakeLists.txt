cmake_minimum_required(VERSION 3.8)
project(f1tenth_gazebo_plugins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)

link_directories(${gazebo_dev_LIBRARY_DIRS})

# f1tenth_kinematic_bicycle_model
add_library(f1tenth_kinematic_bicycle_model SHARED
  src/f1tenth_kinematic_bicycle_model.cpp
)

target_include_directories(f1tenth_kinematic_bicycle_model PUBLIC include)

ament_target_dependencies(f1tenth_kinematic_bicycle_model
  "gazebo_dev"
  "gazebo_ros"
)

ament_export_libraries(f1tenth_kinematic_bicycle_model)

if(ENABLE_PROFILER)
  target_include_directories(f1tenth_kinematic_bicycle_model PUBLIC ${ignition-common3_INCLUDE_DIRS})
  target_link_libraries(f1tenth_kinematic_bicycle_model ${ignition-common3_LIBRARIES})
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/${PROJECT_NAME}.sh.in")


ament_package()
