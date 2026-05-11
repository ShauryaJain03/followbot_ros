cmake_minimum_required(VERSION 3.8)
project(traversability_gridmap)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)


# find dependencies
find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3)
find_package(grid_map_cmake_helpers REQUIRED)
find_package(grid_map_core REQUIRED)
find_package(grid_map_cv REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(grid_map_ros REQUIRED)

grid_map_package()

set(dependencies
  grid_map_msgs
  grid_map_ros
  rclcpp
  sensor_msgs
  std_msgs
  Eigen3
  visualization_msgs
)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  SYSTEM
    ${EIGEN3_INCLUDE_DIR}
)


## Declare a cpp executable
add_executable(
  traversability_node
  src/traversability_node.cpp src/traversabilityGrid.cpp
)

## Specify libraries to link a library or executable target against
ament_target_dependencies(
  traversability_node SYSTEM
  ${dependencies}
)


install(TARGETS traversability_node
  DESTINATION launch
  DESTINATION include
  DESTINATION lib/${PROJECT_NAME}
)

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

ament_export_include_directories(include ${EIGEN3_INCLUDE_DIR})
ament_auto_package(INSTALL_TO_SHARE launch)
