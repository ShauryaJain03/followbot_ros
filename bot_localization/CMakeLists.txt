cmake_minimum_required(VERSION 3.8)
project(bot_localization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(include)
ament_python_install_package(${PROJECT_NAME})

install(
  DIRECTORY include
  DESTINATION include
)


install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  ${PROJECT_NAME}/imu_republisher.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
