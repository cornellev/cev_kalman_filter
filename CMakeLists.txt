
cmake_minimum_required(VERSION 3.8)
project(ackermann_ekf)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES Clang)
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(Eigen3 REQUIRED)

# ROS
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_library(update_model
    src/update_model.cpp
    include/update_model.h
)
ament_target_dependencies(update_model Eigen3)

add_library(state
    src/state.cpp
    include/state.h
)
ament_target_dependencies(state Eigen3)

add_library(sensor
    src/sensor.cpp
    include/sensor.h
)

ament_target_dependencies(sensor Eigen3)

include_directories(update_model include)
include_directories(state include)
include_directories(sensor include)

add_executable(ackermann_ekf src/ackermann_ekf.cpp)
target_link_libraries(ackermann_ekf update_model state sensor)
ament_target_dependencies(ackermann_ekf rclcpp sensor_msgs std_msgs Eigen3)

install(TARGETS
  ackermann_ekf
  DESTINATION lib/${PROJECT_NAME}
)

install (DIRECTORY include/
  DESTINATION share/${PROJECT_NAME}/include
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
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

ament_package()
