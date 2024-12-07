
cmake_minimum_required(VERSION 3.8)
project(cev_localization)

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
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

find_package(cev_msgs REQUIRED)

add_library(estimator
    src/estimator.cpp
    include/estimator.h
)
ament_target_dependencies(estimator Eigen3)

add_library(model
    src/model.cpp
    include/model.h
)

add_library(sensor
    src/sensor.cpp
    include/sensor.h
)

add_library(ros_sensor
    src/ros_sensor.cpp
    include/ros_sensor.h
)

add_library(config_parser
    src/config_parser.cpp
    include/config_parser.h
)

# add_library(updateable
#     src/updateable.cpp
#     include/updateable.h
# )

# add_library(updater
#     src/updater.cpp
#     include/updater.h
# )

include_directories(estimator include)
include_directories(model include)
include_directories(sensor include)
include_directories(ros_sensor include)
include_directories(config_parser include)
# include_directories(updateable include)
# include_directories(updater include)

target_link_libraries(config_parser yaml-cpp)
ament_target_dependencies(config_parser yaml-cpp)

add_executable(ackermann_ekf src/ackermann_ekf.cpp)
# target_link_libraries(ackermann_ekf estimator model sensor ros_sensor updateable updater)
target_link_libraries(ackermann_ekf estimator model sensor ros_sensor yaml-cpp config_parser)
ament_target_dependencies(ackermann_ekf rclcpp sensor_msgs std_msgs Eigen3 cev_msgs tf2 tf2_ros tf2_geometry_msgs nav_msgs)

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
