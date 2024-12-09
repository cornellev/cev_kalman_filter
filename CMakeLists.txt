# Set the project name for your library
project(cev_kalman_filter)

find_package(Eigen3 REQUIRED)

# Add the include directory
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

# Compile the library (static or shared)
add_library(cev_kalman_filter STATIC
    src/estimator.cpp
    src/model.cpp
    src/sensor.cpp
    src/standard_models.cpp
)

target_link_libraries(cev_kalman_filter Eigen3::Eigen)
