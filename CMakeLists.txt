# Set the project name for your library
project(cev_kalman_filter)

# Add the include directory
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

# Compile the library (static or shared)
add_library(cev_kalman_filter STATIC
    src/estimator.cpp
    src/model.cpp
    src/sensor.cpp
    src/standard_models.cpp
)

# Specify the include directory so other projects can find the headers
target_include_directories(cev_kalman_filter PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
