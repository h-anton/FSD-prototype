cmake_minimum_required(VERSION 3.10)
project(calibration)

# Include OpenCV
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# Include Pylon
find_package(pylon 7.4.0 REQUIRED)

# Include Eigen
include_directories(../libraries/Eigen/)

# Include src files
include_directories(include)
add_executable(main src/main.cpp)

target_link_libraries(main pylon::pylon ${OpenCV_LIBS})
