cmake_minimum_required(VERSION 3.10)
project(formula_drive)

# Include OpenCV
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# Include Pylon
find_package(pylon 7.4.0 REQUIRED)

# Include Eigen
include_directories(../libraries/Eigen/)

# Include CUDA
include_directories(/usr/local/cuda-12.2/targets/aarch64-linux/include/)
set(CUDA_TOOLKIT_ROOT_DIR /usr/local/cuda-12.2)
find_package(CUDA REQUIRED)

# Include TensorRT
find_library(LIBNVINFER NAME nvinfer REQUIRED)
find_library(LIBNVINVER_PLUGIN NAME nvinfer_plugin REQUIRED)
find_library(LIBNVPARSERS NAME nvparsers REQUIRED)

# Include src files
include_directories(include)
add_executable(main src/main.cpp)

target_link_libraries(main
					pylon::pylon
					${OpenCV_LIBS}
					opencv_core opencv_cudawarping
					${CUDA_LIBRARIES}
					${LIBNVINFER} ${LIBNVINFER_PLUGIN} ${LIBNVPARSERS})
