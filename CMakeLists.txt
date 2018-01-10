cmake_minimum_required(VERSION 2.8.3)
project(orb_slam)

set( CMAKE_EXPORT_COMPILE_COMMANDS 1 )
set(ROS_BUILD_TYPE Release)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3 -march=native")

# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  pcl_conversions
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

find_package(OpenCV 2.4.3 REQUIRED)
find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(CUDA  REQUIRED)

set(
    CUDA_NVCC_FLAGS
    ${CUDA_NVCC_FLAGS};
    -O3
    #-gencode arch=compute_20,code=[sm_20,sm_21] 666 CHECK THIS DEPEND ON THE PLATFORM
    -gencode arch=compute_30,code=sm_30
    -gencode arch=compute_35,code=sm_35
    -gencode arch=compute_50,code=[sm_50,compute_50]
    -gencode arch=compute_52,code=[sm_52,compute_52]
    -gencode arch=compute_61,code=sm_61
    -gencode arch=compute_62,code=sm_62
 
)
add_definitions(-DGPU)
add_definitions(-DOPENCV)
find_package(Boost REQUIRED COMPONENTS thread)

set(DARKNET_PATH "/home/lucky/MyLib/darknet")
add_definitions(-DDARKNET_FILE_PATH="${DARKNET_PATH}")

SET(CUDA_PROPAGATE_HOST_FLAGS OFF)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES orb_slam
#  CATKIN_DEPENDS cv_bridge pcl_conversions pcl_ros roscpp rospy sensor_msgs std_msgs tf
#  DEPENDS system_lib
)

###########
## Build ##
###########
SET(MyLib /home/lucky/MyLib)
## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include/orb_slam
  ${MyLib}
  ${DARKNET_PATH}/src
  ${DARKNET_PATH}/include 
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${Pangolin_INCLUDE_DIRS}
  ${CUDA_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}
)

## Declare a cpp library
# add_library(orb_slam
#   src/${PROJECT_NAME}/orb_slam.cpp
# )

message("/////////////////////////////////")
message(${CUDA_LIBRARIES})
message(${CUDA_INCLUDE_DIRS})


link_directories(
	${MyLib}
	/usr/local/cuda-8.0/lib64	
)

 cuda_add_library(darknet_c
    ${DARKNET_PATH}/src/activation_layer.c        ${DARKNET_PATH}/src/im2col.c
    ${DARKNET_PATH}/src/activations.c             ${DARKNET_PATH}/src/image.c
    ${DARKNET_PATH}/src/avgpool_layer.c           ${DARKNET_PATH}/src/layer.c
    ${DARKNET_PATH}/src/batchnorm_layer.c         ${DARKNET_PATH}/src/list.c
    ${DARKNET_PATH}/src/blas.c                    ${DARKNET_PATH}/src/local_layer.c
    ${DARKNET_PATH}/src/box.c                     ${DARKNET_PATH}/src/lstm_layer.c
    ${DARKNET_PATH}/src/col2im.c                  ${DARKNET_PATH}/src/matrix.c
    ${DARKNET_PATH}/src/maxpool_layer.c
    ${DARKNET_PATH}/src/connected_layer.c         ${DARKNET_PATH}/src/network.c
    ${DARKNET_PATH}/src/convolutional_layer.c     ${DARKNET_PATH}/src/normalization_layer.c
    ${DARKNET_PATH}/src/cost_layer.c              ${DARKNET_PATH}/src/option_list.c
    ${DARKNET_PATH}/src/crnn_layer.c              ${DARKNET_PATH}/src/parser.c
    ${DARKNET_PATH}/src/crop_layer.c              ${DARKNET_PATH}/src/region_layer.c
    ${DARKNET_PATH}/src/cuda.c                    ${DARKNET_PATH}/src/reorg_layer.c
    ${DARKNET_PATH}/src/data.c                    ${DARKNET_PATH}/src/rnn_layer.c
    ${DARKNET_PATH}/src/deconvolutional_layer.c   ${DARKNET_PATH}/src/route_layer.c
    ${DARKNET_PATH}/src/demo.c                    ${DARKNET_PATH}/src/shortcut_layer.c
    ${DARKNET_PATH}/src/detection_layer.c         ${DARKNET_PATH}/src/softmax_layer.c
    ${DARKNET_PATH}/src/dropout_layer.c           ${DARKNET_PATH}/src/tree.c
    ${DARKNET_PATH}/src/gemm.c                    ${DARKNET_PATH}/src/utils.c
    ${DARKNET_PATH}/src/gru_layer.c

    ${DARKNET_PATH}/src/activation_kernels.cu     ${DARKNET_PATH}/src/deconvolutional_kernels.cu
    ${DARKNET_PATH}/src/avgpool_layer_kernels.cu  ${DARKNET_PATH}/src/dropout_layer_kernels.cu
    ${DARKNET_PATH}/src/blas_kernels.cu           ${DARKNET_PATH}/src/im2col_kernels.cu
    ${DARKNET_PATH}/src/col2im_kernels.cu         ${DARKNET_PATH}/src/maxpool_layer_kernels.cu
    ${DARKNET_PATH}/src/convolutional_kernels.cu  
    ${DARKNET_PATH}/src/crop_layer_kernels.cu
  )

 target_link_libraries(darknet_c
    m
    pthread
    stdc++
    cuda
    cudart
    cublas
    curand
    ${Boost_LIBRARIES}
  )


set(LIBS
${OpenCV_LIBS}
${EIGEN3_LIBS}
${catkin_LIBRARIES}
${PCL_LIBRARIES}
${Pangolin_LIBRARIES}
${CUDA_LIBRARIES}
${MyLib}/darknet/libdarknet.so
${MyLib}/DBoW2/lib/libDBoW2.so
${MyLib}/g2o/lib/libg2o.so
darknet_c
)





cuda_add_library(${PROJECT_NAME} SHARED
src/System.cc
src/Tracking.cc
src/LocalMapping.cc
src/LoopClosing.cc
src/ORBextractor.cc
src/ORBmatcher.cc
src/FrameDrawer.cc
src/Converter.cc
src/MapPoint.cc
src/KeyFrame.cc
src/Map.cc
src/MapDrawer.cc
src/Optimizer.cc
src/PnPsolver.cc
src/Frame.cc
src/KeyFrameDatabase.cc
src/Sim3Solver.cc
src/Initializer.cc
src/Viewer.cc
src/Send.cc
src/AllConfig.cc
src/Objects.cc
src/Object.cc
src/Yolo.cc
src/KeyFrameDrawer.cc
src/PointCloudDrawer.cc
)



target_link_libraries(${PROJECT_NAME}
${LIBS}
)
## Declare a cpp executable
# add_executable(orb_slam_node src/orb_slam_node.cpp)

add_executable(rgbd_tum src/DATASET/rgbd_tum.cc)
target_link_libraries(rgbd_tum ${PROJECT_NAME} ${LIBS})

add_executable(Mono src/ROS/ros_mono.cc)
target_link_libraries(Mono ${PROJECT_NAME} ${LIBS})

add_executable(Stereo src/ROS/ros_stereo.cc)
target_link_libraries(Stereo ${PROJECT_NAME} ${LIBS})

add_executable(RGBD src/ROS/ros_rgbd.cc)
target_link_libraries(RGBD ${PROJECT_NAME} ${LIBS})

