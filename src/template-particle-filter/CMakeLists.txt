cmake_minimum_required(VERSION 2.8.3)
project(particle-filter)

set(CMAKE_CUDA_COMPILER  /usr/local/cuda-11.0/bin/nvcc)

find_package(CUDA REQUIRED)
#Here you can set any ncvv compiler flags, if you so wish
#SET(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} -DMY_DEF=1")

set(CMAKE_BUILD_TYPE "Release")                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS                     
  geometry_msgs                                                           
  sensor_msgs
  roscpp
  rospy
  std_msgs
  message_generation
  tf
)

#find_package(Eigen3 REQUIRED)
#find_package(PCL REQUIRED)
#find_package(OpenCV REQUIRED)
#find_package(Ceres REQUIRED)
## Generate messages in the 'msg' folder
add_message_files(
    FILES
    RPYAxes.msg
#   Message1.msg
#   Message2.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  CATKIN_DEPENDS message_runtime
#  INCLUDE_DIRS include
#  LIBRARIES perception
#  CATKIN_DEPENDS rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
	${catkin_INCLUDE_DIRS} 
	# ${PCL_INCLUDE_DIRS}
        # ${OpenCV_INCLUDE_DIRS}
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs tf
  #DEPENDS EIGEN3 PCL 
  INCLUDE_DIRS include
)


add_executable(SphParticleFilter src/particle-filter.cpp)
target_link_libraries(SphParticleFilter ${catkin_LIBRARIES})

# CMake Indexing
FILE(GLOB_RECURSE LibFiles "include/*")
add_custom_target(headers SOURCES ${LibFiles})




