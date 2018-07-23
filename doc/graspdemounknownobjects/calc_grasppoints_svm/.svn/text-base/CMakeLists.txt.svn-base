cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#FIND_PACKAGE( V4R REQUIRED )
#link_directories(${V4R_DIR}/lib})

#include_directories("/opt/ros/graspdemounknownobjects/learn_grasppoints/include/")
#link_directories("/opt/ros/graspdemounknownobjects/learn_grasppoints/lib/")

#common commands for building c++ executables and libraries
rosbuild_add_library(CHaarFeature src/CHaarFeature.cpp)
rosbuild_add_library(CIntImage_to_Featurevec src/CIntImage_to_Featurevec.cpp)
rosbuild_add_executable(calc_grasppoints src/calc_grasppoints.cpp)
target_link_libraries(CIntImage_to_Featurevec CHaarFeature)
target_link_libraries(calc_grasppoints CIntImage_to_Featurevec)

