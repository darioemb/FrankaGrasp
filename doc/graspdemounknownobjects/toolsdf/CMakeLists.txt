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

#include_directories("/opt/ros/privat/stacks/david/learn_grasppoints/include/")
#link_directories("/opt/ros/privat/stacks/david/learn_grasppoints/lib/")

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#rosbuild_add_executable(pc_merge src/pc_merge.cpp)
rosbuild_add_executable(pc_merge_1cam_nobox src/pc_merge_1cam_nobox.cpp)
#rosbuild_add_executable(pc_merge_with_basket src/pc_merge_with_basket.cpp)
#rosbuild_add_executable(pc_merge_with_basket_rot src/pc_merge_with_basket_rot.cpp)
#rosbuild_add_executable(pc_merge_with_basket_rot_2cams src/pc_merge_with_basket_rot_2cams.cpp)
#rosbuild_add_executable(pc_mirror src/pc_mirror.cpp)
#rosbuild_add_executable(pcd_to_heightsgrid src/pcd_to_heightsgrid.cpp)
#rosbuild_add_executable(pc_cut src/pc_cut.cpp)
#rosbuild_add_executable(pc_inverse_z src/pc_inverse_z.cpp)
#rosbuild_add_executable(edit_featurefile src/edit_featurefile.cpp)
#rosbuild_add_executable(feature_creator src/feature_creator.cpp)
#rosbuild_add_executable(preprocessing src/preprocessing.cpp)
#rosbuild_add_executable(viewgrids src/viewgrids.cpp)
#rosbuild_add_executable(heightsgrid_to_intimage src/heightsgrid_to_intimage.cpp)
#rosbuild_add_executable(intimage_to_featurevec src/intimage_to_featurevec.cpp)
#rosbuild_add_executable(cp_from_many_files_to_one src/cp_from_many_files_to_one.cpp)
#rosbuild_add_executable(svmoutput_to_matrix src/svmoutput_to_matrix.cpp)
#target_link_libraries(pcd_to_heightsgrid v4rtomgine)
#target_link_libraries(heightsgrid_to_intimage v4rtomgine)
#target_link_libraries(viewgrids v4rtomgine)
#target_link_libraries(intimage_to_featurevec CHaarFeature)
#target_link_libraries(example ${PROJECT_NAME})
