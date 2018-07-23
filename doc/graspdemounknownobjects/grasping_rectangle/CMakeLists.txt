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

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})
rosbuild_add_executable(extract_pc_features src/extract_point_cloud_training_data.cpp src/gpc.cpp src/common.cpp src/svm/svm_common.cpp src/thr_pool.cpp src/histogram.cpp src/feature.cpp)
rosbuild_add_compile_flags(extract_pc_features "-O3 -fomit-frame-pointer -ffast-math")

#rosbuild_add_executable(test_pc_features src/point_cloud_feature_test.cpp src/common.cpp src/svm/svm_common.cpp src/thr_pool.cpp src/histogram.cpp src/feature.cpp)
#rosbuild_add_compile_flags(test_pc_features "-O3 -fomit-frame-pointer -ffast-math")

rosbuild_add_executable(calc_intersection src/calcRectIntersection.cpp src/gpc.cpp src/common.cpp)
rosbuild_add_compile_flags(calc_intersection "-O3 -fomit-frame-pointer -ffast-math")

rosbuild_add_executable(showRect src/showRect.cpp src/common.cpp src/svm/svm_common.cpp src/feature.cpp)
rosbuild_add_executable(filterRect src/filterRect.cpp)

rosbuild_add_executable(boost_training src/boostTraining.cpp src/common.cpp src/svm/svm_common.cpp src/thr_pool.cpp src/histogram.cpp src/feature.cpp src/gpc.cpp )
rosbuild_add_compile_flags(boost_training "-O3 -fomit-frame-pointer -ffast-math")

rosbuild_add_executable(baseline src/baseline.cpp src/common.cpp src/svm/svm_common.cpp src/thr_pool.cpp src/histogram.cpp src/feature.cpp src/gpc.cpp )
rosbuild_add_compile_flags(baseline "-O3 -fomit-frame-pointer -ffast-math")

rosbuild_add_executable(background_subtraction src/testBackgroundSubtraction.cpp src/common.cpp src/svm/svm_common.cpp src/feature.cpp )
rosbuild_add_executable(centroid_grasp src/centroid_grasp.cpp src/common.cpp src/svm/svm_common.cpp src/feature.cpp )

rosbuild_add_executable(rank src/rank_main.cpp src/common.cpp src/svm/svm_common.cpp src/thr_pool.cpp src/histogram.cpp src/feature.cpp)
rosbuild_add_compile_flags(rank "-O3 -fomit-frame-pointer -ffast-math")

#rosbuild_add_compile_flags(testee "-O3 -fomit-frame-pointer -ffast-math")
#rosbuild_add_link_flags(testee "-lm")
#rosbuild_add_executable(lala src/lala.cpp)

#rosbuild_add_executable(thr_pool src/thr_pool.cpp src/thr_pool_main.cpp)
#rosbuild_add_compile_flags(thr_pool "-O3 -fomit-frame-pointer -ffast-math -pthread")
#rosbuild_add_link_flags(thr_pool "-lm -pthread")
