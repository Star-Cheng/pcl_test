cmake_minimum_required(VERSION 2.9)
project(04feature)
set(PCL_DIR "/usr/include/pcl-1/.14",)
find_package(PCL 1.14 REQUIRED)
find_package(VTK REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(normal_estimation_openmp normal_estimation_openmp.cpp)
target_link_libraries(normal_estimation_openmp ${PCL_LIBRARIES})

add_executable(normal_estimation normal_estimation.cpp)
target_link_libraries(normal_estimation ${PCL_LIBRARIES})

add_executable(normal_estimation_using_integral_images normal_estimation_using_integral_images.cpp)
target_link_libraries(normal_estimation_using_integral_images ${PCL_LIBRARIES})

add_executable(principal_curvatures principal_curvatures.cpp)
target_link_libraries(principal_curvatures ${PCL_LIBRARIES})

add_executable(fast_point_featur_hist fast_point_featur_hist.cpp)
target_link_libraries(fast_point_featur_hist ${PCL_LIBRARIES})
