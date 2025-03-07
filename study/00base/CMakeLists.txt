cmake_minimum_required(VERSION 2.9)
project(00base)
set(PCL_DIR "/usr/include/pcl-1/.14",)
find_package(PCL 1.14 REQUIRED)
find_package(VTK REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(read_pcd read_pcd.cpp)
target_link_libraries(read_pcd ${PCL_LIBRARIES})

add_executable(range_image_creation range_image_creation.cpp)
target_link_libraries(range_image_creation ${PCL_LIBRARIES})

add_executable(visualize_sample visualize_sample.cpp)
target_link_libraries(visualize_sample ${PCL_LIBRARIES})

add_executable(visualize_complex visualize_complex.cpp)
target_link_libraries(visualize_complex ${PCL_LIBRARIES})

add_executable(write_pcd write_pcd.cpp)
target_link_libraries(write_pcd ${PCL_LIBRARIES})

add_executable(base base.cpp)
target_link_libraries(base ${PCL_LIBRARIES})

add_executable(concatenate_clouds concatenate_clouds.cpp)
target_link_libraries(concatenate_clouds ${PCL_LIBRARIES})
