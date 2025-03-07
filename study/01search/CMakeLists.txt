cmake_minimum_required(VERSION 2.9)
project(01search)
set(PCL_DIR "/usr/include/pcl-1/.14",)
find_package(PCL 1.14 REQUIRED)
find_package(VTK REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(octree_change_detection octree_change_detection.cpp)
target_link_libraries(octree_change_detection ${PCL_LIBRARIES})

add_executable(octree_search octree_search.cpp)
target_link_libraries(octree_search ${PCL_LIBRARIES})

add_executable(search search.cpp)
target_link_libraries(search ${PCL_LIBRARIES})

add_executable(kdtree_search kdtree_search.cpp)
target_link_libraries(kdtree_search ${PCL_LIBRARIES})
