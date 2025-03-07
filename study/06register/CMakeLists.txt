cmake_minimum_required(VERSION 2.9)
project(06register)
set(PCL_DIR "/usr/include/pcl-1/.14",)
find_package(PCL 1.14 REQUIRED)
find_package(VTK REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(icp icp.cpp)
target_link_libraries(icp ${PCL_LIBRARIES})

add_executable(icp2 icp2.cpp)
target_link_libraries(icp2 ${PCL_LIBRARIES})

add_executable(ndt ndt.cpp)
target_link_libraries(ndt ${PCL_LIBRARIES})
