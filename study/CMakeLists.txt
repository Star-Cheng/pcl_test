cmake_minimum_required(VERSION 2.9)
project(study)
set(CMAKE_BUILD_TYPE Debug)  # 设置编译类型为Debug
# set(PCL_DIR "/usr/include/pcl-1.12",)
# find_package(PCL 1.12 REQUIRED)
set(PCL_DIR "/usr/include/pcl-1/.14",)
find_package(PCL 1.14 REQUIRED)
find_package(VTK REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
cmake_minimum_required(VERSION 3.0)

add_executable(demo demo.cpp)

target_link_libraries (demo ${PCL_LIBRARIES})