cmake_minimum_required(VERSION 2.9)
project(pcl_test)

set(CMAKE_BUILD_TYPE Debug)
# set(PCL_DIR "/usr/include/pcl-1.12",)
# find_package(PCL 1.12 REQUIRED)
set(PCL_DIR "/usr/include/pcl-1/.14",)
find_package(PCL 1.14 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(pcl_test pcl_test.cpp)

target_link_libraries (pcl_test ${PCL_LIBRARIES})

install(TARGETS pcl_test RUNTIME DESTINATION bin)
