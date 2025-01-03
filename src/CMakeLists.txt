cmake_minimum_required(VERSION 2.9)
project(pcl_test)
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
# project(PCDToOBJ)



add_executable(ex1 ex1.cpp)
target_link_libraries (ex1 ${PCL_LIBRARIES})
install(TARGETS ex1 RUNTIME DESTINATION bin)

add_executable(ex2 ex2.cpp)
target_link_libraries (ex2 ${PCL_LIBRARIES})
install(TARGETS ex2 RUNTIME DESTINATION bin)

add_executable(ex3 ex3.cpp)
target_link_libraries (ex3 ${PCL_LIBRARIES})
install(TARGETS ex3 RUNTIME DESTINATION bin)
