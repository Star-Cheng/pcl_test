import os

target_dir = input("please input the dir:")
target_listdir = os.listdir(target_dir)
CmakeLists_path = os.path.join(target_dir, "CMakeLists.txt")
CmakeVersion = "cmake_minimum_required(VERSION 2.9)"
project_name = '\nproject({})'.format(os.path.basename(target_dir))
CmakeRequire = """
set(PCL_DIR "/usr/include/pcl-1/.14",)
find_package(PCL 1.14 REQUIRED)
find_package(VTK REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})\n"""
CmakeLists_text = CmakeVersion + project_name + CmakeRequire
print(CmakeLists_text)
with open(CmakeLists_path, "w") as f:
    f.write(CmakeLists_text)
    for cpp_name in target_listdir:
        if cpp_name.endswith(".cpp"):
            cpp_path = os.path.join(target_dir, cpp_name)
            add_executable = "add_executable({} {})".format(cpp_name.split(".")[0], cpp_name)
            target_link_libraries = "target_link_libraries({} ${})".format(cpp_name.split(".")[0], "{PCL_LIBRARIES}")
            f.write("\n")
            f.write(add_executable + "\n")
            f.write(target_link_libraries + "\n")
