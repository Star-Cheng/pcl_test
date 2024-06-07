# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.22

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/gym/Code/PCL/pcl_test

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/gym/Code/PCL/pcl_test/build

# Include any dependencies generated for this target.
include CMakeFiles/pcl_test.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/pcl_test.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/pcl_test.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/pcl_test.dir/flags.make

CMakeFiles/pcl_test.dir/pcl_test.cpp.o: CMakeFiles/pcl_test.dir/flags.make
CMakeFiles/pcl_test.dir/pcl_test.cpp.o: ../pcl_test.cpp
CMakeFiles/pcl_test.dir/pcl_test.cpp.o: CMakeFiles/pcl_test.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/gym/Code/PCL/pcl_test/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/pcl_test.dir/pcl_test.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/pcl_test.dir/pcl_test.cpp.o -MF CMakeFiles/pcl_test.dir/pcl_test.cpp.o.d -o CMakeFiles/pcl_test.dir/pcl_test.cpp.o -c /home/gym/Code/PCL/pcl_test/pcl_test.cpp

CMakeFiles/pcl_test.dir/pcl_test.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/pcl_test.dir/pcl_test.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/gym/Code/PCL/pcl_test/pcl_test.cpp > CMakeFiles/pcl_test.dir/pcl_test.cpp.i

CMakeFiles/pcl_test.dir/pcl_test.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/pcl_test.dir/pcl_test.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/gym/Code/PCL/pcl_test/pcl_test.cpp -o CMakeFiles/pcl_test.dir/pcl_test.cpp.s

# Object files for target pcl_test
pcl_test_OBJECTS = \
"CMakeFiles/pcl_test.dir/pcl_test.cpp.o"

# External object files for target pcl_test
pcl_test_EXTERNAL_OBJECTS =

pcl_test: CMakeFiles/pcl_test.dir/pcl_test.cpp.o
pcl_test: CMakeFiles/pcl_test.dir/build.make
pcl_test: /usr/lib/libpcl_surface.so
pcl_test: /usr/lib/libpcl_keypoints.so
pcl_test: /usr/lib/libpcl_tracking.so
pcl_test: /usr/lib/libpcl_recognition.so
pcl_test: /usr/lib/libpcl_stereo.so
pcl_test: /usr/lib/libpcl_outofcore.so
pcl_test: /usr/lib/libpcl_people.so
pcl_test: /usr/lib/libOpenNI.so
pcl_test: /usr/lib/x86_64-linux-gnu/libusb-1.0.so
pcl_test: /usr/lib/x86_64-linux-gnu/libOpenNI2.so
pcl_test: /usr/lib/x86_64-linux-gnu/libusb-1.0.so
pcl_test: /usr/lib/x86_64-linux-gnu/libflann_cpp.so
pcl_test: /usr/lib/libpcl_registration.so
pcl_test: /usr/lib/libpcl_segmentation.so
pcl_test: /usr/lib/libpcl_features.so
pcl_test: /usr/lib/libpcl_filters.so
pcl_test: /usr/lib/libpcl_sample_consensus.so
pcl_test: /usr/lib/libpcl_ml.so
pcl_test: /usr/lib/libpcl_visualization.so
pcl_test: /usr/lib/libpcl_search.so
pcl_test: /usr/lib/libpcl_kdtree.so
pcl_test: /usr/lib/libpcl_io.so
pcl_test: /usr/lib/libpcl_octree.so
pcl_test: /usr/lib/gcc/x86_64-linux-gnu/11/libgomp.so
pcl_test: /usr/lib/x86_64-linux-gnu/libpthread.a
pcl_test: /usr/lib/x86_64-linux-gnu/libpng.so
pcl_test: /usr/lib/x86_64-linux-gnu/libz.so
pcl_test: /usr/lib/libOpenNI.so
pcl_test: /usr/lib/x86_64-linux-gnu/libusb-1.0.so
pcl_test: /usr/lib/x86_64-linux-gnu/libOpenNI2.so
pcl_test: /usr/lib/x86_64-linux-gnu/libvtkChartsCore-9.1.so.9.1.0
pcl_test: /usr/lib/x86_64-linux-gnu/libvtkInteractionImage-9.1.so.9.1.0
pcl_test: /usr/lib/x86_64-linux-gnu/libvtkIOGeometry-9.1.so.9.1.0
pcl_test: /usr/lib/x86_64-linux-gnu/libjsoncpp.so
pcl_test: /usr/lib/x86_64-linux-gnu/libvtkIOPLY-9.1.so.9.1.0
pcl_test: /usr/lib/x86_64-linux-gnu/libvtkRenderingLOD-9.1.so.9.1.0
pcl_test: /usr/lib/x86_64-linux-gnu/libvtkViewsContext2D-9.1.so.9.1.0
pcl_test: /usr/lib/x86_64-linux-gnu/libvtkViewsCore-9.1.so.9.1.0
pcl_test: /usr/lib/x86_64-linux-gnu/libvtkRenderingContextOpenGL2-9.1.so.9.1.0
pcl_test: /usr/lib/x86_64-linux-gnu/libvtkGUISupportQt-9.1.so.9.1.0
pcl_test: /usr/lib/x86_64-linux-gnu/libvtkInteractionWidgets-9.1.so.9.1.0
pcl_test: /usr/lib/x86_64-linux-gnu/libvtkFiltersModeling-9.1.so.9.1.0
pcl_test: /usr/lib/x86_64-linux-gnu/libvtkInteractionStyle-9.1.so.9.1.0
pcl_test: /usr/lib/x86_64-linux-gnu/libvtkFiltersExtraction-9.1.so.9.1.0
pcl_test: /usr/lib/x86_64-linux-gnu/libvtkIOLegacy-9.1.so.9.1.0
pcl_test: /usr/lib/x86_64-linux-gnu/libvtkIOCore-9.1.so.9.1.0
pcl_test: /usr/lib/x86_64-linux-gnu/libvtkRenderingAnnotation-9.1.so.9.1.0
pcl_test: /usr/lib/x86_64-linux-gnu/libvtkRenderingContext2D-9.1.so.9.1.0
pcl_test: /usr/lib/x86_64-linux-gnu/libvtkRenderingFreeType-9.1.so.9.1.0
pcl_test: /usr/lib/x86_64-linux-gnu/libfreetype.so
pcl_test: /usr/lib/x86_64-linux-gnu/libvtkImagingSources-9.1.so.9.1.0
pcl_test: /usr/lib/x86_64-linux-gnu/libvtkIOImage-9.1.so.9.1.0
pcl_test: /usr/lib/x86_64-linux-gnu/libvtkImagingCore-9.1.so.9.1.0
pcl_test: /usr/lib/x86_64-linux-gnu/libvtkRenderingOpenGL2-9.1.so.9.1.0
pcl_test: /usr/lib/x86_64-linux-gnu/libvtkRenderingUI-9.1.so.9.1.0
pcl_test: /usr/lib/x86_64-linux-gnu/libvtkRenderingCore-9.1.so.9.1.0
pcl_test: /usr/lib/x86_64-linux-gnu/libvtkCommonColor-9.1.so.9.1.0
pcl_test: /usr/lib/x86_64-linux-gnu/libvtkFiltersGeometry-9.1.so.9.1.0
pcl_test: /usr/lib/x86_64-linux-gnu/libvtkFiltersSources-9.1.so.9.1.0
pcl_test: /usr/lib/x86_64-linux-gnu/libvtkFiltersGeneral-9.1.so.9.1.0
pcl_test: /usr/lib/x86_64-linux-gnu/libvtkCommonComputationalGeometry-9.1.so.9.1.0
pcl_test: /usr/lib/x86_64-linux-gnu/libvtkFiltersCore-9.1.so.9.1.0
pcl_test: /usr/lib/x86_64-linux-gnu/libvtkCommonExecutionModel-9.1.so.9.1.0
pcl_test: /usr/lib/x86_64-linux-gnu/libvtkCommonDataModel-9.1.so.9.1.0
pcl_test: /usr/lib/x86_64-linux-gnu/libvtkCommonMisc-9.1.so.9.1.0
pcl_test: /usr/lib/x86_64-linux-gnu/libvtkCommonTransforms-9.1.so.9.1.0
pcl_test: /usr/lib/x86_64-linux-gnu/libvtkCommonMath-9.1.so.9.1.0
pcl_test: /usr/lib/x86_64-linux-gnu/libvtkkissfft-9.1.so.9.1.0
pcl_test: /usr/lib/x86_64-linux-gnu/libGLEW.so
pcl_test: /usr/lib/x86_64-linux-gnu/libX11.so
pcl_test: /usr/lib/x86_64-linux-gnu/libQt5OpenGL.so.5.15.3
pcl_test: /usr/lib/x86_64-linux-gnu/libQt5Widgets.so.5.15.3
pcl_test: /usr/lib/x86_64-linux-gnu/libQt5Gui.so.5.15.3
pcl_test: /usr/lib/x86_64-linux-gnu/libQt5Core.so.5.15.3
pcl_test: /usr/lib/x86_64-linux-gnu/libvtkCommonCore-9.1.so.9.1.0
pcl_test: /usr/lib/x86_64-linux-gnu/libtbb.so.12.5
pcl_test: /usr/lib/x86_64-linux-gnu/libvtksys-9.1.so.9.1.0
pcl_test: /usr/lib/libpcl_common.so
pcl_test: /usr/lib/x86_64-linux-gnu/libboost_system.so.1.74.0
pcl_test: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so.1.74.0
pcl_test: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so.1.74.0
pcl_test: /usr/lib/x86_64-linux-gnu/libboost_serialization.so.1.74.0
pcl_test: /usr/lib/x86_64-linux-gnu/libqhull_r.so.8.0.2
pcl_test: CMakeFiles/pcl_test.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/gym/Code/PCL/pcl_test/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable pcl_test"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/pcl_test.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/pcl_test.dir/build: pcl_test
.PHONY : CMakeFiles/pcl_test.dir/build

CMakeFiles/pcl_test.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/pcl_test.dir/cmake_clean.cmake
.PHONY : CMakeFiles/pcl_test.dir/clean

CMakeFiles/pcl_test.dir/depend:
	cd /home/gym/Code/PCL/pcl_test/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/gym/Code/PCL/pcl_test /home/gym/Code/PCL/pcl_test /home/gym/Code/PCL/pcl_test/build /home/gym/Code/PCL/pcl_test/build /home/gym/Code/PCL/pcl_test/build/CMakeFiles/pcl_test.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/pcl_test.dir/depend

