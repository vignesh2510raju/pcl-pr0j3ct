# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list

# Suppress display of executed commands.
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
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The program to use to edit the cache.
CMAKE_EDIT_COMMAND = /usr/bin/cmake-gui

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/vignesh/pcl-proyect

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/vignesh/pcl-proyect/build

# Include any dependencies generated for this target.
include CMakeFiles/bin2pcd.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/bin2pcd.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/bin2pcd.dir/flags.make

CMakeFiles/bin2pcd.dir/src/bin2pcd.cpp.o: CMakeFiles/bin2pcd.dir/flags.make
CMakeFiles/bin2pcd.dir/src/bin2pcd.cpp.o: ../src/bin2pcd.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/vignesh/pcl-proyect/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/bin2pcd.dir/src/bin2pcd.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/bin2pcd.dir/src/bin2pcd.cpp.o -c /home/vignesh/pcl-proyect/src/bin2pcd.cpp

CMakeFiles/bin2pcd.dir/src/bin2pcd.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/bin2pcd.dir/src/bin2pcd.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/vignesh/pcl-proyect/src/bin2pcd.cpp > CMakeFiles/bin2pcd.dir/src/bin2pcd.cpp.i

CMakeFiles/bin2pcd.dir/src/bin2pcd.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/bin2pcd.dir/src/bin2pcd.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/vignesh/pcl-proyect/src/bin2pcd.cpp -o CMakeFiles/bin2pcd.dir/src/bin2pcd.cpp.s

CMakeFiles/bin2pcd.dir/src/bin2pcd.cpp.o.requires:
.PHONY : CMakeFiles/bin2pcd.dir/src/bin2pcd.cpp.o.requires

CMakeFiles/bin2pcd.dir/src/bin2pcd.cpp.o.provides: CMakeFiles/bin2pcd.dir/src/bin2pcd.cpp.o.requires
	$(MAKE) -f CMakeFiles/bin2pcd.dir/build.make CMakeFiles/bin2pcd.dir/src/bin2pcd.cpp.o.provides.build
.PHONY : CMakeFiles/bin2pcd.dir/src/bin2pcd.cpp.o.provides

CMakeFiles/bin2pcd.dir/src/bin2pcd.cpp.o.provides.build: CMakeFiles/bin2pcd.dir/src/bin2pcd.cpp.o

# Object files for target bin2pcd
bin2pcd_OBJECTS = \
"CMakeFiles/bin2pcd.dir/src/bin2pcd.cpp.o"

# External object files for target bin2pcd
bin2pcd_EXTERNAL_OBJECTS =

bin2pcd: CMakeFiles/bin2pcd.dir/src/bin2pcd.cpp.o
bin2pcd: CMakeFiles/bin2pcd.dir/build.make
bin2pcd: /usr/lib/x86_64-linux-gnu/libboost_system.so
bin2pcd: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
bin2pcd: /usr/lib/x86_64-linux-gnu/libboost_thread.so
bin2pcd: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
bin2pcd: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
bin2pcd: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
bin2pcd: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
bin2pcd: /usr/lib/x86_64-linux-gnu/libpthread.so
bin2pcd: /usr/lib/libpcl_common.so
bin2pcd: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
bin2pcd: /usr/lib/libpcl_kdtree.so
bin2pcd: /usr/lib/libpcl_octree.so
bin2pcd: /usr/lib/libpcl_search.so
bin2pcd: /usr/lib/x86_64-linux-gnu/libqhull.so
bin2pcd: /usr/lib/libpcl_surface.so
bin2pcd: /usr/lib/libpcl_sample_consensus.so
bin2pcd: /usr/lib/libOpenNI.so
bin2pcd: /usr/lib/libOpenNI2.so
bin2pcd: /usr/lib/libvtkCommon.so.5.8.0
bin2pcd: /usr/lib/libvtkFiltering.so.5.8.0
bin2pcd: /usr/lib/libvtkImaging.so.5.8.0
bin2pcd: /usr/lib/libvtkGraphics.so.5.8.0
bin2pcd: /usr/lib/libvtkGenericFiltering.so.5.8.0
bin2pcd: /usr/lib/libvtkIO.so.5.8.0
bin2pcd: /usr/lib/libvtkRendering.so.5.8.0
bin2pcd: /usr/lib/libvtkVolumeRendering.so.5.8.0
bin2pcd: /usr/lib/libvtkHybrid.so.5.8.0
bin2pcd: /usr/lib/libvtkWidgets.so.5.8.0
bin2pcd: /usr/lib/libvtkParallel.so.5.8.0
bin2pcd: /usr/lib/libvtkInfovis.so.5.8.0
bin2pcd: /usr/lib/libvtkGeovis.so.5.8.0
bin2pcd: /usr/lib/libvtkViews.so.5.8.0
bin2pcd: /usr/lib/libvtkCharts.so.5.8.0
bin2pcd: /usr/lib/libpcl_io.so
bin2pcd: /usr/lib/libpcl_filters.so
bin2pcd: /usr/lib/libpcl_features.so
bin2pcd: /usr/lib/libpcl_keypoints.so
bin2pcd: /usr/lib/libpcl_registration.so
bin2pcd: /usr/lib/libpcl_segmentation.so
bin2pcd: /usr/lib/libpcl_recognition.so
bin2pcd: /usr/lib/libpcl_visualization.so
bin2pcd: /usr/lib/libpcl_people.so
bin2pcd: /usr/lib/libpcl_outofcore.so
bin2pcd: /usr/lib/libpcl_tracking.so
bin2pcd: /usr/lib/libpcl_apps.so
bin2pcd: /usr/lib/x86_64-linux-gnu/libboost_system.so
bin2pcd: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
bin2pcd: /usr/lib/x86_64-linux-gnu/libboost_thread.so
bin2pcd: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
bin2pcd: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
bin2pcd: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
bin2pcd: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
bin2pcd: /usr/lib/x86_64-linux-gnu/libpthread.so
bin2pcd: /usr/lib/x86_64-linux-gnu/libqhull.so
bin2pcd: /usr/lib/libOpenNI.so
bin2pcd: /usr/lib/libOpenNI2.so
bin2pcd: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
bin2pcd: /usr/lib/libvtkCommon.so.5.8.0
bin2pcd: /usr/lib/libvtkFiltering.so.5.8.0
bin2pcd: /usr/lib/libvtkImaging.so.5.8.0
bin2pcd: /usr/lib/libvtkGraphics.so.5.8.0
bin2pcd: /usr/lib/libvtkGenericFiltering.so.5.8.0
bin2pcd: /usr/lib/libvtkIO.so.5.8.0
bin2pcd: /usr/lib/libvtkRendering.so.5.8.0
bin2pcd: /usr/lib/libvtkVolumeRendering.so.5.8.0
bin2pcd: /usr/lib/libvtkHybrid.so.5.8.0
bin2pcd: /usr/lib/libvtkWidgets.so.5.8.0
bin2pcd: /usr/lib/libvtkParallel.so.5.8.0
bin2pcd: /usr/lib/libvtkInfovis.so.5.8.0
bin2pcd: /usr/lib/libvtkGeovis.so.5.8.0
bin2pcd: /usr/lib/libvtkViews.so.5.8.0
bin2pcd: /usr/lib/libvtkCharts.so.5.8.0
bin2pcd: /usr/lib/libpcl_common.so
bin2pcd: /usr/lib/libpcl_kdtree.so
bin2pcd: /usr/lib/libpcl_octree.so
bin2pcd: /usr/lib/libpcl_search.so
bin2pcd: /usr/lib/libpcl_surface.so
bin2pcd: /usr/lib/libpcl_sample_consensus.so
bin2pcd: /usr/lib/libpcl_io.so
bin2pcd: /usr/lib/libpcl_filters.so
bin2pcd: /usr/lib/libpcl_features.so
bin2pcd: /usr/lib/libpcl_keypoints.so
bin2pcd: /usr/lib/libpcl_registration.so
bin2pcd: /usr/lib/libpcl_segmentation.so
bin2pcd: /usr/lib/libpcl_recognition.so
bin2pcd: /usr/lib/libpcl_visualization.so
bin2pcd: /usr/lib/libpcl_people.so
bin2pcd: /usr/lib/libpcl_outofcore.so
bin2pcd: /usr/lib/libpcl_tracking.so
bin2pcd: /usr/lib/libpcl_apps.so
bin2pcd: /usr/lib/libvtkViews.so.5.8.0
bin2pcd: /usr/lib/libvtkInfovis.so.5.8.0
bin2pcd: /usr/lib/libvtkWidgets.so.5.8.0
bin2pcd: /usr/lib/libvtkVolumeRendering.so.5.8.0
bin2pcd: /usr/lib/libvtkHybrid.so.5.8.0
bin2pcd: /usr/lib/libvtkParallel.so.5.8.0
bin2pcd: /usr/lib/libvtkRendering.so.5.8.0
bin2pcd: /usr/lib/libvtkImaging.so.5.8.0
bin2pcd: /usr/lib/libvtkGraphics.so.5.8.0
bin2pcd: /usr/lib/libvtkIO.so.5.8.0
bin2pcd: /usr/lib/libvtkFiltering.so.5.8.0
bin2pcd: /usr/lib/libvtkCommon.so.5.8.0
bin2pcd: /usr/lib/libvtksys.so.5.8.0
bin2pcd: CMakeFiles/bin2pcd.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable bin2pcd"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/bin2pcd.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/bin2pcd.dir/build: bin2pcd
.PHONY : CMakeFiles/bin2pcd.dir/build

CMakeFiles/bin2pcd.dir/requires: CMakeFiles/bin2pcd.dir/src/bin2pcd.cpp.o.requires
.PHONY : CMakeFiles/bin2pcd.dir/requires

CMakeFiles/bin2pcd.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/bin2pcd.dir/cmake_clean.cmake
.PHONY : CMakeFiles/bin2pcd.dir/clean

CMakeFiles/bin2pcd.dir/depend:
	cd /home/vignesh/pcl-proyect/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/vignesh/pcl-proyect /home/vignesh/pcl-proyect /home/vignesh/pcl-proyect/build /home/vignesh/pcl-proyect/build /home/vignesh/pcl-proyect/build/CMakeFiles/bin2pcd.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/bin2pcd.dir/depend

