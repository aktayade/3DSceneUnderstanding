# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canoncical targets will work.
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

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/srinath/Desktop/545Project/code/3DSceneUnderstanding/FeatureExtract/visualize

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/srinath/Desktop/545Project/code/3DSceneUnderstanding/FeatureExtract/visualize/build

# Include any dependencies generated for this target.
include src/CMakeFiles/visualizepcd.dir/depend.make

# Include the progress variables for this target.
include src/CMakeFiles/visualizepcd.dir/progress.make

# Include the compile flags for this target's objects.
include src/CMakeFiles/visualizepcd.dir/flags.make

src/CMakeFiles/visualizepcd.dir/main.cpp.o: src/CMakeFiles/visualizepcd.dir/flags.make
src/CMakeFiles/visualizepcd.dir/main.cpp.o: ../src/main.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/srinath/Desktop/545Project/code/3DSceneUnderstanding/FeatureExtract/visualize/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object src/CMakeFiles/visualizepcd.dir/main.cpp.o"
	cd /home/srinath/Desktop/545Project/code/3DSceneUnderstanding/FeatureExtract/visualize/build/src && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/visualizepcd.dir/main.cpp.o -c /home/srinath/Desktop/545Project/code/3DSceneUnderstanding/FeatureExtract/visualize/src/main.cpp

src/CMakeFiles/visualizepcd.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/visualizepcd.dir/main.cpp.i"
	cd /home/srinath/Desktop/545Project/code/3DSceneUnderstanding/FeatureExtract/visualize/build/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/srinath/Desktop/545Project/code/3DSceneUnderstanding/FeatureExtract/visualize/src/main.cpp > CMakeFiles/visualizepcd.dir/main.cpp.i

src/CMakeFiles/visualizepcd.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/visualizepcd.dir/main.cpp.s"
	cd /home/srinath/Desktop/545Project/code/3DSceneUnderstanding/FeatureExtract/visualize/build/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/srinath/Desktop/545Project/code/3DSceneUnderstanding/FeatureExtract/visualize/src/main.cpp -o CMakeFiles/visualizepcd.dir/main.cpp.s

src/CMakeFiles/visualizepcd.dir/main.cpp.o.requires:
.PHONY : src/CMakeFiles/visualizepcd.dir/main.cpp.o.requires

src/CMakeFiles/visualizepcd.dir/main.cpp.o.provides: src/CMakeFiles/visualizepcd.dir/main.cpp.o.requires
	$(MAKE) -f src/CMakeFiles/visualizepcd.dir/build.make src/CMakeFiles/visualizepcd.dir/main.cpp.o.provides.build
.PHONY : src/CMakeFiles/visualizepcd.dir/main.cpp.o.provides

src/CMakeFiles/visualizepcd.dir/main.cpp.o.provides.build: src/CMakeFiles/visualizepcd.dir/main.cpp.o

# Object files for target visualizepcd
visualizepcd_OBJECTS = \
"CMakeFiles/visualizepcd.dir/main.cpp.o"

# External object files for target visualizepcd
visualizepcd_EXTERNAL_OBJECTS =

src/visualizepcd: src/CMakeFiles/visualizepcd.dir/main.cpp.o
src/visualizepcd: /usr/lib/libboost_system-mt.so
src/visualizepcd: /usr/lib/libboost_filesystem-mt.so
src/visualizepcd: /usr/lib/libboost_thread-mt.so
src/visualizepcd: /usr/lib/libboost_date_time-mt.so
src/visualizepcd: /usr/lib/libboost_iostreams-mt.so
src/visualizepcd: /usr/lib/libpcl_common.so
src/visualizepcd: /usr/lib/libflann_cpp_s.a
src/visualizepcd: /usr/lib/libpcl_kdtree.so
src/visualizepcd: /usr/lib/libpcl_octree.so
src/visualizepcd: /usr/lib/libOpenNI.so
src/visualizepcd: /usr/lib/libpcl_io.so
src/visualizepcd: /usr/lib/libpcl_sample_consensus.so
src/visualizepcd: /usr/lib/libpcl_search.so
src/visualizepcd: /usr/lib/libpcl_filters.so
src/visualizepcd: /usr/lib/libpcl_segmentation.so
src/visualizepcd: /usr/lib/libpcl_features.so
src/visualizepcd: /usr/lib/libqhull.so
src/visualizepcd: /usr/lib/libpcl_surface.so
src/visualizepcd: /usr/lib/libpcl_registration.so
src/visualizepcd: /usr/lib/libpcl_visualization.so
src/visualizepcd: /usr/lib/libpcl_keypoints.so
src/visualizepcd: /usr/lib/libpcl_tracking.so
src/visualizepcd: /usr/lib/libpcl_apps.so
src/visualizepcd: /usr/lib/x86_64-linux-gnu/libfreetype.so
src/visualizepcd: /usr/lib/libgl2ps.so
src/visualizepcd: /usr/lib/x86_64-linux-gnu/libXt.so
src/visualizepcd: /usr/lib/libpq.so
src/visualizepcd: /usr/lib/libmysqlclient.so
src/visualizepcd: /usr/lib/x86_64-linux-gnu/libpng.so
src/visualizepcd: /usr/lib/x86_64-linux-gnu/libz.so
src/visualizepcd: /usr/lib/x86_64-linux-gnu/libjpeg.so
src/visualizepcd: /usr/lib/x86_64-linux-gnu/libtiff.so
src/visualizepcd: /usr/lib/x86_64-linux-gnu/libexpat.so
src/visualizepcd: /usr/lib/libavformat.so
src/visualizepcd: /usr/lib/libavcodec.so
src/visualizepcd: /usr/lib/libavutil.so
src/visualizepcd: /usr/lib/libswscale.so
src/visualizepcd: /usr/lib/x86_64-linux-gnu/libGL.so
src/visualizepcd: /usr/lib/openmpi/lib/libmpi.so
src/visualizepcd: /usr/lib/openmpi/lib/libopen-rte.so
src/visualizepcd: /usr/lib/openmpi/lib/libopen-pal.so
src/visualizepcd: /usr/lib/x86_64-linux-gnu/libdl.so
src/visualizepcd: /usr/lib/x86_64-linux-gnu/libnsl.so
src/visualizepcd: /usr/lib/x86_64-linux-gnu/libutil.so
src/visualizepcd: /usr/lib/x86_64-linux-gnu/libm.so
src/visualizepcd: /usr/lib/x86_64-linux-gnu/libdl.so
src/visualizepcd: /usr/lib/x86_64-linux-gnu/libnsl.so
src/visualizepcd: /usr/lib/x86_64-linux-gnu/libutil.so
src/visualizepcd: /usr/lib/x86_64-linux-gnu/libm.so
src/visualizepcd: /usr/lib/openmpi/lib/libmpi_cxx.so
src/visualizepcd: src/CMakeFiles/visualizepcd.dir/build.make
src/visualizepcd: src/CMakeFiles/visualizepcd.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable visualizepcd"
	cd /home/srinath/Desktop/545Project/code/3DSceneUnderstanding/FeatureExtract/visualize/build/src && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/visualizepcd.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/CMakeFiles/visualizepcd.dir/build: src/visualizepcd
.PHONY : src/CMakeFiles/visualizepcd.dir/build

src/CMakeFiles/visualizepcd.dir/requires: src/CMakeFiles/visualizepcd.dir/main.cpp.o.requires
.PHONY : src/CMakeFiles/visualizepcd.dir/requires

src/CMakeFiles/visualizepcd.dir/clean:
	cd /home/srinath/Desktop/545Project/code/3DSceneUnderstanding/FeatureExtract/visualize/build/src && $(CMAKE_COMMAND) -P CMakeFiles/visualizepcd.dir/cmake_clean.cmake
.PHONY : src/CMakeFiles/visualizepcd.dir/clean

src/CMakeFiles/visualizepcd.dir/depend:
	cd /home/srinath/Desktop/545Project/code/3DSceneUnderstanding/FeatureExtract/visualize/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/srinath/Desktop/545Project/code/3DSceneUnderstanding/FeatureExtract/visualize /home/srinath/Desktop/545Project/code/3DSceneUnderstanding/FeatureExtract/visualize/src /home/srinath/Desktop/545Project/code/3DSceneUnderstanding/FeatureExtract/visualize/build /home/srinath/Desktop/545Project/code/3DSceneUnderstanding/FeatureExtract/visualize/build/src /home/srinath/Desktop/545Project/code/3DSceneUnderstanding/FeatureExtract/visualize/build/src/CMakeFiles/visualizepcd.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/CMakeFiles/visualizepcd.dir/depend

