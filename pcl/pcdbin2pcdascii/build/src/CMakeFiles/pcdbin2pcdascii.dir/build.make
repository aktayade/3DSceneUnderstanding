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
CMAKE_SOURCE_DIR = /home/srinath/Desktop/545Project/code/3DSceneUnderstanding/FeatureExtract/pcdbin2pcdascii

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/srinath/Desktop/545Project/code/3DSceneUnderstanding/FeatureExtract/pcdbin2pcdascii/build

# Include any dependencies generated for this target.
include src/CMakeFiles/pcdbin2pcdascii.dir/depend.make

# Include the progress variables for this target.
include src/CMakeFiles/pcdbin2pcdascii.dir/progress.make

# Include the compile flags for this target's objects.
include src/CMakeFiles/pcdbin2pcdascii.dir/flags.make

src/CMakeFiles/pcdbin2pcdascii.dir/main.cpp.o: src/CMakeFiles/pcdbin2pcdascii.dir/flags.make
src/CMakeFiles/pcdbin2pcdascii.dir/main.cpp.o: ../src/main.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/srinath/Desktop/545Project/code/3DSceneUnderstanding/FeatureExtract/pcdbin2pcdascii/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object src/CMakeFiles/pcdbin2pcdascii.dir/main.cpp.o"
	cd /home/srinath/Desktop/545Project/code/3DSceneUnderstanding/FeatureExtract/pcdbin2pcdascii/build/src && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/pcdbin2pcdascii.dir/main.cpp.o -c /home/srinath/Desktop/545Project/code/3DSceneUnderstanding/FeatureExtract/pcdbin2pcdascii/src/main.cpp

src/CMakeFiles/pcdbin2pcdascii.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/pcdbin2pcdascii.dir/main.cpp.i"
	cd /home/srinath/Desktop/545Project/code/3DSceneUnderstanding/FeatureExtract/pcdbin2pcdascii/build/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/srinath/Desktop/545Project/code/3DSceneUnderstanding/FeatureExtract/pcdbin2pcdascii/src/main.cpp > CMakeFiles/pcdbin2pcdascii.dir/main.cpp.i

src/CMakeFiles/pcdbin2pcdascii.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/pcdbin2pcdascii.dir/main.cpp.s"
	cd /home/srinath/Desktop/545Project/code/3DSceneUnderstanding/FeatureExtract/pcdbin2pcdascii/build/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/srinath/Desktop/545Project/code/3DSceneUnderstanding/FeatureExtract/pcdbin2pcdascii/src/main.cpp -o CMakeFiles/pcdbin2pcdascii.dir/main.cpp.s

src/CMakeFiles/pcdbin2pcdascii.dir/main.cpp.o.requires:
.PHONY : src/CMakeFiles/pcdbin2pcdascii.dir/main.cpp.o.requires

src/CMakeFiles/pcdbin2pcdascii.dir/main.cpp.o.provides: src/CMakeFiles/pcdbin2pcdascii.dir/main.cpp.o.requires
	$(MAKE) -f src/CMakeFiles/pcdbin2pcdascii.dir/build.make src/CMakeFiles/pcdbin2pcdascii.dir/main.cpp.o.provides.build
.PHONY : src/CMakeFiles/pcdbin2pcdascii.dir/main.cpp.o.provides

src/CMakeFiles/pcdbin2pcdascii.dir/main.cpp.o.provides.build: src/CMakeFiles/pcdbin2pcdascii.dir/main.cpp.o

# Object files for target pcdbin2pcdascii
pcdbin2pcdascii_OBJECTS = \
"CMakeFiles/pcdbin2pcdascii.dir/main.cpp.o"

# External object files for target pcdbin2pcdascii
pcdbin2pcdascii_EXTERNAL_OBJECTS =

src/pcdbin2pcdascii: src/CMakeFiles/pcdbin2pcdascii.dir/main.cpp.o
src/pcdbin2pcdascii: /usr/lib/libboost_system-mt.so
src/pcdbin2pcdascii: /usr/lib/libboost_filesystem-mt.so
src/pcdbin2pcdascii: /usr/lib/libboost_thread-mt.so
src/pcdbin2pcdascii: /usr/lib/libboost_date_time-mt.so
src/pcdbin2pcdascii: /usr/lib/libboost_iostreams-mt.so
src/pcdbin2pcdascii: /usr/lib/libpcl_common.so
src/pcdbin2pcdascii: /usr/lib/libflann_cpp_s.a
src/pcdbin2pcdascii: /usr/lib/libpcl_kdtree.so
src/pcdbin2pcdascii: /usr/lib/libpcl_octree.so
src/pcdbin2pcdascii: /usr/lib/libOpenNI.so
src/pcdbin2pcdascii: /usr/lib/libpcl_io.so
src/pcdbin2pcdascii: /usr/lib/libpcl_sample_consensus.so
src/pcdbin2pcdascii: /usr/lib/libpcl_search.so
src/pcdbin2pcdascii: /usr/lib/libpcl_filters.so
src/pcdbin2pcdascii: /usr/lib/libpcl_segmentation.so
src/pcdbin2pcdascii: /usr/lib/libpcl_features.so
src/pcdbin2pcdascii: /usr/lib/libqhull.so
src/pcdbin2pcdascii: /usr/lib/libpcl_surface.so
src/pcdbin2pcdascii: /usr/lib/libpcl_registration.so
src/pcdbin2pcdascii: /usr/lib/libpcl_visualization.so
src/pcdbin2pcdascii: /usr/lib/libpcl_keypoints.so
src/pcdbin2pcdascii: /usr/lib/libpcl_tracking.so
src/pcdbin2pcdascii: /usr/lib/libpcl_apps.so
src/pcdbin2pcdascii: /usr/lib/x86_64-linux-gnu/libfreetype.so
src/pcdbin2pcdascii: /usr/lib/libgl2ps.so
src/pcdbin2pcdascii: /usr/lib/x86_64-linux-gnu/libXt.so
src/pcdbin2pcdascii: /usr/lib/libpq.so
src/pcdbin2pcdascii: /usr/lib/libmysqlclient.so
src/pcdbin2pcdascii: /usr/lib/x86_64-linux-gnu/libpng.so
src/pcdbin2pcdascii: /usr/lib/x86_64-linux-gnu/libz.so
src/pcdbin2pcdascii: /usr/lib/x86_64-linux-gnu/libjpeg.so
src/pcdbin2pcdascii: /usr/lib/x86_64-linux-gnu/libtiff.so
src/pcdbin2pcdascii: /usr/lib/x86_64-linux-gnu/libexpat.so
src/pcdbin2pcdascii: /usr/lib/libavformat.so
src/pcdbin2pcdascii: /usr/lib/libavcodec.so
src/pcdbin2pcdascii: /usr/lib/libavutil.so
src/pcdbin2pcdascii: /usr/lib/libswscale.so
src/pcdbin2pcdascii: /usr/lib/x86_64-linux-gnu/libGL.so
src/pcdbin2pcdascii: /usr/lib/openmpi/lib/libmpi.so
src/pcdbin2pcdascii: /usr/lib/openmpi/lib/libopen-rte.so
src/pcdbin2pcdascii: /usr/lib/openmpi/lib/libopen-pal.so
src/pcdbin2pcdascii: /usr/lib/x86_64-linux-gnu/libdl.so
src/pcdbin2pcdascii: /usr/lib/x86_64-linux-gnu/libnsl.so
src/pcdbin2pcdascii: /usr/lib/x86_64-linux-gnu/libutil.so
src/pcdbin2pcdascii: /usr/lib/x86_64-linux-gnu/libm.so
src/pcdbin2pcdascii: /usr/lib/x86_64-linux-gnu/libdl.so
src/pcdbin2pcdascii: /usr/lib/x86_64-linux-gnu/libnsl.so
src/pcdbin2pcdascii: /usr/lib/x86_64-linux-gnu/libutil.so
src/pcdbin2pcdascii: /usr/lib/x86_64-linux-gnu/libm.so
src/pcdbin2pcdascii: /usr/lib/openmpi/lib/libmpi_cxx.so
src/pcdbin2pcdascii: src/CMakeFiles/pcdbin2pcdascii.dir/build.make
src/pcdbin2pcdascii: src/CMakeFiles/pcdbin2pcdascii.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable pcdbin2pcdascii"
	cd /home/srinath/Desktop/545Project/code/3DSceneUnderstanding/FeatureExtract/pcdbin2pcdascii/build/src && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/pcdbin2pcdascii.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/CMakeFiles/pcdbin2pcdascii.dir/build: src/pcdbin2pcdascii
.PHONY : src/CMakeFiles/pcdbin2pcdascii.dir/build

src/CMakeFiles/pcdbin2pcdascii.dir/requires: src/CMakeFiles/pcdbin2pcdascii.dir/main.cpp.o.requires
.PHONY : src/CMakeFiles/pcdbin2pcdascii.dir/requires

src/CMakeFiles/pcdbin2pcdascii.dir/clean:
	cd /home/srinath/Desktop/545Project/code/3DSceneUnderstanding/FeatureExtract/pcdbin2pcdascii/build/src && $(CMAKE_COMMAND) -P CMakeFiles/pcdbin2pcdascii.dir/cmake_clean.cmake
.PHONY : src/CMakeFiles/pcdbin2pcdascii.dir/clean

src/CMakeFiles/pcdbin2pcdascii.dir/depend:
	cd /home/srinath/Desktop/545Project/code/3DSceneUnderstanding/FeatureExtract/pcdbin2pcdascii/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/srinath/Desktop/545Project/code/3DSceneUnderstanding/FeatureExtract/pcdbin2pcdascii /home/srinath/Desktop/545Project/code/3DSceneUnderstanding/FeatureExtract/pcdbin2pcdascii/src /home/srinath/Desktop/545Project/code/3DSceneUnderstanding/FeatureExtract/pcdbin2pcdascii/build /home/srinath/Desktop/545Project/code/3DSceneUnderstanding/FeatureExtract/pcdbin2pcdascii/build/src /home/srinath/Desktop/545Project/code/3DSceneUnderstanding/FeatureExtract/pcdbin2pcdascii/build/src/CMakeFiles/pcdbin2pcdascii.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/CMakeFiles/pcdbin2pcdascii.dir/depend

