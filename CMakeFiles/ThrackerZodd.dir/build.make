# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.6

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


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
CMAKE_COMMAND = /usr/local/Cellar/cmake/3.6.1/bin/cmake

# The command to remove a file.
RM = /usr/local/Cellar/cmake/3.6.1/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /Users/kiranshila/Desktop/thracker_zod

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /Users/kiranshila/Desktop/thracker_zod

# Include any dependencies generated for this target.
include CMakeFiles/ThrackerZodd.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/ThrackerZodd.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/ThrackerZodd.dir/flags.make

CMakeFiles/ThrackerZodd.dir/src/main.cpp.o: CMakeFiles/ThrackerZodd.dir/flags.make
CMakeFiles/ThrackerZodd.dir/src/main.cpp.o: src/main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/kiranshila/Desktop/thracker_zod/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/ThrackerZodd.dir/src/main.cpp.o"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/ThrackerZodd.dir/src/main.cpp.o -c /Users/kiranshila/Desktop/thracker_zod/src/main.cpp

CMakeFiles/ThrackerZodd.dir/src/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ThrackerZodd.dir/src/main.cpp.i"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/kiranshila/Desktop/thracker_zod/src/main.cpp > CMakeFiles/ThrackerZodd.dir/src/main.cpp.i

CMakeFiles/ThrackerZodd.dir/src/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ThrackerZodd.dir/src/main.cpp.s"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/kiranshila/Desktop/thracker_zod/src/main.cpp -o CMakeFiles/ThrackerZodd.dir/src/main.cpp.s

CMakeFiles/ThrackerZodd.dir/src/main.cpp.o.requires:

.PHONY : CMakeFiles/ThrackerZodd.dir/src/main.cpp.o.requires

CMakeFiles/ThrackerZodd.dir/src/main.cpp.o.provides: CMakeFiles/ThrackerZodd.dir/src/main.cpp.o.requires
	$(MAKE) -f CMakeFiles/ThrackerZodd.dir/build.make CMakeFiles/ThrackerZodd.dir/src/main.cpp.o.provides.build
.PHONY : CMakeFiles/ThrackerZodd.dir/src/main.cpp.o.provides

CMakeFiles/ThrackerZodd.dir/src/main.cpp.o.provides.build: CMakeFiles/ThrackerZodd.dir/src/main.cpp.o


# Object files for target ThrackerZodd
ThrackerZodd_OBJECTS = \
"CMakeFiles/ThrackerZodd.dir/src/main.cpp.o"

# External object files for target ThrackerZodd
ThrackerZodd_EXTERNAL_OBJECTS =

bin/ThrackerZodd: CMakeFiles/ThrackerZodd.dir/src/main.cpp.o
bin/ThrackerZodd: CMakeFiles/ThrackerZodd.dir/build.make
bin/ThrackerZodd: /usr/local/lib/libopencv_videostab.2.4.13.dylib
bin/ThrackerZodd: /usr/local/lib/libopencv_ts.a
bin/ThrackerZodd: /usr/local/lib/libopencv_superres.2.4.13.dylib
bin/ThrackerZodd: /usr/local/lib/libopencv_stitching.2.4.13.dylib
bin/ThrackerZodd: /usr/local/lib/libopencv_contrib.2.4.13.dylib
bin/ThrackerZodd: /usr/local/lib/libopencv_nonfree.2.4.13.dylib
bin/ThrackerZodd: /usr/local/lib/libopencv_ocl.2.4.13.dylib
bin/ThrackerZodd: /usr/local/lib/libopencv_gpu.2.4.13.dylib
bin/ThrackerZodd: /usr/local/lib/libopencv_photo.2.4.13.dylib
bin/ThrackerZodd: /usr/local/lib/libopencv_objdetect.2.4.13.dylib
bin/ThrackerZodd: /usr/local/lib/libopencv_legacy.2.4.13.dylib
bin/ThrackerZodd: /usr/local/lib/libopencv_video.2.4.13.dylib
bin/ThrackerZodd: /usr/local/lib/libopencv_ml.2.4.13.dylib
bin/ThrackerZodd: /usr/local/lib/libopencv_calib3d.2.4.13.dylib
bin/ThrackerZodd: /usr/local/lib/libopencv_features2d.2.4.13.dylib
bin/ThrackerZodd: /usr/local/lib/libopencv_highgui.2.4.13.dylib
bin/ThrackerZodd: /usr/local/lib/libopencv_imgproc.2.4.13.dylib
bin/ThrackerZodd: /usr/local/lib/libopencv_flann.2.4.13.dylib
bin/ThrackerZodd: /usr/local/lib/libopencv_core.2.4.13.dylib
bin/ThrackerZodd: CMakeFiles/ThrackerZodd.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/Users/kiranshila/Desktop/thracker_zod/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable bin/ThrackerZodd"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/ThrackerZodd.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/ThrackerZodd.dir/build: bin/ThrackerZodd

.PHONY : CMakeFiles/ThrackerZodd.dir/build

CMakeFiles/ThrackerZodd.dir/requires: CMakeFiles/ThrackerZodd.dir/src/main.cpp.o.requires

.PHONY : CMakeFiles/ThrackerZodd.dir/requires

CMakeFiles/ThrackerZodd.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/ThrackerZodd.dir/cmake_clean.cmake
.PHONY : CMakeFiles/ThrackerZodd.dir/clean

CMakeFiles/ThrackerZodd.dir/depend:
	cd /Users/kiranshila/Desktop/thracker_zod && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/kiranshila/Desktop/thracker_zod /Users/kiranshila/Desktop/thracker_zod /Users/kiranshila/Desktop/thracker_zod /Users/kiranshila/Desktop/thracker_zod /Users/kiranshila/Desktop/thracker_zod/CMakeFiles/ThrackerZodd.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/ThrackerZodd.dir/depend
