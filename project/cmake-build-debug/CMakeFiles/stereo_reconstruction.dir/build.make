# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.13

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
CMAKE_COMMAND = /opt/clion-2018.3.1/bin/cmake/linux/bin/cmake

# The command to remove a file.
RM = /opt/clion-2018.3.1/bin/cmake/linux/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = "/home/wayl/3D Scanning and Motion Capture/Project/StereoReconstruction/project"

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = "/home/wayl/3D Scanning and Motion Capture/Project/StereoReconstruction/project/cmake-build-debug"

# Include any dependencies generated for this target.
include CMakeFiles/stereo_reconstruction.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/stereo_reconstruction.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/stereo_reconstruction.dir/flags.make

CMakeFiles/stereo_reconstruction.dir/main.cpp.o: CMakeFiles/stereo_reconstruction.dir/flags.make
CMakeFiles/stereo_reconstruction.dir/main.cpp.o: ../main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir="/home/wayl/3D Scanning and Motion Capture/Project/StereoReconstruction/project/cmake-build-debug/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/stereo_reconstruction.dir/main.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/stereo_reconstruction.dir/main.cpp.o -c "/home/wayl/3D Scanning and Motion Capture/Project/StereoReconstruction/project/main.cpp"

CMakeFiles/stereo_reconstruction.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/stereo_reconstruction.dir/main.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E "/home/wayl/3D Scanning and Motion Capture/Project/StereoReconstruction/project/main.cpp" > CMakeFiles/stereo_reconstruction.dir/main.cpp.i

CMakeFiles/stereo_reconstruction.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/stereo_reconstruction.dir/main.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S "/home/wayl/3D Scanning and Motion Capture/Project/StereoReconstruction/project/main.cpp" -o CMakeFiles/stereo_reconstruction.dir/main.cpp.s

# Object files for target stereo_reconstruction
stereo_reconstruction_OBJECTS = \
"CMakeFiles/stereo_reconstruction.dir/main.cpp.o"

# External object files for target stereo_reconstruction
stereo_reconstruction_EXTERNAL_OBJECTS =

stereo_reconstruction: CMakeFiles/stereo_reconstruction.dir/main.cpp.o
stereo_reconstruction: CMakeFiles/stereo_reconstruction.dir/build.make
stereo_reconstruction: /usr/local/lib/libceres.a
stereo_reconstruction: /usr/lib/x86_64-linux-gnu/libglog.so
stereo_reconstruction: /usr/lib/x86_64-linux-gnu/libgflags.so.2.2.1
stereo_reconstruction: /usr/lib/x86_64-linux-gnu/libspqr.so
stereo_reconstruction: /usr/lib/x86_64-linux-gnu/libcholmod.so
stereo_reconstruction: /usr/lib/x86_64-linux-gnu/libccolamd.so
stereo_reconstruction: /usr/lib/x86_64-linux-gnu/libcamd.so
stereo_reconstruction: /usr/lib/x86_64-linux-gnu/libcolamd.so
stereo_reconstruction: /usr/lib/x86_64-linux-gnu/libamd.so
stereo_reconstruction: /usr/lib/x86_64-linux-gnu/liblapack.so
stereo_reconstruction: /usr/lib/x86_64-linux-gnu/libf77blas.so
stereo_reconstruction: /usr/lib/x86_64-linux-gnu/libatlas.so
stereo_reconstruction: /usr/lib/x86_64-linux-gnu/libsuitesparseconfig.so
stereo_reconstruction: /usr/lib/x86_64-linux-gnu/librt.so
stereo_reconstruction: /usr/lib/x86_64-linux-gnu/libcxsparse.so
stereo_reconstruction: /usr/lib/x86_64-linux-gnu/liblapack.so
stereo_reconstruction: /usr/lib/x86_64-linux-gnu/libf77blas.so
stereo_reconstruction: /usr/lib/x86_64-linux-gnu/libatlas.so
stereo_reconstruction: /usr/lib/x86_64-linux-gnu/libsuitesparseconfig.so
stereo_reconstruction: /usr/lib/x86_64-linux-gnu/librt.so
stereo_reconstruction: /usr/lib/x86_64-linux-gnu/libcxsparse.so
stereo_reconstruction: CMakeFiles/stereo_reconstruction.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir="/home/wayl/3D Scanning and Motion Capture/Project/StereoReconstruction/project/cmake-build-debug/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable stereo_reconstruction"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/stereo_reconstruction.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/stereo_reconstruction.dir/build: stereo_reconstruction

.PHONY : CMakeFiles/stereo_reconstruction.dir/build

CMakeFiles/stereo_reconstruction.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/stereo_reconstruction.dir/cmake_clean.cmake
.PHONY : CMakeFiles/stereo_reconstruction.dir/clean

CMakeFiles/stereo_reconstruction.dir/depend:
	cd "/home/wayl/3D Scanning and Motion Capture/Project/StereoReconstruction/project/cmake-build-debug" && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" "/home/wayl/3D Scanning and Motion Capture/Project/StereoReconstruction/project" "/home/wayl/3D Scanning and Motion Capture/Project/StereoReconstruction/project" "/home/wayl/3D Scanning and Motion Capture/Project/StereoReconstruction/project/cmake-build-debug" "/home/wayl/3D Scanning and Motion Capture/Project/StereoReconstruction/project/cmake-build-debug" "/home/wayl/3D Scanning and Motion Capture/Project/StereoReconstruction/project/cmake-build-debug/CMakeFiles/stereo_reconstruction.dir/DependInfo.cmake" --color=$(COLOR)
.PHONY : CMakeFiles/stereo_reconstruction.dir/depend

