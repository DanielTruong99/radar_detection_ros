# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

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
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/ryz2/catkin_ws/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/ryz2/catkin_ws/build

# Include any dependencies generated for this target.
include libtorch_demo/src/ColisionDetector/CMakeFiles/ColisionDetector.dir/depend.make

# Include the progress variables for this target.
include libtorch_demo/src/ColisionDetector/CMakeFiles/ColisionDetector.dir/progress.make

# Include the compile flags for this target's objects.
include libtorch_demo/src/ColisionDetector/CMakeFiles/ColisionDetector.dir/flags.make

libtorch_demo/src/ColisionDetector/CMakeFiles/ColisionDetector.dir/ColisionDetector.cpp.o: libtorch_demo/src/ColisionDetector/CMakeFiles/ColisionDetector.dir/flags.make
libtorch_demo/src/ColisionDetector/CMakeFiles/ColisionDetector.dir/ColisionDetector.cpp.o: /home/ryz2/catkin_ws/src/libtorch_demo/src/ColisionDetector/ColisionDetector.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ryz2/catkin_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object libtorch_demo/src/ColisionDetector/CMakeFiles/ColisionDetector.dir/ColisionDetector.cpp.o"
	cd /home/ryz2/catkin_ws/build/libtorch_demo/src/ColisionDetector && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/ColisionDetector.dir/ColisionDetector.cpp.o -c /home/ryz2/catkin_ws/src/libtorch_demo/src/ColisionDetector/ColisionDetector.cpp

libtorch_demo/src/ColisionDetector/CMakeFiles/ColisionDetector.dir/ColisionDetector.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ColisionDetector.dir/ColisionDetector.cpp.i"
	cd /home/ryz2/catkin_ws/build/libtorch_demo/src/ColisionDetector && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ryz2/catkin_ws/src/libtorch_demo/src/ColisionDetector/ColisionDetector.cpp > CMakeFiles/ColisionDetector.dir/ColisionDetector.cpp.i

libtorch_demo/src/ColisionDetector/CMakeFiles/ColisionDetector.dir/ColisionDetector.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ColisionDetector.dir/ColisionDetector.cpp.s"
	cd /home/ryz2/catkin_ws/build/libtorch_demo/src/ColisionDetector && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ryz2/catkin_ws/src/libtorch_demo/src/ColisionDetector/ColisionDetector.cpp -o CMakeFiles/ColisionDetector.dir/ColisionDetector.cpp.s

# Object files for target ColisionDetector
ColisionDetector_OBJECTS = \
"CMakeFiles/ColisionDetector.dir/ColisionDetector.cpp.o"

# External object files for target ColisionDetector
ColisionDetector_EXTERNAL_OBJECTS =

/home/ryz2/catkin_ws/devel/lib/libColisionDetector.a: libtorch_demo/src/ColisionDetector/CMakeFiles/ColisionDetector.dir/ColisionDetector.cpp.o
/home/ryz2/catkin_ws/devel/lib/libColisionDetector.a: libtorch_demo/src/ColisionDetector/CMakeFiles/ColisionDetector.dir/build.make
/home/ryz2/catkin_ws/devel/lib/libColisionDetector.a: libtorch_demo/src/ColisionDetector/CMakeFiles/ColisionDetector.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/ryz2/catkin_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library /home/ryz2/catkin_ws/devel/lib/libColisionDetector.a"
	cd /home/ryz2/catkin_ws/build/libtorch_demo/src/ColisionDetector && $(CMAKE_COMMAND) -P CMakeFiles/ColisionDetector.dir/cmake_clean_target.cmake
	cd /home/ryz2/catkin_ws/build/libtorch_demo/src/ColisionDetector && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/ColisionDetector.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
libtorch_demo/src/ColisionDetector/CMakeFiles/ColisionDetector.dir/build: /home/ryz2/catkin_ws/devel/lib/libColisionDetector.a

.PHONY : libtorch_demo/src/ColisionDetector/CMakeFiles/ColisionDetector.dir/build

libtorch_demo/src/ColisionDetector/CMakeFiles/ColisionDetector.dir/clean:
	cd /home/ryz2/catkin_ws/build/libtorch_demo/src/ColisionDetector && $(CMAKE_COMMAND) -P CMakeFiles/ColisionDetector.dir/cmake_clean.cmake
.PHONY : libtorch_demo/src/ColisionDetector/CMakeFiles/ColisionDetector.dir/clean

libtorch_demo/src/ColisionDetector/CMakeFiles/ColisionDetector.dir/depend:
	cd /home/ryz2/catkin_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ryz2/catkin_ws/src /home/ryz2/catkin_ws/src/libtorch_demo/src/ColisionDetector /home/ryz2/catkin_ws/build /home/ryz2/catkin_ws/build/libtorch_demo/src/ColisionDetector /home/ryz2/catkin_ws/build/libtorch_demo/src/ColisionDetector/CMakeFiles/ColisionDetector.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : libtorch_demo/src/ColisionDetector/CMakeFiles/ColisionDetector.dir/depend

