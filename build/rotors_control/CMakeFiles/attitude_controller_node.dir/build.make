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
CMAKE_SOURCE_DIR = /home/chris/catkin_ws/src/CrazyS/rotors_control

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/chris/catkin_ws/build/rotors_control

# Include any dependencies generated for this target.
include CMakeFiles/attitude_controller_node.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/attitude_controller_node.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/attitude_controller_node.dir/flags.make

CMakeFiles/attitude_controller_node.dir/src/nodes/attitude_controller_node.cpp.o: CMakeFiles/attitude_controller_node.dir/flags.make
CMakeFiles/attitude_controller_node.dir/src/nodes/attitude_controller_node.cpp.o: /home/chris/catkin_ws/src/CrazyS/rotors_control/src/nodes/attitude_controller_node.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/chris/catkin_ws/build/rotors_control/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/attitude_controller_node.dir/src/nodes/attitude_controller_node.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/attitude_controller_node.dir/src/nodes/attitude_controller_node.cpp.o -c /home/chris/catkin_ws/src/CrazyS/rotors_control/src/nodes/attitude_controller_node.cpp

CMakeFiles/attitude_controller_node.dir/src/nodes/attitude_controller_node.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/attitude_controller_node.dir/src/nodes/attitude_controller_node.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/chris/catkin_ws/src/CrazyS/rotors_control/src/nodes/attitude_controller_node.cpp > CMakeFiles/attitude_controller_node.dir/src/nodes/attitude_controller_node.cpp.i

CMakeFiles/attitude_controller_node.dir/src/nodes/attitude_controller_node.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/attitude_controller_node.dir/src/nodes/attitude_controller_node.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/chris/catkin_ws/src/CrazyS/rotors_control/src/nodes/attitude_controller_node.cpp -o CMakeFiles/attitude_controller_node.dir/src/nodes/attitude_controller_node.cpp.s

# Object files for target attitude_controller_node
attitude_controller_node_OBJECTS = \
"CMakeFiles/attitude_controller_node.dir/src/nodes/attitude_controller_node.cpp.o"

# External object files for target attitude_controller_node
attitude_controller_node_EXTERNAL_OBJECTS =

/home/chris/catkin_ws/devel/.private/rotors_control/lib/rotors_control/attitude_controller_node: CMakeFiles/attitude_controller_node.dir/src/nodes/attitude_controller_node.cpp.o
/home/chris/catkin_ws/devel/.private/rotors_control/lib/rotors_control/attitude_controller_node: CMakeFiles/attitude_controller_node.dir/build.make
/home/chris/catkin_ws/devel/.private/rotors_control/lib/rotors_control/attitude_controller_node: /home/chris/catkin_ws/devel/.private/rotors_control/lib/libattitude_controller.so
/home/chris/catkin_ws/devel/.private/rotors_control/lib/rotors_control/attitude_controller_node: /opt/ros/noetic/lib/libroscpp.so
/home/chris/catkin_ws/devel/.private/rotors_control/lib/rotors_control/attitude_controller_node: /usr/lib/aarch64-linux-gnu/libpthread.so
/home/chris/catkin_ws/devel/.private/rotors_control/lib/rotors_control/attitude_controller_node: /usr/lib/aarch64-linux-gnu/libboost_chrono.so.1.71.0
/home/chris/catkin_ws/devel/.private/rotors_control/lib/rotors_control/attitude_controller_node: /usr/lib/aarch64-linux-gnu/libboost_filesystem.so.1.71.0
/home/chris/catkin_ws/devel/.private/rotors_control/lib/rotors_control/attitude_controller_node: /opt/ros/noetic/lib/librosconsole.so
/home/chris/catkin_ws/devel/.private/rotors_control/lib/rotors_control/attitude_controller_node: /opt/ros/noetic/lib/librosconsole_log4cxx.so
/home/chris/catkin_ws/devel/.private/rotors_control/lib/rotors_control/attitude_controller_node: /opt/ros/noetic/lib/librosconsole_backend_interface.so
/home/chris/catkin_ws/devel/.private/rotors_control/lib/rotors_control/attitude_controller_node: /usr/lib/aarch64-linux-gnu/liblog4cxx.so
/home/chris/catkin_ws/devel/.private/rotors_control/lib/rotors_control/attitude_controller_node: /usr/lib/aarch64-linux-gnu/libboost_regex.so.1.71.0
/home/chris/catkin_ws/devel/.private/rotors_control/lib/rotors_control/attitude_controller_node: /opt/ros/noetic/lib/libxmlrpcpp.so
/home/chris/catkin_ws/devel/.private/rotors_control/lib/rotors_control/attitude_controller_node: /opt/ros/noetic/lib/libroscpp_serialization.so
/home/chris/catkin_ws/devel/.private/rotors_control/lib/rotors_control/attitude_controller_node: /opt/ros/noetic/lib/librostime.so
/home/chris/catkin_ws/devel/.private/rotors_control/lib/rotors_control/attitude_controller_node: /usr/lib/aarch64-linux-gnu/libboost_date_time.so.1.71.0
/home/chris/catkin_ws/devel/.private/rotors_control/lib/rotors_control/attitude_controller_node: /opt/ros/noetic/lib/libcpp_common.so
/home/chris/catkin_ws/devel/.private/rotors_control/lib/rotors_control/attitude_controller_node: /usr/lib/aarch64-linux-gnu/libboost_system.so.1.71.0
/home/chris/catkin_ws/devel/.private/rotors_control/lib/rotors_control/attitude_controller_node: /usr/lib/aarch64-linux-gnu/libboost_thread.so.1.71.0
/home/chris/catkin_ws/devel/.private/rotors_control/lib/rotors_control/attitude_controller_node: /usr/lib/aarch64-linux-gnu/libconsole_bridge.so.0.4
/home/chris/catkin_ws/devel/.private/rotors_control/lib/rotors_control/attitude_controller_node: CMakeFiles/attitude_controller_node.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/chris/catkin_ws/build/rotors_control/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable /home/chris/catkin_ws/devel/.private/rotors_control/lib/rotors_control/attitude_controller_node"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/attitude_controller_node.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/attitude_controller_node.dir/build: /home/chris/catkin_ws/devel/.private/rotors_control/lib/rotors_control/attitude_controller_node

.PHONY : CMakeFiles/attitude_controller_node.dir/build

CMakeFiles/attitude_controller_node.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/attitude_controller_node.dir/cmake_clean.cmake
.PHONY : CMakeFiles/attitude_controller_node.dir/clean

CMakeFiles/attitude_controller_node.dir/depend:
	cd /home/chris/catkin_ws/build/rotors_control && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/chris/catkin_ws/src/CrazyS/rotors_control /home/chris/catkin_ws/src/CrazyS/rotors_control /home/chris/catkin_ws/build/rotors_control /home/chris/catkin_ws/build/rotors_control /home/chris/catkin_ws/build/rotors_control/CMakeFiles/attitude_controller_node.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/attitude_controller_node.dir/depend

