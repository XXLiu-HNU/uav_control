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
CMAKE_SOURCE_DIR = /home/xingxun/uav_control/src/ego_planner

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/xingxun/uav_control/build/ego_planner

# Include any dependencies generated for this target.
include CMakeFiles/ego_planner.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/ego_planner.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/ego_planner.dir/flags.make

CMakeFiles/ego_planner.dir/src/planning_visualization.cpp.o: CMakeFiles/ego_planner.dir/flags.make
CMakeFiles/ego_planner.dir/src/planning_visualization.cpp.o: /home/xingxun/uav_control/src/ego_planner/src/planning_visualization.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/xingxun/uav_control/build/ego_planner/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/ego_planner.dir/src/planning_visualization.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/ego_planner.dir/src/planning_visualization.cpp.o -c /home/xingxun/uav_control/src/ego_planner/src/planning_visualization.cpp

CMakeFiles/ego_planner.dir/src/planning_visualization.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ego_planner.dir/src/planning_visualization.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/xingxun/uav_control/src/ego_planner/src/planning_visualization.cpp > CMakeFiles/ego_planner.dir/src/planning_visualization.cpp.i

CMakeFiles/ego_planner.dir/src/planning_visualization.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ego_planner.dir/src/planning_visualization.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/xingxun/uav_control/src/ego_planner/src/planning_visualization.cpp -o CMakeFiles/ego_planner.dir/src/planning_visualization.cpp.s

CMakeFiles/ego_planner.dir/src/polynomial_traj.cpp.o: CMakeFiles/ego_planner.dir/flags.make
CMakeFiles/ego_planner.dir/src/polynomial_traj.cpp.o: /home/xingxun/uav_control/src/ego_planner/src/polynomial_traj.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/xingxun/uav_control/build/ego_planner/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/ego_planner.dir/src/polynomial_traj.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/ego_planner.dir/src/polynomial_traj.cpp.o -c /home/xingxun/uav_control/src/ego_planner/src/polynomial_traj.cpp

CMakeFiles/ego_planner.dir/src/polynomial_traj.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ego_planner.dir/src/polynomial_traj.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/xingxun/uav_control/src/ego_planner/src/polynomial_traj.cpp > CMakeFiles/ego_planner.dir/src/polynomial_traj.cpp.i

CMakeFiles/ego_planner.dir/src/polynomial_traj.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ego_planner.dir/src/polynomial_traj.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/xingxun/uav_control/src/ego_planner/src/polynomial_traj.cpp -o CMakeFiles/ego_planner.dir/src/polynomial_traj.cpp.s

# Object files for target ego_planner
ego_planner_OBJECTS = \
"CMakeFiles/ego_planner.dir/src/planning_visualization.cpp.o" \
"CMakeFiles/ego_planner.dir/src/polynomial_traj.cpp.o"

# External object files for target ego_planner
ego_planner_EXTERNAL_OBJECTS =

/home/xingxun/uav_control/devel/.private/ego_planner/lib/libego_planner.so: CMakeFiles/ego_planner.dir/src/planning_visualization.cpp.o
/home/xingxun/uav_control/devel/.private/ego_planner/lib/libego_planner.so: CMakeFiles/ego_planner.dir/src/polynomial_traj.cpp.o
/home/xingxun/uav_control/devel/.private/ego_planner/lib/libego_planner.so: CMakeFiles/ego_planner.dir/build.make
/home/xingxun/uav_control/devel/.private/ego_planner/lib/libego_planner.so: /opt/ros/noetic/lib/libroscpp.so
/home/xingxun/uav_control/devel/.private/ego_planner/lib/libego_planner.so: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/xingxun/uav_control/devel/.private/ego_planner/lib/libego_planner.so: /usr/lib/x86_64-linux-gnu/libboost_chrono.so.1.71.0
/home/xingxun/uav_control/devel/.private/ego_planner/lib/libego_planner.so: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so.1.71.0
/home/xingxun/uav_control/devel/.private/ego_planner/lib/libego_planner.so: /opt/ros/noetic/lib/librosconsole.so
/home/xingxun/uav_control/devel/.private/ego_planner/lib/libego_planner.so: /opt/ros/noetic/lib/librosconsole_log4cxx.so
/home/xingxun/uav_control/devel/.private/ego_planner/lib/libego_planner.so: /opt/ros/noetic/lib/librosconsole_backend_interface.so
/home/xingxun/uav_control/devel/.private/ego_planner/lib/libego_planner.so: /usr/lib/x86_64-linux-gnu/liblog4cxx.so
/home/xingxun/uav_control/devel/.private/ego_planner/lib/libego_planner.so: /usr/lib/x86_64-linux-gnu/libboost_regex.so.1.71.0
/home/xingxun/uav_control/devel/.private/ego_planner/lib/libego_planner.so: /opt/ros/noetic/lib/libxmlrpcpp.so
/home/xingxun/uav_control/devel/.private/ego_planner/lib/libego_planner.so: /opt/ros/noetic/lib/libroscpp_serialization.so
/home/xingxun/uav_control/devel/.private/ego_planner/lib/libego_planner.so: /opt/ros/noetic/lib/librostime.so
/home/xingxun/uav_control/devel/.private/ego_planner/lib/libego_planner.so: /usr/lib/x86_64-linux-gnu/libboost_date_time.so.1.71.0
/home/xingxun/uav_control/devel/.private/ego_planner/lib/libego_planner.so: /opt/ros/noetic/lib/libcpp_common.so
/home/xingxun/uav_control/devel/.private/ego_planner/lib/libego_planner.so: /usr/lib/x86_64-linux-gnu/libboost_system.so.1.71.0
/home/xingxun/uav_control/devel/.private/ego_planner/lib/libego_planner.so: /usr/lib/x86_64-linux-gnu/libboost_thread.so.1.71.0
/home/xingxun/uav_control/devel/.private/ego_planner/lib/libego_planner.so: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so.0.4
/home/xingxun/uav_control/devel/.private/ego_planner/lib/libego_planner.so: CMakeFiles/ego_planner.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/xingxun/uav_control/build/ego_planner/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX shared library /home/xingxun/uav_control/devel/.private/ego_planner/lib/libego_planner.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/ego_planner.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/ego_planner.dir/build: /home/xingxun/uav_control/devel/.private/ego_planner/lib/libego_planner.so

.PHONY : CMakeFiles/ego_planner.dir/build

CMakeFiles/ego_planner.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/ego_planner.dir/cmake_clean.cmake
.PHONY : CMakeFiles/ego_planner.dir/clean

CMakeFiles/ego_planner.dir/depend:
	cd /home/xingxun/uav_control/build/ego_planner && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/xingxun/uav_control/src/ego_planner /home/xingxun/uav_control/src/ego_planner /home/xingxun/uav_control/build/ego_planner /home/xingxun/uav_control/build/ego_planner /home/xingxun/uav_control/build/ego_planner/CMakeFiles/ego_planner.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/ego_planner.dir/depend
