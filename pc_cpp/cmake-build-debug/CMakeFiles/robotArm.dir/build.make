# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.15

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
CMAKE_COMMAND = /home/bab21/.local/share/JetBrains/Toolbox/apps/CLion/ch-0/193.6911.21/bin/cmake/linux/bin/cmake

# The command to remove a file.
RM = /home/bab21/.local/share/JetBrains/Toolbox/apps/CLion/ch-0/193.6911.21/bin/cmake/linux/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/bab21/PycharmProjects/robotArm/pc_cpp

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/bab21/PycharmProjects/robotArm/pc_cpp/cmake-build-debug

# Include any dependencies generated for this target.
include CMakeFiles/robotArm.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/robotArm.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/robotArm.dir/flags.make

CMakeFiles/robotArm.dir/src/main.cpp.o: CMakeFiles/robotArm.dir/flags.make
CMakeFiles/robotArm.dir/src/main.cpp.o: ../src/main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/bab21/PycharmProjects/robotArm/pc_cpp/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/robotArm.dir/src/main.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/robotArm.dir/src/main.cpp.o -c /home/bab21/PycharmProjects/robotArm/pc_cpp/src/main.cpp

CMakeFiles/robotArm.dir/src/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/robotArm.dir/src/main.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/bab21/PycharmProjects/robotArm/pc_cpp/src/main.cpp > CMakeFiles/robotArm.dir/src/main.cpp.i

CMakeFiles/robotArm.dir/src/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/robotArm.dir/src/main.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/bab21/PycharmProjects/robotArm/pc_cpp/src/main.cpp -o CMakeFiles/robotArm.dir/src/main.cpp.s

# Object files for target robotArm
robotArm_OBJECTS = \
"CMakeFiles/robotArm.dir/src/main.cpp.o"

# External object files for target robotArm
robotArm_EXTERNAL_OBJECTS =

robotArm: CMakeFiles/robotArm.dir/src/main.cpp.o
robotArm: CMakeFiles/robotArm.dir/build.make
robotArm: CMakeFiles/robotArm.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/bab21/PycharmProjects/robotArm/pc_cpp/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable robotArm"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/robotArm.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/robotArm.dir/build: robotArm

.PHONY : CMakeFiles/robotArm.dir/build

CMakeFiles/robotArm.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/robotArm.dir/cmake_clean.cmake
.PHONY : CMakeFiles/robotArm.dir/clean

CMakeFiles/robotArm.dir/depend:
	cd /home/bab21/PycharmProjects/robotArm/pc_cpp/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/bab21/PycharmProjects/robotArm/pc_cpp /home/bab21/PycharmProjects/robotArm/pc_cpp /home/bab21/PycharmProjects/robotArm/pc_cpp/cmake-build-debug /home/bab21/PycharmProjects/robotArm/pc_cpp/cmake-build-debug /home/bab21/PycharmProjects/robotArm/pc_cpp/cmake-build-debug/CMakeFiles/robotArm.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/robotArm.dir/depend

