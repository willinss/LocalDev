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

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/ncslab/Documents/tracking/strucklinux

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/ncslab/Documents/tracking/strucklinux

# Include any dependencies generated for this target.
include GraphUtils/CMakeFiles/GraphUtils.dir/depend.make

# Include the progress variables for this target.
include GraphUtils/CMakeFiles/GraphUtils.dir/progress.make

# Include the compile flags for this target's objects.
include GraphUtils/CMakeFiles/GraphUtils.dir/flags.make

GraphUtils/CMakeFiles/GraphUtils.dir/GraphUtils.cpp.o: GraphUtils/CMakeFiles/GraphUtils.dir/flags.make
GraphUtils/CMakeFiles/GraphUtils.dir/GraphUtils.cpp.o: GraphUtils/GraphUtils.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/ncslab/Documents/tracking/strucklinux/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object GraphUtils/CMakeFiles/GraphUtils.dir/GraphUtils.cpp.o"
	cd /home/ncslab/Documents/tracking/strucklinux/GraphUtils && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/GraphUtils.dir/GraphUtils.cpp.o -c /home/ncslab/Documents/tracking/strucklinux/GraphUtils/GraphUtils.cpp

GraphUtils/CMakeFiles/GraphUtils.dir/GraphUtils.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/GraphUtils.dir/GraphUtils.cpp.i"
	cd /home/ncslab/Documents/tracking/strucklinux/GraphUtils && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/ncslab/Documents/tracking/strucklinux/GraphUtils/GraphUtils.cpp > CMakeFiles/GraphUtils.dir/GraphUtils.cpp.i

GraphUtils/CMakeFiles/GraphUtils.dir/GraphUtils.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/GraphUtils.dir/GraphUtils.cpp.s"
	cd /home/ncslab/Documents/tracking/strucklinux/GraphUtils && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/ncslab/Documents/tracking/strucklinux/GraphUtils/GraphUtils.cpp -o CMakeFiles/GraphUtils.dir/GraphUtils.cpp.s

GraphUtils/CMakeFiles/GraphUtils.dir/GraphUtils.cpp.o.requires:
.PHONY : GraphUtils/CMakeFiles/GraphUtils.dir/GraphUtils.cpp.o.requires

GraphUtils/CMakeFiles/GraphUtils.dir/GraphUtils.cpp.o.provides: GraphUtils/CMakeFiles/GraphUtils.dir/GraphUtils.cpp.o.requires
	$(MAKE) -f GraphUtils/CMakeFiles/GraphUtils.dir/build.make GraphUtils/CMakeFiles/GraphUtils.dir/GraphUtils.cpp.o.provides.build
.PHONY : GraphUtils/CMakeFiles/GraphUtils.dir/GraphUtils.cpp.o.provides

GraphUtils/CMakeFiles/GraphUtils.dir/GraphUtils.cpp.o.provides.build: GraphUtils/CMakeFiles/GraphUtils.dir/GraphUtils.cpp.o

# Object files for target GraphUtils
GraphUtils_OBJECTS = \
"CMakeFiles/GraphUtils.dir/GraphUtils.cpp.o"

# External object files for target GraphUtils
GraphUtils_EXTERNAL_OBJECTS =

GraphUtils/libGraphUtils.a: GraphUtils/CMakeFiles/GraphUtils.dir/GraphUtils.cpp.o
GraphUtils/libGraphUtils.a: GraphUtils/CMakeFiles/GraphUtils.dir/build.make
GraphUtils/libGraphUtils.a: GraphUtils/CMakeFiles/GraphUtils.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX static library libGraphUtils.a"
	cd /home/ncslab/Documents/tracking/strucklinux/GraphUtils && $(CMAKE_COMMAND) -P CMakeFiles/GraphUtils.dir/cmake_clean_target.cmake
	cd /home/ncslab/Documents/tracking/strucklinux/GraphUtils && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/GraphUtils.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
GraphUtils/CMakeFiles/GraphUtils.dir/build: GraphUtils/libGraphUtils.a
.PHONY : GraphUtils/CMakeFiles/GraphUtils.dir/build

GraphUtils/CMakeFiles/GraphUtils.dir/requires: GraphUtils/CMakeFiles/GraphUtils.dir/GraphUtils.cpp.o.requires
.PHONY : GraphUtils/CMakeFiles/GraphUtils.dir/requires

GraphUtils/CMakeFiles/GraphUtils.dir/clean:
	cd /home/ncslab/Documents/tracking/strucklinux/GraphUtils && $(CMAKE_COMMAND) -P CMakeFiles/GraphUtils.dir/cmake_clean.cmake
.PHONY : GraphUtils/CMakeFiles/GraphUtils.dir/clean

GraphUtils/CMakeFiles/GraphUtils.dir/depend:
	cd /home/ncslab/Documents/tracking/strucklinux && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ncslab/Documents/tracking/strucklinux /home/ncslab/Documents/tracking/strucklinux/GraphUtils /home/ncslab/Documents/tracking/strucklinux /home/ncslab/Documents/tracking/strucklinux/GraphUtils /home/ncslab/Documents/tracking/strucklinux/GraphUtils/CMakeFiles/GraphUtils.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : GraphUtils/CMakeFiles/GraphUtils.dir/depend

