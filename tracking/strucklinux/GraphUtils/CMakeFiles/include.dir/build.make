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
include GraphUtils/CMakeFiles/include.dir/depend.make

# Include the progress variables for this target.
include GraphUtils/CMakeFiles/include.dir/progress.make

# Include the compile flags for this target's objects.
include GraphUtils/CMakeFiles/include.dir/flags.make

GraphUtils/CMakeFiles/include.dir/GraphUtils.cpp.o: GraphUtils/CMakeFiles/include.dir/flags.make
GraphUtils/CMakeFiles/include.dir/GraphUtils.cpp.o: GraphUtils/GraphUtils.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/ncslab/Documents/tracking/strucklinux/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object GraphUtils/CMakeFiles/include.dir/GraphUtils.cpp.o"
	cd /home/ncslab/Documents/tracking/strucklinux/GraphUtils && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/include.dir/GraphUtils.cpp.o -c /home/ncslab/Documents/tracking/strucklinux/GraphUtils/GraphUtils.cpp

GraphUtils/CMakeFiles/include.dir/GraphUtils.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/include.dir/GraphUtils.cpp.i"
	cd /home/ncslab/Documents/tracking/strucklinux/GraphUtils && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/ncslab/Documents/tracking/strucklinux/GraphUtils/GraphUtils.cpp > CMakeFiles/include.dir/GraphUtils.cpp.i

GraphUtils/CMakeFiles/include.dir/GraphUtils.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/include.dir/GraphUtils.cpp.s"
	cd /home/ncslab/Documents/tracking/strucklinux/GraphUtils && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/ncslab/Documents/tracking/strucklinux/GraphUtils/GraphUtils.cpp -o CMakeFiles/include.dir/GraphUtils.cpp.s

GraphUtils/CMakeFiles/include.dir/GraphUtils.cpp.o.requires:
.PHONY : GraphUtils/CMakeFiles/include.dir/GraphUtils.cpp.o.requires

GraphUtils/CMakeFiles/include.dir/GraphUtils.cpp.o.provides: GraphUtils/CMakeFiles/include.dir/GraphUtils.cpp.o.requires
	$(MAKE) -f GraphUtils/CMakeFiles/include.dir/build.make GraphUtils/CMakeFiles/include.dir/GraphUtils.cpp.o.provides.build
.PHONY : GraphUtils/CMakeFiles/include.dir/GraphUtils.cpp.o.provides

GraphUtils/CMakeFiles/include.dir/GraphUtils.cpp.o.provides.build: GraphUtils/CMakeFiles/include.dir/GraphUtils.cpp.o

# Object files for target include
include_OBJECTS = \
"CMakeFiles/include.dir/GraphUtils.cpp.o"

# External object files for target include
include_EXTERNAL_OBJECTS =

GraphUtils/libinclude.a: GraphUtils/CMakeFiles/include.dir/GraphUtils.cpp.o
GraphUtils/libinclude.a: GraphUtils/CMakeFiles/include.dir/build.make
GraphUtils/libinclude.a: GraphUtils/CMakeFiles/include.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX static library libinclude.a"
	cd /home/ncslab/Documents/tracking/strucklinux/GraphUtils && $(CMAKE_COMMAND) -P CMakeFiles/include.dir/cmake_clean_target.cmake
	cd /home/ncslab/Documents/tracking/strucklinux/GraphUtils && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/include.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
GraphUtils/CMakeFiles/include.dir/build: GraphUtils/libinclude.a
.PHONY : GraphUtils/CMakeFiles/include.dir/build

GraphUtils/CMakeFiles/include.dir/requires: GraphUtils/CMakeFiles/include.dir/GraphUtils.cpp.o.requires
.PHONY : GraphUtils/CMakeFiles/include.dir/requires

GraphUtils/CMakeFiles/include.dir/clean:
	cd /home/ncslab/Documents/tracking/strucklinux/GraphUtils && $(CMAKE_COMMAND) -P CMakeFiles/include.dir/cmake_clean.cmake
.PHONY : GraphUtils/CMakeFiles/include.dir/clean

GraphUtils/CMakeFiles/include.dir/depend:
	cd /home/ncslab/Documents/tracking/strucklinux && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ncslab/Documents/tracking/strucklinux /home/ncslab/Documents/tracking/strucklinux/GraphUtils /home/ncslab/Documents/tracking/strucklinux /home/ncslab/Documents/tracking/strucklinux/GraphUtils /home/ncslab/Documents/tracking/strucklinux/GraphUtils/CMakeFiles/include.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : GraphUtils/CMakeFiles/include.dir/depend

