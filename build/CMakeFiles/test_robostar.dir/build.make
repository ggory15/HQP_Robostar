# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list

# Produce verbose output by default.
VERBOSE = 1

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
CMAKE_SOURCE_DIR = /home/dyros/HQP_Robostar

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/dyros/HQP_Robostar/build

# Include any dependencies generated for this target.
include CMakeFiles/test_robostar.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/test_robostar.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/test_robostar.dir/flags.make

CMakeFiles/test_robostar.dir/demo-robostar/demo.cpp.o: CMakeFiles/test_robostar.dir/flags.make
CMakeFiles/test_robostar.dir/demo-robostar/demo.cpp.o: ../demo-robostar/demo.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/dyros/HQP_Robostar/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/test_robostar.dir/demo-robostar/demo.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/test_robostar.dir/demo-robostar/demo.cpp.o -c /home/dyros/HQP_Robostar/demo-robostar/demo.cpp

CMakeFiles/test_robostar.dir/demo-robostar/demo.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/test_robostar.dir/demo-robostar/demo.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/dyros/HQP_Robostar/demo-robostar/demo.cpp > CMakeFiles/test_robostar.dir/demo-robostar/demo.cpp.i

CMakeFiles/test_robostar.dir/demo-robostar/demo.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/test_robostar.dir/demo-robostar/demo.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/dyros/HQP_Robostar/demo-robostar/demo.cpp -o CMakeFiles/test_robostar.dir/demo-robostar/demo.cpp.s

CMakeFiles/test_robostar.dir/demo-robostar/demo.cpp.o.requires:
.PHONY : CMakeFiles/test_robostar.dir/demo-robostar/demo.cpp.o.requires

CMakeFiles/test_robostar.dir/demo-robostar/demo.cpp.o.provides: CMakeFiles/test_robostar.dir/demo-robostar/demo.cpp.o.requires
	$(MAKE) -f CMakeFiles/test_robostar.dir/build.make CMakeFiles/test_robostar.dir/demo-robostar/demo.cpp.o.provides.build
.PHONY : CMakeFiles/test_robostar.dir/demo-robostar/demo.cpp.o.provides

CMakeFiles/test_robostar.dir/demo-robostar/demo.cpp.o.provides.build: CMakeFiles/test_robostar.dir/demo-robostar/demo.cpp.o

CMakeFiles/test_robostar.dir/src/hardware/ethercat_elmo.cpp.o: CMakeFiles/test_robostar.dir/flags.make
CMakeFiles/test_robostar.dir/src/hardware/ethercat_elmo.cpp.o: ../src/hardware/ethercat_elmo.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/dyros/HQP_Robostar/build/CMakeFiles $(CMAKE_PROGRESS_2)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/test_robostar.dir/src/hardware/ethercat_elmo.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/test_robostar.dir/src/hardware/ethercat_elmo.cpp.o -c /home/dyros/HQP_Robostar/src/hardware/ethercat_elmo.cpp

CMakeFiles/test_robostar.dir/src/hardware/ethercat_elmo.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/test_robostar.dir/src/hardware/ethercat_elmo.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/dyros/HQP_Robostar/src/hardware/ethercat_elmo.cpp > CMakeFiles/test_robostar.dir/src/hardware/ethercat_elmo.cpp.i

CMakeFiles/test_robostar.dir/src/hardware/ethercat_elmo.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/test_robostar.dir/src/hardware/ethercat_elmo.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/dyros/HQP_Robostar/src/hardware/ethercat_elmo.cpp -o CMakeFiles/test_robostar.dir/src/hardware/ethercat_elmo.cpp.s

CMakeFiles/test_robostar.dir/src/hardware/ethercat_elmo.cpp.o.requires:
.PHONY : CMakeFiles/test_robostar.dir/src/hardware/ethercat_elmo.cpp.o.requires

CMakeFiles/test_robostar.dir/src/hardware/ethercat_elmo.cpp.o.provides: CMakeFiles/test_robostar.dir/src/hardware/ethercat_elmo.cpp.o.requires
	$(MAKE) -f CMakeFiles/test_robostar.dir/build.make CMakeFiles/test_robostar.dir/src/hardware/ethercat_elmo.cpp.o.provides.build
.PHONY : CMakeFiles/test_robostar.dir/src/hardware/ethercat_elmo.cpp.o.provides

CMakeFiles/test_robostar.dir/src/hardware/ethercat_elmo.cpp.o.provides.build: CMakeFiles/test_robostar.dir/src/hardware/ethercat_elmo.cpp.o

# Object files for target test_robostar
test_robostar_OBJECTS = \
"CMakeFiles/test_robostar.dir/demo-robostar/demo.cpp.o" \
"CMakeFiles/test_robostar.dir/src/hardware/ethercat_elmo.cpp.o"

# External object files for target test_robostar
test_robostar_EXTERNAL_OBJECTS =

test_robostar: CMakeFiles/test_robostar.dir/demo-robostar/demo.cpp.o
test_robostar: CMakeFiles/test_robostar.dir/src/hardware/ethercat_elmo.cpp.o
test_robostar: CMakeFiles/test_robostar.dir/build.make
test_robostar: src/libHQP.so
test_robostar: CMakeFiles/test_robostar.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable test_robostar"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/test_robostar.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/test_robostar.dir/build: test_robostar
.PHONY : CMakeFiles/test_robostar.dir/build

CMakeFiles/test_robostar.dir/requires: CMakeFiles/test_robostar.dir/demo-robostar/demo.cpp.o.requires
CMakeFiles/test_robostar.dir/requires: CMakeFiles/test_robostar.dir/src/hardware/ethercat_elmo.cpp.o.requires
.PHONY : CMakeFiles/test_robostar.dir/requires

CMakeFiles/test_robostar.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/test_robostar.dir/cmake_clean.cmake
.PHONY : CMakeFiles/test_robostar.dir/clean

CMakeFiles/test_robostar.dir/depend:
	cd /home/dyros/HQP_Robostar/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/dyros/HQP_Robostar /home/dyros/HQP_Robostar /home/dyros/HQP_Robostar/build /home/dyros/HQP_Robostar/build /home/dyros/HQP_Robostar/build/CMakeFiles/test_robostar.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/test_robostar.dir/depend

