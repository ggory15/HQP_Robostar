# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.5

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


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
CMAKE_SOURCE_DIR = /home/ggory15/Robostar

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/ggory15/Robostar/build

# Utility rule file for distdir.

# Include the progress variables for this target.
include CMakeFiles/distdir.dir/progress.make

CMakeFiles/distdir:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/ggory15/Robostar/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Generating dist directory..."
	cd /home/ggory15/Robostar && rm -f /tmp/HQP.tar && /home/ggory15/Robostar/cmake/git-archive-all.sh --prefix HQP-UNKNOWN-dirty/ HQP.tar && cd /home/ggory15/Robostar/build/ && ( test -d HQP-UNKNOWN-dirty && find HQP-UNKNOWN-dirty/ -type d -print0 | xargs -0 chmod a+w || true ) && rm -rf HQP-UNKNOWN-dirty/ && /bin/tar xf /home/ggory15/Robostar/HQP.tar && echo UNKNOWN-dirty > /home/ggory15/Robostar/build/HQP-UNKNOWN-dirty/.version && /home/ggory15/Robostar/cmake/gitlog-to-changelog > /home/ggory15/Robostar/build/HQP-UNKNOWN-dirty/ChangeLog && rm -f /home/ggory15/Robostar/HQP.tar

distdir: CMakeFiles/distdir
distdir: CMakeFiles/distdir.dir/build.make

.PHONY : distdir

# Rule to build all files generated by this target.
CMakeFiles/distdir.dir/build: distdir

.PHONY : CMakeFiles/distdir.dir/build

CMakeFiles/distdir.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/distdir.dir/cmake_clean.cmake
.PHONY : CMakeFiles/distdir.dir/clean

CMakeFiles/distdir.dir/depend:
	cd /home/ggory15/Robostar/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ggory15/Robostar /home/ggory15/Robostar /home/ggory15/Robostar/build /home/ggory15/Robostar/build /home/ggory15/Robostar/build/CMakeFiles/distdir.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/distdir.dir/depend

