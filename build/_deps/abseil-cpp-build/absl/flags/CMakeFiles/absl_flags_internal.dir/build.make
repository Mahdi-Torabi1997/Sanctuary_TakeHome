# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.22

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
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
RM = /usr/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/mato/cpp_projects/Sanctuary

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/mato/cpp_projects/Sanctuary/build

# Include any dependencies generated for this target.
include _deps/abseil-cpp-build/absl/flags/CMakeFiles/absl_flags_internal.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include _deps/abseil-cpp-build/absl/flags/CMakeFiles/absl_flags_internal.dir/compiler_depend.make

# Include the progress variables for this target.
include _deps/abseil-cpp-build/absl/flags/CMakeFiles/absl_flags_internal.dir/progress.make

# Include the compile flags for this target's objects.
include _deps/abseil-cpp-build/absl/flags/CMakeFiles/absl_flags_internal.dir/flags.make

_deps/abseil-cpp-build/absl/flags/CMakeFiles/absl_flags_internal.dir/internal/flag.cc.o: _deps/abseil-cpp-build/absl/flags/CMakeFiles/absl_flags_internal.dir/flags.make
_deps/abseil-cpp-build/absl/flags/CMakeFiles/absl_flags_internal.dir/internal/flag.cc.o: _deps/abseil-cpp-src/absl/flags/internal/flag.cc
_deps/abseil-cpp-build/absl/flags/CMakeFiles/absl_flags_internal.dir/internal/flag.cc.o: _deps/abseil-cpp-build/absl/flags/CMakeFiles/absl_flags_internal.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/mato/cpp_projects/Sanctuary/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object _deps/abseil-cpp-build/absl/flags/CMakeFiles/absl_flags_internal.dir/internal/flag.cc.o"
	cd /home/mato/cpp_projects/Sanctuary/build/_deps/abseil-cpp-build/absl/flags && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT _deps/abseil-cpp-build/absl/flags/CMakeFiles/absl_flags_internal.dir/internal/flag.cc.o -MF CMakeFiles/absl_flags_internal.dir/internal/flag.cc.o.d -o CMakeFiles/absl_flags_internal.dir/internal/flag.cc.o -c /home/mato/cpp_projects/Sanctuary/build/_deps/abseil-cpp-src/absl/flags/internal/flag.cc

_deps/abseil-cpp-build/absl/flags/CMakeFiles/absl_flags_internal.dir/internal/flag.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/absl_flags_internal.dir/internal/flag.cc.i"
	cd /home/mato/cpp_projects/Sanctuary/build/_deps/abseil-cpp-build/absl/flags && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/mato/cpp_projects/Sanctuary/build/_deps/abseil-cpp-src/absl/flags/internal/flag.cc > CMakeFiles/absl_flags_internal.dir/internal/flag.cc.i

_deps/abseil-cpp-build/absl/flags/CMakeFiles/absl_flags_internal.dir/internal/flag.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/absl_flags_internal.dir/internal/flag.cc.s"
	cd /home/mato/cpp_projects/Sanctuary/build/_deps/abseil-cpp-build/absl/flags && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/mato/cpp_projects/Sanctuary/build/_deps/abseil-cpp-src/absl/flags/internal/flag.cc -o CMakeFiles/absl_flags_internal.dir/internal/flag.cc.s

# Object files for target absl_flags_internal
absl_flags_internal_OBJECTS = \
"CMakeFiles/absl_flags_internal.dir/internal/flag.cc.o"

# External object files for target absl_flags_internal
absl_flags_internal_EXTERNAL_OBJECTS =

_deps/abseil-cpp-build/absl/flags/libabsl_flags_internal.a: _deps/abseil-cpp-build/absl/flags/CMakeFiles/absl_flags_internal.dir/internal/flag.cc.o
_deps/abseil-cpp-build/absl/flags/libabsl_flags_internal.a: _deps/abseil-cpp-build/absl/flags/CMakeFiles/absl_flags_internal.dir/build.make
_deps/abseil-cpp-build/absl/flags/libabsl_flags_internal.a: _deps/abseil-cpp-build/absl/flags/CMakeFiles/absl_flags_internal.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/mato/cpp_projects/Sanctuary/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library libabsl_flags_internal.a"
	cd /home/mato/cpp_projects/Sanctuary/build/_deps/abseil-cpp-build/absl/flags && $(CMAKE_COMMAND) -P CMakeFiles/absl_flags_internal.dir/cmake_clean_target.cmake
	cd /home/mato/cpp_projects/Sanctuary/build/_deps/abseil-cpp-build/absl/flags && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/absl_flags_internal.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
_deps/abseil-cpp-build/absl/flags/CMakeFiles/absl_flags_internal.dir/build: _deps/abseil-cpp-build/absl/flags/libabsl_flags_internal.a
.PHONY : _deps/abseil-cpp-build/absl/flags/CMakeFiles/absl_flags_internal.dir/build

_deps/abseil-cpp-build/absl/flags/CMakeFiles/absl_flags_internal.dir/clean:
	cd /home/mato/cpp_projects/Sanctuary/build/_deps/abseil-cpp-build/absl/flags && $(CMAKE_COMMAND) -P CMakeFiles/absl_flags_internal.dir/cmake_clean.cmake
.PHONY : _deps/abseil-cpp-build/absl/flags/CMakeFiles/absl_flags_internal.dir/clean

_deps/abseil-cpp-build/absl/flags/CMakeFiles/absl_flags_internal.dir/depend:
	cd /home/mato/cpp_projects/Sanctuary/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/mato/cpp_projects/Sanctuary /home/mato/cpp_projects/Sanctuary/build/_deps/abseil-cpp-src/absl/flags /home/mato/cpp_projects/Sanctuary/build /home/mato/cpp_projects/Sanctuary/build/_deps/abseil-cpp-build/absl/flags /home/mato/cpp_projects/Sanctuary/build/_deps/abseil-cpp-build/absl/flags/CMakeFiles/absl_flags_internal.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : _deps/abseil-cpp-build/absl/flags/CMakeFiles/absl_flags_internal.dir/depend

