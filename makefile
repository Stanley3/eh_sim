# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.9

# Default target executed when no arguments are given to make.
default_target: all

.PHONY : default_target

# Allow only one "make -f Makefile2" at a time, but pass parallelism.
.NOTPARALLEL:


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
CMAKE_COMMAND = /usr/local/Cellar/cmake/3.9.4/bin/cmake

# The command to remove a file.
RM = /usr/local/Cellar/cmake/3.9.4/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /Users/jo/Documents/workspace/eh_sim

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /Users/jo/Documents/workspace/eh_sim

#=============================================================================
# Targets provided globally by CMake.

# Special rule for the target rebuild_cache
rebuild_cache:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --cyan "Running CMake to regenerate build system..."
	/usr/local/Cellar/cmake/3.9.4/bin/cmake -H$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR)
.PHONY : rebuild_cache

# Special rule for the target rebuild_cache
rebuild_cache/fast: rebuild_cache

.PHONY : rebuild_cache/fast

# Special rule for the target edit_cache
edit_cache:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --cyan "Running CMake cache editor..."
	/usr/local/Cellar/cmake/3.9.4/bin/ccmake -H$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR)
.PHONY : edit_cache

# Special rule for the target edit_cache
edit_cache/fast: edit_cache

.PHONY : edit_cache/fast

# The main all target
all: cmake_check_build_system
	$(CMAKE_COMMAND) -E cmake_progress_start /Users/jo/Documents/workspace/eh_sim/CMakeFiles /Users/jo/Documents/workspace/eh_sim/CMakeFiles/progress.marks
	$(MAKE) -f CMakeFiles/Makefile2 all
	$(CMAKE_COMMAND) -E cmake_progress_start /Users/jo/Documents/workspace/eh_sim/CMakeFiles 0
.PHONY : all

# The main clean target
clean:
	$(MAKE) -f CMakeFiles/Makefile2 clean
.PHONY : clean

# The main clean target
clean/fast: clean

.PHONY : clean/fast

# Prepare targets for installation.
preinstall: all
	$(MAKE) -f CMakeFiles/Makefile2 preinstall
.PHONY : preinstall

# Prepare targets for installation.
preinstall/fast:
	$(MAKE) -f CMakeFiles/Makefile2 preinstall
.PHONY : preinstall/fast

# clear depends
depend:
	$(CMAKE_COMMAND) -H$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR) --check-build-system CMakeFiles/Makefile.cmake 1
.PHONY : depend

#=============================================================================
# Target rules for targets named main

# Build rule for target.
main: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 main
.PHONY : main

# fast build rule for target.
main/fast:
	$(MAKE) -f CMakeFiles/main.dir/build.make CMakeFiles/main.dir/build
.PHONY : main/fast

#=============================================================================
# Target rules for targets named UTILS

# Build rule for target.
UTILS: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 UTILS
.PHONY : UTILS

# fast build rule for target.
UTILS/fast:
	$(MAKE) -f utils/CMakeFiles/UTILS.dir/build.make utils/CMakeFiles/UTILS.dir/build
.PHONY : UTILS/fast

gc_multi.o: gc_multi.cpp.o

.PHONY : gc_multi.o

# target to build an object file
gc_multi.cpp.o:
	$(MAKE) -f CMakeFiles/main.dir/build.make CMakeFiles/main.dir/gc_multi.cpp.o
.PHONY : gc_multi.cpp.o

gc_multi.i: gc_multi.cpp.i

.PHONY : gc_multi.i

# target to preprocess a source file
gc_multi.cpp.i:
	$(MAKE) -f CMakeFiles/main.dir/build.make CMakeFiles/main.dir/gc_multi.cpp.i
.PHONY : gc_multi.cpp.i

gc_multi.s: gc_multi.cpp.s

.PHONY : gc_multi.s

# target to generate assembly for a file
gc_multi.cpp.s:
	$(MAKE) -f CMakeFiles/main.dir/build.make CMakeFiles/main.dir/gc_multi.cpp.s
.PHONY : gc_multi.cpp.s

main.o: main.cpp.o

.PHONY : main.o

# target to build an object file
main.cpp.o:
	$(MAKE) -f CMakeFiles/main.dir/build.make CMakeFiles/main.dir/main.cpp.o
.PHONY : main.cpp.o

main.i: main.cpp.i

.PHONY : main.i

# target to preprocess a source file
main.cpp.i:
	$(MAKE) -f CMakeFiles/main.dir/build.make CMakeFiles/main.dir/main.cpp.i
.PHONY : main.cpp.i

main.s: main.cpp.s

.PHONY : main.s

# target to generate assembly for a file
main.cpp.s:
	$(MAKE) -f CMakeFiles/main.dir/build.make CMakeFiles/main.dir/main.cpp.s
.PHONY : main.cpp.s

test_m.o: test_m.cpp.o

.PHONY : test_m.o

# target to build an object file
test_m.cpp.o:
	$(MAKE) -f CMakeFiles/main.dir/build.make CMakeFiles/main.dir/test_m.cpp.o
.PHONY : test_m.cpp.o

test_m.i: test_m.cpp.i

.PHONY : test_m.i

# target to preprocess a source file
test_m.cpp.i:
	$(MAKE) -f CMakeFiles/main.dir/build.make CMakeFiles/main.dir/test_m.cpp.i
.PHONY : test_m.cpp.i

test_m.s: test_m.cpp.s

.PHONY : test_m.s

# target to generate assembly for a file
test_m.cpp.s:
	$(MAKE) -f CMakeFiles/main.dir/build.make CMakeFiles/main.dir/test_m.cpp.s
.PHONY : test_m.cpp.s

visual_template_match.o: visual_template_match.cpp.o

.PHONY : visual_template_match.o

# target to build an object file
visual_template_match.cpp.o:
	$(MAKE) -f CMakeFiles/main.dir/build.make CMakeFiles/main.dir/visual_template_match.cpp.o
.PHONY : visual_template_match.cpp.o

visual_template_match.i: visual_template_match.cpp.i

.PHONY : visual_template_match.i

# target to preprocess a source file
visual_template_match.cpp.i:
	$(MAKE) -f CMakeFiles/main.dir/build.make CMakeFiles/main.dir/visual_template_match.cpp.i
.PHONY : visual_template_match.cpp.i

visual_template_match.s: visual_template_match.cpp.s

.PHONY : visual_template_match.s

# target to generate assembly for a file
visual_template_match.cpp.s:
	$(MAKE) -f CMakeFiles/main.dir/build.make CMakeFiles/main.dir/visual_template_match.cpp.s
.PHONY : visual_template_match.cpp.s

# Help Target
help:
	@echo "The following are some of the valid targets for this Makefile:"
	@echo "... all (the default if no target is provided)"
	@echo "... clean"
	@echo "... depend"
	@echo "... rebuild_cache"
	@echo "... edit_cache"
	@echo "... main"
	@echo "... UTILS"
	@echo "... gc_multi.o"
	@echo "... gc_multi.i"
	@echo "... gc_multi.s"
	@echo "... main.o"
	@echo "... main.i"
	@echo "... main.s"
	@echo "... test_m.o"
	@echo "... test_m.i"
	@echo "... test_m.s"
	@echo "... visual_template_match.o"
	@echo "... visual_template_match.i"
	@echo "... visual_template_match.s"
.PHONY : help



#=============================================================================
# Special targets to cleanup operation of make.

# Special rule to run CMake to check the build system integrity.
# No rule that depends on this can have commands that come from listfiles
# because they might be regenerated.
cmake_check_build_system:
	$(CMAKE_COMMAND) -H$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR) --check-build-system CMakeFiles/Makefile.cmake 0
.PHONY : cmake_check_build_system

