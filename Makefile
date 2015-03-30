# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

# Default target executed when no arguments are given to make.
default_target: all
.PHONY : default_target

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
CMAKE_SOURCE_DIR = /home/shiv/RPA-Tracking-Human-Trajectories

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/shiv/RPA-Tracking-Human-Trajectories

#=============================================================================
# Targets provided globally by CMake.

# Special rule for the target edit_cache
edit_cache:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --cyan "Running interactive CMake command-line interface..."
	/usr/bin/cmake -i .
.PHONY : edit_cache

# Special rule for the target edit_cache
edit_cache/fast: edit_cache
.PHONY : edit_cache/fast

# Special rule for the target rebuild_cache
rebuild_cache:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --cyan "Running CMake to regenerate build system..."
	/usr/bin/cmake -H$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR)
.PHONY : rebuild_cache

# Special rule for the target rebuild_cache
rebuild_cache/fast: rebuild_cache
.PHONY : rebuild_cache/fast

# The main all target
all: cmake_check_build_system
	$(CMAKE_COMMAND) -E cmake_progress_start /home/shiv/RPA-Tracking-Human-Trajectories/CMakeFiles /home/shiv/RPA-Tracking-Human-Trajectories/CMakeFiles/progress.marks
	$(MAKE) -f CMakeFiles/Makefile2 all
	$(CMAKE_COMMAND) -E cmake_progress_start /home/shiv/RPA-Tracking-Human-Trajectories/CMakeFiles 0
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
# Target rules for targets named ground_based_rgbd_people_detector

# Build rule for target.
ground_based_rgbd_people_detector: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 ground_based_rgbd_people_detector
.PHONY : ground_based_rgbd_people_detector

# fast build rule for target.
ground_based_rgbd_people_detector/fast:
	$(MAKE) -f CMakeFiles/ground_based_rgbd_people_detector.dir/build.make CMakeFiles/ground_based_rgbd_people_detector.dir/build
.PHONY : ground_based_rgbd_people_detector/fast

src/Macros.o: src/Macros.cpp.o
.PHONY : src/Macros.o

# target to build an object file
src/Macros.cpp.o:
	$(MAKE) -f CMakeFiles/ground_based_rgbd_people_detector.dir/build.make CMakeFiles/ground_based_rgbd_people_detector.dir/src/Macros.cpp.o
.PHONY : src/Macros.cpp.o

src/Macros.i: src/Macros.cpp.i
.PHONY : src/Macros.i

# target to preprocess a source file
src/Macros.cpp.i:
	$(MAKE) -f CMakeFiles/ground_based_rgbd_people_detector.dir/build.make CMakeFiles/ground_based_rgbd_people_detector.dir/src/Macros.cpp.i
.PHONY : src/Macros.cpp.i

src/Macros.s: src/Macros.cpp.s
.PHONY : src/Macros.s

# target to generate assembly for a file
src/Macros.cpp.s:
	$(MAKE) -f CMakeFiles/ground_based_rgbd_people_detector.dir/build.make CMakeFiles/ground_based_rgbd_people_detector.dir/src/Macros.cpp.s
.PHONY : src/Macros.cpp.s

src/Person.o: src/Person.cpp.o
.PHONY : src/Person.o

# target to build an object file
src/Person.cpp.o:
	$(MAKE) -f CMakeFiles/ground_based_rgbd_people_detector.dir/build.make CMakeFiles/ground_based_rgbd_people_detector.dir/src/Person.cpp.o
.PHONY : src/Person.cpp.o

src/Person.i: src/Person.cpp.i
.PHONY : src/Person.i

# target to preprocess a source file
src/Person.cpp.i:
	$(MAKE) -f CMakeFiles/ground_based_rgbd_people_detector.dir/build.make CMakeFiles/ground_based_rgbd_people_detector.dir/src/Person.cpp.i
.PHONY : src/Person.cpp.i

src/Person.s: src/Person.cpp.s
.PHONY : src/Person.s

# target to generate assembly for a file
src/Person.cpp.s:
	$(MAKE) -f CMakeFiles/ground_based_rgbd_people_detector.dir/build.make CMakeFiles/ground_based_rgbd_people_detector.dir/src/Person.cpp.s
.PHONY : src/Person.cpp.s

src/Trajectory.o: src/Trajectory.cpp.o
.PHONY : src/Trajectory.o

# target to build an object file
src/Trajectory.cpp.o:
	$(MAKE) -f CMakeFiles/ground_based_rgbd_people_detector.dir/build.make CMakeFiles/ground_based_rgbd_people_detector.dir/src/Trajectory.cpp.o
.PHONY : src/Trajectory.cpp.o

src/Trajectory.i: src/Trajectory.cpp.i
.PHONY : src/Trajectory.i

# target to preprocess a source file
src/Trajectory.cpp.i:
	$(MAKE) -f CMakeFiles/ground_based_rgbd_people_detector.dir/build.make CMakeFiles/ground_based_rgbd_people_detector.dir/src/Trajectory.cpp.i
.PHONY : src/Trajectory.cpp.i

src/Trajectory.s: src/Trajectory.cpp.s
.PHONY : src/Trajectory.s

# target to generate assembly for a file
src/Trajectory.cpp.s:
	$(MAKE) -f CMakeFiles/ground_based_rgbd_people_detector.dir/build.make CMakeFiles/ground_based_rgbd_people_detector.dir/src/Trajectory.cpp.s
.PHONY : src/Trajectory.cpp.s

src/main_ground_based_people_detection.o: src/main_ground_based_people_detection.cpp.o
.PHONY : src/main_ground_based_people_detection.o

# target to build an object file
src/main_ground_based_people_detection.cpp.o:
	$(MAKE) -f CMakeFiles/ground_based_rgbd_people_detector.dir/build.make CMakeFiles/ground_based_rgbd_people_detector.dir/src/main_ground_based_people_detection.cpp.o
.PHONY : src/main_ground_based_people_detection.cpp.o

src/main_ground_based_people_detection.i: src/main_ground_based_people_detection.cpp.i
.PHONY : src/main_ground_based_people_detection.i

# target to preprocess a source file
src/main_ground_based_people_detection.cpp.i:
	$(MAKE) -f CMakeFiles/ground_based_rgbd_people_detector.dir/build.make CMakeFiles/ground_based_rgbd_people_detector.dir/src/main_ground_based_people_detection.cpp.i
.PHONY : src/main_ground_based_people_detection.cpp.i

src/main_ground_based_people_detection.s: src/main_ground_based_people_detection.cpp.s
.PHONY : src/main_ground_based_people_detection.s

# target to generate assembly for a file
src/main_ground_based_people_detection.cpp.s:
	$(MAKE) -f CMakeFiles/ground_based_rgbd_people_detector.dir/build.make CMakeFiles/ground_based_rgbd_people_detector.dir/src/main_ground_based_people_detection.cpp.s
.PHONY : src/main_ground_based_people_detection.cpp.s

# Help Target
help:
	@echo "The following are some of the valid targets for this Makefile:"
	@echo "... all (the default if no target is provided)"
	@echo "... clean"
	@echo "... depend"
	@echo "... edit_cache"
	@echo "... ground_based_rgbd_people_detector"
	@echo "... rebuild_cache"
	@echo "... src/Macros.o"
	@echo "... src/Macros.i"
	@echo "... src/Macros.s"
	@echo "... src/Person.o"
	@echo "... src/Person.i"
	@echo "... src/Person.s"
	@echo "... src/Trajectory.o"
	@echo "... src/Trajectory.i"
	@echo "... src/Trajectory.s"
	@echo "... src/main_ground_based_people_detection.o"
	@echo "... src/main_ground_based_people_detection.i"
	@echo "... src/main_ground_based_people_detection.s"
.PHONY : help



#=============================================================================
# Special targets to cleanup operation of make.

# Special rule to run CMake to check the build system integrity.
# No rule that depends on this can have commands that come from listfiles
# because they might be regenerated.
cmake_check_build_system:
	$(CMAKE_COMMAND) -H$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR) --check-build-system CMakeFiles/Makefile.cmake 0
.PHONY : cmake_check_build_system

