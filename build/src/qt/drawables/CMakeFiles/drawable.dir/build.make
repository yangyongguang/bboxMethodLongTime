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
CMAKE_SOURCE_DIR = /home/yyg/code/lidarCode

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/yyg/code/lidarCode/build

# Include any dependencies generated for this target.
include src/qt/drawables/CMakeFiles/drawable.dir/depend.make

# Include the progress variables for this target.
include src/qt/drawables/CMakeFiles/drawable.dir/progress.make

# Include the compile flags for this target's objects.
include src/qt/drawables/CMakeFiles/drawable.dir/flags.make

src/qt/drawables/CMakeFiles/drawable.dir/drawable_cloud.cpp.o: src/qt/drawables/CMakeFiles/drawable.dir/flags.make
src/qt/drawables/CMakeFiles/drawable.dir/drawable_cloud.cpp.o: ../src/qt/drawables/drawable_cloud.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/yyg/code/lidarCode/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/qt/drawables/CMakeFiles/drawable.dir/drawable_cloud.cpp.o"
	cd /home/yyg/code/lidarCode/build/src/qt/drawables && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/drawable.dir/drawable_cloud.cpp.o -c /home/yyg/code/lidarCode/src/qt/drawables/drawable_cloud.cpp

src/qt/drawables/CMakeFiles/drawable.dir/drawable_cloud.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/drawable.dir/drawable_cloud.cpp.i"
	cd /home/yyg/code/lidarCode/build/src/qt/drawables && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/yyg/code/lidarCode/src/qt/drawables/drawable_cloud.cpp > CMakeFiles/drawable.dir/drawable_cloud.cpp.i

src/qt/drawables/CMakeFiles/drawable.dir/drawable_cloud.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/drawable.dir/drawable_cloud.cpp.s"
	cd /home/yyg/code/lidarCode/build/src/qt/drawables && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/yyg/code/lidarCode/src/qt/drawables/drawable_cloud.cpp -o CMakeFiles/drawable.dir/drawable_cloud.cpp.s

src/qt/drawables/CMakeFiles/drawable.dir/drawable_cloud.cpp.o.requires:

.PHONY : src/qt/drawables/CMakeFiles/drawable.dir/drawable_cloud.cpp.o.requires

src/qt/drawables/CMakeFiles/drawable.dir/drawable_cloud.cpp.o.provides: src/qt/drawables/CMakeFiles/drawable.dir/drawable_cloud.cpp.o.requires
	$(MAKE) -f src/qt/drawables/CMakeFiles/drawable.dir/build.make src/qt/drawables/CMakeFiles/drawable.dir/drawable_cloud.cpp.o.provides.build
.PHONY : src/qt/drawables/CMakeFiles/drawable.dir/drawable_cloud.cpp.o.provides

src/qt/drawables/CMakeFiles/drawable.dir/drawable_cloud.cpp.o.provides.build: src/qt/drawables/CMakeFiles/drawable.dir/drawable_cloud.cpp.o


src/qt/drawables/CMakeFiles/drawable.dir/drawable_line.cpp.o: src/qt/drawables/CMakeFiles/drawable.dir/flags.make
src/qt/drawables/CMakeFiles/drawable.dir/drawable_line.cpp.o: ../src/qt/drawables/drawable_line.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/yyg/code/lidarCode/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object src/qt/drawables/CMakeFiles/drawable.dir/drawable_line.cpp.o"
	cd /home/yyg/code/lidarCode/build/src/qt/drawables && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/drawable.dir/drawable_line.cpp.o -c /home/yyg/code/lidarCode/src/qt/drawables/drawable_line.cpp

src/qt/drawables/CMakeFiles/drawable.dir/drawable_line.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/drawable.dir/drawable_line.cpp.i"
	cd /home/yyg/code/lidarCode/build/src/qt/drawables && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/yyg/code/lidarCode/src/qt/drawables/drawable_line.cpp > CMakeFiles/drawable.dir/drawable_line.cpp.i

src/qt/drawables/CMakeFiles/drawable.dir/drawable_line.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/drawable.dir/drawable_line.cpp.s"
	cd /home/yyg/code/lidarCode/build/src/qt/drawables && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/yyg/code/lidarCode/src/qt/drawables/drawable_line.cpp -o CMakeFiles/drawable.dir/drawable_line.cpp.s

src/qt/drawables/CMakeFiles/drawable.dir/drawable_line.cpp.o.requires:

.PHONY : src/qt/drawables/CMakeFiles/drawable.dir/drawable_line.cpp.o.requires

src/qt/drawables/CMakeFiles/drawable.dir/drawable_line.cpp.o.provides: src/qt/drawables/CMakeFiles/drawable.dir/drawable_line.cpp.o.requires
	$(MAKE) -f src/qt/drawables/CMakeFiles/drawable.dir/build.make src/qt/drawables/CMakeFiles/drawable.dir/drawable_line.cpp.o.provides.build
.PHONY : src/qt/drawables/CMakeFiles/drawable.dir/drawable_line.cpp.o.provides

src/qt/drawables/CMakeFiles/drawable.dir/drawable_line.cpp.o.provides.build: src/qt/drawables/CMakeFiles/drawable.dir/drawable_line.cpp.o


src/qt/drawables/CMakeFiles/drawable.dir/drawable_selectable_cloud.cpp.o: src/qt/drawables/CMakeFiles/drawable.dir/flags.make
src/qt/drawables/CMakeFiles/drawable.dir/drawable_selectable_cloud.cpp.o: ../src/qt/drawables/drawable_selectable_cloud.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/yyg/code/lidarCode/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object src/qt/drawables/CMakeFiles/drawable.dir/drawable_selectable_cloud.cpp.o"
	cd /home/yyg/code/lidarCode/build/src/qt/drawables && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/drawable.dir/drawable_selectable_cloud.cpp.o -c /home/yyg/code/lidarCode/src/qt/drawables/drawable_selectable_cloud.cpp

src/qt/drawables/CMakeFiles/drawable.dir/drawable_selectable_cloud.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/drawable.dir/drawable_selectable_cloud.cpp.i"
	cd /home/yyg/code/lidarCode/build/src/qt/drawables && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/yyg/code/lidarCode/src/qt/drawables/drawable_selectable_cloud.cpp > CMakeFiles/drawable.dir/drawable_selectable_cloud.cpp.i

src/qt/drawables/CMakeFiles/drawable.dir/drawable_selectable_cloud.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/drawable.dir/drawable_selectable_cloud.cpp.s"
	cd /home/yyg/code/lidarCode/build/src/qt/drawables && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/yyg/code/lidarCode/src/qt/drawables/drawable_selectable_cloud.cpp -o CMakeFiles/drawable.dir/drawable_selectable_cloud.cpp.s

src/qt/drawables/CMakeFiles/drawable.dir/drawable_selectable_cloud.cpp.o.requires:

.PHONY : src/qt/drawables/CMakeFiles/drawable.dir/drawable_selectable_cloud.cpp.o.requires

src/qt/drawables/CMakeFiles/drawable.dir/drawable_selectable_cloud.cpp.o.provides: src/qt/drawables/CMakeFiles/drawable.dir/drawable_selectable_cloud.cpp.o.requires
	$(MAKE) -f src/qt/drawables/CMakeFiles/drawable.dir/build.make src/qt/drawables/CMakeFiles/drawable.dir/drawable_selectable_cloud.cpp.o.provides.build
.PHONY : src/qt/drawables/CMakeFiles/drawable.dir/drawable_selectable_cloud.cpp.o.provides

src/qt/drawables/CMakeFiles/drawable.dir/drawable_selectable_cloud.cpp.o.provides.build: src/qt/drawables/CMakeFiles/drawable.dir/drawable_selectable_cloud.cpp.o


# Object files for target drawable
drawable_OBJECTS = \
"CMakeFiles/drawable.dir/drawable_cloud.cpp.o" \
"CMakeFiles/drawable.dir/drawable_line.cpp.o" \
"CMakeFiles/drawable.dir/drawable_selectable_cloud.cpp.o"

# External object files for target drawable
drawable_EXTERNAL_OBJECTS =

src/qt/drawables/libdrawable.so: src/qt/drawables/CMakeFiles/drawable.dir/drawable_cloud.cpp.o
src/qt/drawables/libdrawable.so: src/qt/drawables/CMakeFiles/drawable.dir/drawable_line.cpp.o
src/qt/drawables/libdrawable.so: src/qt/drawables/CMakeFiles/drawable.dir/drawable_selectable_cloud.cpp.o
src/qt/drawables/libdrawable.so: src/qt/drawables/CMakeFiles/drawable.dir/build.make
src/qt/drawables/libdrawable.so: src/qt/utils/libutils.so
src/qt/drawables/libdrawable.so: /usr/lib/x86_64-linux-gnu/libQGLViewer.so
src/qt/drawables/libdrawable.so: /opt/Qt5.6.0/5.6/gcc_64/lib/libQt5Xml.so.5.6.0
src/qt/drawables/libdrawable.so: /opt/Qt5.6.0/5.6/gcc_64/lib/libQt5OpenGL.so.5.6.0
src/qt/drawables/libdrawable.so: /usr/lib/x86_64-linux-gnu/libGL.so
src/qt/drawables/libdrawable.so: /usr/lib/x86_64-linux-gnu/libGLU.so
src/qt/drawables/libdrawable.so: /usr/local/lib/libboost_system.so
src/qt/drawables/libdrawable.so: /usr/local/lib/libboost_filesystem.so
src/qt/drawables/libdrawable.so: /usr/local/lib/libboost_regex.so
src/qt/drawables/libdrawable.so: /usr/local/lib/libboost_program_options.so
src/qt/drawables/libdrawable.so: /opt/Qt5.6.0/5.6/gcc_64/lib/libQt5Widgets.so.5.6.0
src/qt/drawables/libdrawable.so: /opt/Qt5.6.0/5.6/gcc_64/lib/libQt5Gui.so.5.6.0
src/qt/drawables/libdrawable.so: /opt/Qt5.6.0/5.6/gcc_64/lib/libQt5Core.so.5.6.0
src/qt/drawables/libdrawable.so: src/qt/drawables/CMakeFiles/drawable.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/yyg/code/lidarCode/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Linking CXX shared library libdrawable.so"
	cd /home/yyg/code/lidarCode/build/src/qt/drawables && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/drawable.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/qt/drawables/CMakeFiles/drawable.dir/build: src/qt/drawables/libdrawable.so

.PHONY : src/qt/drawables/CMakeFiles/drawable.dir/build

src/qt/drawables/CMakeFiles/drawable.dir/requires: src/qt/drawables/CMakeFiles/drawable.dir/drawable_cloud.cpp.o.requires
src/qt/drawables/CMakeFiles/drawable.dir/requires: src/qt/drawables/CMakeFiles/drawable.dir/drawable_line.cpp.o.requires
src/qt/drawables/CMakeFiles/drawable.dir/requires: src/qt/drawables/CMakeFiles/drawable.dir/drawable_selectable_cloud.cpp.o.requires

.PHONY : src/qt/drawables/CMakeFiles/drawable.dir/requires

src/qt/drawables/CMakeFiles/drawable.dir/clean:
	cd /home/yyg/code/lidarCode/build/src/qt/drawables && $(CMAKE_COMMAND) -P CMakeFiles/drawable.dir/cmake_clean.cmake
.PHONY : src/qt/drawables/CMakeFiles/drawable.dir/clean

src/qt/drawables/CMakeFiles/drawable.dir/depend:
	cd /home/yyg/code/lidarCode/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/yyg/code/lidarCode /home/yyg/code/lidarCode/src/qt/drawables /home/yyg/code/lidarCode/build /home/yyg/code/lidarCode/build/src/qt/drawables /home/yyg/code/lidarCode/build/src/qt/drawables/CMakeFiles/drawable.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/qt/drawables/CMakeFiles/drawable.dir/depend

