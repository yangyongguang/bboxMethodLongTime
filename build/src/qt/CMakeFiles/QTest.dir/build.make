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
include src/qt/CMakeFiles/QTest.dir/depend.make

# Include the progress variables for this target.
include src/qt/CMakeFiles/QTest.dir/progress.make

# Include the compile flags for this target's objects.
include src/qt/CMakeFiles/QTest.dir/flags.make

src/qt/CMakeFiles/QTest.dir/main.cpp.o: src/qt/CMakeFiles/QTest.dir/flags.make
src/qt/CMakeFiles/QTest.dir/main.cpp.o: ../src/qt/main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/yyg/code/lidarCode/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/qt/CMakeFiles/QTest.dir/main.cpp.o"
	cd /home/yyg/code/lidarCode/build/src/qt && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/QTest.dir/main.cpp.o -c /home/yyg/code/lidarCode/src/qt/main.cpp

src/qt/CMakeFiles/QTest.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/QTest.dir/main.cpp.i"
	cd /home/yyg/code/lidarCode/build/src/qt && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/yyg/code/lidarCode/src/qt/main.cpp > CMakeFiles/QTest.dir/main.cpp.i

src/qt/CMakeFiles/QTest.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/QTest.dir/main.cpp.s"
	cd /home/yyg/code/lidarCode/build/src/qt && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/yyg/code/lidarCode/src/qt/main.cpp -o CMakeFiles/QTest.dir/main.cpp.s

src/qt/CMakeFiles/QTest.dir/main.cpp.o.requires:

.PHONY : src/qt/CMakeFiles/QTest.dir/main.cpp.o.requires

src/qt/CMakeFiles/QTest.dir/main.cpp.o.provides: src/qt/CMakeFiles/QTest.dir/main.cpp.o.requires
	$(MAKE) -f src/qt/CMakeFiles/QTest.dir/build.make src/qt/CMakeFiles/QTest.dir/main.cpp.o.provides.build
.PHONY : src/qt/CMakeFiles/QTest.dir/main.cpp.o.provides

src/qt/CMakeFiles/QTest.dir/main.cpp.o.provides.build: src/qt/CMakeFiles/QTest.dir/main.cpp.o


# Object files for target QTest
QTest_OBJECTS = \
"CMakeFiles/QTest.dir/main.cpp.o"

# External object files for target QTest
QTest_EXTERNAL_OBJECTS =

src/qt/QTest: src/qt/CMakeFiles/QTest.dir/main.cpp.o
src/qt/QTest: src/qt/CMakeFiles/QTest.dir/build.make
src/qt/QTest: src/qt/widgets/libqt_widgets.so
src/qt/QTest: src/groundRemove/libgroundRemove.so
src/qt/QTest: src/groundRemove/libcomponent_clustering.a
src/qt/QTest: src/groundRemove/libbox_fitting.a
src/qt/QTest: src/groundRemove/libimm_ukf_pda.a
src/qt/QTest: src/qt/viewer/libviewer.so
src/qt/QTest: src/qt/utils/libobject.so
src/qt/QTest: src/qt/drawables/libdrawable.so
src/qt/QTest: /usr/lib/x86_64-linux-gnu/libQGLViewer.so
src/qt/QTest: /opt/Qt5.6.0/5.6/gcc_64/lib/libQt5Xml.so.5.6.0
src/qt/QTest: /opt/Qt5.6.0/5.6/gcc_64/lib/libQt5OpenGL.so.5.6.0
src/qt/QTest: /opt/Qt5.6.0/5.6/gcc_64/lib/libQt5Widgets.so.5.6.0
src/qt/QTest: /opt/Qt5.6.0/5.6/gcc_64/lib/libQt5Gui.so.5.6.0
src/qt/QTest: /opt/Qt5.6.0/5.6/gcc_64/lib/libQt5Core.so.5.6.0
src/qt/QTest: /usr/lib/x86_64-linux-gnu/libGL.so
src/qt/QTest: /usr/lib/x86_64-linux-gnu/libGLU.so
src/qt/QTest: src/qt/utils/libutils.so
src/qt/QTest: src/groundRemove/libbin.so
src/qt/QTest: src/groundRemove/libsegment.so
src/qt/QTest: src/groundRemove/libconvex_hull.a
src/qt/QTest: /usr/local/lib/libopencv_dnn.so.3.4.0
src/qt/QTest: /usr/local/lib/libopencv_ml.so.3.4.0
src/qt/QTest: /usr/local/lib/libopencv_objdetect.so.3.4.0
src/qt/QTest: /usr/local/lib/libopencv_shape.so.3.4.0
src/qt/QTest: /usr/local/lib/libopencv_stitching.so.3.4.0
src/qt/QTest: /usr/local/lib/libopencv_superres.so.3.4.0
src/qt/QTest: /usr/local/lib/libopencv_videostab.so.3.4.0
src/qt/QTest: /usr/local/lib/libopencv_calib3d.so.3.4.0
src/qt/QTest: /usr/local/lib/libopencv_features2d.so.3.4.0
src/qt/QTest: /usr/local/lib/libopencv_flann.so.3.4.0
src/qt/QTest: /usr/local/lib/libopencv_highgui.so.3.4.0
src/qt/QTest: /usr/local/lib/libopencv_photo.so.3.4.0
src/qt/QTest: /usr/local/lib/libopencv_video.so.3.4.0
src/qt/QTest: /usr/local/lib/libopencv_videoio.so.3.4.0
src/qt/QTest: /usr/local/lib/libopencv_imgcodecs.so.3.4.0
src/qt/QTest: /usr/local/lib/libopencv_imgproc.so.3.4.0
src/qt/QTest: /usr/local/lib/libopencv_core.so.3.4.0
src/qt/QTest: src/groundRemove/libcloud.so
src/qt/QTest: /usr/local/lib/libboost_system.so
src/qt/QTest: /usr/local/lib/libboost_filesystem.so
src/qt/QTest: /usr/local/lib/libboost_regex.so
src/qt/QTest: /usr/local/lib/libboost_program_options.so
src/qt/QTest: src/qt/CMakeFiles/QTest.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/yyg/code/lidarCode/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable QTest"
	cd /home/yyg/code/lidarCode/build/src/qt && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/QTest.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/qt/CMakeFiles/QTest.dir/build: src/qt/QTest

.PHONY : src/qt/CMakeFiles/QTest.dir/build

src/qt/CMakeFiles/QTest.dir/requires: src/qt/CMakeFiles/QTest.dir/main.cpp.o.requires

.PHONY : src/qt/CMakeFiles/QTest.dir/requires

src/qt/CMakeFiles/QTest.dir/clean:
	cd /home/yyg/code/lidarCode/build/src/qt && $(CMAKE_COMMAND) -P CMakeFiles/QTest.dir/cmake_clean.cmake
.PHONY : src/qt/CMakeFiles/QTest.dir/clean

src/qt/CMakeFiles/QTest.dir/depend:
	cd /home/yyg/code/lidarCode/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/yyg/code/lidarCode /home/yyg/code/lidarCode/src/qt /home/yyg/code/lidarCode/build /home/yyg/code/lidarCode/build/src/qt /home/yyg/code/lidarCode/build/src/qt/CMakeFiles/QTest.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/qt/CMakeFiles/QTest.dir/depend

