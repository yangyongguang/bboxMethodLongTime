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
include src/qt/widgets/CMakeFiles/qt_widgets.dir/depend.make

# Include the progress variables for this target.
include src/qt/widgets/CMakeFiles/qt_widgets.dir/progress.make

# Include the compile flags for this target's objects.
include src/qt/widgets/CMakeFiles/qt_widgets.dir/flags.make

src/qt/widgets/ui_mainwindow.h: ../src/qt/widgets/ui/mainwindow.ui
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/yyg/code/lidarCode/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Generating ui_mainwindow.h"
	cd /home/yyg/code/lidarCode/build/src/qt/widgets && /opt/Qt5.6.0/5.6/gcc_64/bin/uic -o /home/yyg/code/lidarCode/build/src/qt/widgets/ui_mainwindow.h /home/yyg/code/lidarCode/src/qt/widgets/ui/mainwindow.ui

src/qt/widgets/moc_base_viewer_widget.cpp: ../src/qt/widgets/base_viewer_widget.h
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/yyg/code/lidarCode/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Generating moc_base_viewer_widget.cpp"
	cd /home/yyg/code/lidarCode/build/src/qt/widgets && /opt/Qt5.6.0/5.6/gcc_64/bin/moc @/home/yyg/code/lidarCode/build/src/qt/widgets/moc_base_viewer_widget.cpp_parameters

src/qt/widgets/moc_mainwindow.cpp: ../src/qt/widgets/mainwindow.h
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/yyg/code/lidarCode/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Generating moc_mainwindow.cpp"
	cd /home/yyg/code/lidarCode/build/src/qt/widgets && /opt/Qt5.6.0/5.6/gcc_64/bin/moc @/home/yyg/code/lidarCode/build/src/qt/widgets/moc_mainwindow.cpp_parameters

src/qt/widgets/CMakeFiles/qt_widgets.dir/base_viewer_widget.cpp.o: src/qt/widgets/CMakeFiles/qt_widgets.dir/flags.make
src/qt/widgets/CMakeFiles/qt_widgets.dir/base_viewer_widget.cpp.o: ../src/qt/widgets/base_viewer_widget.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/yyg/code/lidarCode/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object src/qt/widgets/CMakeFiles/qt_widgets.dir/base_viewer_widget.cpp.o"
	cd /home/yyg/code/lidarCode/build/src/qt/widgets && /usr/bin/g++-5   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/qt_widgets.dir/base_viewer_widget.cpp.o -c /home/yyg/code/lidarCode/src/qt/widgets/base_viewer_widget.cpp

src/qt/widgets/CMakeFiles/qt_widgets.dir/base_viewer_widget.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/qt_widgets.dir/base_viewer_widget.cpp.i"
	cd /home/yyg/code/lidarCode/build/src/qt/widgets && /usr/bin/g++-5  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/yyg/code/lidarCode/src/qt/widgets/base_viewer_widget.cpp > CMakeFiles/qt_widgets.dir/base_viewer_widget.cpp.i

src/qt/widgets/CMakeFiles/qt_widgets.dir/base_viewer_widget.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/qt_widgets.dir/base_viewer_widget.cpp.s"
	cd /home/yyg/code/lidarCode/build/src/qt/widgets && /usr/bin/g++-5  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/yyg/code/lidarCode/src/qt/widgets/base_viewer_widget.cpp -o CMakeFiles/qt_widgets.dir/base_viewer_widget.cpp.s

src/qt/widgets/CMakeFiles/qt_widgets.dir/base_viewer_widget.cpp.o.requires:

.PHONY : src/qt/widgets/CMakeFiles/qt_widgets.dir/base_viewer_widget.cpp.o.requires

src/qt/widgets/CMakeFiles/qt_widgets.dir/base_viewer_widget.cpp.o.provides: src/qt/widgets/CMakeFiles/qt_widgets.dir/base_viewer_widget.cpp.o.requires
	$(MAKE) -f src/qt/widgets/CMakeFiles/qt_widgets.dir/build.make src/qt/widgets/CMakeFiles/qt_widgets.dir/base_viewer_widget.cpp.o.provides.build
.PHONY : src/qt/widgets/CMakeFiles/qt_widgets.dir/base_viewer_widget.cpp.o.provides

src/qt/widgets/CMakeFiles/qt_widgets.dir/base_viewer_widget.cpp.o.provides.build: src/qt/widgets/CMakeFiles/qt_widgets.dir/base_viewer_widget.cpp.o


src/qt/widgets/CMakeFiles/qt_widgets.dir/mainwindow.cpp.o: src/qt/widgets/CMakeFiles/qt_widgets.dir/flags.make
src/qt/widgets/CMakeFiles/qt_widgets.dir/mainwindow.cpp.o: ../src/qt/widgets/mainwindow.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/yyg/code/lidarCode/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object src/qt/widgets/CMakeFiles/qt_widgets.dir/mainwindow.cpp.o"
	cd /home/yyg/code/lidarCode/build/src/qt/widgets && /usr/bin/g++-5   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/qt_widgets.dir/mainwindow.cpp.o -c /home/yyg/code/lidarCode/src/qt/widgets/mainwindow.cpp

src/qt/widgets/CMakeFiles/qt_widgets.dir/mainwindow.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/qt_widgets.dir/mainwindow.cpp.i"
	cd /home/yyg/code/lidarCode/build/src/qt/widgets && /usr/bin/g++-5  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/yyg/code/lidarCode/src/qt/widgets/mainwindow.cpp > CMakeFiles/qt_widgets.dir/mainwindow.cpp.i

src/qt/widgets/CMakeFiles/qt_widgets.dir/mainwindow.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/qt_widgets.dir/mainwindow.cpp.s"
	cd /home/yyg/code/lidarCode/build/src/qt/widgets && /usr/bin/g++-5  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/yyg/code/lidarCode/src/qt/widgets/mainwindow.cpp -o CMakeFiles/qt_widgets.dir/mainwindow.cpp.s

src/qt/widgets/CMakeFiles/qt_widgets.dir/mainwindow.cpp.o.requires:

.PHONY : src/qt/widgets/CMakeFiles/qt_widgets.dir/mainwindow.cpp.o.requires

src/qt/widgets/CMakeFiles/qt_widgets.dir/mainwindow.cpp.o.provides: src/qt/widgets/CMakeFiles/qt_widgets.dir/mainwindow.cpp.o.requires
	$(MAKE) -f src/qt/widgets/CMakeFiles/qt_widgets.dir/build.make src/qt/widgets/CMakeFiles/qt_widgets.dir/mainwindow.cpp.o.provides.build
.PHONY : src/qt/widgets/CMakeFiles/qt_widgets.dir/mainwindow.cpp.o.provides

src/qt/widgets/CMakeFiles/qt_widgets.dir/mainwindow.cpp.o.provides.build: src/qt/widgets/CMakeFiles/qt_widgets.dir/mainwindow.cpp.o


src/qt/widgets/CMakeFiles/qt_widgets.dir/moc_base_viewer_widget.cpp.o: src/qt/widgets/CMakeFiles/qt_widgets.dir/flags.make
src/qt/widgets/CMakeFiles/qt_widgets.dir/moc_base_viewer_widget.cpp.o: src/qt/widgets/moc_base_viewer_widget.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/yyg/code/lidarCode/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object src/qt/widgets/CMakeFiles/qt_widgets.dir/moc_base_viewer_widget.cpp.o"
	cd /home/yyg/code/lidarCode/build/src/qt/widgets && /usr/bin/g++-5   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/qt_widgets.dir/moc_base_viewer_widget.cpp.o -c /home/yyg/code/lidarCode/build/src/qt/widgets/moc_base_viewer_widget.cpp

src/qt/widgets/CMakeFiles/qt_widgets.dir/moc_base_viewer_widget.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/qt_widgets.dir/moc_base_viewer_widget.cpp.i"
	cd /home/yyg/code/lidarCode/build/src/qt/widgets && /usr/bin/g++-5  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/yyg/code/lidarCode/build/src/qt/widgets/moc_base_viewer_widget.cpp > CMakeFiles/qt_widgets.dir/moc_base_viewer_widget.cpp.i

src/qt/widgets/CMakeFiles/qt_widgets.dir/moc_base_viewer_widget.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/qt_widgets.dir/moc_base_viewer_widget.cpp.s"
	cd /home/yyg/code/lidarCode/build/src/qt/widgets && /usr/bin/g++-5  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/yyg/code/lidarCode/build/src/qt/widgets/moc_base_viewer_widget.cpp -o CMakeFiles/qt_widgets.dir/moc_base_viewer_widget.cpp.s

src/qt/widgets/CMakeFiles/qt_widgets.dir/moc_base_viewer_widget.cpp.o.requires:

.PHONY : src/qt/widgets/CMakeFiles/qt_widgets.dir/moc_base_viewer_widget.cpp.o.requires

src/qt/widgets/CMakeFiles/qt_widgets.dir/moc_base_viewer_widget.cpp.o.provides: src/qt/widgets/CMakeFiles/qt_widgets.dir/moc_base_viewer_widget.cpp.o.requires
	$(MAKE) -f src/qt/widgets/CMakeFiles/qt_widgets.dir/build.make src/qt/widgets/CMakeFiles/qt_widgets.dir/moc_base_viewer_widget.cpp.o.provides.build
.PHONY : src/qt/widgets/CMakeFiles/qt_widgets.dir/moc_base_viewer_widget.cpp.o.provides

src/qt/widgets/CMakeFiles/qt_widgets.dir/moc_base_viewer_widget.cpp.o.provides.build: src/qt/widgets/CMakeFiles/qt_widgets.dir/moc_base_viewer_widget.cpp.o


src/qt/widgets/CMakeFiles/qt_widgets.dir/moc_mainwindow.cpp.o: src/qt/widgets/CMakeFiles/qt_widgets.dir/flags.make
src/qt/widgets/CMakeFiles/qt_widgets.dir/moc_mainwindow.cpp.o: src/qt/widgets/moc_mainwindow.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/yyg/code/lidarCode/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Building CXX object src/qt/widgets/CMakeFiles/qt_widgets.dir/moc_mainwindow.cpp.o"
	cd /home/yyg/code/lidarCode/build/src/qt/widgets && /usr/bin/g++-5   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/qt_widgets.dir/moc_mainwindow.cpp.o -c /home/yyg/code/lidarCode/build/src/qt/widgets/moc_mainwindow.cpp

src/qt/widgets/CMakeFiles/qt_widgets.dir/moc_mainwindow.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/qt_widgets.dir/moc_mainwindow.cpp.i"
	cd /home/yyg/code/lidarCode/build/src/qt/widgets && /usr/bin/g++-5  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/yyg/code/lidarCode/build/src/qt/widgets/moc_mainwindow.cpp > CMakeFiles/qt_widgets.dir/moc_mainwindow.cpp.i

src/qt/widgets/CMakeFiles/qt_widgets.dir/moc_mainwindow.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/qt_widgets.dir/moc_mainwindow.cpp.s"
	cd /home/yyg/code/lidarCode/build/src/qt/widgets && /usr/bin/g++-5  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/yyg/code/lidarCode/build/src/qt/widgets/moc_mainwindow.cpp -o CMakeFiles/qt_widgets.dir/moc_mainwindow.cpp.s

src/qt/widgets/CMakeFiles/qt_widgets.dir/moc_mainwindow.cpp.o.requires:

.PHONY : src/qt/widgets/CMakeFiles/qt_widgets.dir/moc_mainwindow.cpp.o.requires

src/qt/widgets/CMakeFiles/qt_widgets.dir/moc_mainwindow.cpp.o.provides: src/qt/widgets/CMakeFiles/qt_widgets.dir/moc_mainwindow.cpp.o.requires
	$(MAKE) -f src/qt/widgets/CMakeFiles/qt_widgets.dir/build.make src/qt/widgets/CMakeFiles/qt_widgets.dir/moc_mainwindow.cpp.o.provides.build
.PHONY : src/qt/widgets/CMakeFiles/qt_widgets.dir/moc_mainwindow.cpp.o.provides

src/qt/widgets/CMakeFiles/qt_widgets.dir/moc_mainwindow.cpp.o.provides.build: src/qt/widgets/CMakeFiles/qt_widgets.dir/moc_mainwindow.cpp.o


# Object files for target qt_widgets
qt_widgets_OBJECTS = \
"CMakeFiles/qt_widgets.dir/base_viewer_widget.cpp.o" \
"CMakeFiles/qt_widgets.dir/mainwindow.cpp.o" \
"CMakeFiles/qt_widgets.dir/moc_base_viewer_widget.cpp.o" \
"CMakeFiles/qt_widgets.dir/moc_mainwindow.cpp.o"

# External object files for target qt_widgets
qt_widgets_EXTERNAL_OBJECTS =

src/qt/widgets/libqt_widgets.so: src/qt/widgets/CMakeFiles/qt_widgets.dir/base_viewer_widget.cpp.o
src/qt/widgets/libqt_widgets.so: src/qt/widgets/CMakeFiles/qt_widgets.dir/mainwindow.cpp.o
src/qt/widgets/libqt_widgets.so: src/qt/widgets/CMakeFiles/qt_widgets.dir/moc_base_viewer_widget.cpp.o
src/qt/widgets/libqt_widgets.so: src/qt/widgets/CMakeFiles/qt_widgets.dir/moc_mainwindow.cpp.o
src/qt/widgets/libqt_widgets.so: src/qt/widgets/CMakeFiles/qt_widgets.dir/build.make
src/qt/widgets/libqt_widgets.so: src/qt/viewer/libviewer.so
src/qt/widgets/libqt_widgets.so: src/qt/drawables/libdrawable.so
src/qt/widgets/libqt_widgets.so: src/qt/utils/libutils.so
src/qt/widgets/libqt_widgets.so: /usr/local/lib/libopencv_dnn.so.3.4.0
src/qt/widgets/libqt_widgets.so: /usr/local/lib/libopencv_ml.so.3.4.0
src/qt/widgets/libqt_widgets.so: /usr/local/lib/libopencv_objdetect.so.3.4.0
src/qt/widgets/libqt_widgets.so: /usr/local/lib/libopencv_shape.so.3.4.0
src/qt/widgets/libqt_widgets.so: /usr/local/lib/libopencv_stitching.so.3.4.0
src/qt/widgets/libqt_widgets.so: /usr/local/lib/libopencv_superres.so.3.4.0
src/qt/widgets/libqt_widgets.so: /usr/local/lib/libopencv_videostab.so.3.4.0
src/qt/widgets/libqt_widgets.so: /usr/lib/x86_64-linux-gnu/libQGLViewer.so
src/qt/widgets/libqt_widgets.so: /opt/Qt5.6.0/5.6/gcc_64/lib/libQt5Xml.so.5.6.0
src/qt/widgets/libqt_widgets.so: /opt/Qt5.6.0/5.6/gcc_64/lib/libQt5OpenGL.so.5.6.0
src/qt/widgets/libqt_widgets.so: /usr/lib/x86_64-linux-gnu/libGL.so
src/qt/widgets/libqt_widgets.so: /usr/lib/x86_64-linux-gnu/libGLU.so
src/qt/widgets/libqt_widgets.so: /usr/local/lib/libboost_system.so
src/qt/widgets/libqt_widgets.so: /usr/local/lib/libboost_filesystem.so
src/qt/widgets/libqt_widgets.so: /usr/local/lib/libboost_regex.so
src/qt/widgets/libqt_widgets.so: /usr/local/lib/libboost_program_options.so
src/qt/widgets/libqt_widgets.so: /opt/Qt5.6.0/5.6/gcc_64/lib/libQt5Widgets.so.5.6.0
src/qt/widgets/libqt_widgets.so: /opt/Qt5.6.0/5.6/gcc_64/lib/libQt5Gui.so.5.6.0
src/qt/widgets/libqt_widgets.so: /opt/Qt5.6.0/5.6/gcc_64/lib/libQt5Core.so.5.6.0
src/qt/widgets/libqt_widgets.so: src/qt/utils/libobject.so
src/qt/widgets/libqt_widgets.so: /usr/local/lib/libopencv_calib3d.so.3.4.0
src/qt/widgets/libqt_widgets.so: /usr/local/lib/libopencv_features2d.so.3.4.0
src/qt/widgets/libqt_widgets.so: /usr/local/lib/libopencv_flann.so.3.4.0
src/qt/widgets/libqt_widgets.so: /usr/local/lib/libopencv_highgui.so.3.4.0
src/qt/widgets/libqt_widgets.so: /usr/local/lib/libopencv_photo.so.3.4.0
src/qt/widgets/libqt_widgets.so: /usr/local/lib/libopencv_video.so.3.4.0
src/qt/widgets/libqt_widgets.so: /usr/local/lib/libopencv_videoio.so.3.4.0
src/qt/widgets/libqt_widgets.so: /usr/local/lib/libopencv_imgcodecs.so.3.4.0
src/qt/widgets/libqt_widgets.so: /usr/local/lib/libopencv_imgproc.so.3.4.0
src/qt/widgets/libqt_widgets.so: /usr/local/lib/libopencv_core.so.3.4.0
src/qt/widgets/libqt_widgets.so: src/qt/widgets/CMakeFiles/qt_widgets.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/yyg/code/lidarCode/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Linking CXX shared library libqt_widgets.so"
	cd /home/yyg/code/lidarCode/build/src/qt/widgets && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/qt_widgets.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/qt/widgets/CMakeFiles/qt_widgets.dir/build: src/qt/widgets/libqt_widgets.so

.PHONY : src/qt/widgets/CMakeFiles/qt_widgets.dir/build

src/qt/widgets/CMakeFiles/qt_widgets.dir/requires: src/qt/widgets/CMakeFiles/qt_widgets.dir/base_viewer_widget.cpp.o.requires
src/qt/widgets/CMakeFiles/qt_widgets.dir/requires: src/qt/widgets/CMakeFiles/qt_widgets.dir/mainwindow.cpp.o.requires
src/qt/widgets/CMakeFiles/qt_widgets.dir/requires: src/qt/widgets/CMakeFiles/qt_widgets.dir/moc_base_viewer_widget.cpp.o.requires
src/qt/widgets/CMakeFiles/qt_widgets.dir/requires: src/qt/widgets/CMakeFiles/qt_widgets.dir/moc_mainwindow.cpp.o.requires

.PHONY : src/qt/widgets/CMakeFiles/qt_widgets.dir/requires

src/qt/widgets/CMakeFiles/qt_widgets.dir/clean:
	cd /home/yyg/code/lidarCode/build/src/qt/widgets && $(CMAKE_COMMAND) -P CMakeFiles/qt_widgets.dir/cmake_clean.cmake
.PHONY : src/qt/widgets/CMakeFiles/qt_widgets.dir/clean

src/qt/widgets/CMakeFiles/qt_widgets.dir/depend: src/qt/widgets/ui_mainwindow.h
src/qt/widgets/CMakeFiles/qt_widgets.dir/depend: src/qt/widgets/moc_base_viewer_widget.cpp
src/qt/widgets/CMakeFiles/qt_widgets.dir/depend: src/qt/widgets/moc_mainwindow.cpp
	cd /home/yyg/code/lidarCode/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/yyg/code/lidarCode /home/yyg/code/lidarCode/src/qt/widgets /home/yyg/code/lidarCode/build /home/yyg/code/lidarCode/build/src/qt/widgets /home/yyg/code/lidarCode/build/src/qt/widgets/CMakeFiles/qt_widgets.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/qt/widgets/CMakeFiles/qt_widgets.dir/depend

