# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.24

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
CMAKE_COMMAND = /usr/local/bin/cmake

# The command to remove a file.
RM = /usr/local/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /media/iaitech/DATA/programs/20221212

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /media/iaitech/DATA/programs/20221212

# Include any dependencies generated for this target.
include CMakeFiles/VA.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/VA.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/VA.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/VA.dir/flags.make

CMakeFiles/VA.dir/main.cpp.o: CMakeFiles/VA.dir/flags.make
CMakeFiles/VA.dir/main.cpp.o: main.cpp
CMakeFiles/VA.dir/main.cpp.o: CMakeFiles/VA.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/media/iaitech/DATA/programs/20221212/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/VA.dir/main.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/VA.dir/main.cpp.o -MF CMakeFiles/VA.dir/main.cpp.o.d -o CMakeFiles/VA.dir/main.cpp.o -c /media/iaitech/DATA/programs/20221212/main.cpp

CMakeFiles/VA.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/VA.dir/main.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /media/iaitech/DATA/programs/20221212/main.cpp > CMakeFiles/VA.dir/main.cpp.i

CMakeFiles/VA.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/VA.dir/main.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /media/iaitech/DATA/programs/20221212/main.cpp -o CMakeFiles/VA.dir/main.cpp.s

CMakeFiles/VA.dir/detector.cpp.o: CMakeFiles/VA.dir/flags.make
CMakeFiles/VA.dir/detector.cpp.o: detector.cpp
CMakeFiles/VA.dir/detector.cpp.o: CMakeFiles/VA.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/media/iaitech/DATA/programs/20221212/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/VA.dir/detector.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/VA.dir/detector.cpp.o -MF CMakeFiles/VA.dir/detector.cpp.o.d -o CMakeFiles/VA.dir/detector.cpp.o -c /media/iaitech/DATA/programs/20221212/detector.cpp

CMakeFiles/VA.dir/detector.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/VA.dir/detector.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /media/iaitech/DATA/programs/20221212/detector.cpp > CMakeFiles/VA.dir/detector.cpp.i

CMakeFiles/VA.dir/detector.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/VA.dir/detector.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /media/iaitech/DATA/programs/20221212/detector.cpp -o CMakeFiles/VA.dir/detector.cpp.s

CMakeFiles/VA.dir/HungarianAlg.cpp.o: CMakeFiles/VA.dir/flags.make
CMakeFiles/VA.dir/HungarianAlg.cpp.o: HungarianAlg.cpp
CMakeFiles/VA.dir/HungarianAlg.cpp.o: CMakeFiles/VA.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/media/iaitech/DATA/programs/20221212/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/VA.dir/HungarianAlg.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/VA.dir/HungarianAlg.cpp.o -MF CMakeFiles/VA.dir/HungarianAlg.cpp.o.d -o CMakeFiles/VA.dir/HungarianAlg.cpp.o -c /media/iaitech/DATA/programs/20221212/HungarianAlg.cpp

CMakeFiles/VA.dir/HungarianAlg.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/VA.dir/HungarianAlg.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /media/iaitech/DATA/programs/20221212/HungarianAlg.cpp > CMakeFiles/VA.dir/HungarianAlg.cpp.i

CMakeFiles/VA.dir/HungarianAlg.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/VA.dir/HungarianAlg.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /media/iaitech/DATA/programs/20221212/HungarianAlg.cpp -o CMakeFiles/VA.dir/HungarianAlg.cpp.s

CMakeFiles/VA.dir/Kalman.cpp.o: CMakeFiles/VA.dir/flags.make
CMakeFiles/VA.dir/Kalman.cpp.o: Kalman.cpp
CMakeFiles/VA.dir/Kalman.cpp.o: CMakeFiles/VA.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/media/iaitech/DATA/programs/20221212/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/VA.dir/Kalman.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/VA.dir/Kalman.cpp.o -MF CMakeFiles/VA.dir/Kalman.cpp.o.d -o CMakeFiles/VA.dir/Kalman.cpp.o -c /media/iaitech/DATA/programs/20221212/Kalman.cpp

CMakeFiles/VA.dir/Kalman.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/VA.dir/Kalman.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /media/iaitech/DATA/programs/20221212/Kalman.cpp > CMakeFiles/VA.dir/Kalman.cpp.i

CMakeFiles/VA.dir/Kalman.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/VA.dir/Kalman.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /media/iaitech/DATA/programs/20221212/Kalman.cpp -o CMakeFiles/VA.dir/Kalman.cpp.s

CMakeFiles/VA.dir/Multitracker.cpp.o: CMakeFiles/VA.dir/flags.make
CMakeFiles/VA.dir/Multitracker.cpp.o: Multitracker.cpp
CMakeFiles/VA.dir/Multitracker.cpp.o: CMakeFiles/VA.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/media/iaitech/DATA/programs/20221212/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object CMakeFiles/VA.dir/Multitracker.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/VA.dir/Multitracker.cpp.o -MF CMakeFiles/VA.dir/Multitracker.cpp.o.d -o CMakeFiles/VA.dir/Multitracker.cpp.o -c /media/iaitech/DATA/programs/20221212/Multitracker.cpp

CMakeFiles/VA.dir/Multitracker.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/VA.dir/Multitracker.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /media/iaitech/DATA/programs/20221212/Multitracker.cpp > CMakeFiles/VA.dir/Multitracker.cpp.i

CMakeFiles/VA.dir/Multitracker.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/VA.dir/Multitracker.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /media/iaitech/DATA/programs/20221212/Multitracker.cpp -o CMakeFiles/VA.dir/Multitracker.cpp.s

CMakeFiles/VA.dir/yolo_utils.cpp.o: CMakeFiles/VA.dir/flags.make
CMakeFiles/VA.dir/yolo_utils.cpp.o: yolo_utils.cpp
CMakeFiles/VA.dir/yolo_utils.cpp.o: CMakeFiles/VA.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/media/iaitech/DATA/programs/20221212/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object CMakeFiles/VA.dir/yolo_utils.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/VA.dir/yolo_utils.cpp.o -MF CMakeFiles/VA.dir/yolo_utils.cpp.o.d -o CMakeFiles/VA.dir/yolo_utils.cpp.o -c /media/iaitech/DATA/programs/20221212/yolo_utils.cpp

CMakeFiles/VA.dir/yolo_utils.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/VA.dir/yolo_utils.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /media/iaitech/DATA/programs/20221212/yolo_utils.cpp > CMakeFiles/VA.dir/yolo_utils.cpp.i

CMakeFiles/VA.dir/yolo_utils.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/VA.dir/yolo_utils.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /media/iaitech/DATA/programs/20221212/yolo_utils.cpp -o CMakeFiles/VA.dir/yolo_utils.cpp.s

CMakeFiles/VA.dir/Track.cpp.o: CMakeFiles/VA.dir/flags.make
CMakeFiles/VA.dir/Track.cpp.o: Track.cpp
CMakeFiles/VA.dir/Track.cpp.o: CMakeFiles/VA.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/media/iaitech/DATA/programs/20221212/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Building CXX object CMakeFiles/VA.dir/Track.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/VA.dir/Track.cpp.o -MF CMakeFiles/VA.dir/Track.cpp.o.d -o CMakeFiles/VA.dir/Track.cpp.o -c /media/iaitech/DATA/programs/20221212/Track.cpp

CMakeFiles/VA.dir/Track.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/VA.dir/Track.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /media/iaitech/DATA/programs/20221212/Track.cpp > CMakeFiles/VA.dir/Track.cpp.i

CMakeFiles/VA.dir/Track.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/VA.dir/Track.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /media/iaitech/DATA/programs/20221212/Track.cpp -o CMakeFiles/VA.dir/Track.cpp.s

CMakeFiles/VA.dir/Display.cpp.o: CMakeFiles/VA.dir/flags.make
CMakeFiles/VA.dir/Display.cpp.o: Display.cpp
CMakeFiles/VA.dir/Display.cpp.o: CMakeFiles/VA.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/media/iaitech/DATA/programs/20221212/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Building CXX object CMakeFiles/VA.dir/Display.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/VA.dir/Display.cpp.o -MF CMakeFiles/VA.dir/Display.cpp.o.d -o CMakeFiles/VA.dir/Display.cpp.o -c /media/iaitech/DATA/programs/20221212/Display.cpp

CMakeFiles/VA.dir/Display.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/VA.dir/Display.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /media/iaitech/DATA/programs/20221212/Display.cpp > CMakeFiles/VA.dir/Display.cpp.i

CMakeFiles/VA.dir/Display.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/VA.dir/Display.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /media/iaitech/DATA/programs/20221212/Display.cpp -o CMakeFiles/VA.dir/Display.cpp.s

CMakeFiles/VA.dir/SpeedEstimator.cpp.o: CMakeFiles/VA.dir/flags.make
CMakeFiles/VA.dir/SpeedEstimator.cpp.o: SpeedEstimator.cpp
CMakeFiles/VA.dir/SpeedEstimator.cpp.o: CMakeFiles/VA.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/media/iaitech/DATA/programs/20221212/CMakeFiles --progress-num=$(CMAKE_PROGRESS_9) "Building CXX object CMakeFiles/VA.dir/SpeedEstimator.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/VA.dir/SpeedEstimator.cpp.o -MF CMakeFiles/VA.dir/SpeedEstimator.cpp.o.d -o CMakeFiles/VA.dir/SpeedEstimator.cpp.o -c /media/iaitech/DATA/programs/20221212/SpeedEstimator.cpp

CMakeFiles/VA.dir/SpeedEstimator.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/VA.dir/SpeedEstimator.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /media/iaitech/DATA/programs/20221212/SpeedEstimator.cpp > CMakeFiles/VA.dir/SpeedEstimator.cpp.i

CMakeFiles/VA.dir/SpeedEstimator.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/VA.dir/SpeedEstimator.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /media/iaitech/DATA/programs/20221212/SpeedEstimator.cpp -o CMakeFiles/VA.dir/SpeedEstimator.cpp.s

CMakeFiles/VA.dir/Intersection.cpp.o: CMakeFiles/VA.dir/flags.make
CMakeFiles/VA.dir/Intersection.cpp.o: Intersection.cpp
CMakeFiles/VA.dir/Intersection.cpp.o: CMakeFiles/VA.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/media/iaitech/DATA/programs/20221212/CMakeFiles --progress-num=$(CMAKE_PROGRESS_10) "Building CXX object CMakeFiles/VA.dir/Intersection.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/VA.dir/Intersection.cpp.o -MF CMakeFiles/VA.dir/Intersection.cpp.o.d -o CMakeFiles/VA.dir/Intersection.cpp.o -c /media/iaitech/DATA/programs/20221212/Intersection.cpp

CMakeFiles/VA.dir/Intersection.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/VA.dir/Intersection.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /media/iaitech/DATA/programs/20221212/Intersection.cpp > CMakeFiles/VA.dir/Intersection.cpp.i

CMakeFiles/VA.dir/Intersection.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/VA.dir/Intersection.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /media/iaitech/DATA/programs/20221212/Intersection.cpp -o CMakeFiles/VA.dir/Intersection.cpp.s

CMakeFiles/VA.dir/MultiCams.cpp.o: CMakeFiles/VA.dir/flags.make
CMakeFiles/VA.dir/MultiCams.cpp.o: MultiCams.cpp
CMakeFiles/VA.dir/MultiCams.cpp.o: CMakeFiles/VA.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/media/iaitech/DATA/programs/20221212/CMakeFiles --progress-num=$(CMAKE_PROGRESS_11) "Building CXX object CMakeFiles/VA.dir/MultiCams.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/VA.dir/MultiCams.cpp.o -MF CMakeFiles/VA.dir/MultiCams.cpp.o.d -o CMakeFiles/VA.dir/MultiCams.cpp.o -c /media/iaitech/DATA/programs/20221212/MultiCams.cpp

CMakeFiles/VA.dir/MultiCams.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/VA.dir/MultiCams.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /media/iaitech/DATA/programs/20221212/MultiCams.cpp > CMakeFiles/VA.dir/MultiCams.cpp.i

CMakeFiles/VA.dir/MultiCams.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/VA.dir/MultiCams.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /media/iaitech/DATA/programs/20221212/MultiCams.cpp -o CMakeFiles/VA.dir/MultiCams.cpp.s

# Object files for target VA
VA_OBJECTS = \
"CMakeFiles/VA.dir/main.cpp.o" \
"CMakeFiles/VA.dir/detector.cpp.o" \
"CMakeFiles/VA.dir/HungarianAlg.cpp.o" \
"CMakeFiles/VA.dir/Kalman.cpp.o" \
"CMakeFiles/VA.dir/Multitracker.cpp.o" \
"CMakeFiles/VA.dir/yolo_utils.cpp.o" \
"CMakeFiles/VA.dir/Track.cpp.o" \
"CMakeFiles/VA.dir/Display.cpp.o" \
"CMakeFiles/VA.dir/SpeedEstimator.cpp.o" \
"CMakeFiles/VA.dir/Intersection.cpp.o" \
"CMakeFiles/VA.dir/MultiCams.cpp.o"

# External object files for target VA
VA_EXTERNAL_OBJECTS =

VA: CMakeFiles/VA.dir/main.cpp.o
VA: CMakeFiles/VA.dir/detector.cpp.o
VA: CMakeFiles/VA.dir/HungarianAlg.cpp.o
VA: CMakeFiles/VA.dir/Kalman.cpp.o
VA: CMakeFiles/VA.dir/Multitracker.cpp.o
VA: CMakeFiles/VA.dir/yolo_utils.cpp.o
VA: CMakeFiles/VA.dir/Track.cpp.o
VA: CMakeFiles/VA.dir/Display.cpp.o
VA: CMakeFiles/VA.dir/SpeedEstimator.cpp.o
VA: CMakeFiles/VA.dir/Intersection.cpp.o
VA: CMakeFiles/VA.dir/MultiCams.cpp.o
VA: CMakeFiles/VA.dir/build.make
VA: /home/iaitech/onnxruntime/build/Linux/Release/libonnxruntime.so
VA: /usr/local/lib/libopencv_gapi.so.4.6.0
VA: /usr/local/lib/libopencv_stitching.so.4.6.0
VA: /usr/local/lib/libopencv_aruco.so.4.6.0
VA: /usr/local/lib/libopencv_barcode.so.4.6.0
VA: /usr/local/lib/libopencv_bgsegm.so.4.6.0
VA: /usr/local/lib/libopencv_bioinspired.so.4.6.0
VA: /usr/local/lib/libopencv_ccalib.so.4.6.0
VA: /usr/local/lib/libopencv_cudabgsegm.so.4.6.0
VA: /usr/local/lib/libopencv_cudafeatures2d.so.4.6.0
VA: /usr/local/lib/libopencv_cudaobjdetect.so.4.6.0
VA: /usr/local/lib/libopencv_cudastereo.so.4.6.0
VA: /usr/local/lib/libopencv_dnn_objdetect.so.4.6.0
VA: /usr/local/lib/libopencv_dnn_superres.so.4.6.0
VA: /usr/local/lib/libopencv_dpm.so.4.6.0
VA: /usr/local/lib/libopencv_face.so.4.6.0
VA: /usr/local/lib/libopencv_freetype.so.4.6.0
VA: /usr/local/lib/libopencv_fuzzy.so.4.6.0
VA: /usr/local/lib/libopencv_hfs.so.4.6.0
VA: /usr/local/lib/libopencv_img_hash.so.4.6.0
VA: /usr/local/lib/libopencv_intensity_transform.so.4.6.0
VA: /usr/local/lib/libopencv_line_descriptor.so.4.6.0
VA: /usr/local/lib/libopencv_mcc.so.4.6.0
VA: /usr/local/lib/libopencv_quality.so.4.6.0
VA: /usr/local/lib/libopencv_rapid.so.4.6.0
VA: /usr/local/lib/libopencv_reg.so.4.6.0
VA: /usr/local/lib/libopencv_rgbd.so.4.6.0
VA: /usr/local/lib/libopencv_saliency.so.4.6.0
VA: /usr/local/lib/libopencv_stereo.so.4.6.0
VA: /usr/local/lib/libopencv_structured_light.so.4.6.0
VA: /usr/local/lib/libopencv_superres.so.4.6.0
VA: /usr/local/lib/libopencv_surface_matching.so.4.6.0
VA: /usr/local/lib/libopencv_tracking.so.4.6.0
VA: /usr/local/lib/libopencv_videostab.so.4.6.0
VA: /usr/local/lib/libopencv_wechat_qrcode.so.4.6.0
VA: /usr/local/lib/libopencv_xfeatures2d.so.4.6.0
VA: /usr/local/lib/libopencv_xobjdetect.so.4.6.0
VA: /usr/local/lib/libopencv_xphoto.so.4.6.0
VA: /usr/local/lib/libopencv_shape.so.4.6.0
VA: /usr/local/lib/libopencv_highgui.so.4.6.0
VA: /usr/local/lib/libopencv_datasets.so.4.6.0
VA: /usr/local/lib/libopencv_plot.so.4.6.0
VA: /usr/local/lib/libopencv_text.so.4.6.0
VA: /usr/local/lib/libopencv_ml.so.4.6.0
VA: /usr/local/lib/libopencv_phase_unwrapping.so.4.6.0
VA: /usr/local/lib/libopencv_cudacodec.so.4.6.0
VA: /usr/local/lib/libopencv_videoio.so.4.6.0
VA: /usr/local/lib/libopencv_cudaoptflow.so.4.6.0
VA: /usr/local/lib/libopencv_cudalegacy.so.4.6.0
VA: /usr/local/lib/libopencv_cudawarping.so.4.6.0
VA: /usr/local/lib/libopencv_optflow.so.4.6.0
VA: /usr/local/lib/libopencv_ximgproc.so.4.6.0
VA: /usr/local/lib/libopencv_video.so.4.6.0
VA: /usr/local/lib/libopencv_imgcodecs.so.4.6.0
VA: /usr/local/lib/libopencv_objdetect.so.4.6.0
VA: /usr/local/lib/libopencv_calib3d.so.4.6.0
VA: /usr/local/lib/libopencv_dnn.so.4.6.0
VA: /usr/local/lib/libopencv_features2d.so.4.6.0
VA: /usr/local/lib/libopencv_flann.so.4.6.0
VA: /usr/local/lib/libopencv_photo.so.4.6.0
VA: /usr/local/lib/libopencv_cudaimgproc.so.4.6.0
VA: /usr/local/lib/libopencv_cudafilters.so.4.6.0
VA: /usr/local/lib/libopencv_imgproc.so.4.6.0
VA: /usr/local/lib/libopencv_cudaarithm.so.4.6.0
VA: /usr/local/lib/libopencv_core.so.4.6.0
VA: /usr/local/lib/libopencv_cudev.so.4.6.0
VA: CMakeFiles/VA.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/media/iaitech/DATA/programs/20221212/CMakeFiles --progress-num=$(CMAKE_PROGRESS_12) "Linking CXX executable VA"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/VA.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/VA.dir/build: VA
.PHONY : CMakeFiles/VA.dir/build

CMakeFiles/VA.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/VA.dir/cmake_clean.cmake
.PHONY : CMakeFiles/VA.dir/clean

CMakeFiles/VA.dir/depend:
	cd /media/iaitech/DATA/programs/20221212 && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /media/iaitech/DATA/programs/20221212 /media/iaitech/DATA/programs/20221212 /media/iaitech/DATA/programs/20221212 /media/iaitech/DATA/programs/20221212 /media/iaitech/DATA/programs/20221212/CMakeFiles/VA.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/VA.dir/depend

