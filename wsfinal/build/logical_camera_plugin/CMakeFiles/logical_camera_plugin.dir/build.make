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
CMAKE_SOURCE_DIR = /home/iaguiluz/UCMEng_Collaboratory03/wsfinal/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/iaguiluz/UCMEng_Collaboratory03/wsfinal/build

# Include any dependencies generated for this target.
include logical_camera_plugin/CMakeFiles/logical_camera_plugin.dir/depend.make

# Include the progress variables for this target.
include logical_camera_plugin/CMakeFiles/logical_camera_plugin.dir/progress.make

# Include the compile flags for this target's objects.
include logical_camera_plugin/CMakeFiles/logical_camera_plugin.dir/flags.make

logical_camera_plugin/CMakeFiles/logical_camera_plugin.dir/src/logical_camera_plugin.cpp.o: logical_camera_plugin/CMakeFiles/logical_camera_plugin.dir/flags.make
logical_camera_plugin/CMakeFiles/logical_camera_plugin.dir/src/logical_camera_plugin.cpp.o: /home/iaguiluz/UCMEng_Collaboratory03/wsfinal/src/logical_camera_plugin/src/logical_camera_plugin.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/iaguiluz/UCMEng_Collaboratory03/wsfinal/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object logical_camera_plugin/CMakeFiles/logical_camera_plugin.dir/src/logical_camera_plugin.cpp.o"
	cd /home/iaguiluz/UCMEng_Collaboratory03/wsfinal/build/logical_camera_plugin && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/logical_camera_plugin.dir/src/logical_camera_plugin.cpp.o -c /home/iaguiluz/UCMEng_Collaboratory03/wsfinal/src/logical_camera_plugin/src/logical_camera_plugin.cpp

logical_camera_plugin/CMakeFiles/logical_camera_plugin.dir/src/logical_camera_plugin.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/logical_camera_plugin.dir/src/logical_camera_plugin.cpp.i"
	cd /home/iaguiluz/UCMEng_Collaboratory03/wsfinal/build/logical_camera_plugin && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/iaguiluz/UCMEng_Collaboratory03/wsfinal/src/logical_camera_plugin/src/logical_camera_plugin.cpp > CMakeFiles/logical_camera_plugin.dir/src/logical_camera_plugin.cpp.i

logical_camera_plugin/CMakeFiles/logical_camera_plugin.dir/src/logical_camera_plugin.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/logical_camera_plugin.dir/src/logical_camera_plugin.cpp.s"
	cd /home/iaguiluz/UCMEng_Collaboratory03/wsfinal/build/logical_camera_plugin && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/iaguiluz/UCMEng_Collaboratory03/wsfinal/src/logical_camera_plugin/src/logical_camera_plugin.cpp -o CMakeFiles/logical_camera_plugin.dir/src/logical_camera_plugin.cpp.s

logical_camera_plugin/CMakeFiles/logical_camera_plugin.dir/src/logical_camera_plugin.cpp.o.requires:

.PHONY : logical_camera_plugin/CMakeFiles/logical_camera_plugin.dir/src/logical_camera_plugin.cpp.o.requires

logical_camera_plugin/CMakeFiles/logical_camera_plugin.dir/src/logical_camera_plugin.cpp.o.provides: logical_camera_plugin/CMakeFiles/logical_camera_plugin.dir/src/logical_camera_plugin.cpp.o.requires
	$(MAKE) -f logical_camera_plugin/CMakeFiles/logical_camera_plugin.dir/build.make logical_camera_plugin/CMakeFiles/logical_camera_plugin.dir/src/logical_camera_plugin.cpp.o.provides.build
.PHONY : logical_camera_plugin/CMakeFiles/logical_camera_plugin.dir/src/logical_camera_plugin.cpp.o.provides

logical_camera_plugin/CMakeFiles/logical_camera_plugin.dir/src/logical_camera_plugin.cpp.o.provides.build: logical_camera_plugin/CMakeFiles/logical_camera_plugin.dir/src/logical_camera_plugin.cpp.o


# Object files for target logical_camera_plugin
logical_camera_plugin_OBJECTS = \
"CMakeFiles/logical_camera_plugin.dir/src/logical_camera_plugin.cpp.o"

# External object files for target logical_camera_plugin
logical_camera_plugin_EXTERNAL_OBJECTS =

/home/iaguiluz/UCMEng_Collaboratory03/wsfinal/devel/lib/liblogical_camera_plugin.so: logical_camera_plugin/CMakeFiles/logical_camera_plugin.dir/src/logical_camera_plugin.cpp.o
/home/iaguiluz/UCMEng_Collaboratory03/wsfinal/devel/lib/liblogical_camera_plugin.so: logical_camera_plugin/CMakeFiles/logical_camera_plugin.dir/build.make
/home/iaguiluz/UCMEng_Collaboratory03/wsfinal/devel/lib/liblogical_camera_plugin.so: /opt/ros/kinetic/lib/libvision_reconfigure.so
/home/iaguiluz/UCMEng_Collaboratory03/wsfinal/devel/lib/liblogical_camera_plugin.so: /opt/ros/kinetic/lib/libgazebo_ros_utils.so
/home/iaguiluz/UCMEng_Collaboratory03/wsfinal/devel/lib/liblogical_camera_plugin.so: /opt/ros/kinetic/lib/libgazebo_ros_camera_utils.so
/home/iaguiluz/UCMEng_Collaboratory03/wsfinal/devel/lib/liblogical_camera_plugin.so: /opt/ros/kinetic/lib/libgazebo_ros_camera.so
/home/iaguiluz/UCMEng_Collaboratory03/wsfinal/devel/lib/liblogical_camera_plugin.so: /opt/ros/kinetic/lib/libgazebo_ros_multicamera.so
/home/iaguiluz/UCMEng_Collaboratory03/wsfinal/devel/lib/liblogical_camera_plugin.so: /opt/ros/kinetic/lib/libgazebo_ros_depth_camera.so
/home/iaguiluz/UCMEng_Collaboratory03/wsfinal/devel/lib/liblogical_camera_plugin.so: /opt/ros/kinetic/lib/libgazebo_ros_openni_kinect.so
/home/iaguiluz/UCMEng_Collaboratory03/wsfinal/devel/lib/liblogical_camera_plugin.so: /opt/ros/kinetic/lib/libgazebo_ros_gpu_laser.so
/home/iaguiluz/UCMEng_Collaboratory03/wsfinal/devel/lib/liblogical_camera_plugin.so: /opt/ros/kinetic/lib/libgazebo_ros_laser.so
/home/iaguiluz/UCMEng_Collaboratory03/wsfinal/devel/lib/liblogical_camera_plugin.so: /opt/ros/kinetic/lib/libgazebo_ros_block_laser.so
/home/iaguiluz/UCMEng_Collaboratory03/wsfinal/devel/lib/liblogical_camera_plugin.so: /opt/ros/kinetic/lib/libgazebo_ros_p3d.so
/home/iaguiluz/UCMEng_Collaboratory03/wsfinal/devel/lib/liblogical_camera_plugin.so: /opt/ros/kinetic/lib/libgazebo_ros_imu.so
/home/iaguiluz/UCMEng_Collaboratory03/wsfinal/devel/lib/liblogical_camera_plugin.so: /opt/ros/kinetic/lib/libgazebo_ros_f3d.so
/home/iaguiluz/UCMEng_Collaboratory03/wsfinal/devel/lib/liblogical_camera_plugin.so: /opt/ros/kinetic/lib/libgazebo_ros_ft_sensor.so
/home/iaguiluz/UCMEng_Collaboratory03/wsfinal/devel/lib/liblogical_camera_plugin.so: /opt/ros/kinetic/lib/libgazebo_ros_bumper.so
/home/iaguiluz/UCMEng_Collaboratory03/wsfinal/devel/lib/liblogical_camera_plugin.so: /opt/ros/kinetic/lib/libgazebo_ros_template.so
/home/iaguiluz/UCMEng_Collaboratory03/wsfinal/devel/lib/liblogical_camera_plugin.so: /opt/ros/kinetic/lib/libgazebo_ros_projector.so
/home/iaguiluz/UCMEng_Collaboratory03/wsfinal/devel/lib/liblogical_camera_plugin.so: /opt/ros/kinetic/lib/libgazebo_ros_prosilica.so
/home/iaguiluz/UCMEng_Collaboratory03/wsfinal/devel/lib/liblogical_camera_plugin.so: /opt/ros/kinetic/lib/libgazebo_ros_force.so
/home/iaguiluz/UCMEng_Collaboratory03/wsfinal/devel/lib/liblogical_camera_plugin.so: /opt/ros/kinetic/lib/libgazebo_ros_joint_trajectory.so
/home/iaguiluz/UCMEng_Collaboratory03/wsfinal/devel/lib/liblogical_camera_plugin.so: /opt/ros/kinetic/lib/libgazebo_ros_joint_state_publisher.so
/home/iaguiluz/UCMEng_Collaboratory03/wsfinal/devel/lib/liblogical_camera_plugin.so: /opt/ros/kinetic/lib/libgazebo_ros_joint_pose_trajectory.so
/home/iaguiluz/UCMEng_Collaboratory03/wsfinal/devel/lib/liblogical_camera_plugin.so: /opt/ros/kinetic/lib/libgazebo_ros_diff_drive.so
/home/iaguiluz/UCMEng_Collaboratory03/wsfinal/devel/lib/liblogical_camera_plugin.so: /opt/ros/kinetic/lib/libgazebo_ros_tricycle_drive.so
/home/iaguiluz/UCMEng_Collaboratory03/wsfinal/devel/lib/liblogical_camera_plugin.so: /opt/ros/kinetic/lib/libgazebo_ros_skid_steer_drive.so
/home/iaguiluz/UCMEng_Collaboratory03/wsfinal/devel/lib/liblogical_camera_plugin.so: /opt/ros/kinetic/lib/libgazebo_ros_video.so
/home/iaguiluz/UCMEng_Collaboratory03/wsfinal/devel/lib/liblogical_camera_plugin.so: /opt/ros/kinetic/lib/libgazebo_ros_planar_move.so
/home/iaguiluz/UCMEng_Collaboratory03/wsfinal/devel/lib/liblogical_camera_plugin.so: /opt/ros/kinetic/lib/libgazebo_ros_vacuum_gripper.so
/home/iaguiluz/UCMEng_Collaboratory03/wsfinal/devel/lib/liblogical_camera_plugin.so: /opt/ros/kinetic/lib/libnodeletlib.so
/home/iaguiluz/UCMEng_Collaboratory03/wsfinal/devel/lib/liblogical_camera_plugin.so: /opt/ros/kinetic/lib/libbondcpp.so
/home/iaguiluz/UCMEng_Collaboratory03/wsfinal/devel/lib/liblogical_camera_plugin.so: /usr/lib/x86_64-linux-gnu/libuuid.so
/home/iaguiluz/UCMEng_Collaboratory03/wsfinal/devel/lib/liblogical_camera_plugin.so: /opt/ros/kinetic/lib/liburdf.so
/home/iaguiluz/UCMEng_Collaboratory03/wsfinal/devel/lib/liblogical_camera_plugin.so: /usr/lib/x86_64-linux-gnu/liburdfdom_sensor.so
/home/iaguiluz/UCMEng_Collaboratory03/wsfinal/devel/lib/liblogical_camera_plugin.so: /usr/lib/x86_64-linux-gnu/liburdfdom_model_state.so
/home/iaguiluz/UCMEng_Collaboratory03/wsfinal/devel/lib/liblogical_camera_plugin.so: /usr/lib/x86_64-linux-gnu/liburdfdom_model.so
/home/iaguiluz/UCMEng_Collaboratory03/wsfinal/devel/lib/liblogical_camera_plugin.so: /usr/lib/x86_64-linux-gnu/liburdfdom_world.so
/home/iaguiluz/UCMEng_Collaboratory03/wsfinal/devel/lib/liblogical_camera_plugin.so: /opt/ros/kinetic/lib/librosconsole_bridge.so
/home/iaguiluz/UCMEng_Collaboratory03/wsfinal/devel/lib/liblogical_camera_plugin.so: /opt/ros/kinetic/lib/libimage_transport.so
/home/iaguiluz/UCMEng_Collaboratory03/wsfinal/devel/lib/liblogical_camera_plugin.so: /opt/ros/kinetic/lib/libclass_loader.so
/home/iaguiluz/UCMEng_Collaboratory03/wsfinal/devel/lib/liblogical_camera_plugin.so: /usr/lib/libPocoFoundation.so
/home/iaguiluz/UCMEng_Collaboratory03/wsfinal/devel/lib/liblogical_camera_plugin.so: /usr/lib/x86_64-linux-gnu/libdl.so
/home/iaguiluz/UCMEng_Collaboratory03/wsfinal/devel/lib/liblogical_camera_plugin.so: /opt/ros/kinetic/lib/libcamera_info_manager.so
/home/iaguiluz/UCMEng_Collaboratory03/wsfinal/devel/lib/liblogical_camera_plugin.so: /opt/ros/kinetic/lib/libgazebo_ros_api_plugin.so
/home/iaguiluz/UCMEng_Collaboratory03/wsfinal/devel/lib/liblogical_camera_plugin.so: /opt/ros/kinetic/lib/libgazebo_ros_paths_plugin.so
/home/iaguiluz/UCMEng_Collaboratory03/wsfinal/devel/lib/liblogical_camera_plugin.so: /opt/ros/kinetic/lib/libroslib.so
/home/iaguiluz/UCMEng_Collaboratory03/wsfinal/devel/lib/liblogical_camera_plugin.so: /opt/ros/kinetic/lib/librospack.so
/home/iaguiluz/UCMEng_Collaboratory03/wsfinal/devel/lib/liblogical_camera_plugin.so: /usr/lib/x86_64-linux-gnu/libpython2.7.so
/home/iaguiluz/UCMEng_Collaboratory03/wsfinal/devel/lib/liblogical_camera_plugin.so: /usr/lib/x86_64-linux-gnu/libboost_program_options.so
/home/iaguiluz/UCMEng_Collaboratory03/wsfinal/devel/lib/liblogical_camera_plugin.so: /usr/lib/x86_64-linux-gnu/libtinyxml.so
/home/iaguiluz/UCMEng_Collaboratory03/wsfinal/devel/lib/liblogical_camera_plugin.so: /opt/ros/kinetic/lib/libtf.so
/home/iaguiluz/UCMEng_Collaboratory03/wsfinal/devel/lib/liblogical_camera_plugin.so: /opt/ros/kinetic/lib/libtf2_ros.so
/home/iaguiluz/UCMEng_Collaboratory03/wsfinal/devel/lib/liblogical_camera_plugin.so: /opt/ros/kinetic/lib/libactionlib.so
/home/iaguiluz/UCMEng_Collaboratory03/wsfinal/devel/lib/liblogical_camera_plugin.so: /opt/ros/kinetic/lib/libmessage_filters.so
/home/iaguiluz/UCMEng_Collaboratory03/wsfinal/devel/lib/liblogical_camera_plugin.so: /opt/ros/kinetic/lib/libtf2.so
/home/iaguiluz/UCMEng_Collaboratory03/wsfinal/devel/lib/liblogical_camera_plugin.so: /opt/ros/kinetic/lib/libdynamic_reconfigure_config_init_mutex.so
/home/iaguiluz/UCMEng_Collaboratory03/wsfinal/devel/lib/liblogical_camera_plugin.so: /opt/ros/kinetic/lib/libroscpp.so
/home/iaguiluz/UCMEng_Collaboratory03/wsfinal/devel/lib/liblogical_camera_plugin.so: /usr/lib/x86_64-linux-gnu/libboost_signals.so
/home/iaguiluz/UCMEng_Collaboratory03/wsfinal/devel/lib/liblogical_camera_plugin.so: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
/home/iaguiluz/UCMEng_Collaboratory03/wsfinal/devel/lib/liblogical_camera_plugin.so: /opt/ros/kinetic/lib/librosconsole.so
/home/iaguiluz/UCMEng_Collaboratory03/wsfinal/devel/lib/liblogical_camera_plugin.so: /opt/ros/kinetic/lib/librosconsole_log4cxx.so
/home/iaguiluz/UCMEng_Collaboratory03/wsfinal/devel/lib/liblogical_camera_plugin.so: /opt/ros/kinetic/lib/librosconsole_backend_interface.so
/home/iaguiluz/UCMEng_Collaboratory03/wsfinal/devel/lib/liblogical_camera_plugin.so: /usr/lib/x86_64-linux-gnu/liblog4cxx.so
/home/iaguiluz/UCMEng_Collaboratory03/wsfinal/devel/lib/liblogical_camera_plugin.so: /usr/lib/x86_64-linux-gnu/libboost_regex.so
/home/iaguiluz/UCMEng_Collaboratory03/wsfinal/devel/lib/liblogical_camera_plugin.so: /opt/ros/kinetic/lib/libroscpp_serialization.so
/home/iaguiluz/UCMEng_Collaboratory03/wsfinal/devel/lib/liblogical_camera_plugin.so: /opt/ros/kinetic/lib/librostime.so
/home/iaguiluz/UCMEng_Collaboratory03/wsfinal/devel/lib/liblogical_camera_plugin.so: /opt/ros/kinetic/lib/libxmlrpcpp.so
/home/iaguiluz/UCMEng_Collaboratory03/wsfinal/devel/lib/liblogical_camera_plugin.so: /opt/ros/kinetic/lib/libcpp_common.so
/home/iaguiluz/UCMEng_Collaboratory03/wsfinal/devel/lib/liblogical_camera_plugin.so: /usr/lib/x86_64-linux-gnu/libboost_system.so
/home/iaguiluz/UCMEng_Collaboratory03/wsfinal/devel/lib/liblogical_camera_plugin.so: /usr/lib/x86_64-linux-gnu/libboost_thread.so
/home/iaguiluz/UCMEng_Collaboratory03/wsfinal/devel/lib/liblogical_camera_plugin.so: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
/home/iaguiluz/UCMEng_Collaboratory03/wsfinal/devel/lib/liblogical_camera_plugin.so: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
/home/iaguiluz/UCMEng_Collaboratory03/wsfinal/devel/lib/liblogical_camera_plugin.so: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
/home/iaguiluz/UCMEng_Collaboratory03/wsfinal/devel/lib/liblogical_camera_plugin.so: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/iaguiluz/UCMEng_Collaboratory03/wsfinal/devel/lib/liblogical_camera_plugin.so: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so
/home/iaguiluz/UCMEng_Collaboratory03/wsfinal/devel/lib/liblogical_camera_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo.so
/home/iaguiluz/UCMEng_Collaboratory03/wsfinal/devel/lib/liblogical_camera_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_client.so
/home/iaguiluz/UCMEng_Collaboratory03/wsfinal/devel/lib/liblogical_camera_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_gui.so
/home/iaguiluz/UCMEng_Collaboratory03/wsfinal/devel/lib/liblogical_camera_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_sensors.so
/home/iaguiluz/UCMEng_Collaboratory03/wsfinal/devel/lib/liblogical_camera_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_rendering.so
/home/iaguiluz/UCMEng_Collaboratory03/wsfinal/devel/lib/liblogical_camera_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_physics.so
/home/iaguiluz/UCMEng_Collaboratory03/wsfinal/devel/lib/liblogical_camera_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_ode.so
/home/iaguiluz/UCMEng_Collaboratory03/wsfinal/devel/lib/liblogical_camera_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_transport.so
/home/iaguiluz/UCMEng_Collaboratory03/wsfinal/devel/lib/liblogical_camera_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_msgs.so
/home/iaguiluz/UCMEng_Collaboratory03/wsfinal/devel/lib/liblogical_camera_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_util.so
/home/iaguiluz/UCMEng_Collaboratory03/wsfinal/devel/lib/liblogical_camera_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_common.so
/home/iaguiluz/UCMEng_Collaboratory03/wsfinal/devel/lib/liblogical_camera_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_gimpact.so
/home/iaguiluz/UCMEng_Collaboratory03/wsfinal/devel/lib/liblogical_camera_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_opcode.so
/home/iaguiluz/UCMEng_Collaboratory03/wsfinal/devel/lib/liblogical_camera_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_opende_ou.so
/home/iaguiluz/UCMEng_Collaboratory03/wsfinal/devel/lib/liblogical_camera_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_math.so
/home/iaguiluz/UCMEng_Collaboratory03/wsfinal/devel/lib/liblogical_camera_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_ccd.so
/home/iaguiluz/UCMEng_Collaboratory03/wsfinal/devel/lib/liblogical_camera_plugin.so: /usr/lib/x86_64-linux-gnu/libboost_thread.so
/home/iaguiluz/UCMEng_Collaboratory03/wsfinal/devel/lib/liblogical_camera_plugin.so: /usr/lib/x86_64-linux-gnu/libboost_signals.so
/home/iaguiluz/UCMEng_Collaboratory03/wsfinal/devel/lib/liblogical_camera_plugin.so: /usr/lib/x86_64-linux-gnu/libboost_system.so
/home/iaguiluz/UCMEng_Collaboratory03/wsfinal/devel/lib/liblogical_camera_plugin.so: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
/home/iaguiluz/UCMEng_Collaboratory03/wsfinal/devel/lib/liblogical_camera_plugin.so: /usr/lib/x86_64-linux-gnu/libboost_program_options.so
/home/iaguiluz/UCMEng_Collaboratory03/wsfinal/devel/lib/liblogical_camera_plugin.so: /usr/lib/x86_64-linux-gnu/libboost_regex.so
/home/iaguiluz/UCMEng_Collaboratory03/wsfinal/devel/lib/liblogical_camera_plugin.so: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
/home/iaguiluz/UCMEng_Collaboratory03/wsfinal/devel/lib/liblogical_camera_plugin.so: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
/home/iaguiluz/UCMEng_Collaboratory03/wsfinal/devel/lib/liblogical_camera_plugin.so: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
/home/iaguiluz/UCMEng_Collaboratory03/wsfinal/devel/lib/liblogical_camera_plugin.so: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
/home/iaguiluz/UCMEng_Collaboratory03/wsfinal/devel/lib/liblogical_camera_plugin.so: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/iaguiluz/UCMEng_Collaboratory03/wsfinal/devel/lib/liblogical_camera_plugin.so: /usr/lib/x86_64-linux-gnu/libprotobuf.so
/home/iaguiluz/UCMEng_Collaboratory03/wsfinal/devel/lib/liblogical_camera_plugin.so: /usr/lib/x86_64-linux-gnu/libsdformat.so
/home/iaguiluz/UCMEng_Collaboratory03/wsfinal/devel/lib/liblogical_camera_plugin.so: /usr/lib/x86_64-linux-gnu/libignition-math2.so
/home/iaguiluz/UCMEng_Collaboratory03/wsfinal/devel/lib/liblogical_camera_plugin.so: /usr/lib/x86_64-linux-gnu/libOgreMain.so
/home/iaguiluz/UCMEng_Collaboratory03/wsfinal/devel/lib/liblogical_camera_plugin.so: /usr/lib/x86_64-linux-gnu/libboost_thread.so
/home/iaguiluz/UCMEng_Collaboratory03/wsfinal/devel/lib/liblogical_camera_plugin.so: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
/home/iaguiluz/UCMEng_Collaboratory03/wsfinal/devel/lib/liblogical_camera_plugin.so: /usr/lib/x86_64-linux-gnu/libboost_system.so
/home/iaguiluz/UCMEng_Collaboratory03/wsfinal/devel/lib/liblogical_camera_plugin.so: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
/home/iaguiluz/UCMEng_Collaboratory03/wsfinal/devel/lib/liblogical_camera_plugin.so: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
/home/iaguiluz/UCMEng_Collaboratory03/wsfinal/devel/lib/liblogical_camera_plugin.so: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/iaguiluz/UCMEng_Collaboratory03/wsfinal/devel/lib/liblogical_camera_plugin.so: /usr/lib/x86_64-linux-gnu/libOgreTerrain.so
/home/iaguiluz/UCMEng_Collaboratory03/wsfinal/devel/lib/liblogical_camera_plugin.so: /usr/lib/x86_64-linux-gnu/libOgrePaging.so
/home/iaguiluz/UCMEng_Collaboratory03/wsfinal/devel/lib/liblogical_camera_plugin.so: /usr/lib/x86_64-linux-gnu/libignition-math2.so
/home/iaguiluz/UCMEng_Collaboratory03/wsfinal/devel/lib/liblogical_camera_plugin.so: /usr/lib/x86_64-linux-gnu/libboost_program_options.so
/home/iaguiluz/UCMEng_Collaboratory03/wsfinal/devel/lib/liblogical_camera_plugin.so: /usr/lib/x86_64-linux-gnu/libtinyxml.so
/home/iaguiluz/UCMEng_Collaboratory03/wsfinal/devel/lib/liblogical_camera_plugin.so: /opt/ros/kinetic/lib/libtf.so
/home/iaguiluz/UCMEng_Collaboratory03/wsfinal/devel/lib/liblogical_camera_plugin.so: /opt/ros/kinetic/lib/libtf2_ros.so
/home/iaguiluz/UCMEng_Collaboratory03/wsfinal/devel/lib/liblogical_camera_plugin.so: /opt/ros/kinetic/lib/libactionlib.so
/home/iaguiluz/UCMEng_Collaboratory03/wsfinal/devel/lib/liblogical_camera_plugin.so: /opt/ros/kinetic/lib/libmessage_filters.so
/home/iaguiluz/UCMEng_Collaboratory03/wsfinal/devel/lib/liblogical_camera_plugin.so: /opt/ros/kinetic/lib/libtf2.so
/home/iaguiluz/UCMEng_Collaboratory03/wsfinal/devel/lib/liblogical_camera_plugin.so: /opt/ros/kinetic/lib/libdynamic_reconfigure_config_init_mutex.so
/home/iaguiluz/UCMEng_Collaboratory03/wsfinal/devel/lib/liblogical_camera_plugin.so: /opt/ros/kinetic/lib/libroscpp.so
/home/iaguiluz/UCMEng_Collaboratory03/wsfinal/devel/lib/liblogical_camera_plugin.so: /usr/lib/x86_64-linux-gnu/libboost_signals.so
/home/iaguiluz/UCMEng_Collaboratory03/wsfinal/devel/lib/liblogical_camera_plugin.so: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
/home/iaguiluz/UCMEng_Collaboratory03/wsfinal/devel/lib/liblogical_camera_plugin.so: /opt/ros/kinetic/lib/librosconsole.so
/home/iaguiluz/UCMEng_Collaboratory03/wsfinal/devel/lib/liblogical_camera_plugin.so: /opt/ros/kinetic/lib/librosconsole_log4cxx.so
/home/iaguiluz/UCMEng_Collaboratory03/wsfinal/devel/lib/liblogical_camera_plugin.so: /opt/ros/kinetic/lib/librosconsole_backend_interface.so
/home/iaguiluz/UCMEng_Collaboratory03/wsfinal/devel/lib/liblogical_camera_plugin.so: /usr/lib/x86_64-linux-gnu/liblog4cxx.so
/home/iaguiluz/UCMEng_Collaboratory03/wsfinal/devel/lib/liblogical_camera_plugin.so: /usr/lib/x86_64-linux-gnu/libboost_regex.so
/home/iaguiluz/UCMEng_Collaboratory03/wsfinal/devel/lib/liblogical_camera_plugin.so: /opt/ros/kinetic/lib/libroscpp_serialization.so
/home/iaguiluz/UCMEng_Collaboratory03/wsfinal/devel/lib/liblogical_camera_plugin.so: /opt/ros/kinetic/lib/librostime.so
/home/iaguiluz/UCMEng_Collaboratory03/wsfinal/devel/lib/liblogical_camera_plugin.so: /opt/ros/kinetic/lib/libxmlrpcpp.so
/home/iaguiluz/UCMEng_Collaboratory03/wsfinal/devel/lib/liblogical_camera_plugin.so: /opt/ros/kinetic/lib/libcpp_common.so
/home/iaguiluz/UCMEng_Collaboratory03/wsfinal/devel/lib/liblogical_camera_plugin.so: /usr/lib/x86_64-linux-gnu/libboost_system.so
/home/iaguiluz/UCMEng_Collaboratory03/wsfinal/devel/lib/liblogical_camera_plugin.so: /usr/lib/x86_64-linux-gnu/libboost_thread.so
/home/iaguiluz/UCMEng_Collaboratory03/wsfinal/devel/lib/liblogical_camera_plugin.so: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
/home/iaguiluz/UCMEng_Collaboratory03/wsfinal/devel/lib/liblogical_camera_plugin.so: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
/home/iaguiluz/UCMEng_Collaboratory03/wsfinal/devel/lib/liblogical_camera_plugin.so: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
/home/iaguiluz/UCMEng_Collaboratory03/wsfinal/devel/lib/liblogical_camera_plugin.so: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/iaguiluz/UCMEng_Collaboratory03/wsfinal/devel/lib/liblogical_camera_plugin.so: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so
/home/iaguiluz/UCMEng_Collaboratory03/wsfinal/devel/lib/liblogical_camera_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo.so
/home/iaguiluz/UCMEng_Collaboratory03/wsfinal/devel/lib/liblogical_camera_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_client.so
/home/iaguiluz/UCMEng_Collaboratory03/wsfinal/devel/lib/liblogical_camera_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_gui.so
/home/iaguiluz/UCMEng_Collaboratory03/wsfinal/devel/lib/liblogical_camera_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_sensors.so
/home/iaguiluz/UCMEng_Collaboratory03/wsfinal/devel/lib/liblogical_camera_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_rendering.so
/home/iaguiluz/UCMEng_Collaboratory03/wsfinal/devel/lib/liblogical_camera_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_physics.so
/home/iaguiluz/UCMEng_Collaboratory03/wsfinal/devel/lib/liblogical_camera_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_ode.so
/home/iaguiluz/UCMEng_Collaboratory03/wsfinal/devel/lib/liblogical_camera_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_transport.so
/home/iaguiluz/UCMEng_Collaboratory03/wsfinal/devel/lib/liblogical_camera_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_msgs.so
/home/iaguiluz/UCMEng_Collaboratory03/wsfinal/devel/lib/liblogical_camera_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_util.so
/home/iaguiluz/UCMEng_Collaboratory03/wsfinal/devel/lib/liblogical_camera_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_common.so
/home/iaguiluz/UCMEng_Collaboratory03/wsfinal/devel/lib/liblogical_camera_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_gimpact.so
/home/iaguiluz/UCMEng_Collaboratory03/wsfinal/devel/lib/liblogical_camera_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_opcode.so
/home/iaguiluz/UCMEng_Collaboratory03/wsfinal/devel/lib/liblogical_camera_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_opende_ou.so
/home/iaguiluz/UCMEng_Collaboratory03/wsfinal/devel/lib/liblogical_camera_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_math.so
/home/iaguiluz/UCMEng_Collaboratory03/wsfinal/devel/lib/liblogical_camera_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_ccd.so
/home/iaguiluz/UCMEng_Collaboratory03/wsfinal/devel/lib/liblogical_camera_plugin.so: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
/home/iaguiluz/UCMEng_Collaboratory03/wsfinal/devel/lib/liblogical_camera_plugin.so: /usr/lib/x86_64-linux-gnu/libprotobuf.so
/home/iaguiluz/UCMEng_Collaboratory03/wsfinal/devel/lib/liblogical_camera_plugin.so: /usr/lib/x86_64-linux-gnu/libsdformat.so
/home/iaguiluz/UCMEng_Collaboratory03/wsfinal/devel/lib/liblogical_camera_plugin.so: /usr/lib/x86_64-linux-gnu/libOgreMain.so
/home/iaguiluz/UCMEng_Collaboratory03/wsfinal/devel/lib/liblogical_camera_plugin.so: /usr/lib/x86_64-linux-gnu/libOgreTerrain.so
/home/iaguiluz/UCMEng_Collaboratory03/wsfinal/devel/lib/liblogical_camera_plugin.so: /usr/lib/x86_64-linux-gnu/libOgrePaging.so
/home/iaguiluz/UCMEng_Collaboratory03/wsfinal/devel/lib/liblogical_camera_plugin.so: logical_camera_plugin/CMakeFiles/logical_camera_plugin.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/iaguiluz/UCMEng_Collaboratory03/wsfinal/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX shared library /home/iaguiluz/UCMEng_Collaboratory03/wsfinal/devel/lib/liblogical_camera_plugin.so"
	cd /home/iaguiluz/UCMEng_Collaboratory03/wsfinal/build/logical_camera_plugin && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/logical_camera_plugin.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
logical_camera_plugin/CMakeFiles/logical_camera_plugin.dir/build: /home/iaguiluz/UCMEng_Collaboratory03/wsfinal/devel/lib/liblogical_camera_plugin.so

.PHONY : logical_camera_plugin/CMakeFiles/logical_camera_plugin.dir/build

logical_camera_plugin/CMakeFiles/logical_camera_plugin.dir/requires: logical_camera_plugin/CMakeFiles/logical_camera_plugin.dir/src/logical_camera_plugin.cpp.o.requires

.PHONY : logical_camera_plugin/CMakeFiles/logical_camera_plugin.dir/requires

logical_camera_plugin/CMakeFiles/logical_camera_plugin.dir/clean:
	cd /home/iaguiluz/UCMEng_Collaboratory03/wsfinal/build/logical_camera_plugin && $(CMAKE_COMMAND) -P CMakeFiles/logical_camera_plugin.dir/cmake_clean.cmake
.PHONY : logical_camera_plugin/CMakeFiles/logical_camera_plugin.dir/clean

logical_camera_plugin/CMakeFiles/logical_camera_plugin.dir/depend:
	cd /home/iaguiluz/UCMEng_Collaboratory03/wsfinal/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/iaguiluz/UCMEng_Collaboratory03/wsfinal/src /home/iaguiluz/UCMEng_Collaboratory03/wsfinal/src/logical_camera_plugin /home/iaguiluz/UCMEng_Collaboratory03/wsfinal/build /home/iaguiluz/UCMEng_Collaboratory03/wsfinal/build/logical_camera_plugin /home/iaguiluz/UCMEng_Collaboratory03/wsfinal/build/logical_camera_plugin/CMakeFiles/logical_camera_plugin.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : logical_camera_plugin/CMakeFiles/logical_camera_plugin.dir/depend

