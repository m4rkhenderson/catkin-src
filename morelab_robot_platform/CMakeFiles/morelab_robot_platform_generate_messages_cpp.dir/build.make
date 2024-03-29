# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

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

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/tungxt/rosproject/src/morelab_robot_platform

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/tungxt/rosproject/src/morelab_robot_platform

# Utility rule file for morelab_robot_platform_generate_messages_cpp.

# Include the progress variables for this target.
include CMakeFiles/morelab_robot_platform_generate_messages_cpp.dir/progress.make

CMakeFiles/morelab_robot_platform_generate_messages_cpp: devel/include/morelab_robot_platform/KeyStroke.h
CMakeFiles/morelab_robot_platform_generate_messages_cpp: devel/include/morelab_robot_platform/BatteryLevel.h
CMakeFiles/morelab_robot_platform_generate_messages_cpp: devel/include/morelab_robot_platform/Ping.h
CMakeFiles/morelab_robot_platform_generate_messages_cpp: devel/include/morelab_robot_platform/Voltages.h
CMakeFiles/morelab_robot_platform_generate_messages_cpp: devel/include/morelab_robot_platform/Velocity.h
CMakeFiles/morelab_robot_platform_generate_messages_cpp: devel/include/morelab_robot_platform/ADC.h
CMakeFiles/morelab_robot_platform_generate_messages_cpp: devel/include/morelab_robot_platform/Distances.h
CMakeFiles/morelab_robot_platform_generate_messages_cpp: devel/include/morelab_robot_platform/Speech.h
CMakeFiles/morelab_robot_platform_generate_messages_cpp: devel/include/morelab_robot_platform/StopSlidingSignal.h
CMakeFiles/morelab_robot_platform_generate_messages_cpp: devel/include/morelab_robot_platform/SpeedWheel.h
CMakeFiles/morelab_robot_platform_generate_messages_cpp: devel/include/morelab_robot_platform/DistanceWheel.h
CMakeFiles/morelab_robot_platform_generate_messages_cpp: devel/include/morelab_robot_platform/DriveWithPower.h
CMakeFiles/morelab_robot_platform_generate_messages_cpp: devel/include/morelab_robot_platform/Accelerate.h
CMakeFiles/morelab_robot_platform_generate_messages_cpp: devel/include/morelab_robot_platform/GetHeading.h
CMakeFiles/morelab_robot_platform_generate_messages_cpp: devel/include/morelab_robot_platform/Rotate.h
CMakeFiles/morelab_robot_platform_generate_messages_cpp: devel/include/morelab_robot_platform/DriveWithSpeed.h
CMakeFiles/morelab_robot_platform_generate_messages_cpp: devel/include/morelab_robot_platform/GetStatus.h
CMakeFiles/morelab_robot_platform_generate_messages_cpp: devel/include/morelab_robot_platform/ResetEncoder.h
CMakeFiles/morelab_robot_platform_generate_messages_cpp: devel/include/morelab_robot_platform/StopAtDistance.h
CMakeFiles/morelab_robot_platform_generate_messages_cpp: devel/include/morelab_robot_platform/GetSpeed.h
CMakeFiles/morelab_robot_platform_generate_messages_cpp: devel/include/morelab_robot_platform/DriveWithDistance.h
CMakeFiles/morelab_robot_platform_generate_messages_cpp: devel/include/morelab_robot_platform/GetDistance.h

devel/include/morelab_robot_platform/KeyStroke.h: /opt/ros/hydro/share/gencpp/cmake/../../../lib/gencpp/gen_cpp.py
devel/include/morelab_robot_platform/KeyStroke.h: msg/KeyStroke.msg
devel/include/morelab_robot_platform/KeyStroke.h: /opt/ros/hydro/share/gencpp/cmake/../msg.h.template
	$(CMAKE_COMMAND) -E cmake_progress_report /home/tungxt/rosproject/src/morelab_robot_platform/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold "Generating C++ code from morelab_robot_platform/KeyStroke.msg"
	catkin_generated/env_cached.sh /usr/bin/python /opt/ros/hydro/share/gencpp/cmake/../../../lib/gencpp/gen_cpp.py /home/tungxt/rosproject/src/morelab_robot_platform/msg/KeyStroke.msg -Imorelab_robot_platform:/home/tungxt/rosproject/src/morelab_robot_platform/msg -Isensor_msgs:/opt/ros/hydro/share/sensor_msgs/cmake/../msg -Igeometry_msgs:/opt/ros/hydro/share/geometry_msgs/cmake/../msg -Iroscpp:/opt/ros/hydro/share/roscpp/cmake/../msg -Istd_msgs:/opt/ros/hydro/share/std_msgs/cmake/../msg -Itf:/opt/ros/hydro/share/tf/cmake/../msg -p morelab_robot_platform -o /home/tungxt/rosproject/src/morelab_robot_platform/devel/include/morelab_robot_platform -e /opt/ros/hydro/share/gencpp/cmake/..

devel/include/morelab_robot_platform/BatteryLevel.h: /opt/ros/hydro/share/gencpp/cmake/../../../lib/gencpp/gen_cpp.py
devel/include/morelab_robot_platform/BatteryLevel.h: msg/BatteryLevel.msg
devel/include/morelab_robot_platform/BatteryLevel.h: /opt/ros/hydro/share/gencpp/cmake/../msg.h.template
	$(CMAKE_COMMAND) -E cmake_progress_report /home/tungxt/rosproject/src/morelab_robot_platform/CMakeFiles $(CMAKE_PROGRESS_2)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold "Generating C++ code from morelab_robot_platform/BatteryLevel.msg"
	catkin_generated/env_cached.sh /usr/bin/python /opt/ros/hydro/share/gencpp/cmake/../../../lib/gencpp/gen_cpp.py /home/tungxt/rosproject/src/morelab_robot_platform/msg/BatteryLevel.msg -Imorelab_robot_platform:/home/tungxt/rosproject/src/morelab_robot_platform/msg -Isensor_msgs:/opt/ros/hydro/share/sensor_msgs/cmake/../msg -Igeometry_msgs:/opt/ros/hydro/share/geometry_msgs/cmake/../msg -Iroscpp:/opt/ros/hydro/share/roscpp/cmake/../msg -Istd_msgs:/opt/ros/hydro/share/std_msgs/cmake/../msg -Itf:/opt/ros/hydro/share/tf/cmake/../msg -p morelab_robot_platform -o /home/tungxt/rosproject/src/morelab_robot_platform/devel/include/morelab_robot_platform -e /opt/ros/hydro/share/gencpp/cmake/..

devel/include/morelab_robot_platform/Ping.h: /opt/ros/hydro/share/gencpp/cmake/../../../lib/gencpp/gen_cpp.py
devel/include/morelab_robot_platform/Ping.h: msg/Ping.msg
devel/include/morelab_robot_platform/Ping.h: /opt/ros/hydro/share/gencpp/cmake/../msg.h.template
	$(CMAKE_COMMAND) -E cmake_progress_report /home/tungxt/rosproject/src/morelab_robot_platform/CMakeFiles $(CMAKE_PROGRESS_3)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold "Generating C++ code from morelab_robot_platform/Ping.msg"
	catkin_generated/env_cached.sh /usr/bin/python /opt/ros/hydro/share/gencpp/cmake/../../../lib/gencpp/gen_cpp.py /home/tungxt/rosproject/src/morelab_robot_platform/msg/Ping.msg -Imorelab_robot_platform:/home/tungxt/rosproject/src/morelab_robot_platform/msg -Isensor_msgs:/opt/ros/hydro/share/sensor_msgs/cmake/../msg -Igeometry_msgs:/opt/ros/hydro/share/geometry_msgs/cmake/../msg -Iroscpp:/opt/ros/hydro/share/roscpp/cmake/../msg -Istd_msgs:/opt/ros/hydro/share/std_msgs/cmake/../msg -Itf:/opt/ros/hydro/share/tf/cmake/../msg -p morelab_robot_platform -o /home/tungxt/rosproject/src/morelab_robot_platform/devel/include/morelab_robot_platform -e /opt/ros/hydro/share/gencpp/cmake/..

devel/include/morelab_robot_platform/Voltages.h: /opt/ros/hydro/share/gencpp/cmake/../../../lib/gencpp/gen_cpp.py
devel/include/morelab_robot_platform/Voltages.h: msg/Voltages.msg
devel/include/morelab_robot_platform/Voltages.h: /opt/ros/hydro/share/gencpp/cmake/../msg.h.template
	$(CMAKE_COMMAND) -E cmake_progress_report /home/tungxt/rosproject/src/morelab_robot_platform/CMakeFiles $(CMAKE_PROGRESS_4)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold "Generating C++ code from morelab_robot_platform/Voltages.msg"
	catkin_generated/env_cached.sh /usr/bin/python /opt/ros/hydro/share/gencpp/cmake/../../../lib/gencpp/gen_cpp.py /home/tungxt/rosproject/src/morelab_robot_platform/msg/Voltages.msg -Imorelab_robot_platform:/home/tungxt/rosproject/src/morelab_robot_platform/msg -Isensor_msgs:/opt/ros/hydro/share/sensor_msgs/cmake/../msg -Igeometry_msgs:/opt/ros/hydro/share/geometry_msgs/cmake/../msg -Iroscpp:/opt/ros/hydro/share/roscpp/cmake/../msg -Istd_msgs:/opt/ros/hydro/share/std_msgs/cmake/../msg -Itf:/opt/ros/hydro/share/tf/cmake/../msg -p morelab_robot_platform -o /home/tungxt/rosproject/src/morelab_robot_platform/devel/include/morelab_robot_platform -e /opt/ros/hydro/share/gencpp/cmake/..

devel/include/morelab_robot_platform/Velocity.h: /opt/ros/hydro/share/gencpp/cmake/../../../lib/gencpp/gen_cpp.py
devel/include/morelab_robot_platform/Velocity.h: msg/Velocity.msg
devel/include/morelab_robot_platform/Velocity.h: /opt/ros/hydro/share/gencpp/cmake/../msg.h.template
	$(CMAKE_COMMAND) -E cmake_progress_report /home/tungxt/rosproject/src/morelab_robot_platform/CMakeFiles $(CMAKE_PROGRESS_5)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold "Generating C++ code from morelab_robot_platform/Velocity.msg"
	catkin_generated/env_cached.sh /usr/bin/python /opt/ros/hydro/share/gencpp/cmake/../../../lib/gencpp/gen_cpp.py /home/tungxt/rosproject/src/morelab_robot_platform/msg/Velocity.msg -Imorelab_robot_platform:/home/tungxt/rosproject/src/morelab_robot_platform/msg -Isensor_msgs:/opt/ros/hydro/share/sensor_msgs/cmake/../msg -Igeometry_msgs:/opt/ros/hydro/share/geometry_msgs/cmake/../msg -Iroscpp:/opt/ros/hydro/share/roscpp/cmake/../msg -Istd_msgs:/opt/ros/hydro/share/std_msgs/cmake/../msg -Itf:/opt/ros/hydro/share/tf/cmake/../msg -p morelab_robot_platform -o /home/tungxt/rosproject/src/morelab_robot_platform/devel/include/morelab_robot_platform -e /opt/ros/hydro/share/gencpp/cmake/..

devel/include/morelab_robot_platform/ADC.h: /opt/ros/hydro/share/gencpp/cmake/../../../lib/gencpp/gen_cpp.py
devel/include/morelab_robot_platform/ADC.h: msg/ADC.msg
devel/include/morelab_robot_platform/ADC.h: /opt/ros/hydro/share/gencpp/cmake/../msg.h.template
	$(CMAKE_COMMAND) -E cmake_progress_report /home/tungxt/rosproject/src/morelab_robot_platform/CMakeFiles $(CMAKE_PROGRESS_6)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold "Generating C++ code from morelab_robot_platform/ADC.msg"
	catkin_generated/env_cached.sh /usr/bin/python /opt/ros/hydro/share/gencpp/cmake/../../../lib/gencpp/gen_cpp.py /home/tungxt/rosproject/src/morelab_robot_platform/msg/ADC.msg -Imorelab_robot_platform:/home/tungxt/rosproject/src/morelab_robot_platform/msg -Isensor_msgs:/opt/ros/hydro/share/sensor_msgs/cmake/../msg -Igeometry_msgs:/opt/ros/hydro/share/geometry_msgs/cmake/../msg -Iroscpp:/opt/ros/hydro/share/roscpp/cmake/../msg -Istd_msgs:/opt/ros/hydro/share/std_msgs/cmake/../msg -Itf:/opt/ros/hydro/share/tf/cmake/../msg -p morelab_robot_platform -o /home/tungxt/rosproject/src/morelab_robot_platform/devel/include/morelab_robot_platform -e /opt/ros/hydro/share/gencpp/cmake/..

devel/include/morelab_robot_platform/Distances.h: /opt/ros/hydro/share/gencpp/cmake/../../../lib/gencpp/gen_cpp.py
devel/include/morelab_robot_platform/Distances.h: msg/Distances.msg
devel/include/morelab_robot_platform/Distances.h: /opt/ros/hydro/share/gencpp/cmake/../msg.h.template
	$(CMAKE_COMMAND) -E cmake_progress_report /home/tungxt/rosproject/src/morelab_robot_platform/CMakeFiles $(CMAKE_PROGRESS_7)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold "Generating C++ code from morelab_robot_platform/Distances.msg"
	catkin_generated/env_cached.sh /usr/bin/python /opt/ros/hydro/share/gencpp/cmake/../../../lib/gencpp/gen_cpp.py /home/tungxt/rosproject/src/morelab_robot_platform/msg/Distances.msg -Imorelab_robot_platform:/home/tungxt/rosproject/src/morelab_robot_platform/msg -Isensor_msgs:/opt/ros/hydro/share/sensor_msgs/cmake/../msg -Igeometry_msgs:/opt/ros/hydro/share/geometry_msgs/cmake/../msg -Iroscpp:/opt/ros/hydro/share/roscpp/cmake/../msg -Istd_msgs:/opt/ros/hydro/share/std_msgs/cmake/../msg -Itf:/opt/ros/hydro/share/tf/cmake/../msg -p morelab_robot_platform -o /home/tungxt/rosproject/src/morelab_robot_platform/devel/include/morelab_robot_platform -e /opt/ros/hydro/share/gencpp/cmake/..

devel/include/morelab_robot_platform/Speech.h: /opt/ros/hydro/share/gencpp/cmake/../../../lib/gencpp/gen_cpp.py
devel/include/morelab_robot_platform/Speech.h: msg/Speech.msg
devel/include/morelab_robot_platform/Speech.h: /opt/ros/hydro/share/gencpp/cmake/../msg.h.template
	$(CMAKE_COMMAND) -E cmake_progress_report /home/tungxt/rosproject/src/morelab_robot_platform/CMakeFiles $(CMAKE_PROGRESS_8)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold "Generating C++ code from morelab_robot_platform/Speech.msg"
	catkin_generated/env_cached.sh /usr/bin/python /opt/ros/hydro/share/gencpp/cmake/../../../lib/gencpp/gen_cpp.py /home/tungxt/rosproject/src/morelab_robot_platform/msg/Speech.msg -Imorelab_robot_platform:/home/tungxt/rosproject/src/morelab_robot_platform/msg -Isensor_msgs:/opt/ros/hydro/share/sensor_msgs/cmake/../msg -Igeometry_msgs:/opt/ros/hydro/share/geometry_msgs/cmake/../msg -Iroscpp:/opt/ros/hydro/share/roscpp/cmake/../msg -Istd_msgs:/opt/ros/hydro/share/std_msgs/cmake/../msg -Itf:/opt/ros/hydro/share/tf/cmake/../msg -p morelab_robot_platform -o /home/tungxt/rosproject/src/morelab_robot_platform/devel/include/morelab_robot_platform -e /opt/ros/hydro/share/gencpp/cmake/..

devel/include/morelab_robot_platform/StopSlidingSignal.h: /opt/ros/hydro/share/gencpp/cmake/../../../lib/gencpp/gen_cpp.py
devel/include/morelab_robot_platform/StopSlidingSignal.h: msg/StopSlidingSignal.msg
devel/include/morelab_robot_platform/StopSlidingSignal.h: /opt/ros/hydro/share/gencpp/cmake/../msg.h.template
	$(CMAKE_COMMAND) -E cmake_progress_report /home/tungxt/rosproject/src/morelab_robot_platform/CMakeFiles $(CMAKE_PROGRESS_9)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold "Generating C++ code from morelab_robot_platform/StopSlidingSignal.msg"
	catkin_generated/env_cached.sh /usr/bin/python /opt/ros/hydro/share/gencpp/cmake/../../../lib/gencpp/gen_cpp.py /home/tungxt/rosproject/src/morelab_robot_platform/msg/StopSlidingSignal.msg -Imorelab_robot_platform:/home/tungxt/rosproject/src/morelab_robot_platform/msg -Isensor_msgs:/opt/ros/hydro/share/sensor_msgs/cmake/../msg -Igeometry_msgs:/opt/ros/hydro/share/geometry_msgs/cmake/../msg -Iroscpp:/opt/ros/hydro/share/roscpp/cmake/../msg -Istd_msgs:/opt/ros/hydro/share/std_msgs/cmake/../msg -Itf:/opt/ros/hydro/share/tf/cmake/../msg -p morelab_robot_platform -o /home/tungxt/rosproject/src/morelab_robot_platform/devel/include/morelab_robot_platform -e /opt/ros/hydro/share/gencpp/cmake/..

devel/include/morelab_robot_platform/SpeedWheel.h: /opt/ros/hydro/share/gencpp/cmake/../../../lib/gencpp/gen_cpp.py
devel/include/morelab_robot_platform/SpeedWheel.h: msg/SpeedWheel.msg
devel/include/morelab_robot_platform/SpeedWheel.h: /opt/ros/hydro/share/gencpp/cmake/../msg.h.template
	$(CMAKE_COMMAND) -E cmake_progress_report /home/tungxt/rosproject/src/morelab_robot_platform/CMakeFiles $(CMAKE_PROGRESS_10)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold "Generating C++ code from morelab_robot_platform/SpeedWheel.msg"
	catkin_generated/env_cached.sh /usr/bin/python /opt/ros/hydro/share/gencpp/cmake/../../../lib/gencpp/gen_cpp.py /home/tungxt/rosproject/src/morelab_robot_platform/msg/SpeedWheel.msg -Imorelab_robot_platform:/home/tungxt/rosproject/src/morelab_robot_platform/msg -Isensor_msgs:/opt/ros/hydro/share/sensor_msgs/cmake/../msg -Igeometry_msgs:/opt/ros/hydro/share/geometry_msgs/cmake/../msg -Iroscpp:/opt/ros/hydro/share/roscpp/cmake/../msg -Istd_msgs:/opt/ros/hydro/share/std_msgs/cmake/../msg -Itf:/opt/ros/hydro/share/tf/cmake/../msg -p morelab_robot_platform -o /home/tungxt/rosproject/src/morelab_robot_platform/devel/include/morelab_robot_platform -e /opt/ros/hydro/share/gencpp/cmake/..

devel/include/morelab_robot_platform/DistanceWheel.h: /opt/ros/hydro/share/gencpp/cmake/../../../lib/gencpp/gen_cpp.py
devel/include/morelab_robot_platform/DistanceWheel.h: msg/DistanceWheel.msg
devel/include/morelab_robot_platform/DistanceWheel.h: /opt/ros/hydro/share/gencpp/cmake/../msg.h.template
	$(CMAKE_COMMAND) -E cmake_progress_report /home/tungxt/rosproject/src/morelab_robot_platform/CMakeFiles $(CMAKE_PROGRESS_11)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold "Generating C++ code from morelab_robot_platform/DistanceWheel.msg"
	catkin_generated/env_cached.sh /usr/bin/python /opt/ros/hydro/share/gencpp/cmake/../../../lib/gencpp/gen_cpp.py /home/tungxt/rosproject/src/morelab_robot_platform/msg/DistanceWheel.msg -Imorelab_robot_platform:/home/tungxt/rosproject/src/morelab_robot_platform/msg -Isensor_msgs:/opt/ros/hydro/share/sensor_msgs/cmake/../msg -Igeometry_msgs:/opt/ros/hydro/share/geometry_msgs/cmake/../msg -Iroscpp:/opt/ros/hydro/share/roscpp/cmake/../msg -Istd_msgs:/opt/ros/hydro/share/std_msgs/cmake/../msg -Itf:/opt/ros/hydro/share/tf/cmake/../msg -p morelab_robot_platform -o /home/tungxt/rosproject/src/morelab_robot_platform/devel/include/morelab_robot_platform -e /opt/ros/hydro/share/gencpp/cmake/..

devel/include/morelab_robot_platform/DriveWithPower.h: /opt/ros/hydro/share/gencpp/cmake/../../../lib/gencpp/gen_cpp.py
devel/include/morelab_robot_platform/DriveWithPower.h: srv/DriveWithPower.srv
devel/include/morelab_robot_platform/DriveWithPower.h: /opt/ros/hydro/share/gencpp/cmake/../msg.h.template
devel/include/morelab_robot_platform/DriveWithPower.h: /opt/ros/hydro/share/gencpp/cmake/../srv.h.template
	$(CMAKE_COMMAND) -E cmake_progress_report /home/tungxt/rosproject/src/morelab_robot_platform/CMakeFiles $(CMAKE_PROGRESS_12)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold "Generating C++ code from morelab_robot_platform/DriveWithPower.srv"
	catkin_generated/env_cached.sh /usr/bin/python /opt/ros/hydro/share/gencpp/cmake/../../../lib/gencpp/gen_cpp.py /home/tungxt/rosproject/src/morelab_robot_platform/srv/DriveWithPower.srv -Imorelab_robot_platform:/home/tungxt/rosproject/src/morelab_robot_platform/msg -Isensor_msgs:/opt/ros/hydro/share/sensor_msgs/cmake/../msg -Igeometry_msgs:/opt/ros/hydro/share/geometry_msgs/cmake/../msg -Iroscpp:/opt/ros/hydro/share/roscpp/cmake/../msg -Istd_msgs:/opt/ros/hydro/share/std_msgs/cmake/../msg -Itf:/opt/ros/hydro/share/tf/cmake/../msg -p morelab_robot_platform -o /home/tungxt/rosproject/src/morelab_robot_platform/devel/include/morelab_robot_platform -e /opt/ros/hydro/share/gencpp/cmake/..

devel/include/morelab_robot_platform/Accelerate.h: /opt/ros/hydro/share/gencpp/cmake/../../../lib/gencpp/gen_cpp.py
devel/include/morelab_robot_platform/Accelerate.h: srv/Accelerate.srv
devel/include/morelab_robot_platform/Accelerate.h: /opt/ros/hydro/share/gencpp/cmake/../msg.h.template
devel/include/morelab_robot_platform/Accelerate.h: /opt/ros/hydro/share/gencpp/cmake/../srv.h.template
	$(CMAKE_COMMAND) -E cmake_progress_report /home/tungxt/rosproject/src/morelab_robot_platform/CMakeFiles $(CMAKE_PROGRESS_13)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold "Generating C++ code from morelab_robot_platform/Accelerate.srv"
	catkin_generated/env_cached.sh /usr/bin/python /opt/ros/hydro/share/gencpp/cmake/../../../lib/gencpp/gen_cpp.py /home/tungxt/rosproject/src/morelab_robot_platform/srv/Accelerate.srv -Imorelab_robot_platform:/home/tungxt/rosproject/src/morelab_robot_platform/msg -Isensor_msgs:/opt/ros/hydro/share/sensor_msgs/cmake/../msg -Igeometry_msgs:/opt/ros/hydro/share/geometry_msgs/cmake/../msg -Iroscpp:/opt/ros/hydro/share/roscpp/cmake/../msg -Istd_msgs:/opt/ros/hydro/share/std_msgs/cmake/../msg -Itf:/opt/ros/hydro/share/tf/cmake/../msg -p morelab_robot_platform -o /home/tungxt/rosproject/src/morelab_robot_platform/devel/include/morelab_robot_platform -e /opt/ros/hydro/share/gencpp/cmake/..

devel/include/morelab_robot_platform/GetHeading.h: /opt/ros/hydro/share/gencpp/cmake/../../../lib/gencpp/gen_cpp.py
devel/include/morelab_robot_platform/GetHeading.h: srv/GetHeading.srv
devel/include/morelab_robot_platform/GetHeading.h: /opt/ros/hydro/share/gencpp/cmake/../msg.h.template
devel/include/morelab_robot_platform/GetHeading.h: /opt/ros/hydro/share/gencpp/cmake/../srv.h.template
	$(CMAKE_COMMAND) -E cmake_progress_report /home/tungxt/rosproject/src/morelab_robot_platform/CMakeFiles $(CMAKE_PROGRESS_14)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold "Generating C++ code from morelab_robot_platform/GetHeading.srv"
	catkin_generated/env_cached.sh /usr/bin/python /opt/ros/hydro/share/gencpp/cmake/../../../lib/gencpp/gen_cpp.py /home/tungxt/rosproject/src/morelab_robot_platform/srv/GetHeading.srv -Imorelab_robot_platform:/home/tungxt/rosproject/src/morelab_robot_platform/msg -Isensor_msgs:/opt/ros/hydro/share/sensor_msgs/cmake/../msg -Igeometry_msgs:/opt/ros/hydro/share/geometry_msgs/cmake/../msg -Iroscpp:/opt/ros/hydro/share/roscpp/cmake/../msg -Istd_msgs:/opt/ros/hydro/share/std_msgs/cmake/../msg -Itf:/opt/ros/hydro/share/tf/cmake/../msg -p morelab_robot_platform -o /home/tungxt/rosproject/src/morelab_robot_platform/devel/include/morelab_robot_platform -e /opt/ros/hydro/share/gencpp/cmake/..

devel/include/morelab_robot_platform/Rotate.h: /opt/ros/hydro/share/gencpp/cmake/../../../lib/gencpp/gen_cpp.py
devel/include/morelab_robot_platform/Rotate.h: srv/Rotate.srv
devel/include/morelab_robot_platform/Rotate.h: /opt/ros/hydro/share/gencpp/cmake/../msg.h.template
devel/include/morelab_robot_platform/Rotate.h: /opt/ros/hydro/share/gencpp/cmake/../srv.h.template
	$(CMAKE_COMMAND) -E cmake_progress_report /home/tungxt/rosproject/src/morelab_robot_platform/CMakeFiles $(CMAKE_PROGRESS_15)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold "Generating C++ code from morelab_robot_platform/Rotate.srv"
	catkin_generated/env_cached.sh /usr/bin/python /opt/ros/hydro/share/gencpp/cmake/../../../lib/gencpp/gen_cpp.py /home/tungxt/rosproject/src/morelab_robot_platform/srv/Rotate.srv -Imorelab_robot_platform:/home/tungxt/rosproject/src/morelab_robot_platform/msg -Isensor_msgs:/opt/ros/hydro/share/sensor_msgs/cmake/../msg -Igeometry_msgs:/opt/ros/hydro/share/geometry_msgs/cmake/../msg -Iroscpp:/opt/ros/hydro/share/roscpp/cmake/../msg -Istd_msgs:/opt/ros/hydro/share/std_msgs/cmake/../msg -Itf:/opt/ros/hydro/share/tf/cmake/../msg -p morelab_robot_platform -o /home/tungxt/rosproject/src/morelab_robot_platform/devel/include/morelab_robot_platform -e /opt/ros/hydro/share/gencpp/cmake/..

devel/include/morelab_robot_platform/DriveWithSpeed.h: /opt/ros/hydro/share/gencpp/cmake/../../../lib/gencpp/gen_cpp.py
devel/include/morelab_robot_platform/DriveWithSpeed.h: srv/DriveWithSpeed.srv
devel/include/morelab_robot_platform/DriveWithSpeed.h: /opt/ros/hydro/share/gencpp/cmake/../msg.h.template
devel/include/morelab_robot_platform/DriveWithSpeed.h: /opt/ros/hydro/share/gencpp/cmake/../srv.h.template
	$(CMAKE_COMMAND) -E cmake_progress_report /home/tungxt/rosproject/src/morelab_robot_platform/CMakeFiles $(CMAKE_PROGRESS_16)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold "Generating C++ code from morelab_robot_platform/DriveWithSpeed.srv"
	catkin_generated/env_cached.sh /usr/bin/python /opt/ros/hydro/share/gencpp/cmake/../../../lib/gencpp/gen_cpp.py /home/tungxt/rosproject/src/morelab_robot_platform/srv/DriveWithSpeed.srv -Imorelab_robot_platform:/home/tungxt/rosproject/src/morelab_robot_platform/msg -Isensor_msgs:/opt/ros/hydro/share/sensor_msgs/cmake/../msg -Igeometry_msgs:/opt/ros/hydro/share/geometry_msgs/cmake/../msg -Iroscpp:/opt/ros/hydro/share/roscpp/cmake/../msg -Istd_msgs:/opt/ros/hydro/share/std_msgs/cmake/../msg -Itf:/opt/ros/hydro/share/tf/cmake/../msg -p morelab_robot_platform -o /home/tungxt/rosproject/src/morelab_robot_platform/devel/include/morelab_robot_platform -e /opt/ros/hydro/share/gencpp/cmake/..

devel/include/morelab_robot_platform/GetStatus.h: /opt/ros/hydro/share/gencpp/cmake/../../../lib/gencpp/gen_cpp.py
devel/include/morelab_robot_platform/GetStatus.h: srv/GetStatus.srv
devel/include/morelab_robot_platform/GetStatus.h: /opt/ros/hydro/share/gencpp/cmake/../msg.h.template
devel/include/morelab_robot_platform/GetStatus.h: /opt/ros/hydro/share/gencpp/cmake/../srv.h.template
	$(CMAKE_COMMAND) -E cmake_progress_report /home/tungxt/rosproject/src/morelab_robot_platform/CMakeFiles $(CMAKE_PROGRESS_17)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold "Generating C++ code from morelab_robot_platform/GetStatus.srv"
	catkin_generated/env_cached.sh /usr/bin/python /opt/ros/hydro/share/gencpp/cmake/../../../lib/gencpp/gen_cpp.py /home/tungxt/rosproject/src/morelab_robot_platform/srv/GetStatus.srv -Imorelab_robot_platform:/home/tungxt/rosproject/src/morelab_robot_platform/msg -Isensor_msgs:/opt/ros/hydro/share/sensor_msgs/cmake/../msg -Igeometry_msgs:/opt/ros/hydro/share/geometry_msgs/cmake/../msg -Iroscpp:/opt/ros/hydro/share/roscpp/cmake/../msg -Istd_msgs:/opt/ros/hydro/share/std_msgs/cmake/../msg -Itf:/opt/ros/hydro/share/tf/cmake/../msg -p morelab_robot_platform -o /home/tungxt/rosproject/src/morelab_robot_platform/devel/include/morelab_robot_platform -e /opt/ros/hydro/share/gencpp/cmake/..

devel/include/morelab_robot_platform/ResetEncoder.h: /opt/ros/hydro/share/gencpp/cmake/../../../lib/gencpp/gen_cpp.py
devel/include/morelab_robot_platform/ResetEncoder.h: srv/ResetEncoder.srv
devel/include/morelab_robot_platform/ResetEncoder.h: /opt/ros/hydro/share/gencpp/cmake/../msg.h.template
devel/include/morelab_robot_platform/ResetEncoder.h: /opt/ros/hydro/share/gencpp/cmake/../srv.h.template
	$(CMAKE_COMMAND) -E cmake_progress_report /home/tungxt/rosproject/src/morelab_robot_platform/CMakeFiles $(CMAKE_PROGRESS_18)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold "Generating C++ code from morelab_robot_platform/ResetEncoder.srv"
	catkin_generated/env_cached.sh /usr/bin/python /opt/ros/hydro/share/gencpp/cmake/../../../lib/gencpp/gen_cpp.py /home/tungxt/rosproject/src/morelab_robot_platform/srv/ResetEncoder.srv -Imorelab_robot_platform:/home/tungxt/rosproject/src/morelab_robot_platform/msg -Isensor_msgs:/opt/ros/hydro/share/sensor_msgs/cmake/../msg -Igeometry_msgs:/opt/ros/hydro/share/geometry_msgs/cmake/../msg -Iroscpp:/opt/ros/hydro/share/roscpp/cmake/../msg -Istd_msgs:/opt/ros/hydro/share/std_msgs/cmake/../msg -Itf:/opt/ros/hydro/share/tf/cmake/../msg -p morelab_robot_platform -o /home/tungxt/rosproject/src/morelab_robot_platform/devel/include/morelab_robot_platform -e /opt/ros/hydro/share/gencpp/cmake/..

devel/include/morelab_robot_platform/StopAtDistance.h: /opt/ros/hydro/share/gencpp/cmake/../../../lib/gencpp/gen_cpp.py
devel/include/morelab_robot_platform/StopAtDistance.h: srv/StopAtDistance.srv
devel/include/morelab_robot_platform/StopAtDistance.h: /opt/ros/hydro/share/gencpp/cmake/../msg.h.template
devel/include/morelab_robot_platform/StopAtDistance.h: /opt/ros/hydro/share/gencpp/cmake/../srv.h.template
	$(CMAKE_COMMAND) -E cmake_progress_report /home/tungxt/rosproject/src/morelab_robot_platform/CMakeFiles $(CMAKE_PROGRESS_19)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold "Generating C++ code from morelab_robot_platform/StopAtDistance.srv"
	catkin_generated/env_cached.sh /usr/bin/python /opt/ros/hydro/share/gencpp/cmake/../../../lib/gencpp/gen_cpp.py /home/tungxt/rosproject/src/morelab_robot_platform/srv/StopAtDistance.srv -Imorelab_robot_platform:/home/tungxt/rosproject/src/morelab_robot_platform/msg -Isensor_msgs:/opt/ros/hydro/share/sensor_msgs/cmake/../msg -Igeometry_msgs:/opt/ros/hydro/share/geometry_msgs/cmake/../msg -Iroscpp:/opt/ros/hydro/share/roscpp/cmake/../msg -Istd_msgs:/opt/ros/hydro/share/std_msgs/cmake/../msg -Itf:/opt/ros/hydro/share/tf/cmake/../msg -p morelab_robot_platform -o /home/tungxt/rosproject/src/morelab_robot_platform/devel/include/morelab_robot_platform -e /opt/ros/hydro/share/gencpp/cmake/..

devel/include/morelab_robot_platform/GetSpeed.h: /opt/ros/hydro/share/gencpp/cmake/../../../lib/gencpp/gen_cpp.py
devel/include/morelab_robot_platform/GetSpeed.h: srv/GetSpeed.srv
devel/include/morelab_robot_platform/GetSpeed.h: /opt/ros/hydro/share/gencpp/cmake/../msg.h.template
devel/include/morelab_robot_platform/GetSpeed.h: /opt/ros/hydro/share/gencpp/cmake/../srv.h.template
	$(CMAKE_COMMAND) -E cmake_progress_report /home/tungxt/rosproject/src/morelab_robot_platform/CMakeFiles $(CMAKE_PROGRESS_20)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold "Generating C++ code from morelab_robot_platform/GetSpeed.srv"
	catkin_generated/env_cached.sh /usr/bin/python /opt/ros/hydro/share/gencpp/cmake/../../../lib/gencpp/gen_cpp.py /home/tungxt/rosproject/src/morelab_robot_platform/srv/GetSpeed.srv -Imorelab_robot_platform:/home/tungxt/rosproject/src/morelab_robot_platform/msg -Isensor_msgs:/opt/ros/hydro/share/sensor_msgs/cmake/../msg -Igeometry_msgs:/opt/ros/hydro/share/geometry_msgs/cmake/../msg -Iroscpp:/opt/ros/hydro/share/roscpp/cmake/../msg -Istd_msgs:/opt/ros/hydro/share/std_msgs/cmake/../msg -Itf:/opt/ros/hydro/share/tf/cmake/../msg -p morelab_robot_platform -o /home/tungxt/rosproject/src/morelab_robot_platform/devel/include/morelab_robot_platform -e /opt/ros/hydro/share/gencpp/cmake/..

devel/include/morelab_robot_platform/DriveWithDistance.h: /opt/ros/hydro/share/gencpp/cmake/../../../lib/gencpp/gen_cpp.py
devel/include/morelab_robot_platform/DriveWithDistance.h: srv/DriveWithDistance.srv
devel/include/morelab_robot_platform/DriveWithDistance.h: /opt/ros/hydro/share/gencpp/cmake/../msg.h.template
devel/include/morelab_robot_platform/DriveWithDistance.h: /opt/ros/hydro/share/gencpp/cmake/../srv.h.template
	$(CMAKE_COMMAND) -E cmake_progress_report /home/tungxt/rosproject/src/morelab_robot_platform/CMakeFiles $(CMAKE_PROGRESS_21)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold "Generating C++ code from morelab_robot_platform/DriveWithDistance.srv"
	catkin_generated/env_cached.sh /usr/bin/python /opt/ros/hydro/share/gencpp/cmake/../../../lib/gencpp/gen_cpp.py /home/tungxt/rosproject/src/morelab_robot_platform/srv/DriveWithDistance.srv -Imorelab_robot_platform:/home/tungxt/rosproject/src/morelab_robot_platform/msg -Isensor_msgs:/opt/ros/hydro/share/sensor_msgs/cmake/../msg -Igeometry_msgs:/opt/ros/hydro/share/geometry_msgs/cmake/../msg -Iroscpp:/opt/ros/hydro/share/roscpp/cmake/../msg -Istd_msgs:/opt/ros/hydro/share/std_msgs/cmake/../msg -Itf:/opt/ros/hydro/share/tf/cmake/../msg -p morelab_robot_platform -o /home/tungxt/rosproject/src/morelab_robot_platform/devel/include/morelab_robot_platform -e /opt/ros/hydro/share/gencpp/cmake/..

devel/include/morelab_robot_platform/GetDistance.h: /opt/ros/hydro/share/gencpp/cmake/../../../lib/gencpp/gen_cpp.py
devel/include/morelab_robot_platform/GetDistance.h: srv/GetDistance.srv
devel/include/morelab_robot_platform/GetDistance.h: /opt/ros/hydro/share/gencpp/cmake/../msg.h.template
devel/include/morelab_robot_platform/GetDistance.h: /opt/ros/hydro/share/gencpp/cmake/../srv.h.template
	$(CMAKE_COMMAND) -E cmake_progress_report /home/tungxt/rosproject/src/morelab_robot_platform/CMakeFiles $(CMAKE_PROGRESS_22)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold "Generating C++ code from morelab_robot_platform/GetDistance.srv"
	catkin_generated/env_cached.sh /usr/bin/python /opt/ros/hydro/share/gencpp/cmake/../../../lib/gencpp/gen_cpp.py /home/tungxt/rosproject/src/morelab_robot_platform/srv/GetDistance.srv -Imorelab_robot_platform:/home/tungxt/rosproject/src/morelab_robot_platform/msg -Isensor_msgs:/opt/ros/hydro/share/sensor_msgs/cmake/../msg -Igeometry_msgs:/opt/ros/hydro/share/geometry_msgs/cmake/../msg -Iroscpp:/opt/ros/hydro/share/roscpp/cmake/../msg -Istd_msgs:/opt/ros/hydro/share/std_msgs/cmake/../msg -Itf:/opt/ros/hydro/share/tf/cmake/../msg -p morelab_robot_platform -o /home/tungxt/rosproject/src/morelab_robot_platform/devel/include/morelab_robot_platform -e /opt/ros/hydro/share/gencpp/cmake/..

morelab_robot_platform_generate_messages_cpp: CMakeFiles/morelab_robot_platform_generate_messages_cpp
morelab_robot_platform_generate_messages_cpp: devel/include/morelab_robot_platform/KeyStroke.h
morelab_robot_platform_generate_messages_cpp: devel/include/morelab_robot_platform/BatteryLevel.h
morelab_robot_platform_generate_messages_cpp: devel/include/morelab_robot_platform/Ping.h
morelab_robot_platform_generate_messages_cpp: devel/include/morelab_robot_platform/Voltages.h
morelab_robot_platform_generate_messages_cpp: devel/include/morelab_robot_platform/Velocity.h
morelab_robot_platform_generate_messages_cpp: devel/include/morelab_robot_platform/ADC.h
morelab_robot_platform_generate_messages_cpp: devel/include/morelab_robot_platform/Distances.h
morelab_robot_platform_generate_messages_cpp: devel/include/morelab_robot_platform/Speech.h
morelab_robot_platform_generate_messages_cpp: devel/include/morelab_robot_platform/StopSlidingSignal.h
morelab_robot_platform_generate_messages_cpp: devel/include/morelab_robot_platform/SpeedWheel.h
morelab_robot_platform_generate_messages_cpp: devel/include/morelab_robot_platform/DistanceWheel.h
morelab_robot_platform_generate_messages_cpp: devel/include/morelab_robot_platform/DriveWithPower.h
morelab_robot_platform_generate_messages_cpp: devel/include/morelab_robot_platform/Accelerate.h
morelab_robot_platform_generate_messages_cpp: devel/include/morelab_robot_platform/GetHeading.h
morelab_robot_platform_generate_messages_cpp: devel/include/morelab_robot_platform/Rotate.h
morelab_robot_platform_generate_messages_cpp: devel/include/morelab_robot_platform/DriveWithSpeed.h
morelab_robot_platform_generate_messages_cpp: devel/include/morelab_robot_platform/GetStatus.h
morelab_robot_platform_generate_messages_cpp: devel/include/morelab_robot_platform/ResetEncoder.h
morelab_robot_platform_generate_messages_cpp: devel/include/morelab_robot_platform/StopAtDistance.h
morelab_robot_platform_generate_messages_cpp: devel/include/morelab_robot_platform/GetSpeed.h
morelab_robot_platform_generate_messages_cpp: devel/include/morelab_robot_platform/DriveWithDistance.h
morelab_robot_platform_generate_messages_cpp: devel/include/morelab_robot_platform/GetDistance.h
morelab_robot_platform_generate_messages_cpp: CMakeFiles/morelab_robot_platform_generate_messages_cpp.dir/build.make
.PHONY : morelab_robot_platform_generate_messages_cpp

# Rule to build all files generated by this target.
CMakeFiles/morelab_robot_platform_generate_messages_cpp.dir/build: morelab_robot_platform_generate_messages_cpp
.PHONY : CMakeFiles/morelab_robot_platform_generate_messages_cpp.dir/build

CMakeFiles/morelab_robot_platform_generate_messages_cpp.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/morelab_robot_platform_generate_messages_cpp.dir/cmake_clean.cmake
.PHONY : CMakeFiles/morelab_robot_platform_generate_messages_cpp.dir/clean

CMakeFiles/morelab_robot_platform_generate_messages_cpp.dir/depend:
	cd /home/tungxt/rosproject/src/morelab_robot_platform && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/tungxt/rosproject/src/morelab_robot_platform /home/tungxt/rosproject/src/morelab_robot_platform /home/tungxt/rosproject/src/morelab_robot_platform /home/tungxt/rosproject/src/morelab_robot_platform /home/tungxt/rosproject/src/morelab_robot_platform/CMakeFiles/morelab_robot_platform_generate_messages_cpp.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/morelab_robot_platform_generate_messages_cpp.dir/depend

