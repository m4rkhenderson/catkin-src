# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "morelab_robot_platform: 11 messages, 11 services")

set(MSG_I_FLAGS "-Imorelab_robot_platform:/home/tungxt/rosproject/src/morelab_robot_platform/msg;-Isensor_msgs:/opt/ros/hydro/share/sensor_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/hydro/share/geometry_msgs/cmake/../msg;-Iroscpp:/opt/ros/hydro/share/roscpp/cmake/../msg;-Istd_msgs:/opt/ros/hydro/share/std_msgs/cmake/../msg;-Itf:/opt/ros/hydro/share/tf/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(morelab_robot_platform_generate_messages ALL)

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(morelab_robot_platform
  "/home/tungxt/rosproject/src/morelab_robot_platform/msg/KeyStroke.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/morelab_robot_platform
)
_generate_msg_cpp(morelab_robot_platform
  "/home/tungxt/rosproject/src/morelab_robot_platform/msg/BatteryLevel.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/morelab_robot_platform
)
_generate_msg_cpp(morelab_robot_platform
  "/home/tungxt/rosproject/src/morelab_robot_platform/msg/Ping.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/morelab_robot_platform
)
_generate_msg_cpp(morelab_robot_platform
  "/home/tungxt/rosproject/src/morelab_robot_platform/msg/Voltages.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/morelab_robot_platform
)
_generate_msg_cpp(morelab_robot_platform
  "/home/tungxt/rosproject/src/morelab_robot_platform/msg/Velocity.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/morelab_robot_platform
)
_generate_msg_cpp(morelab_robot_platform
  "/home/tungxt/rosproject/src/morelab_robot_platform/msg/ADC.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/morelab_robot_platform
)
_generate_msg_cpp(morelab_robot_platform
  "/home/tungxt/rosproject/src/morelab_robot_platform/msg/Distances.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/morelab_robot_platform
)
_generate_msg_cpp(morelab_robot_platform
  "/home/tungxt/rosproject/src/morelab_robot_platform/msg/Speech.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/morelab_robot_platform
)
_generate_msg_cpp(morelab_robot_platform
  "/home/tungxt/rosproject/src/morelab_robot_platform/msg/StopSlidingSignal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/morelab_robot_platform
)
_generate_msg_cpp(morelab_robot_platform
  "/home/tungxt/rosproject/src/morelab_robot_platform/msg/SpeedWheel.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/morelab_robot_platform
)
_generate_msg_cpp(morelab_robot_platform
  "/home/tungxt/rosproject/src/morelab_robot_platform/msg/DistanceWheel.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/morelab_robot_platform
)

### Generating Services
_generate_srv_cpp(morelab_robot_platform
  "/home/tungxt/rosproject/src/morelab_robot_platform/srv/DriveWithPower.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/morelab_robot_platform
)
_generate_srv_cpp(morelab_robot_platform
  "/home/tungxt/rosproject/src/morelab_robot_platform/srv/Accelerate.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/morelab_robot_platform
)
_generate_srv_cpp(morelab_robot_platform
  "/home/tungxt/rosproject/src/morelab_robot_platform/srv/GetHeading.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/morelab_robot_platform
)
_generate_srv_cpp(morelab_robot_platform
  "/home/tungxt/rosproject/src/morelab_robot_platform/srv/Rotate.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/morelab_robot_platform
)
_generate_srv_cpp(morelab_robot_platform
  "/home/tungxt/rosproject/src/morelab_robot_platform/srv/DriveWithSpeed.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/morelab_robot_platform
)
_generate_srv_cpp(morelab_robot_platform
  "/home/tungxt/rosproject/src/morelab_robot_platform/srv/GetStatus.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/morelab_robot_platform
)
_generate_srv_cpp(morelab_robot_platform
  "/home/tungxt/rosproject/src/morelab_robot_platform/srv/ResetEncoder.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/morelab_robot_platform
)
_generate_srv_cpp(morelab_robot_platform
  "/home/tungxt/rosproject/src/morelab_robot_platform/srv/StopAtDistance.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/morelab_robot_platform
)
_generate_srv_cpp(morelab_robot_platform
  "/home/tungxt/rosproject/src/morelab_robot_platform/srv/GetSpeed.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/morelab_robot_platform
)
_generate_srv_cpp(morelab_robot_platform
  "/home/tungxt/rosproject/src/morelab_robot_platform/srv/DriveWithDistance.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/morelab_robot_platform
)
_generate_srv_cpp(morelab_robot_platform
  "/home/tungxt/rosproject/src/morelab_robot_platform/srv/GetDistance.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/morelab_robot_platform
)

### Generating Module File
_generate_module_cpp(morelab_robot_platform
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/morelab_robot_platform
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(morelab_robot_platform_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(morelab_robot_platform_generate_messages morelab_robot_platform_generate_messages_cpp)

# target for backward compatibility
add_custom_target(morelab_robot_platform_gencpp)
add_dependencies(morelab_robot_platform_gencpp morelab_robot_platform_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS morelab_robot_platform_generate_messages_cpp)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(morelab_robot_platform
  "/home/tungxt/rosproject/src/morelab_robot_platform/msg/KeyStroke.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/morelab_robot_platform
)
_generate_msg_lisp(morelab_robot_platform
  "/home/tungxt/rosproject/src/morelab_robot_platform/msg/BatteryLevel.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/morelab_robot_platform
)
_generate_msg_lisp(morelab_robot_platform
  "/home/tungxt/rosproject/src/morelab_robot_platform/msg/Ping.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/morelab_robot_platform
)
_generate_msg_lisp(morelab_robot_platform
  "/home/tungxt/rosproject/src/morelab_robot_platform/msg/Voltages.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/morelab_robot_platform
)
_generate_msg_lisp(morelab_robot_platform
  "/home/tungxt/rosproject/src/morelab_robot_platform/msg/Velocity.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/morelab_robot_platform
)
_generate_msg_lisp(morelab_robot_platform
  "/home/tungxt/rosproject/src/morelab_robot_platform/msg/ADC.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/morelab_robot_platform
)
_generate_msg_lisp(morelab_robot_platform
  "/home/tungxt/rosproject/src/morelab_robot_platform/msg/Distances.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/morelab_robot_platform
)
_generate_msg_lisp(morelab_robot_platform
  "/home/tungxt/rosproject/src/morelab_robot_platform/msg/Speech.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/morelab_robot_platform
)
_generate_msg_lisp(morelab_robot_platform
  "/home/tungxt/rosproject/src/morelab_robot_platform/msg/StopSlidingSignal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/morelab_robot_platform
)
_generate_msg_lisp(morelab_robot_platform
  "/home/tungxt/rosproject/src/morelab_robot_platform/msg/SpeedWheel.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/morelab_robot_platform
)
_generate_msg_lisp(morelab_robot_platform
  "/home/tungxt/rosproject/src/morelab_robot_platform/msg/DistanceWheel.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/morelab_robot_platform
)

### Generating Services
_generate_srv_lisp(morelab_robot_platform
  "/home/tungxt/rosproject/src/morelab_robot_platform/srv/DriveWithPower.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/morelab_robot_platform
)
_generate_srv_lisp(morelab_robot_platform
  "/home/tungxt/rosproject/src/morelab_robot_platform/srv/Accelerate.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/morelab_robot_platform
)
_generate_srv_lisp(morelab_robot_platform
  "/home/tungxt/rosproject/src/morelab_robot_platform/srv/GetHeading.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/morelab_robot_platform
)
_generate_srv_lisp(morelab_robot_platform
  "/home/tungxt/rosproject/src/morelab_robot_platform/srv/Rotate.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/morelab_robot_platform
)
_generate_srv_lisp(morelab_robot_platform
  "/home/tungxt/rosproject/src/morelab_robot_platform/srv/DriveWithSpeed.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/morelab_robot_platform
)
_generate_srv_lisp(morelab_robot_platform
  "/home/tungxt/rosproject/src/morelab_robot_platform/srv/GetStatus.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/morelab_robot_platform
)
_generate_srv_lisp(morelab_robot_platform
  "/home/tungxt/rosproject/src/morelab_robot_platform/srv/ResetEncoder.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/morelab_robot_platform
)
_generate_srv_lisp(morelab_robot_platform
  "/home/tungxt/rosproject/src/morelab_robot_platform/srv/StopAtDistance.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/morelab_robot_platform
)
_generate_srv_lisp(morelab_robot_platform
  "/home/tungxt/rosproject/src/morelab_robot_platform/srv/GetSpeed.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/morelab_robot_platform
)
_generate_srv_lisp(morelab_robot_platform
  "/home/tungxt/rosproject/src/morelab_robot_platform/srv/DriveWithDistance.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/morelab_robot_platform
)
_generate_srv_lisp(morelab_robot_platform
  "/home/tungxt/rosproject/src/morelab_robot_platform/srv/GetDistance.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/morelab_robot_platform
)

### Generating Module File
_generate_module_lisp(morelab_robot_platform
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/morelab_robot_platform
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(morelab_robot_platform_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(morelab_robot_platform_generate_messages morelab_robot_platform_generate_messages_lisp)

# target for backward compatibility
add_custom_target(morelab_robot_platform_genlisp)
add_dependencies(morelab_robot_platform_genlisp morelab_robot_platform_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS morelab_robot_platform_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(morelab_robot_platform
  "/home/tungxt/rosproject/src/morelab_robot_platform/msg/KeyStroke.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/morelab_robot_platform
)
_generate_msg_py(morelab_robot_platform
  "/home/tungxt/rosproject/src/morelab_robot_platform/msg/BatteryLevel.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/morelab_robot_platform
)
_generate_msg_py(morelab_robot_platform
  "/home/tungxt/rosproject/src/morelab_robot_platform/msg/Ping.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/morelab_robot_platform
)
_generate_msg_py(morelab_robot_platform
  "/home/tungxt/rosproject/src/morelab_robot_platform/msg/Voltages.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/morelab_robot_platform
)
_generate_msg_py(morelab_robot_platform
  "/home/tungxt/rosproject/src/morelab_robot_platform/msg/Velocity.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/morelab_robot_platform
)
_generate_msg_py(morelab_robot_platform
  "/home/tungxt/rosproject/src/morelab_robot_platform/msg/ADC.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/morelab_robot_platform
)
_generate_msg_py(morelab_robot_platform
  "/home/tungxt/rosproject/src/morelab_robot_platform/msg/Distances.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/morelab_robot_platform
)
_generate_msg_py(morelab_robot_platform
  "/home/tungxt/rosproject/src/morelab_robot_platform/msg/Speech.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/morelab_robot_platform
)
_generate_msg_py(morelab_robot_platform
  "/home/tungxt/rosproject/src/morelab_robot_platform/msg/StopSlidingSignal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/morelab_robot_platform
)
_generate_msg_py(morelab_robot_platform
  "/home/tungxt/rosproject/src/morelab_robot_platform/msg/SpeedWheel.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/morelab_robot_platform
)
_generate_msg_py(morelab_robot_platform
  "/home/tungxt/rosproject/src/morelab_robot_platform/msg/DistanceWheel.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/morelab_robot_platform
)

### Generating Services
_generate_srv_py(morelab_robot_platform
  "/home/tungxt/rosproject/src/morelab_robot_platform/srv/DriveWithPower.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/morelab_robot_platform
)
_generate_srv_py(morelab_robot_platform
  "/home/tungxt/rosproject/src/morelab_robot_platform/srv/Accelerate.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/morelab_robot_platform
)
_generate_srv_py(morelab_robot_platform
  "/home/tungxt/rosproject/src/morelab_robot_platform/srv/GetHeading.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/morelab_robot_platform
)
_generate_srv_py(morelab_robot_platform
  "/home/tungxt/rosproject/src/morelab_robot_platform/srv/Rotate.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/morelab_robot_platform
)
_generate_srv_py(morelab_robot_platform
  "/home/tungxt/rosproject/src/morelab_robot_platform/srv/DriveWithSpeed.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/morelab_robot_platform
)
_generate_srv_py(morelab_robot_platform
  "/home/tungxt/rosproject/src/morelab_robot_platform/srv/GetStatus.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/morelab_robot_platform
)
_generate_srv_py(morelab_robot_platform
  "/home/tungxt/rosproject/src/morelab_robot_platform/srv/ResetEncoder.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/morelab_robot_platform
)
_generate_srv_py(morelab_robot_platform
  "/home/tungxt/rosproject/src/morelab_robot_platform/srv/StopAtDistance.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/morelab_robot_platform
)
_generate_srv_py(morelab_robot_platform
  "/home/tungxt/rosproject/src/morelab_robot_platform/srv/GetSpeed.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/morelab_robot_platform
)
_generate_srv_py(morelab_robot_platform
  "/home/tungxt/rosproject/src/morelab_robot_platform/srv/DriveWithDistance.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/morelab_robot_platform
)
_generate_srv_py(morelab_robot_platform
  "/home/tungxt/rosproject/src/morelab_robot_platform/srv/GetDistance.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/morelab_robot_platform
)

### Generating Module File
_generate_module_py(morelab_robot_platform
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/morelab_robot_platform
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(morelab_robot_platform_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(morelab_robot_platform_generate_messages morelab_robot_platform_generate_messages_py)

# target for backward compatibility
add_custom_target(morelab_robot_platform_genpy)
add_dependencies(morelab_robot_platform_genpy morelab_robot_platform_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS morelab_robot_platform_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/morelab_robot_platform)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/morelab_robot_platform
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
add_dependencies(morelab_robot_platform_generate_messages_cpp sensor_msgs_generate_messages_cpp)
add_dependencies(morelab_robot_platform_generate_messages_cpp geometry_msgs_generate_messages_cpp)
add_dependencies(morelab_robot_platform_generate_messages_cpp roscpp_generate_messages_cpp)
add_dependencies(morelab_robot_platform_generate_messages_cpp std_msgs_generate_messages_cpp)
add_dependencies(morelab_robot_platform_generate_messages_cpp tf_generate_messages_cpp)

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/morelab_robot_platform)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/morelab_robot_platform
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
add_dependencies(morelab_robot_platform_generate_messages_lisp sensor_msgs_generate_messages_lisp)
add_dependencies(morelab_robot_platform_generate_messages_lisp geometry_msgs_generate_messages_lisp)
add_dependencies(morelab_robot_platform_generate_messages_lisp roscpp_generate_messages_lisp)
add_dependencies(morelab_robot_platform_generate_messages_lisp std_msgs_generate_messages_lisp)
add_dependencies(morelab_robot_platform_generate_messages_lisp tf_generate_messages_lisp)

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/morelab_robot_platform)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/morelab_robot_platform\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/morelab_robot_platform
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
add_dependencies(morelab_robot_platform_generate_messages_py sensor_msgs_generate_messages_py)
add_dependencies(morelab_robot_platform_generate_messages_py geometry_msgs_generate_messages_py)
add_dependencies(morelab_robot_platform_generate_messages_py roscpp_generate_messages_py)
add_dependencies(morelab_robot_platform_generate_messages_py std_msgs_generate_messages_py)
add_dependencies(morelab_robot_platform_generate_messages_py tf_generate_messages_py)
