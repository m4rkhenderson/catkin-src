# generated from genmsg/cmake/pkg-msg-paths.cmake.em

# message include dirs in installspace
_prepend_path("${morelab_robot_platform_DIR}/.." "msg" morelab_robot_platform_MSG_INCLUDE_DIRS UNIQUE)
set(morelab_robot_platform_MSG_DEPENDENCIES sensor_msgs;geometry_msgs;roscpp;std_msgs;tf)
