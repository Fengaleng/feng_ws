# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include;/usr/include/eigen3".split(';') if "${prefix}/include;/usr/include/eigen3" != "" else []
PROJECT_CATKIN_DEPENDS = "dynamic_reconfigure;geometry_msgs;mav_msgs;nav_msgs;roscpp;sensor_msgs;trajectory_msgs;message_generation;std_msgs;geometry_msgs;traj_gen;position_control;message_runtime".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-lquat_att_control".split(';') if "-lquat_att_control" != "" else []
PROJECT_NAME = "attitude_control"
PROJECT_SPACE_DIR = "/home/fechec/feng_ws/install"
PROJECT_VERSION = "0.0.0"
