# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "attitude_control: 3 messages, 0 services")

set(MSG_I_FLAGS "-Iattitude_control:/home/fechec/feng_ws/src/attitude_control/msg;-Iattitude_control:/home/fechec/feng_ws/src;-Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/melodic/share/geometry_msgs/cmake/../msg;-Iposition_control:/home/fechec/feng_ws/src;-Itraj_gen:/home/fechec/feng_ws/src/traj_gen/msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(attitude_control_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/fechec/feng_ws/src/traj_gen/msg/min_snap_traj.msg" NAME_WE)
add_custom_target(_attitude_control_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "attitude_control" "/home/fechec/feng_ws/src/traj_gen/msg/min_snap_traj.msg" "geometry_msgs/Vector3:std_msgs/Header"
)

get_filename_component(_filename "/home/fechec/feng_ws/src/attitude_control/msg/uav_state.msg" NAME_WE)
add_custom_target(_attitude_control_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "attitude_control" "/home/fechec/feng_ws/src/attitude_control/msg/uav_state.msg" "geometry_msgs/Vector3:std_msgs/Header"
)

get_filename_component(_filename "/home/fechec/feng_ws/src/position_control/msg/des_acc_ang.msg" NAME_WE)
add_custom_target(_attitude_control_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "attitude_control" "/home/fechec/feng_ws/src/position_control/msg/des_acc_ang.msg" "geometry_msgs/Vector3:std_msgs/Header"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(attitude_control
  "/home/fechec/feng_ws/src/traj_gen/msg/min_snap_traj.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/attitude_control
)
_generate_msg_cpp(attitude_control
  "/home/fechec/feng_ws/src/position_control/msg/des_acc_ang.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/attitude_control
)
_generate_msg_cpp(attitude_control
  "/home/fechec/feng_ws/src/attitude_control/msg/uav_state.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/attitude_control
)

### Generating Services

### Generating Module File
_generate_module_cpp(attitude_control
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/attitude_control
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(attitude_control_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(attitude_control_generate_messages attitude_control_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/fechec/feng_ws/src/traj_gen/msg/min_snap_traj.msg" NAME_WE)
add_dependencies(attitude_control_generate_messages_cpp _attitude_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/fechec/feng_ws/src/attitude_control/msg/uav_state.msg" NAME_WE)
add_dependencies(attitude_control_generate_messages_cpp _attitude_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/fechec/feng_ws/src/position_control/msg/des_acc_ang.msg" NAME_WE)
add_dependencies(attitude_control_generate_messages_cpp _attitude_control_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(attitude_control_gencpp)
add_dependencies(attitude_control_gencpp attitude_control_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS attitude_control_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(attitude_control
  "/home/fechec/feng_ws/src/traj_gen/msg/min_snap_traj.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/attitude_control
)
_generate_msg_eus(attitude_control
  "/home/fechec/feng_ws/src/position_control/msg/des_acc_ang.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/attitude_control
)
_generate_msg_eus(attitude_control
  "/home/fechec/feng_ws/src/attitude_control/msg/uav_state.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/attitude_control
)

### Generating Services

### Generating Module File
_generate_module_eus(attitude_control
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/attitude_control
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(attitude_control_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(attitude_control_generate_messages attitude_control_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/fechec/feng_ws/src/traj_gen/msg/min_snap_traj.msg" NAME_WE)
add_dependencies(attitude_control_generate_messages_eus _attitude_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/fechec/feng_ws/src/attitude_control/msg/uav_state.msg" NAME_WE)
add_dependencies(attitude_control_generate_messages_eus _attitude_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/fechec/feng_ws/src/position_control/msg/des_acc_ang.msg" NAME_WE)
add_dependencies(attitude_control_generate_messages_eus _attitude_control_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(attitude_control_geneus)
add_dependencies(attitude_control_geneus attitude_control_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS attitude_control_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(attitude_control
  "/home/fechec/feng_ws/src/traj_gen/msg/min_snap_traj.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/attitude_control
)
_generate_msg_lisp(attitude_control
  "/home/fechec/feng_ws/src/position_control/msg/des_acc_ang.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/attitude_control
)
_generate_msg_lisp(attitude_control
  "/home/fechec/feng_ws/src/attitude_control/msg/uav_state.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/attitude_control
)

### Generating Services

### Generating Module File
_generate_module_lisp(attitude_control
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/attitude_control
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(attitude_control_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(attitude_control_generate_messages attitude_control_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/fechec/feng_ws/src/traj_gen/msg/min_snap_traj.msg" NAME_WE)
add_dependencies(attitude_control_generate_messages_lisp _attitude_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/fechec/feng_ws/src/attitude_control/msg/uav_state.msg" NAME_WE)
add_dependencies(attitude_control_generate_messages_lisp _attitude_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/fechec/feng_ws/src/position_control/msg/des_acc_ang.msg" NAME_WE)
add_dependencies(attitude_control_generate_messages_lisp _attitude_control_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(attitude_control_genlisp)
add_dependencies(attitude_control_genlisp attitude_control_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS attitude_control_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(attitude_control
  "/home/fechec/feng_ws/src/traj_gen/msg/min_snap_traj.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/attitude_control
)
_generate_msg_nodejs(attitude_control
  "/home/fechec/feng_ws/src/position_control/msg/des_acc_ang.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/attitude_control
)
_generate_msg_nodejs(attitude_control
  "/home/fechec/feng_ws/src/attitude_control/msg/uav_state.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/attitude_control
)

### Generating Services

### Generating Module File
_generate_module_nodejs(attitude_control
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/attitude_control
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(attitude_control_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(attitude_control_generate_messages attitude_control_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/fechec/feng_ws/src/traj_gen/msg/min_snap_traj.msg" NAME_WE)
add_dependencies(attitude_control_generate_messages_nodejs _attitude_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/fechec/feng_ws/src/attitude_control/msg/uav_state.msg" NAME_WE)
add_dependencies(attitude_control_generate_messages_nodejs _attitude_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/fechec/feng_ws/src/position_control/msg/des_acc_ang.msg" NAME_WE)
add_dependencies(attitude_control_generate_messages_nodejs _attitude_control_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(attitude_control_gennodejs)
add_dependencies(attitude_control_gennodejs attitude_control_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS attitude_control_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(attitude_control
  "/home/fechec/feng_ws/src/traj_gen/msg/min_snap_traj.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/attitude_control
)
_generate_msg_py(attitude_control
  "/home/fechec/feng_ws/src/position_control/msg/des_acc_ang.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/attitude_control
)
_generate_msg_py(attitude_control
  "/home/fechec/feng_ws/src/attitude_control/msg/uav_state.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/attitude_control
)

### Generating Services

### Generating Module File
_generate_module_py(attitude_control
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/attitude_control
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(attitude_control_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(attitude_control_generate_messages attitude_control_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/fechec/feng_ws/src/traj_gen/msg/min_snap_traj.msg" NAME_WE)
add_dependencies(attitude_control_generate_messages_py _attitude_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/fechec/feng_ws/src/attitude_control/msg/uav_state.msg" NAME_WE)
add_dependencies(attitude_control_generate_messages_py _attitude_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/fechec/feng_ws/src/position_control/msg/des_acc_ang.msg" NAME_WE)
add_dependencies(attitude_control_generate_messages_py _attitude_control_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(attitude_control_genpy)
add_dependencies(attitude_control_genpy attitude_control_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS attitude_control_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/attitude_control)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/attitude_control
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(attitude_control_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(attitude_control_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()
if(TARGET position_control_generate_messages_cpp)
  add_dependencies(attitude_control_generate_messages_cpp position_control_generate_messages_cpp)
endif()
if(TARGET traj_gen_generate_messages_cpp)
  add_dependencies(attitude_control_generate_messages_cpp traj_gen_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/attitude_control)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/attitude_control
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(attitude_control_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(attitude_control_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()
if(TARGET position_control_generate_messages_eus)
  add_dependencies(attitude_control_generate_messages_eus position_control_generate_messages_eus)
endif()
if(TARGET traj_gen_generate_messages_eus)
  add_dependencies(attitude_control_generate_messages_eus traj_gen_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/attitude_control)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/attitude_control
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(attitude_control_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(attitude_control_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()
if(TARGET position_control_generate_messages_lisp)
  add_dependencies(attitude_control_generate_messages_lisp position_control_generate_messages_lisp)
endif()
if(TARGET traj_gen_generate_messages_lisp)
  add_dependencies(attitude_control_generate_messages_lisp traj_gen_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/attitude_control)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/attitude_control
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(attitude_control_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(attitude_control_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()
if(TARGET position_control_generate_messages_nodejs)
  add_dependencies(attitude_control_generate_messages_nodejs position_control_generate_messages_nodejs)
endif()
if(TARGET traj_gen_generate_messages_nodejs)
  add_dependencies(attitude_control_generate_messages_nodejs traj_gen_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/attitude_control)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/attitude_control\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/attitude_control
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(attitude_control_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(attitude_control_generate_messages_py geometry_msgs_generate_messages_py)
endif()
if(TARGET position_control_generate_messages_py)
  add_dependencies(attitude_control_generate_messages_py position_control_generate_messages_py)
endif()
if(TARGET traj_gen_generate_messages_py)
  add_dependencies(attitude_control_generate_messages_py traj_gen_generate_messages_py)
endif()
