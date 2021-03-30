# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "position_control: 3 messages, 0 services")

set(MSG_I_FLAGS "-Iposition_control:/home/fechec/feng_ws/src;-Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/melodic/share/geometry_msgs/cmake/../msg;-Itraj_gen:/home/fechec/feng_ws/src/traj_gen/msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(position_control_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/fechec/feng_ws/src/traj_gen/msg/min_snap_traj.msg" NAME_WE)
add_custom_target(_position_control_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "position_control" "/home/fechec/feng_ws/src/traj_gen/msg/min_snap_traj.msg" "geometry_msgs/Vector3:std_msgs/Header"
)

get_filename_component(_filename "/home/fechec/feng_ws/src/position_control/msg/des_acc_ang.msg" NAME_WE)
add_custom_target(_position_control_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "position_control" "/home/fechec/feng_ws/src/position_control/msg/des_acc_ang.msg" "geometry_msgs/Vector3:std_msgs/Header"
)

get_filename_component(_filename "/home/fechec/feng_ws/src/target_generation/msg/target_dect_msg.msg" NAME_WE)
add_custom_target(_position_control_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "position_control" "/home/fechec/feng_ws/src/target_generation/msg/target_dect_msg.msg" "geometry_msgs/Vector3:std_msgs/Header"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(position_control
  "/home/fechec/feng_ws/src/traj_gen/msg/min_snap_traj.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/position_control
)
_generate_msg_cpp(position_control
  "/home/fechec/feng_ws/src/position_control/msg/des_acc_ang.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/position_control
)
_generate_msg_cpp(position_control
  "/home/fechec/feng_ws/src/target_generation/msg/target_dect_msg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/position_control
)

### Generating Services

### Generating Module File
_generate_module_cpp(position_control
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/position_control
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(position_control_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(position_control_generate_messages position_control_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/fechec/feng_ws/src/traj_gen/msg/min_snap_traj.msg" NAME_WE)
add_dependencies(position_control_generate_messages_cpp _position_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/fechec/feng_ws/src/position_control/msg/des_acc_ang.msg" NAME_WE)
add_dependencies(position_control_generate_messages_cpp _position_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/fechec/feng_ws/src/target_generation/msg/target_dect_msg.msg" NAME_WE)
add_dependencies(position_control_generate_messages_cpp _position_control_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(position_control_gencpp)
add_dependencies(position_control_gencpp position_control_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS position_control_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(position_control
  "/home/fechec/feng_ws/src/traj_gen/msg/min_snap_traj.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/position_control
)
_generate_msg_eus(position_control
  "/home/fechec/feng_ws/src/position_control/msg/des_acc_ang.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/position_control
)
_generate_msg_eus(position_control
  "/home/fechec/feng_ws/src/target_generation/msg/target_dect_msg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/position_control
)

### Generating Services

### Generating Module File
_generate_module_eus(position_control
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/position_control
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(position_control_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(position_control_generate_messages position_control_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/fechec/feng_ws/src/traj_gen/msg/min_snap_traj.msg" NAME_WE)
add_dependencies(position_control_generate_messages_eus _position_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/fechec/feng_ws/src/position_control/msg/des_acc_ang.msg" NAME_WE)
add_dependencies(position_control_generate_messages_eus _position_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/fechec/feng_ws/src/target_generation/msg/target_dect_msg.msg" NAME_WE)
add_dependencies(position_control_generate_messages_eus _position_control_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(position_control_geneus)
add_dependencies(position_control_geneus position_control_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS position_control_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(position_control
  "/home/fechec/feng_ws/src/traj_gen/msg/min_snap_traj.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/position_control
)
_generate_msg_lisp(position_control
  "/home/fechec/feng_ws/src/position_control/msg/des_acc_ang.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/position_control
)
_generate_msg_lisp(position_control
  "/home/fechec/feng_ws/src/target_generation/msg/target_dect_msg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/position_control
)

### Generating Services

### Generating Module File
_generate_module_lisp(position_control
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/position_control
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(position_control_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(position_control_generate_messages position_control_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/fechec/feng_ws/src/traj_gen/msg/min_snap_traj.msg" NAME_WE)
add_dependencies(position_control_generate_messages_lisp _position_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/fechec/feng_ws/src/position_control/msg/des_acc_ang.msg" NAME_WE)
add_dependencies(position_control_generate_messages_lisp _position_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/fechec/feng_ws/src/target_generation/msg/target_dect_msg.msg" NAME_WE)
add_dependencies(position_control_generate_messages_lisp _position_control_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(position_control_genlisp)
add_dependencies(position_control_genlisp position_control_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS position_control_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(position_control
  "/home/fechec/feng_ws/src/traj_gen/msg/min_snap_traj.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/position_control
)
_generate_msg_nodejs(position_control
  "/home/fechec/feng_ws/src/position_control/msg/des_acc_ang.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/position_control
)
_generate_msg_nodejs(position_control
  "/home/fechec/feng_ws/src/target_generation/msg/target_dect_msg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/position_control
)

### Generating Services

### Generating Module File
_generate_module_nodejs(position_control
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/position_control
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(position_control_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(position_control_generate_messages position_control_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/fechec/feng_ws/src/traj_gen/msg/min_snap_traj.msg" NAME_WE)
add_dependencies(position_control_generate_messages_nodejs _position_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/fechec/feng_ws/src/position_control/msg/des_acc_ang.msg" NAME_WE)
add_dependencies(position_control_generate_messages_nodejs _position_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/fechec/feng_ws/src/target_generation/msg/target_dect_msg.msg" NAME_WE)
add_dependencies(position_control_generate_messages_nodejs _position_control_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(position_control_gennodejs)
add_dependencies(position_control_gennodejs position_control_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS position_control_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(position_control
  "/home/fechec/feng_ws/src/traj_gen/msg/min_snap_traj.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/position_control
)
_generate_msg_py(position_control
  "/home/fechec/feng_ws/src/position_control/msg/des_acc_ang.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/position_control
)
_generate_msg_py(position_control
  "/home/fechec/feng_ws/src/target_generation/msg/target_dect_msg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/position_control
)

### Generating Services

### Generating Module File
_generate_module_py(position_control
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/position_control
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(position_control_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(position_control_generate_messages position_control_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/fechec/feng_ws/src/traj_gen/msg/min_snap_traj.msg" NAME_WE)
add_dependencies(position_control_generate_messages_py _position_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/fechec/feng_ws/src/position_control/msg/des_acc_ang.msg" NAME_WE)
add_dependencies(position_control_generate_messages_py _position_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/fechec/feng_ws/src/target_generation/msg/target_dect_msg.msg" NAME_WE)
add_dependencies(position_control_generate_messages_py _position_control_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(position_control_genpy)
add_dependencies(position_control_genpy position_control_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS position_control_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/position_control)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/position_control
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(position_control_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(position_control_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()
if(TARGET traj_gen_generate_messages_cpp)
  add_dependencies(position_control_generate_messages_cpp traj_gen_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/position_control)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/position_control
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(position_control_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(position_control_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()
if(TARGET traj_gen_generate_messages_eus)
  add_dependencies(position_control_generate_messages_eus traj_gen_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/position_control)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/position_control
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(position_control_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(position_control_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()
if(TARGET traj_gen_generate_messages_lisp)
  add_dependencies(position_control_generate_messages_lisp traj_gen_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/position_control)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/position_control
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(position_control_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(position_control_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()
if(TARGET traj_gen_generate_messages_nodejs)
  add_dependencies(position_control_generate_messages_nodejs traj_gen_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/position_control)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/position_control\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/position_control
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(position_control_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(position_control_generate_messages_py geometry_msgs_generate_messages_py)
endif()
if(TARGET traj_gen_generate_messages_py)
  add_dependencies(position_control_generate_messages_py traj_gen_generate_messages_py)
endif()
