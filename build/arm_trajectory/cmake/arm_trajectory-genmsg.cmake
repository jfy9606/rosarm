# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "arm_trajectory: 2 messages, 1 services")

set(MSG_I_FLAGS "-Iarm_trajectory:/home/jfy/arm/catkin/src/arm_trajectory/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(arm_trajectory_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/jfy/arm/catkin/src/arm_trajectory/msg/TrajectoryPoint.msg" NAME_WE)
add_custom_target(_arm_trajectory_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "arm_trajectory" "/home/jfy/arm/catkin/src/arm_trajectory/msg/TrajectoryPoint.msg" "geometry_msgs/Pose:geometry_msgs/Point:std_msgs/Header:geometry_msgs/Quaternion"
)

get_filename_component(_filename "/home/jfy/arm/catkin/src/arm_trajectory/msg/TrajectoryPath.msg" NAME_WE)
add_custom_target(_arm_trajectory_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "arm_trajectory" "/home/jfy/arm/catkin/src/arm_trajectory/msg/TrajectoryPath.msg" "geometry_msgs/Point:geometry_msgs/Quaternion:std_msgs/Header:arm_trajectory/TrajectoryPoint:geometry_msgs/Pose"
)

get_filename_component(_filename "/home/jfy/arm/catkin/src/arm_trajectory/srv/PlanTrajectory.srv" NAME_WE)
add_custom_target(_arm_trajectory_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "arm_trajectory" "/home/jfy/arm/catkin/src/arm_trajectory/srv/PlanTrajectory.srv" "geometry_msgs/Point:geometry_msgs/Quaternion:arm_trajectory/TrajectoryPath:std_msgs/Header:arm_trajectory/TrajectoryPoint:geometry_msgs/Pose"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(arm_trajectory
  "/home/jfy/arm/catkin/src/arm_trajectory/msg/TrajectoryPoint.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/arm_trajectory
)
_generate_msg_cpp(arm_trajectory
  "/home/jfy/arm/catkin/src/arm_trajectory/msg/TrajectoryPath.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/jfy/arm/catkin/src/arm_trajectory/msg/TrajectoryPoint.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/arm_trajectory
)

### Generating Services
_generate_srv_cpp(arm_trajectory
  "/home/jfy/arm/catkin/src/arm_trajectory/srv/PlanTrajectory.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/home/jfy/arm/catkin/src/arm_trajectory/msg/TrajectoryPath.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/jfy/arm/catkin/src/arm_trajectory/msg/TrajectoryPoint.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/arm_trajectory
)

### Generating Module File
_generate_module_cpp(arm_trajectory
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/arm_trajectory
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(arm_trajectory_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(arm_trajectory_generate_messages arm_trajectory_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/jfy/arm/catkin/src/arm_trajectory/msg/TrajectoryPoint.msg" NAME_WE)
add_dependencies(arm_trajectory_generate_messages_cpp _arm_trajectory_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jfy/arm/catkin/src/arm_trajectory/msg/TrajectoryPath.msg" NAME_WE)
add_dependencies(arm_trajectory_generate_messages_cpp _arm_trajectory_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jfy/arm/catkin/src/arm_trajectory/srv/PlanTrajectory.srv" NAME_WE)
add_dependencies(arm_trajectory_generate_messages_cpp _arm_trajectory_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(arm_trajectory_gencpp)
add_dependencies(arm_trajectory_gencpp arm_trajectory_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS arm_trajectory_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(arm_trajectory
  "/home/jfy/arm/catkin/src/arm_trajectory/msg/TrajectoryPoint.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/arm_trajectory
)
_generate_msg_eus(arm_trajectory
  "/home/jfy/arm/catkin/src/arm_trajectory/msg/TrajectoryPath.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/jfy/arm/catkin/src/arm_trajectory/msg/TrajectoryPoint.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/arm_trajectory
)

### Generating Services
_generate_srv_eus(arm_trajectory
  "/home/jfy/arm/catkin/src/arm_trajectory/srv/PlanTrajectory.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/home/jfy/arm/catkin/src/arm_trajectory/msg/TrajectoryPath.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/jfy/arm/catkin/src/arm_trajectory/msg/TrajectoryPoint.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/arm_trajectory
)

### Generating Module File
_generate_module_eus(arm_trajectory
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/arm_trajectory
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(arm_trajectory_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(arm_trajectory_generate_messages arm_trajectory_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/jfy/arm/catkin/src/arm_trajectory/msg/TrajectoryPoint.msg" NAME_WE)
add_dependencies(arm_trajectory_generate_messages_eus _arm_trajectory_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jfy/arm/catkin/src/arm_trajectory/msg/TrajectoryPath.msg" NAME_WE)
add_dependencies(arm_trajectory_generate_messages_eus _arm_trajectory_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jfy/arm/catkin/src/arm_trajectory/srv/PlanTrajectory.srv" NAME_WE)
add_dependencies(arm_trajectory_generate_messages_eus _arm_trajectory_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(arm_trajectory_geneus)
add_dependencies(arm_trajectory_geneus arm_trajectory_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS arm_trajectory_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(arm_trajectory
  "/home/jfy/arm/catkin/src/arm_trajectory/msg/TrajectoryPoint.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/arm_trajectory
)
_generate_msg_lisp(arm_trajectory
  "/home/jfy/arm/catkin/src/arm_trajectory/msg/TrajectoryPath.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/jfy/arm/catkin/src/arm_trajectory/msg/TrajectoryPoint.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/arm_trajectory
)

### Generating Services
_generate_srv_lisp(arm_trajectory
  "/home/jfy/arm/catkin/src/arm_trajectory/srv/PlanTrajectory.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/home/jfy/arm/catkin/src/arm_trajectory/msg/TrajectoryPath.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/jfy/arm/catkin/src/arm_trajectory/msg/TrajectoryPoint.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/arm_trajectory
)

### Generating Module File
_generate_module_lisp(arm_trajectory
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/arm_trajectory
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(arm_trajectory_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(arm_trajectory_generate_messages arm_trajectory_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/jfy/arm/catkin/src/arm_trajectory/msg/TrajectoryPoint.msg" NAME_WE)
add_dependencies(arm_trajectory_generate_messages_lisp _arm_trajectory_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jfy/arm/catkin/src/arm_trajectory/msg/TrajectoryPath.msg" NAME_WE)
add_dependencies(arm_trajectory_generate_messages_lisp _arm_trajectory_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jfy/arm/catkin/src/arm_trajectory/srv/PlanTrajectory.srv" NAME_WE)
add_dependencies(arm_trajectory_generate_messages_lisp _arm_trajectory_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(arm_trajectory_genlisp)
add_dependencies(arm_trajectory_genlisp arm_trajectory_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS arm_trajectory_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(arm_trajectory
  "/home/jfy/arm/catkin/src/arm_trajectory/msg/TrajectoryPoint.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/arm_trajectory
)
_generate_msg_nodejs(arm_trajectory
  "/home/jfy/arm/catkin/src/arm_trajectory/msg/TrajectoryPath.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/jfy/arm/catkin/src/arm_trajectory/msg/TrajectoryPoint.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/arm_trajectory
)

### Generating Services
_generate_srv_nodejs(arm_trajectory
  "/home/jfy/arm/catkin/src/arm_trajectory/srv/PlanTrajectory.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/home/jfy/arm/catkin/src/arm_trajectory/msg/TrajectoryPath.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/jfy/arm/catkin/src/arm_trajectory/msg/TrajectoryPoint.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/arm_trajectory
)

### Generating Module File
_generate_module_nodejs(arm_trajectory
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/arm_trajectory
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(arm_trajectory_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(arm_trajectory_generate_messages arm_trajectory_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/jfy/arm/catkin/src/arm_trajectory/msg/TrajectoryPoint.msg" NAME_WE)
add_dependencies(arm_trajectory_generate_messages_nodejs _arm_trajectory_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jfy/arm/catkin/src/arm_trajectory/msg/TrajectoryPath.msg" NAME_WE)
add_dependencies(arm_trajectory_generate_messages_nodejs _arm_trajectory_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jfy/arm/catkin/src/arm_trajectory/srv/PlanTrajectory.srv" NAME_WE)
add_dependencies(arm_trajectory_generate_messages_nodejs _arm_trajectory_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(arm_trajectory_gennodejs)
add_dependencies(arm_trajectory_gennodejs arm_trajectory_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS arm_trajectory_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(arm_trajectory
  "/home/jfy/arm/catkin/src/arm_trajectory/msg/TrajectoryPoint.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/arm_trajectory
)
_generate_msg_py(arm_trajectory
  "/home/jfy/arm/catkin/src/arm_trajectory/msg/TrajectoryPath.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/jfy/arm/catkin/src/arm_trajectory/msg/TrajectoryPoint.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/arm_trajectory
)

### Generating Services
_generate_srv_py(arm_trajectory
  "/home/jfy/arm/catkin/src/arm_trajectory/srv/PlanTrajectory.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/home/jfy/arm/catkin/src/arm_trajectory/msg/TrajectoryPath.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/jfy/arm/catkin/src/arm_trajectory/msg/TrajectoryPoint.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/arm_trajectory
)

### Generating Module File
_generate_module_py(arm_trajectory
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/arm_trajectory
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(arm_trajectory_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(arm_trajectory_generate_messages arm_trajectory_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/jfy/arm/catkin/src/arm_trajectory/msg/TrajectoryPoint.msg" NAME_WE)
add_dependencies(arm_trajectory_generate_messages_py _arm_trajectory_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jfy/arm/catkin/src/arm_trajectory/msg/TrajectoryPath.msg" NAME_WE)
add_dependencies(arm_trajectory_generate_messages_py _arm_trajectory_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jfy/arm/catkin/src/arm_trajectory/srv/PlanTrajectory.srv" NAME_WE)
add_dependencies(arm_trajectory_generate_messages_py _arm_trajectory_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(arm_trajectory_genpy)
add_dependencies(arm_trajectory_genpy arm_trajectory_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS arm_trajectory_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/arm_trajectory)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/arm_trajectory
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(arm_trajectory_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(arm_trajectory_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/arm_trajectory)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/arm_trajectory
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(arm_trajectory_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(arm_trajectory_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/arm_trajectory)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/arm_trajectory
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(arm_trajectory_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(arm_trajectory_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/arm_trajectory)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/arm_trajectory
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(arm_trajectory_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(arm_trajectory_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/arm_trajectory)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/arm_trajectory\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/arm_trajectory
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(arm_trajectory_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(arm_trajectory_generate_messages_py geometry_msgs_generate_messages_py)
endif()
