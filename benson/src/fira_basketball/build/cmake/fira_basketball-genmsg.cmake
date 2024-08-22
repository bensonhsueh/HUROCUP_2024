# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "fira_basketball: 1 messages, 0 services")

set(MSG_I_FLAGS "-Ifira_basketball:/home/robotis/benson_ws/src/fira_basketball/msg;-Igeometry_msgs:/opt/ros/kinetic/share/geometry_msgs/cmake/../msg;-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(fira_basketball_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/robotis/benson_ws/src/fira_basketball/msg/ThreeDouble.msg" NAME_WE)
add_custom_target(_fira_basketball_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "fira_basketball" "/home/robotis/benson_ws/src/fira_basketball/msg/ThreeDouble.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(fira_basketball
  "/home/robotis/benson_ws/src/fira_basketball/msg/ThreeDouble.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/fira_basketball
)

### Generating Services

### Generating Module File
_generate_module_cpp(fira_basketball
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/fira_basketball
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(fira_basketball_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(fira_basketball_generate_messages fira_basketball_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/robotis/benson_ws/src/fira_basketball/msg/ThreeDouble.msg" NAME_WE)
add_dependencies(fira_basketball_generate_messages_cpp _fira_basketball_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(fira_basketball_gencpp)
add_dependencies(fira_basketball_gencpp fira_basketball_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS fira_basketball_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(fira_basketball
  "/home/robotis/benson_ws/src/fira_basketball/msg/ThreeDouble.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/fira_basketball
)

### Generating Services

### Generating Module File
_generate_module_eus(fira_basketball
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/fira_basketball
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(fira_basketball_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(fira_basketball_generate_messages fira_basketball_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/robotis/benson_ws/src/fira_basketball/msg/ThreeDouble.msg" NAME_WE)
add_dependencies(fira_basketball_generate_messages_eus _fira_basketball_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(fira_basketball_geneus)
add_dependencies(fira_basketball_geneus fira_basketball_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS fira_basketball_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(fira_basketball
  "/home/robotis/benson_ws/src/fira_basketball/msg/ThreeDouble.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/fira_basketball
)

### Generating Services

### Generating Module File
_generate_module_lisp(fira_basketball
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/fira_basketball
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(fira_basketball_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(fira_basketball_generate_messages fira_basketball_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/robotis/benson_ws/src/fira_basketball/msg/ThreeDouble.msg" NAME_WE)
add_dependencies(fira_basketball_generate_messages_lisp _fira_basketball_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(fira_basketball_genlisp)
add_dependencies(fira_basketball_genlisp fira_basketball_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS fira_basketball_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(fira_basketball
  "/home/robotis/benson_ws/src/fira_basketball/msg/ThreeDouble.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/fira_basketball
)

### Generating Services

### Generating Module File
_generate_module_nodejs(fira_basketball
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/fira_basketball
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(fira_basketball_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(fira_basketball_generate_messages fira_basketball_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/robotis/benson_ws/src/fira_basketball/msg/ThreeDouble.msg" NAME_WE)
add_dependencies(fira_basketball_generate_messages_nodejs _fira_basketball_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(fira_basketball_gennodejs)
add_dependencies(fira_basketball_gennodejs fira_basketball_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS fira_basketball_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(fira_basketball
  "/home/robotis/benson_ws/src/fira_basketball/msg/ThreeDouble.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/fira_basketball
)

### Generating Services

### Generating Module File
_generate_module_py(fira_basketball
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/fira_basketball
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(fira_basketball_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(fira_basketball_generate_messages fira_basketball_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/robotis/benson_ws/src/fira_basketball/msg/ThreeDouble.msg" NAME_WE)
add_dependencies(fira_basketball_generate_messages_py _fira_basketball_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(fira_basketball_genpy)
add_dependencies(fira_basketball_genpy fira_basketball_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS fira_basketball_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/fira_basketball)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/fira_basketball
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(fira_basketball_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/fira_basketball)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/fira_basketball
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(fira_basketball_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/fira_basketball)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/fira_basketball
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(fira_basketball_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/fira_basketball)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/fira_basketball
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(fira_basketball_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/fira_basketball)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/fira_basketball\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/fira_basketball
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(fira_basketball_generate_messages_py geometry_msgs_generate_messages_py)
endif()
