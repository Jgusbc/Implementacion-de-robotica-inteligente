# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "megatron: 1 messages, 0 services")

set(MSG_I_FLAGS "-Imegatron:/home/puzzlebot/puzzlebot_ws/src/megatron/msg;-Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(megatron_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/puzzlebot/puzzlebot_ws/src/megatron/msg/values.msg" NAME_WE)
add_custom_target(_megatron_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "megatron" "/home/puzzlebot/puzzlebot_ws/src/megatron/msg/values.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(megatron
  "/home/puzzlebot/puzzlebot_ws/src/megatron/msg/values.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/megatron
)

### Generating Services

### Generating Module File
_generate_module_cpp(megatron
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/megatron
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(megatron_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(megatron_generate_messages megatron_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/puzzlebot/puzzlebot_ws/src/megatron/msg/values.msg" NAME_WE)
add_dependencies(megatron_generate_messages_cpp _megatron_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(megatron_gencpp)
add_dependencies(megatron_gencpp megatron_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS megatron_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(megatron
  "/home/puzzlebot/puzzlebot_ws/src/megatron/msg/values.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/megatron
)

### Generating Services

### Generating Module File
_generate_module_eus(megatron
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/megatron
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(megatron_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(megatron_generate_messages megatron_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/puzzlebot/puzzlebot_ws/src/megatron/msg/values.msg" NAME_WE)
add_dependencies(megatron_generate_messages_eus _megatron_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(megatron_geneus)
add_dependencies(megatron_geneus megatron_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS megatron_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(megatron
  "/home/puzzlebot/puzzlebot_ws/src/megatron/msg/values.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/megatron
)

### Generating Services

### Generating Module File
_generate_module_lisp(megatron
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/megatron
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(megatron_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(megatron_generate_messages megatron_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/puzzlebot/puzzlebot_ws/src/megatron/msg/values.msg" NAME_WE)
add_dependencies(megatron_generate_messages_lisp _megatron_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(megatron_genlisp)
add_dependencies(megatron_genlisp megatron_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS megatron_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(megatron
  "/home/puzzlebot/puzzlebot_ws/src/megatron/msg/values.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/megatron
)

### Generating Services

### Generating Module File
_generate_module_nodejs(megatron
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/megatron
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(megatron_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(megatron_generate_messages megatron_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/puzzlebot/puzzlebot_ws/src/megatron/msg/values.msg" NAME_WE)
add_dependencies(megatron_generate_messages_nodejs _megatron_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(megatron_gennodejs)
add_dependencies(megatron_gennodejs megatron_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS megatron_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(megatron
  "/home/puzzlebot/puzzlebot_ws/src/megatron/msg/values.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/megatron
)

### Generating Services

### Generating Module File
_generate_module_py(megatron
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/megatron
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(megatron_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(megatron_generate_messages megatron_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/puzzlebot/puzzlebot_ws/src/megatron/msg/values.msg" NAME_WE)
add_dependencies(megatron_generate_messages_py _megatron_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(megatron_genpy)
add_dependencies(megatron_genpy megatron_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS megatron_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/megatron)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/megatron
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(megatron_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/megatron)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/megatron
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(megatron_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/megatron)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/megatron
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(megatron_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/megatron)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/megatron
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(megatron_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/megatron)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/megatron\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/megatron
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(megatron_generate_messages_py std_msgs_generate_messages_py)
endif()
