# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "navplay: 3 messages, 0 services")

set(MSG_I_FLAGS "-Inavplay:/media/rebeater/hd_data2/workspace/CLionProjects/LooselyCouple2020_cpp/App/RtSimulation/src/navplay/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(navplay_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/media/rebeater/hd_data2/workspace/CLionProjects/LooselyCouple2020_cpp/App/RtSimulation/src/navplay/msg/gnss.msg" NAME_WE)
add_custom_target(_navplay_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "navplay" "/media/rebeater/hd_data2/workspace/CLionProjects/LooselyCouple2020_cpp/App/RtSimulation/src/navplay/msg/gnss.msg" ""
)

get_filename_component(_filename "/media/rebeater/hd_data2/workspace/CLionProjects/LooselyCouple2020_cpp/App/RtSimulation/src/navplay/msg/imu.msg" NAME_WE)
add_custom_target(_navplay_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "navplay" "/media/rebeater/hd_data2/workspace/CLionProjects/LooselyCouple2020_cpp/App/RtSimulation/src/navplay/msg/imu.msg" ""
)

get_filename_component(_filename "/media/rebeater/hd_data2/workspace/CLionProjects/LooselyCouple2020_cpp/App/RtSimulation/src/navplay/msg/vel.msg" NAME_WE)
add_custom_target(_navplay_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "navplay" "/media/rebeater/hd_data2/workspace/CLionProjects/LooselyCouple2020_cpp/App/RtSimulation/src/navplay/msg/vel.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(navplay
  "/media/rebeater/hd_data2/workspace/CLionProjects/LooselyCouple2020_cpp/App/RtSimulation/src/navplay/msg/gnss.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/navplay
)
_generate_msg_cpp(navplay
  "/media/rebeater/hd_data2/workspace/CLionProjects/LooselyCouple2020_cpp/App/RtSimulation/src/navplay/msg/imu.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/navplay
)
_generate_msg_cpp(navplay
  "/media/rebeater/hd_data2/workspace/CLionProjects/LooselyCouple2020_cpp/App/RtSimulation/src/navplay/msg/vel.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/navplay
)

### Generating Services

### Generating Module File
_generate_module_cpp(navplay
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/navplay
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(navplay_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(navplay_generate_messages navplay_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/media/rebeater/hd_data2/workspace/CLionProjects/LooselyCouple2020_cpp/App/RtSimulation/src/navplay/msg/gnss.msg" NAME_WE)
add_dependencies(navplay_generate_messages_cpp _navplay_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/rebeater/hd_data2/workspace/CLionProjects/LooselyCouple2020_cpp/App/RtSimulation/src/navplay/msg/imu.msg" NAME_WE)
add_dependencies(navplay_generate_messages_cpp _navplay_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/rebeater/hd_data2/workspace/CLionProjects/LooselyCouple2020_cpp/App/RtSimulation/src/navplay/msg/vel.msg" NAME_WE)
add_dependencies(navplay_generate_messages_cpp _navplay_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(navplay_gencpp)
add_dependencies(navplay_gencpp navplay_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS navplay_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(navplay
  "/media/rebeater/hd_data2/workspace/CLionProjects/LooselyCouple2020_cpp/App/RtSimulation/src/navplay/msg/gnss.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/navplay
)
_generate_msg_eus(navplay
  "/media/rebeater/hd_data2/workspace/CLionProjects/LooselyCouple2020_cpp/App/RtSimulation/src/navplay/msg/imu.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/navplay
)
_generate_msg_eus(navplay
  "/media/rebeater/hd_data2/workspace/CLionProjects/LooselyCouple2020_cpp/App/RtSimulation/src/navplay/msg/vel.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/navplay
)

### Generating Services

### Generating Module File
_generate_module_eus(navplay
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/navplay
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(navplay_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(navplay_generate_messages navplay_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/media/rebeater/hd_data2/workspace/CLionProjects/LooselyCouple2020_cpp/App/RtSimulation/src/navplay/msg/gnss.msg" NAME_WE)
add_dependencies(navplay_generate_messages_eus _navplay_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/rebeater/hd_data2/workspace/CLionProjects/LooselyCouple2020_cpp/App/RtSimulation/src/navplay/msg/imu.msg" NAME_WE)
add_dependencies(navplay_generate_messages_eus _navplay_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/rebeater/hd_data2/workspace/CLionProjects/LooselyCouple2020_cpp/App/RtSimulation/src/navplay/msg/vel.msg" NAME_WE)
add_dependencies(navplay_generate_messages_eus _navplay_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(navplay_geneus)
add_dependencies(navplay_geneus navplay_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS navplay_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(navplay
  "/media/rebeater/hd_data2/workspace/CLionProjects/LooselyCouple2020_cpp/App/RtSimulation/src/navplay/msg/gnss.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/navplay
)
_generate_msg_lisp(navplay
  "/media/rebeater/hd_data2/workspace/CLionProjects/LooselyCouple2020_cpp/App/RtSimulation/src/navplay/msg/imu.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/navplay
)
_generate_msg_lisp(navplay
  "/media/rebeater/hd_data2/workspace/CLionProjects/LooselyCouple2020_cpp/App/RtSimulation/src/navplay/msg/vel.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/navplay
)

### Generating Services

### Generating Module File
_generate_module_lisp(navplay
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/navplay
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(navplay_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(navplay_generate_messages navplay_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/media/rebeater/hd_data2/workspace/CLionProjects/LooselyCouple2020_cpp/App/RtSimulation/src/navplay/msg/gnss.msg" NAME_WE)
add_dependencies(navplay_generate_messages_lisp _navplay_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/rebeater/hd_data2/workspace/CLionProjects/LooselyCouple2020_cpp/App/RtSimulation/src/navplay/msg/imu.msg" NAME_WE)
add_dependencies(navplay_generate_messages_lisp _navplay_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/rebeater/hd_data2/workspace/CLionProjects/LooselyCouple2020_cpp/App/RtSimulation/src/navplay/msg/vel.msg" NAME_WE)
add_dependencies(navplay_generate_messages_lisp _navplay_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(navplay_genlisp)
add_dependencies(navplay_genlisp navplay_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS navplay_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(navplay
  "/media/rebeater/hd_data2/workspace/CLionProjects/LooselyCouple2020_cpp/App/RtSimulation/src/navplay/msg/gnss.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/navplay
)
_generate_msg_nodejs(navplay
  "/media/rebeater/hd_data2/workspace/CLionProjects/LooselyCouple2020_cpp/App/RtSimulation/src/navplay/msg/imu.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/navplay
)
_generate_msg_nodejs(navplay
  "/media/rebeater/hd_data2/workspace/CLionProjects/LooselyCouple2020_cpp/App/RtSimulation/src/navplay/msg/vel.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/navplay
)

### Generating Services

### Generating Module File
_generate_module_nodejs(navplay
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/navplay
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(navplay_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(navplay_generate_messages navplay_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/media/rebeater/hd_data2/workspace/CLionProjects/LooselyCouple2020_cpp/App/RtSimulation/src/navplay/msg/gnss.msg" NAME_WE)
add_dependencies(navplay_generate_messages_nodejs _navplay_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/rebeater/hd_data2/workspace/CLionProjects/LooselyCouple2020_cpp/App/RtSimulation/src/navplay/msg/imu.msg" NAME_WE)
add_dependencies(navplay_generate_messages_nodejs _navplay_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/rebeater/hd_data2/workspace/CLionProjects/LooselyCouple2020_cpp/App/RtSimulation/src/navplay/msg/vel.msg" NAME_WE)
add_dependencies(navplay_generate_messages_nodejs _navplay_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(navplay_gennodejs)
add_dependencies(navplay_gennodejs navplay_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS navplay_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(navplay
  "/media/rebeater/hd_data2/workspace/CLionProjects/LooselyCouple2020_cpp/App/RtSimulation/src/navplay/msg/gnss.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/navplay
)
_generate_msg_py(navplay
  "/media/rebeater/hd_data2/workspace/CLionProjects/LooselyCouple2020_cpp/App/RtSimulation/src/navplay/msg/imu.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/navplay
)
_generate_msg_py(navplay
  "/media/rebeater/hd_data2/workspace/CLionProjects/LooselyCouple2020_cpp/App/RtSimulation/src/navplay/msg/vel.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/navplay
)

### Generating Services

### Generating Module File
_generate_module_py(navplay
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/navplay
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(navplay_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(navplay_generate_messages navplay_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/media/rebeater/hd_data2/workspace/CLionProjects/LooselyCouple2020_cpp/App/RtSimulation/src/navplay/msg/gnss.msg" NAME_WE)
add_dependencies(navplay_generate_messages_py _navplay_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/rebeater/hd_data2/workspace/CLionProjects/LooselyCouple2020_cpp/App/RtSimulation/src/navplay/msg/imu.msg" NAME_WE)
add_dependencies(navplay_generate_messages_py _navplay_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/rebeater/hd_data2/workspace/CLionProjects/LooselyCouple2020_cpp/App/RtSimulation/src/navplay/msg/vel.msg" NAME_WE)
add_dependencies(navplay_generate_messages_py _navplay_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(navplay_genpy)
add_dependencies(navplay_genpy navplay_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS navplay_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/navplay)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/navplay
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(navplay_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/navplay)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/navplay
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(navplay_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/navplay)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/navplay
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(navplay_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/navplay)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/navplay
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(navplay_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/navplay)
  install(CODE "execute_process(COMMAND \"/home/rebeater/miniconda3/envs/ros/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/navplay\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/navplay
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(navplay_generate_messages_py std_msgs_generate_messages_py)
endif()
