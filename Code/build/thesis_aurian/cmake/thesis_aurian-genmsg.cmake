# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "thesis_aurian: 1 messages, 0 services")

set(MSG_I_FLAGS "-Ithesis_aurian:/home/laboinmastudent/Bureau/Drone_thesis_2017/Code/src/thesis_aurian/msg;-Iardrone_autonomy:/opt/ros/indigo/share/ardrone_autonomy/cmake/../msg;-Igeometry_msgs:/opt/ros/indigo/share/geometry_msgs/cmake/../msg;-Istd_msgs:/opt/ros/indigo/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(thesis_aurian_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/laboinmastudent/Bureau/Drone_thesis_2017/Code/src/thesis_aurian/msg/refdoors.msg" NAME_WE)
add_custom_target(_thesis_aurian_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "thesis_aurian" "/home/laboinmastudent/Bureau/Drone_thesis_2017/Code/src/thesis_aurian/msg/refdoors.msg" ""
)

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(thesis_aurian
  "/home/laboinmastudent/Bureau/Drone_thesis_2017/Code/src/thesis_aurian/msg/refdoors.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/thesis_aurian
)

### Generating Services

### Generating Module File
_generate_module_cpp(thesis_aurian
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/thesis_aurian
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(thesis_aurian_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(thesis_aurian_generate_messages thesis_aurian_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/laboinmastudent/Bureau/Drone_thesis_2017/Code/src/thesis_aurian/msg/refdoors.msg" NAME_WE)
add_dependencies(thesis_aurian_generate_messages_cpp _thesis_aurian_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(thesis_aurian_gencpp)
add_dependencies(thesis_aurian_gencpp thesis_aurian_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS thesis_aurian_generate_messages_cpp)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(thesis_aurian
  "/home/laboinmastudent/Bureau/Drone_thesis_2017/Code/src/thesis_aurian/msg/refdoors.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/thesis_aurian
)

### Generating Services

### Generating Module File
_generate_module_lisp(thesis_aurian
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/thesis_aurian
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(thesis_aurian_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(thesis_aurian_generate_messages thesis_aurian_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/laboinmastudent/Bureau/Drone_thesis_2017/Code/src/thesis_aurian/msg/refdoors.msg" NAME_WE)
add_dependencies(thesis_aurian_generate_messages_lisp _thesis_aurian_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(thesis_aurian_genlisp)
add_dependencies(thesis_aurian_genlisp thesis_aurian_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS thesis_aurian_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(thesis_aurian
  "/home/laboinmastudent/Bureau/Drone_thesis_2017/Code/src/thesis_aurian/msg/refdoors.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/thesis_aurian
)

### Generating Services

### Generating Module File
_generate_module_py(thesis_aurian
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/thesis_aurian
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(thesis_aurian_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(thesis_aurian_generate_messages thesis_aurian_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/laboinmastudent/Bureau/Drone_thesis_2017/Code/src/thesis_aurian/msg/refdoors.msg" NAME_WE)
add_dependencies(thesis_aurian_generate_messages_py _thesis_aurian_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(thesis_aurian_genpy)
add_dependencies(thesis_aurian_genpy thesis_aurian_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS thesis_aurian_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/thesis_aurian)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/thesis_aurian
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET ardrone_autonomy_generate_messages_cpp)
  add_dependencies(thesis_aurian_generate_messages_cpp ardrone_autonomy_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(thesis_aurian_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(thesis_aurian_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/thesis_aurian)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/thesis_aurian
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET ardrone_autonomy_generate_messages_lisp)
  add_dependencies(thesis_aurian_generate_messages_lisp ardrone_autonomy_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(thesis_aurian_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(thesis_aurian_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/thesis_aurian)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/thesis_aurian\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/thesis_aurian
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET ardrone_autonomy_generate_messages_py)
  add_dependencies(thesis_aurian_generate_messages_py ardrone_autonomy_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(thesis_aurian_generate_messages_py geometry_msgs_generate_messages_py)
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(thesis_aurian_generate_messages_py std_msgs_generate_messages_py)
endif()
