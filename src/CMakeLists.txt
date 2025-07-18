# toplevel CMakeLists.txt for a catkin workspace
# catkin/cmake/toplevel.cmake

cmake_minimum_required(VERSION 3.0.2)

project(Project)

set(CATKIN_TOPLEVEL TRUE)

# search for catkin within the workspace
set(_cmd "catkin_find_pkg" "catkin" "${CMAKE_SOURCE_DIR}")
execute_process(COMMAND ${_cmd}
  RESULT_VARIABLE _res
  OUTPUT_VARIABLE _out
  ERROR_VARIABLE _err
  OUTPUT_STRIP_TRAILING_WHITESPACE
  ERROR_STRIP_TRAILING_WHITESPACE
)
if(NOT _res EQUAL 0 AND NOT _res EQUAL 2)
  # searching fot catkin resulted in an error
  string(REPLACE ";" " " _cmd_str "${_cmd}")
  message(FATAL_ERROR "Search for 'catkin' in workspace failed (${_cmd_str}): ${_err}")
endif()

# include catkin from workspace or via find_package()
if(_res EQUAL 0)
  set(catkin_EXTRAS_DIR "${CMAKE_SOURCE_DIR}/${_out}/cmake")
  # include all.cmake without add_subdirectory to let it operate in same scope
  include(${catkin_EXTRAS_DIR}/all.cmake NO_POLICY_SCOPE)
  add_subdirectory("${_out}")

else()
  # use either CMAKE_PREFIX_PATH explicitly passed to CMake as a command line argument
  # or CMAKE_PREFIX_PATH from the environment
  if(NOT DEFINED CMAKE_PREFIX_PATH)
    if(NOT "$ENV{CMAKE_PREFIX_PATH}" STREQUAL "")
      if(NOT WIN32)
        string(REPLACE ":" ";" CMAKE_PREFIX_PATH $ENV{CMAKE_PREFIX_PATH})
      else()
        set(CMAKE_PREFIX_PATH $ENV{CMAKE_PREFIX_PATH})
      endif()
    endif()
  endif()

  # list of catkin workspaces
  set(catkin_search_path "")
  foreach(path ${CMAKE_PREFIX_PATH})
    if(EXISTS "${path}/.catkin")
      list(FIND catkin_search_path ${path} _index)
      if(_index EQUAL -1)
        list(APPEND catkin_search_path ${path})
      endif()
    endif()
  endforeach()

  # search for catkin in all workspaces
  set(CATKIN_TOPLEVEL_FIND_PACKAGE TRUE)
  find_package(catkin QUIET
    NO_POLICY_SCOPE
    PATHS ${catkin_search_path}
    NO_DEFAULT_PATH NO_CMAKE_FIND_ROOT_PATH)
  unset(CATKIN_TOPLEVEL_FIND_PACKAGE)

  if(NOT catkin_FOUND)
    message(FATAL_ERROR "find_package(catkin) failed. catkin was neither found in the workspace nor in the CMAKE_PREFIX_PATH. One reason may be that no ROS setup.sh was sourced before.")
  endif()
endif()


set(YANTHRA_INSTALL_DIR        ${CMAKE_INSTALL_PREFIX})
set(COTTON_DETECT_INSTALL_DIR  ${YANTHRA_INSTALL_DIR}/lib/cotton_detection)
set(COTTON_DETECT_PROGRAM      ${COTTON_DETECT_INSTALL_DIR}/cotton_detect_ml)
set(RS_CAPTURE_S640_PROGRAM    ${COTTON_DETECT_INSTALL_DIR}/rs-capture_single640)
set(PATTERN_FINDER_INSTALL_DIR ${YANTHRA_INSTALL_DIR}/lib/pattern_finder)
set(ARUCO_FINDER_PROGRAM        /home/ubuntu/pragati/src/OakDTools/ArucoDetectYanthra.py) 
#set(ARUCO_FINDER_PROGRAM       ${PATTERN_FINDER_INSTALL_DIR}/aruco_finder)
set(YANTHRA_DATA_DIR           /home/ubuntu)
set(CALIBRATION_MODE_DATA_DIR  ${YANTHRA_DATA_DIR}/calibration_mode)
set(TEST140_MODE_DATA_DIR      ${YANTHRA_DATA_DIR}/test140_mode)
set(YANTHRA_DATA_DEBUG_DIR     ${YANTHRA_DATA_DIR}/debug)
set(YANTHRA_DATA_INPUT_DIR     ${YANTHRA_DATA_DIR}/pragati/inputs)
set(YANTHRA_DATA_OUTPUT_DIR    ${YANTHRA_DATA_DIR}/pragati/outputs)

add_compile_options(
  -DARUCO_FINDER_PROGRAM="${ARUCO_FINDER_PROGRAM}"
  -DCALIBRATION_MODE_DATA_DIR="${CALIBRATION_MODE_DATA_DIR}"
  -DCOTTON_DETECT_PROGRAM="${COTTON_DETECT_PROGRAM}"
  -DRS_CAPTURE_S640_PROGRAM="${RS_CAPTURE_S640_PROGRAM}"
  -DTEST140_MODE_DATA_DIR="${TEST140_MODE_DATA_DIR}"
  -DYANTHRA_DATA_DEBUG_DIR="${YANTHRA_DATA_DEBUG_DIR}"
  -DYANTHRA_DATA_DIR="${YANTHRA_DATA_DIR}"
  -DYANTHRA_DATA_INPUT_DIR="${YANTHRA_DATA_INPUT_DIR}"
  -DYANTHRA_DATA_OUTPUT_DIR="${YANTHRA_DATA_OUTPUT_DIR}"
  -DVERSION="${GIT_VERSION}"
)


catkin_workspace()


