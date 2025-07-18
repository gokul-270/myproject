cmake_minimum_required(VERSION 2.6)
project(pattern_finder)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(catkin REQUIRED COMPONENTS
	rospy
	roscpp
	message_generation
)
find_package(OpenCV 4.0 REQUIRED)
find_package(PCL 1.2 REQUIRED)

catkin_package(
)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(project_aruco_finder_srcs src/aruco_finder.cpp)

include_directories(${PCL_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
	

add_executable(aruco_finder ${project_aruco_finder_srcs})
target_link_libraries(aruco_finder ${OpenCV_LIBRARIES}  ${PCL_LIBRARIES}
	${YANTHRA_XC_UTIL_LIBS} udev
)
if (CMAKE_CROSSCOMPILING)
  link_directories(${pcl_stage}/usr/lib)
  target_link_libraries(aruco_finder pcl_io_ply)
endif()

install(TARGETS
	aruco_finder
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS
	aruco_finder
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
