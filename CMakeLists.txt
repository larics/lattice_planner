cmake_minimum_required(VERSION 2.8.3)
project(lattice_planner)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
  tf2
  tf2_ros
  agv_control_msgs
)

#find_package(
#  OpenCV REQUIRED COMPONENTS flann # for matrix.h
#)
find_package(PkgConfig REQUIRED)
pkg_search_module(MATIO REQUIRED matio)
# HDF5 is required by MATIO
find_package(HDF5 REQUIRED)
if(NOT "${MATIO_LIBRARIES}" STREQUAL "")
  list(REMOVE_ITEM MATIO_LIBRARIES "hdf5")
endif()

###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs tf tf2 tf2_ros agv_control_msgs
)


###########
## Build ##
###########

include_directories(${catkin_INCLUDE_DIRS} ${MATIO_INCLUDE_DIRS})

## Declare a cpp executable
add_executable(pathPlanner src/pathPlanner.cpp src/graphSearch.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(pathPlanner ${catkin_LIBRARIES} ${MATIO_LIBRARIES} ${HDF5_LIBRARIES})

add_dependencies(pathPlanner agv_control_msgs_generate_messages_cpp)



#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS lattice_planner lattice_planner_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_lattice_planner.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
