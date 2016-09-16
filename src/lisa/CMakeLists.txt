cmake_minimum_required(VERSION 2.8.3)
project(lisa)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy

  angles
  geometry_msgs
  message_generation
  sensor_msgs
  std_msgs
  nav_msgs
  tf
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
#add_message_files(
#  FILES
#)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

# Generate added messages and services with any dependencies listed here
#generate_messages(
#  DEPENDENCIES
#  std_msgs
#)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES lisa
#  CATKIN_DEPENDS roscpp rospy std_msgs
  CATKIN_DEPENDS
    roscpp
    rospy

    geometry_msgs
    message_runtime
    std_msgs
    nav_msgs
    tf
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
# add_library(lisa
#   src/${PROJECT_NAME}/lisa.cpp
# )

## Declare a cpp executable
# add_executable(lisa_node src/lisa_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(lisa_node lisa_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(lisa_node
#   ${catkin_LIBRARIES}
# )

## Localizer Node
add_executable(localizer src/localizer.cpp)
target_link_libraries(localizer ${catkin_LIBRARIES})
add_dependencies(localizer ${catkin_EXPORTED_TARGETS})

## Navigator Node
add_executable(navigator src/navigator.cpp)
target_link_libraries(navigator ${catkin_LIBRARIES})
add_dependencies(navigator ${catkin_EXPORTED_TARGETS})

## Waypoint Manager Node
add_executable(waypoint_manager src/waypoint_manager.cpp)
target_link_libraries(waypoint_manager ${catkin_LIBRARIES})
add_dependencies(waypoint_manager ${catkin_EXPORTED_TARGETS})

## Teleoperator (Keyboard) Node
add_executable(teleop_key src/teleop_key.cpp)
target_link_libraries(teleop_key ${catkin_LIBRARIES})
add_dependencies(teleop_key ${catkin_EXPORTED_TARGETS})

## Simulator Nodes
add_executable(imu_simulator src/imu_simulator.cpp)
target_link_libraries(imu_simulator ${catkin_LIBRARIES})
add_dependencies(imu_simulator ${catkin_EXPORTED_TARGETS})

add_executable(wheel_encoder_simulator src/wheel_encoder_simulator.cpp)
target_link_libraries(wheel_encoder_simulator ${catkin_LIBRARIES})
add_dependencies(wheel_encoder_simulator ${catkin_EXPORTED_TARGETS})

## Occupancy Map Node
add_executable(occupancy_map src/occupancy_map.cpp)
target_link_libraries(occupancy_map ${catkin_LIBRARIES})
add_dependencies(occupancy_map ${catkin_EXPORTED_TARGETS})

#add_executable(listener src/listener.cpp)
#target_link_libraries(listener ${catkin_LIBRARIES})
#add_dependencies(listener beginner_tutorials_generate_messages_cpp)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
install(PROGRAMS
  wheel_encoder
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark executables and/or libraries for installation
# install(TARGETS lisa lisa_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_lisa.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
