cmake_minimum_required(VERSION 2.8.3)
project(torob)

# Activate C++11 :
SET(CMAKE_CXX_FLAGS "-std=c++11" )
# SET(CMAKE_CXX_FLAGS "-std=c++14 -Wall -Wextra" )

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  std_msgs
  message_generation
  nav_msgs
  geometry_msgs
  sensor_msgs
  visualization_msgs
  torob_msgs
  urg_node
  roslaunch
)

roslaunch_add_file_check(launch)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

# Boost :
find_package( Boost 1.58 COMPONENTS geometry thread graph random)
link_directories ( ${Boost_LIBRARY_DIRS} )
include_directories ( ${Boost_INCLUDE_DIRS} )

# Get SDL2 :
INCLUDE(FindPkgConfig)
PKG_SEARCH_MODULE(SDL2 REQUIRED sdl2)
INCLUDE_DIRECTORIES(${SDL2_INCLUDE_DIRS})

# project configuration :
include_directories(${PROJECT_SOURCE_DIR} ${PROJECT_SOURCE_DIR}/src)

# Link libraries :
link_libraries( ${Boost_LIBRARIES} )

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

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
# add_message_files(
#     FILES
#     Node.msg
#     Edge.msg
#     VectorMap.msg
# )

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

## Generate added messages and services with any dependencies listed here
# generate_messages(
#     DEPENDENCIES
#     std_msgs
# )

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
#  LIBRARIES torob
#  CATKIN_DEPENDS hokuyo roscpp std_msgs
#  CATKIN_DEPENDS message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories( include ${catkin_INCLUDE_DIRS} )

## Build talker and listener:

#add_executable( sandbox src/main_sandbox.cpp )
#target_link_libraries( sandbox ${catkin_LIBRARIES} )

## Build regular nodes:

add_executable( random_goal src/main_random_goal.cpp )
target_link_libraries( random_goal ${catkin_LIBRARIES} )

add_executable( vmap_move src/rosvmap.cpp src/ogmap.cpp src/visimap.cpp src/impact.cpp src/float2.cpp src/data.cpp src/main_vmap_move.cpp )
target_link_libraries( vmap_move ${Boost_LIBRARIES} ${catkin_LIBRARIES} yaml-cpp)

add_executable( ring_move src/float2.cpp src/impact.cpp src/ring.cpp src/main_ring_move.cpp )
target_link_libraries( ring_move ${Boost_LIBRARIES} ${catkin_LIBRARIES} )

add_executable( merge src/rosvmap.cpp src/ogmap.cpp src/visimap.cpp src/impact.cpp src/float2.cpp src/data.cpp src/main_merge.cpp )
target_link_libraries( merge ${Boost_LIBRARIES} ${catkin_LIBRARIES} yaml-cpp)

add_executable( scan_to_vmap src/rosvmap.cpp src/ogmap.cpp src/visimap.cpp src/impact.cpp src/float2.cpp src/data.cpp src/main_scan_to_vmap.cpp )
target_link_libraries( scan_to_vmap ${Boost_LIBRARIES} ${catkin_LIBRARIES} yaml-cpp)

add_executable( net_send src/rosvmap.cpp src/ogmap.cpp src/visimap.cpp src/impact.cpp src/float2.cpp src/data.cpp src/main_net_send.cpp src/net.cpp )
target_link_libraries( net_send ${Boost_LIBRARIES} ${catkin_LIBRARIES} yaml-cpp)

add_executable( net_receive src/rosvmap.cpp src/ogmap.cpp src/visimap.cpp src/impact.cpp src/float2.cpp src/data.cpp src/main_net_receive.cpp src/net.cpp )
target_link_libraries( net_receive ${Boost_LIBRARIES} ${catkin_LIBRARIES} yaml-cpp)

add_executable( map_to_vmap src/rosvmap.cpp src/ogmap.cpp src/visimap.cpp src/impact.cpp src/float2.cpp src/data.cpp src/main_map_to_vmap.cpp )
target_link_libraries( map_to_vmap ${Boost_LIBRARIES} ${catkin_LIBRARIES} yaml-cpp)

add_executable( vmap_server src/rosvmap.cpp src/ogmap.cpp src/visimap.cpp src/impact.cpp src/float2.cpp src/data.cpp src/main_vmap_server.cpp )
target_link_libraries( vmap_server ${Boost_LIBRARIES} ${catkin_LIBRARIES} yaml-cpp)

#TODO: vmap_to_graph_vmap, vmap_saver

## Build SDL2 nodes:

add_executable( viewer src/rosvmap.cpp src/ring.cpp src/ogmap.cpp src/visimap.cpp src/impact.cpp src/float2.cpp src/impact.cpp src/data.cpp src/main_viewer.cpp src/roscontrol.cpp src/frame.cpp )
target_link_libraries( viewer ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${SDL2_LIBRARIES} yaml-cpp SDL2_gfx)


## Build depreciated nodes:

add_executable( vmap_to_map src/rosvmap.cpp src/ogmap.cpp src/visimap.cpp src/impact.cpp src/float2.cpp src/data.cpp src/map_publisher.cpp  src/main_vmap_to_map.cpp )
target_link_libraries( vmap_to_map ${Boost_LIBRARIES} ${catkin_LIBRARIES} yaml-cpp)


## Declare a cpp library
# add_library(torob
#   src/${PROJECT_NAME}/torob.cpp
# )

## Declare a cpp executable
# add_executable(torob_node src/torob_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(torob_node torob_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(torob_node
#   ${catkin_LIBRARIES}
# )

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
# install(TARGETS torob torob_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_torob.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
