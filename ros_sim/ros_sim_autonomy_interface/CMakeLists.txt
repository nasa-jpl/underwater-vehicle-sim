cmake_minimum_required(VERSION 2.8.3)
project(ros_sim_autonomy_interface)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
add_compile_options($<BUILD_INTERFACE:-Wall>)

add_definitions(-DEIGEN_DONT_VECTORIZE -DEIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
             actionlib_msgs
             roscpp
             tf2
             tf2_ros
             tf2_geometry_msgs
             underwater_vehicle_msgs
             propulsion_controller
             nav_msgs
             geometry_msgs)

find_package (Eigen3 3.3 REQUIRED NO_MODULE)

find_library(ALG_LIB alg_lib)
find_library(UNDERWATER_AUTONOMY_LIB underwater_autonomy)
find_path(UNDERWATER_AUTONOMY_INCLUDE underwater_autonomy)
find_library(VENT_PLANNER_LIB vent_behaviors)
find_path(VENT_PLANNER_INCLUDE vent_behaviors)
find_library(NAVIGATION_PLANNER_LIB navigation_behaviors)
find_path(NAVIGATION_PLANNER_INCLUDE navigation_behaviors)
find_library(EXPLORE_PLANNER_LIB explore_behaviors)
find_path(EXPLORE_PLANNER_INCLUDE explore_behaviors)

find_package (Eigen3 3.3 REQUIRED NO_MODULE)

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
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
#   FILES
#   Message1.msg
#   Message2.msg
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
#   DynamicLawnmowerRos.action
#   WaypointsRos.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   geometry_msgs
#   propulsion_controller
#   actionlib_msgs  # Or other packages containing msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
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
   INCLUDE_DIRS include
   LIBRARIES ros_sim_autonomy_interface
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
    src/ros_sim_plan_server/RosSimPlanServer.cpp
    src/ros_sim_autonomy_interface/ROSSimVehicleInterface.cpp
    src/ros_sim_plan_server/command_executors/WaypointsSimCommandExecutor.cpp
    src/ros_sim_plan_server/command_executors/FollowHeadingSimCommandExecutor.cpp
    src/ros_sim_plan_server/command_executors/HoldDepthSimCommandExecutor.cpp
    src/ros_sim_plan_server/command_executors/YoYoSimCommandExecutor.cpp
    src/ros_sim_plan_server/command_executors/CircleSimCommandExecutor.cpp
)
#target_include_directories(${PROJECT_NAME} PUBLIC /usr/local/include/bfl)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
 add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

 target_include_directories(ros_sim_autonomy_interface PUBLIC 
  ${NAVIGATION_PLANNER_INCLUDE} 
  ${VENT_PLANNER_INCLUDE} 
  ${EXPLORE_PLANNER_INCLUDE} 
  ${UNDERWATER_AUTONOMY_INCLUDE})
 target_link_libraries(ros_sim_autonomy_interface
   ${NAVIGATION_PLANNER_LIB}
   ${VENT_PLANNER_LIB}
   ${EXPLORE_PLANNER_LIB}
   ${UNDERWATER_AUTONOMY_LIB}
   ${catkin_LIBRARIES}
 )

## Declare a C++ executable
 add_executable(ros_sim_plan_server_node src/ROSSimPlanServerNode.cpp)
# set_target_properties(ros_sim_plan_server_node PROPERTIES OUTPUT_NAME node PREFIX "")
 add_dependencies(ros_sim_plan_server_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
 target_include_directories(ros_sim_plan_server_node PRIVATE 
  ${NAVIGATION_PLANNER_INCLUDE} 
  ${VENT_PLANNER_INCLUDE}
  ${EXPLORE_PLANNER_INCLUDE} 
  ${UNDERWATER_AUTONOMY_INCLUDE})
 target_link_libraries(ros_sim_plan_server_node
   ros_sim_autonomy_interface
   ${NAVIGATION_PLANNER_LIB}
   ${VENT_PLANNER_LIB}
   ${EXPLORE_PLANNER_LIB}
   ${UNDERWATER_AUTONOMY_LIB}
   ${ALG_LIB}
   ${catkin_LIBRARIES}
 )

  add_executable(ros_sim_true_navigation_node src/ros_sim_navigation/ROSSimTrueNavigationNode.cpp)
# set_target_properties(ros_sim_true_navigation_node PROPERTIES OUTPUT_NAME node PREFIX "")
 add_dependencies(ros_sim_true_navigation_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  target_include_directories(ros_sim_true_navigation_node PRIVATE ${EIGEN3_INCLUDE_DIR})
 target_link_libraries(ros_sim_true_navigation_node
   underwater_vehicle_msgs
   ${catkin_LIBRARIES}
 )

  add_executable(ros_sim_dr_navigation_node src/ros_sim_navigation/ROSSimDRNavigationNode.cpp)
# set_target_properties(ros_sim_dr_navigation_node PROPERTIES OUTPUT_NAME node PREFIX "")
 add_dependencies(ros_sim_dr_navigation_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  target_include_directories(ros_sim_dr_navigation_node PRIVATE  ${UNDERWATER_AUTONOMY_INCLUDE})
 target_link_libraries(ros_sim_dr_navigation_node
   ros_sim_autonomy_interface
   underwater_vehicle_msgs
   ${UNDERWATER_AUTONOMY_LIB}
   ${catkin_LIBRARIES}
 )

  add_executable(ros_sim_sb_navigation_node src/ros_sim_navigation/ROSSimSBNavigationNode.cpp)
# set_target_properties(ros_sim_sb_navigation_node PROPERTIES OUTPUT_NAME node PREFIX "")
 add_dependencies(ros_sim_sb_navigation_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  target_include_directories(ros_sim_sb_navigation_node PRIVATE  ${UNDERWATER_AUTONOMY_INCLUDE})
 target_link_libraries(ros_sim_sb_navigation_node
   ros_sim_autonomy_interface
   underwater_vehicle_msgs
   ${UNDERWATER_AUTONOMY_LIB}
   ${ALG_LIB}
   ${catkin_LIBRARIES}
 )



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
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
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
if (CATKIN_ENABLE_TESTING)
 find_package(rostest REQUIRED)

 add_rostest_gtest(follow_heading_sim_command_executor_test
                   test/ros_sim_plan_server/command_executors/FollowHeadingSimCommandExecutor.test
                   test/ros_sim_plan_server/command_executors/FollowHeadingSimCommandExecutor_test.cpp)
 target_link_libraries(follow_heading_sim_command_executor_test ros_sim_autonomy_interface ${catkin_LIBRARIES} )

 add_rostest_gtest(hold_depth_sim_command_executor_test
                   test/ros_sim_plan_server/command_executors/HoldDepthSimCommandExecutor.test
                   test/ros_sim_plan_server/command_executors/HoldDepthSimCommandExecutor_test.cpp)
 target_link_libraries(hold_depth_sim_command_executor_test ros_sim_autonomy_interface ${catkin_LIBRARIES} )

 add_rostest_gtest(yoyo_sim_command_executor_test
                   test/ros_sim_plan_server/command_executors/YoYoSimCommandExecutor.test
                   test/ros_sim_plan_server/command_executors/YoYoSimCommandExecutor_test.cpp)
 target_link_libraries(yoyo_sim_command_executor_test ros_sim_autonomy_interface ${catkin_LIBRARIES} )

 add_rostest_gtest(waypoints_sim_command_executor_test
                   test/ros_sim_plan_server/command_executors/WaypointsSimCommandExecutor.test
                   test/ros_sim_plan_server/command_executors/WaypointsSimCommandExecutor_test.cpp)
 target_link_libraries(waypoints_sim_command_executor_test ros_sim_autonomy_interface ${catkin_LIBRARIES} )

 add_rostest_gtest(ros_sim_vehicle_interface_test
                   test/ros_sim_autonomy_interface/ROSSimVehicleInterface.test
                   test/ros_sim_autonomy_interface/ROSSimVehicleInterface_test.cpp)
 target_link_libraries(ros_sim_vehicle_interface_test ros_sim_autonomy_interface ${catkin_LIBRARIES} )

endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
