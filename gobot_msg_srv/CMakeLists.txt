cmake_minimum_required(VERSION 2.8.3)
project(gobot_msg_srv)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    message_generation
    actionlib
    actionlib_msgs

)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


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
add_message_files(
    FILES
    BatteryMsg.msg
    BumperMsg.msg
    CliffMsg.msg
    IrMsg.msg
    OdomTestMsg.msg
    ProximityMsg.msg
    SonarMsg.msg
    WeightMsg.msg
    GyroMsg.msg
)

## Generate services in the 'srv' folder
add_service_files(
    FILES
    GetEncoders.srv
    InitBack.srv
    IsCharging.srv
    Port.srv
    PortLaser.srv
    RecoveredPosition.srv
    SendMessageToPc.srv
    SetSpeeds.srv
    LedStrip.srv
    SetBattery.srv
    SendMap.srv
    GetGobotStatus.srv
    SetGobotStatus.srv
    GetString.srv
    SetString.srv
    GetInt.srv
    SetInt.srv
    GetStringArray.srv
    SetStringArray.srv
    GetIntArray.srv
    SetIntArray.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
)

catkin_package(
  INCLUDE_DIRS include
    LIBRARIES
        robot_class
#  LIBRARIES gobot_msg_srv
    CATKIN_DEPENDS 
        message_runtime
        actionlib
        actionlib_msgs
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

add_library(robot_class
                src/get_robot_class.cpp
                src/set_robot_class.cpp
                src/robot_command.cpp)
add_dependencies(robot_class ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(robot_class ${catkin_LIBRARIES} ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY} )


install(TARGETS
            robot_class
       ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       )