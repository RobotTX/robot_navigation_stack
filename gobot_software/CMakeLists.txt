cmake_minimum_required(VERSION 2.8.3)
project(gobot_software)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    roscpp
    tf
    actionlib
    hector_exploration_node
    geometry_msgs
    gobot_msg_srv
    actionlib_msgs
    nav_msgs
    move_base_msgs
    sensor_msgs
    std_msgs
    std_srvs
)

#add_service_files(
    #FILES
#)

#generate_messages(
#    DEPENDENCIES
#    std_msgs  # and/or other packages containing depended messages
#)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS 
    roscpp
    tf
    actionlib
    hector_exploration_node
    geometry_msgs
    gobot_msg_srv
    actionlib_msgs
    nav_msgs
    move_base_msgs
    sensor_msgs
    std_msgs
    std_srvs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
    include 
    ${catkin_INCLUDE_DIRS}
)

add_executable(map_transfer src/map_transfer.cpp)
target_link_libraries(map_transfer ${catkin_LIBRARIES} ${gobot_software} ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY} )
add_dependencies(map_transfer ${${gobot_msg_srv}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


add_executable(robot_pos_transfer src/robot_pos_transfer.cpp)
target_link_libraries(robot_pos_transfer ${catkin_LIBRARIES} ${gobot_software} ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY} )
add_dependencies(robot_pos_transfer ${${gobot_msg_srv}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


add_executable(read_new_map src/read_new_map.cpp)
target_link_libraries(read_new_map ${catkin_LIBRARIES} ${gobot_software} ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY} )
add_dependencies(read_new_map ${${gobot_msg_srv}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


add_executable(teleop src/teleop.cpp)
target_link_libraries(teleop ${catkin_LIBRARIES} ${gobot_software} ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY} )
add_dependencies(teleop ${${gobot_msg_srv}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


add_executable(play_path src/playPath.cpp)
target_link_libraries(play_path ${catkin_LIBRARIES} ${gobot_software} ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY} )
add_dependencies(play_path ${${gobot_msg_srv}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


add_executable(send_laser_data src/laser.cpp)
target_link_libraries(send_laser_data ${catkin_LIBRARIES} ${gobot_software} ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY} )
add_dependencies(send_laser_data ${${gobot_msg_srv}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


add_executable(recover_position src/recover_position.cpp)
target_link_libraries(recover_position ${catkin_LIBRARIES} ${gobot_software} ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY} )
add_dependencies(recover_position ${${gobot_msg_srv}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


add_executable(auto_docking src/auto_docking.cpp)
target_link_libraries(auto_docking ${catkin_LIBRARIES} ${gobot_software} ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY} )
add_dependencies(auto_docking ${${gobot_msg_srv}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


add_executable(battery_check src/battery_check.cpp)
target_link_libraries(battery_check ${catkin_LIBRARIES} ${gobot_software} )
add_dependencies(battery_check ${${gobot_msg_srv}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


add_executable(ping_server_new src/ping_server_new.cpp)
target_link_libraries(ping_server_new ${catkin_LIBRARIES} ${gobot_software} ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY} )
add_dependencies(ping_server_new ${${gobot_msg_srv}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


add_executable(command_system_new src/command_system_new.cpp)
target_link_libraries(command_system_new ${catkin_LIBRARIES} ${gobot_software} ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY} )
add_dependencies(command_system_new ${${gobot_msg_srv}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


#add_library(ThreadPool include/gobot_software/ThreadPool.h)