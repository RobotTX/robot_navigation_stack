cmake_minimum_required(VERSION 2.8.3)
project(gobot_software)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    hector_exploration_node
    gobot_msg_srv
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

add_library(robot_move src/robot_move.cpp)
target_link_libraries(robot_move ${catkin_LIBRARIES} ${gobot_msg_srv} ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY})


add_executable(auto_docking src/auto_docking.cpp)
target_link_libraries(auto_docking ${catkin_LIBRARIES} ${gobot_software} ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY})
add_dependencies(auto_docking ${${gobot_msg_srv}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(detection_alignment src/detection_alignment.cpp)
target_link_libraries(detection_alignment ${catkin_LIBRARIES} ${gobot_software} ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY})
add_dependencies(detection_alignment ${${gobot_msg_srv}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(battery_monitor src/battery_monitor.cpp)
target_link_libraries(battery_monitor ${catkin_LIBRARIES} ${gobot_software} )
add_dependencies(battery_monitor ${${gobot_msg_srv}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(gobot_status src/gobot_status.cpp)
target_link_libraries(gobot_status ${catkin_LIBRARIES} ${gobot_software} ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY} )
add_dependencies(gobot_status ${${gobot_msg_srv}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(move_it src/move_it.cpp)
target_link_libraries(move_it ${catkin_LIBRARIES} ${gobot_software} ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY} )
add_dependencies(move_it ${${gobot_msg_srv}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(scan_controller src/scan_controller.cpp)
target_link_libraries(scan_controller ${catkin_LIBRARIES} ${hector_exploration_node})
add_dependencies(scan_controller ${${gobot_msg_srv}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(tcp_transfer_map src/tcp_transfer_map.cpp)
target_link_libraries(tcp_transfer_map ${catkin_LIBRARIES} ${gobot_software} ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY} )
add_dependencies(tcp_transfer_map ${${gobot_msg_srv}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(tcp_robot_pose src/tcp_robot_pose.cpp)
target_link_libraries(tcp_robot_pose ${catkin_LIBRARIES} ${gobot_software} ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY} )
add_dependencies(tcp_robot_pose ${${gobot_msg_srv}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(tcp_read_map src/tcp_read_map.cpp)
target_link_libraries(tcp_read_map ${catkin_LIBRARIES} ${gobot_software} ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY} )
add_dependencies(tcp_read_map ${${gobot_msg_srv}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(tcp_read_audio src/tcp_read_audio.cpp)
target_link_libraries(tcp_read_audio ${catkin_LIBRARIES} ${gobot_software} ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY} )
add_dependencies(tcp_read_audio ${${gobot_msg_srv}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(tcp_teleop_control src/tcp_teleop_control.cpp)
target_link_libraries(tcp_teleop_control ${catkin_LIBRARIES} ${gobot_software} ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY} )
add_dependencies(tcp_teleop_control ${${gobot_msg_srv}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(tcp_transfer_laser src/tcp_transfer_laser.cpp)
target_link_libraries(tcp_transfer_laser ${catkin_LIBRARIES} ${gobot_software} ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY} )
add_dependencies(tcp_transfer_laser ${${gobot_msg_srv}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(tcp_ping_server_node src/tcp_ping_server_node.cpp)
target_link_libraries(tcp_ping_server_node ${catkin_LIBRARIES} ${gobot_software} ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY} )
add_dependencies(tcp_ping_server_node ${${gobot_msg_srv}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(tcp_check_servers src/tcp_check_servers.cpp)
target_link_libraries(tcp_check_servers ${catkin_LIBRARIES} ${gobot_software} ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY} )
add_dependencies(tcp_check_servers ${${gobot_msg_srv}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(tcp_command_system src/tcp_command_system.cpp)
target_link_libraries(tcp_command_system ${catkin_LIBRARIES} ${gobot_software} ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY} )
add_dependencies(tcp_command_system ${${gobot_msg_srv}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#add_library(ThreadPool include/gobot_software/ThreadPool.h)
