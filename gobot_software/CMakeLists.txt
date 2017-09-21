cmake_minimum_required(VERSION 2.8.3)
project(gobot_software)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    tf
    nav_msgs
    std_msgs
    actionlib
    actionlib_msgs
    message_generation
    gobot_base
)

add_service_files(
    FILES
    Port.srv
    PortLaser.srv
    SendMessageToPc.srv
    RecoveredPosition.srv
    SetDockStatus.srv
    GetDockStatus.srv
)

generate_messages(
    DEPENDENCIES
    std_msgs  # and/or other packages containing depended messages
)

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
    CATKIN_DEPENDS roscpp rospy std_msgs tf message_runtime
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
add_dependencies(map_transfer gobot_software_generate_messages_cpp)

add_executable(robot_pos_transfer src/robot_pos_transfer.cpp)
target_link_libraries(robot_pos_transfer ${catkin_LIBRARIES} ${gobot_software} ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY} )
add_dependencies(robot_pos_transfer gobot_software_generate_messages_cpp)

add_executable(command_system src/command_system.cpp)
target_link_libraries(command_system ${catkin_LIBRARIES} ${gobot_software} ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY} )
add_dependencies(command_system gobot_software_generate_messages_cpp)

add_executable(read_new_map src/read_new_map.cpp)
target_link_libraries(read_new_map ${catkin_LIBRARIES} ${gobot_software} ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY} )
add_dependencies(read_new_map gobot_software_generate_messages_cpp)

add_executable(teleop src/teleop.cpp)
target_link_libraries(teleop ${catkin_LIBRARIES} ${gobot_software} ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY} )
add_dependencies(teleop gobot_software_generate_messages_cpp)

add_executable(play_path src/playPath.cpp)
target_link_libraries(play_path ${catkin_LIBRARIES} ${gobot_software} ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY} )
add_dependencies(play_path gobot_software_generate_messages_cpp)

add_executable(send_laser_data src/laser.cpp)
target_link_libraries(send_laser_data ${catkin_LIBRARIES} ${gobot_software} ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY} )
add_dependencies(send_laser_data gobot_software_generate_messages_cpp)

add_library(cluster_optimized src/cluster_optimized.cpp)
target_link_libraries(cluster_optimized ${catkin_LIBRARIES})

add_executable(isLocalized_optimized src/isLocalized_optimized.cpp)
target_link_libraries(isLocalized_optimized ${catkin_LIBRARIES} ${gobot_software} cluster_optimized)
add_dependencies(isLocalized_optimized gobot_software_generate_messages_cpp)

add_executable(recover_position src/recover_position.cpp)
target_link_libraries(recover_position ${catkin_LIBRARIES} ${gobot_software} ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY} )
add_dependencies(recover_position gobot_software_generate_messages_cpp)

add_executable(ping_server src/ping_server.cpp)
target_link_libraries(ping_server ${catkin_LIBRARIES} ${gobot_software} ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY} )
add_dependencies(ping_server gobot_software_generate_messages_cpp)

add_executable(backup_server src/backup_server.cpp)
target_link_libraries(backup_server ${catkin_LIBRARIES} ${gobot_software} ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY} )
add_dependencies(backup_server gobot_software_generate_messages_cpp)

add_executable(auto_docking src/auto_docking.cpp)
target_link_libraries(auto_docking ${catkin_LIBRARIES} ${gobot_software} ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY} )
add_dependencies(auto_docking gobot_software_generate_messages_cpp)

#add_executable(initialPosePublisher src/initialPosePublisher.cpp)
#target_link_libraries(initialPosePublisher ${catkin_LIBRARIES} ${gobot_software} )
#add_dependencies(initialPosePublisher gobot_software_generate_messages_cpp)

