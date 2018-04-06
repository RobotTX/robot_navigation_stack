cmake_minimum_required(VERSION 2.8.3)
project(gobot_software)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    roscpp
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

add_executable(auto_docking src/auto_docking.cpp)
target_link_libraries(auto_docking ${catkin_LIBRARIES} ${gobot_software} ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY})
add_dependencies(auto_docking ${${gobot_msg_srv}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


add_executable(gobot_status src/gobot_status.cpp)
target_link_libraries(gobot_status ${catkin_LIBRARIES} ${gobot_software} ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY} )
add_dependencies(gobot_status ${${gobot_msg_srv}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


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

add_executable(move_it src/move_it.cpp)
target_link_libraries(move_it ${catkin_LIBRARIES} ${gobot_software} ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY} )
add_dependencies(move_it ${${gobot_msg_srv}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(send_laser_data src/laser.cpp)
target_link_libraries(send_laser_data ${catkin_LIBRARIES} ${gobot_software} ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY} )
add_dependencies(send_laser_data ${${gobot_msg_srv}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


add_executable(battery_check src/battery_check.cpp)
target_link_libraries(battery_check ${catkin_LIBRARIES} ${gobot_software} )
add_dependencies(battery_check ${${gobot_msg_srv}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


add_executable(ping_servers src/ping_servers.cpp)
target_link_libraries(ping_servers ${catkin_LIBRARIES} ${gobot_software} ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY} )
add_dependencies(ping_servers ${${gobot_msg_srv}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(check_servers src/check_servers.cpp)
target_link_libraries(check_servers ${catkin_LIBRARIES} ${gobot_software} ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY} )
add_dependencies(check_servers ${${gobot_msg_srv}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(command_system src/command_system.cpp)
target_link_libraries(command_system ${catkin_LIBRARIES} ${gobot_software} ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY} )
add_dependencies(command_system ${${gobot_msg_srv}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#add_library(ThreadPool include/gobot_software/ThreadPool.h)