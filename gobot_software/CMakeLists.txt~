cmake_minimum_required(VERSION 2.8.3)
project(gobot_software)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  nav_msgs
  std_msgs
  actionlib
  actionlib_msgs
  message_generation
)

add_service_files(
  FILES
  Port.srv
  PortLaser.srv
  SendMessageToPc.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs  # and/or other packages containing depended messages
)

#set(SFML_STATIC_LIBRARIES TRUE)
# define the SFML_STATIC macro if static build was chosen
#if(SFML_STATIC_LIBRARIES)
#    add_definitions(-DSFML_STATIC)
#endif()

#set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "~/catkin_ws/src/sfml/build/SFML-2.3.2/share/SFML/cmake/Modules")
#find_package(SFML COMPONENTS network system)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy std_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(include ${catkin_INCLUDE_DIRS})

#add_executable(map_transfer src/map_transfer.cpp)
#target_link_libraries(map_transfer ${catkin_LIBRARIES})
#target_link_libraries(map_transfer ${SFML_LIBRARIES})
#add_dependencies(map_transfer gobot_communication_gencpp)

add_executable(map_transfer src/map_transfer.cpp)
target_link_libraries(map_transfer ${catkin_LIBRARIES})
target_link_libraries(map_transfer ${gobot_software} ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY} )
add_dependencies(map_transfer gobot_software_generate_messages_cpp)


add_executable(robot_pos_transfer src/robot_pos_transfer.cpp)
target_link_libraries(robot_pos_transfer ${catkin_LIBRARIES})
target_link_libraries(robot_pos_transfer ${gobot_software} ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY} )
add_dependencies(robot_pos_transfer gobot_software_generate_messages_cpp)


add_executable(map_metadata_transfer src/map_metadata_transfer.cpp)
target_link_libraries(map_metadata_transfer ${catkin_LIBRARIES})
target_link_libraries(map_metadata_transfer ${gobot_software} ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY} )
add_dependencies(map_metadata_transfer gobot_software_generate_messages_cpp)


add_executable(command_system src/command_system.cpp)
target_link_libraries(command_system ${catkin_LIBRARIES})
target_link_libraries(command_system ${gobot_software} ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY} )
add_dependencies(command_system gobot_software_generate_messages_cpp)


add_executable(read_new_map src/read_new_map.cpp)
target_link_libraries(read_new_map ${catkin_LIBRARIES})
target_link_libraries(read_new_map ${gobot_software} ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY} )
add_dependencies(read_new_map gobot_software_generate_messages_cpp)

add_executable(play_path src/playPath.cpp)
target_link_libraries(play_path ${catkin_LIBRARIES})
target_link_libraries(play_path ${gobot_software} ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY} )
add_dependencies(play_path gobot_software_generate_messages_cpp)

add_executable(send_laser_data src/laser.cpp)
target_link_libraries(send_laser_data ${catkin_LIBRARIES})
target_link_libraries(send_laser_data ${gobot_software} ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY} )
add_dependencies(send_laser_data gobot_software_generate_messages_cpp)

add_executable(play_path_new src/play_path_new.cpp)
target_link_libraries(play_path_new ${catkin_LIBRARIES})
target_link_libraries(play_path_new ${gobot_software} ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY} )
add_dependencies(play_path_new gobot_software_generate_messages_cpp)
