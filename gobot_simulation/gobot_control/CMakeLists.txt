cmake_minimum_required(VERSION 2.8.3)
project(gobot_control)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    message_generation
    gobot_base
    gazebo_plugins
    gazebo_ros
)

add_service_files(
  FILES
  SetBattery.srv
)

find_package (gazebo REQUIRED)

generate_messages(
    DEPENDENCIES
    std_msgs
)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS roscpp rospy std_msgs gobot_base gazebo_plugins gazebo_ros
)

link_directories(${GAZEBO_LIBRARY_DIRS})

include_directories(
    include 
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIR} 
    ${GAZEBO_INCLUDE_DIRS}
)


add_executable(ir_controller src/ir_controller.cpp)
target_link_libraries(ir_controller ${catkin_LIBRARIES} ${gobot_control})
add_dependencies(ir_controller gobot_control_generate_messages_cpp)

add_executable(bumper_controller src/bumper_controller.cpp)
target_link_libraries(bumper_controller ${catkin_LIBRARIES} ${gobot_control})
add_dependencies(bumper_controller gobot_control_generate_messages_cpp)

add_executable(proximity_controller src/proximity_controller.cpp)
target_link_libraries(proximity_controller ${catkin_LIBRARIES} ${gobot_control})
add_dependencies(proximity_controller gobot_control_generate_messages_cpp)

add_executable(battery_controller src/battery_controller.cpp)
target_link_libraries(battery_controller ${catkin_LIBRARIES} ${gobot_control})
add_dependencies(battery_controller gobot_control_generate_messages_cpp)

add_executable(sonar_controller src/sonar_controller.cpp)
target_link_libraries(sonar_controller ${catkin_LIBRARIES} ${gobot_control})
add_dependencies(sonar_controller gobot_control_generate_messages_cpp)

add_library(gobot_diff_drive_plugin src/gobot_diff_drive_plugin.cpp)
target_link_libraries(gobot_diff_drive_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${gobot_control})
add_dependencies(gobot_diff_drive_plugin gobot_control_generate_messages_cpp)