cmake_minimum_required(VERSION 2.8.3)
project(gobot_function)
## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    gobot_msg_srv
    hector_nav_msgs
    cv_bridge
    image_transport
)

find_package(OpenCV REQUIRED)
catkin_package(
CATKIN_DEPENDS 
    cv_bridge 
    image_transport  
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
        cv_bridge
        image_transport
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


add_executable(move_function src/move_function.cpp)
target_link_libraries(move_function ${catkin_LIBRARIES})
add_dependencies(move_function ${${gobot_msg_srv}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(docking_function src/docking_function.cpp)
target_link_libraries(docking_function ${catkin_LIBRARIES})
add_dependencies(docking_function ${${gobot_msg_srv}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(battery_monitor_function src/battery_monitor_function.cpp)
target_link_libraries(battery_monitor_function ${catkin_LIBRARIES})
add_dependencies(battery_monitor_function ${${gobot_msg_srv}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(scan_function src/scan_function.cpp)
target_link_libraries(scan_function ${catkin_LIBRARIES})
add_dependencies(scan_function ${${gobot_msg_srv}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(detection_function src/detection_function.cpp)
target_link_libraries(detection_function ${catkin_LIBRARIES})
add_dependencies(detection_function ${${gobot_msg_srv}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(shape_detection_node src/shape_detection_node.cpp)
target_link_libraries(shape_detection_node ${OpenCV_LIBS} ${catkin_LIBRARIES})
add_dependencies(shape_detection_node ${${gobot_msg_srv}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})