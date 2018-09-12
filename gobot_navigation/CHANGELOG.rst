^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for Developed Mobile Robot - MOBOT
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.2.5 (2018-08-16)
-------------------
* last date of Tx

1.2.4 (2018-08-15)
-------------------
* revise teleop node
* teleop speed follow the navigation speed set by user

1.2.3 (2018-08-01)
-------------------
* add rough alignment feature for object tracking
* add drawings for sensor layout and equation for movement along object frame axis in sensor, detection and move nodes

1.2.2 (2018-07-27)
-------------------
* complete object tracking feature

1.2.2 (2018-07-26)
-------------------
* add ethernet support for wireless bridge with IP 192.168.188.[1-100]. If wireless bridge IP domain name changes, ping wifi script need to be changed accordingly

1.2.1 (2018-07-25)
-------------------
* object tracking done
* add object tracking codes into gobot_function package
* add object tracking function. Going to combine it with UI side in the next version
* all yaw conversions between UI frame and ROBOT frame are executed in command_system node

1.2.0 (2018-07-20)
-------------------
* complete audio (mp3/wav) files transfer feature
* complete set sound volume feature because of different audio files have different strength for same volume

1.1.7 (2018-07-17)
-------------------
* add pre-load .mp3 files for auto-scanning, complete scanning
* still working on object tracking based on image
* delete duplicate open source packages such as move_base

1.1.6 (2018-07-16)
-------------------
* #permission for .cfg file
  sudo find ~/catkin_ws/src/robot_navigation_stack/ -name "*.cfg" -exec chmod +x {} \;
* add robot_move_class class for robot move functions
* move robot functions to new package gobot_function
* standardlize gobot_software nodes name
* change "ls01B" package name to "ls01b" to avoid catkin_make warning

1.1.5 (2018-07-12)
-------------------
* restart network-manager when assigned wifi can not be found
* robot will resume to work if low battery auto docking is triggered

1.1.4 (2018-07-11)
-------------------
* add pre-load .mp3 files for auto-docking, complete auto-docking, low battery warning, reload map, abort navigation operations
* add service to control electro-magnet on/off via stm32
* add topic for magnet connection status by reading it from stm32

1.1.3 (2018-07-06)
-------------------
* add feature of playing pre-load .mp3 files for startup, scan, poweroff operations

1.1.2 (2018-06-24)
-------------------
* add robot mode (auto/manual) into the feedback message to UI side
* fix gazebo scanning mode manual control issue

1.1.1 (2018-06-21)
-------------------
* add LeiShen ls01b lidar driver, now support this 28m range 2D lidar
* working on robot_localization package

1.1.0 (2018-06-13)
-------------------
* add yocs_velocity_smoother package to smooth raw cmd_vel
* try to integrate gyro data into system by using extend Kalman Filter

1.0.4 (2018-05-24)
-------------------
* revised robot behavior after completing auto docking
  If robot start scanning from docking station, it will go auto docking; otherwise, it will stop where auto scanning ends

1.0.3 (2018-05-23)
-------------------
* add support to ethernet connection for robot and UI communication. Now robot can directly connect to PC via ethernet
  It is required to set robot Ethernet connection to be "Wired Ethernet" with static IP:1.2.5.1
  It is required to set PC Ethernet connection to be static IP:1.2.5.2

1.0.2 (2018-05-16)
-------------------
* add range_sensor_layer for sonar sensors
* add publisher for viewing all sensors information

1.0.1 (2018-01-01)
-------------------
* new product based on new design

1.0.0 (2017-09-11)
-------------------
* prototyping
