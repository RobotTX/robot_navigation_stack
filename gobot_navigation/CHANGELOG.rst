^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for developed mobile robot
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.1 (2018-06-21)
-------------------
* add LeiShen ls01B lidar driver, now support this 28m range 2D lidar
* work on robot_localization package

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
