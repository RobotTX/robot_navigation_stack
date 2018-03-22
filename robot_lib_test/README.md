# robot_library

### Overview
This package is for any user to integrate their higher level application into the current robot system.

This package includes:
  * class constructor/destructor/initializer
  * subscriber callback functions
  * some conversion functions
    * degree-radian conversion
    * yaw-quaternion conversion
  * programmable text-to-speech functions 
    * support English and Chinese
  * base sensor functions
    * set LED color and mode
    * set sound 
    * get battery data
    * get gyro data
	* get load weight
  * base motor functions
    * set motor velocities
    * set linear/angular velocity limit
    * get encoder data
    * get motor velocities
  * localization and map functions
    * get robot current pose
    * save current map to local path
    * get map medadata
  * target points and routes functions
    * assign single point for robot
    * assign routes for robot
    * get current goal data
    * get current goal status
  * robot motion control functions
    * start/pause/stop robot
    * start/stop auto charging
    * shutdown robot system
  * obstacle detection and avoidance functions
    * get laser data
    * get sonar data
    * get planned path from current pose to goal pose
    * make plan path for given two poses
    * clear costmap
  * robot library example nodes
    * sensor and motor test
    * localization and map test
    * point/route and motion control test
    * obstacle detection and avoidance test
  
By calling the mentioned functions, users are able to control and monitor the provided robot platform.



### Configuration
 * This package was tested on <strong> Ubuntu 16.04 with ROS Kinetic </strong>
 * The provided robot library only worked with <strong> designated robot platform </strong>
 * It is strongly recommanded that users do not change the source code of provided library to avoid any error
 
 
 
### Run example nodes
To run `sensor&motor test`:
```
 $ roslaunch robot_lib_test hardware_test.launch
```

To run `localization&map test`:
```
 $ roslaunch robot_lib_test localization_map_test.launch
```

To run `goal&motion test`:
```
 $ roslaunch robot_lib_test point_route_test.launch
```

To run `obstacle detection&avoidance test`:
```
 $ roslaunch robot_lib_test obstacle_detection_avoidance.launch
```



### Others
* Refer to `robot_lib_test` package for configuring `CMakeLists.txt` and `package.xml` in order to use robot library
* Load <strong> robot_lib_params </strong> in `.launch` file that robot library object requires for initialization
* Refer to any example node in `robot_lib_test` package to learn how to use commands in robot library 
