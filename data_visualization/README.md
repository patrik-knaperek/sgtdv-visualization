# **DataVisualization package**

___

© **SGT Driverless**

**Authors:** Patrik Knaperek

**Objective:** Visualize data from nodes in RViz. 
___

The `data_visualization` node subscribes to `sgtdv_msgs` topics and transforms them into `visualization_msgs`, allowing for the topic data visualization in RViz. Additionally, using `SGT_VISUALIZE` macro defined in [`SGT_Macros.h`](../../SGT_Macros.h), extra data is published from intermediate calculations inside nodes.

### Related packages
* [`camera_driver`](/src/camera_driver/README.md)
* [`lidar_cone_detection`](/src/lidar_cone_detection/README.md)
* [`fusion`](/src/fusion/README.md)
* [`mapper`](/src/mapper/README.md)
* [`path_planning`](/src/path_planning/README.md)
* [`path_tracking`](/src/path_tracking/README.md)

### Topic conversions
* `/camera/cones` [[`sgtdv_msgs/ConeStampedArr`](/src/sgtdv_msgs/msg/ConeStampedArr.msg)] → `/camera/cones/marker` [[`visualization_msgs/MarkerArray`](https://docs.ros.org/en/noetic/api/visualization_msgs/html/msg/MarkerArray.html)]
* `/lidar/cones` [[`sgtdv_msgs/Point2DStampedArr`](/src/sgtdv_msgs/msg/Point2DStampedArr.msg)] → `/lidar/cones/marker` [[`visualization_msgs/MarkerArray`](https://docs.ros.org/en/noetic/api/visualization_msgs/html/msg/MarkerArray.html)]
* `/fusion/cones` [[`sgtdv_msgs/ConeWithCovStampedArr`](/src/sgtdv_msgs/msg/ConeWithCovStampedArr.msg)] → `/fusion/cones/marker` [[`visualization_msgs/MarkerArray`](https://docs.ros.org/en/noetic/api/visualization_msgs/html/msg/MarkerArray.html)]
* `/slam/pose` [[`sgtdv_msgs/CarPose`](/src/sgtdv_msgs/msg/CarPose.msg)] → `/slam/pose/marker` [`visualization_msgs/Marker`]
* `/slam/map` [[`sgtdv_msgs/ConeArr`](/src/sgtdv_msgs/msg/ConeArr.msg)] → `/slam/map/marker` [`visualization_msgs/Marker`]
* `/path_planning/trajectory` [[`sgtdv_msgs/Trajectory`](/src/sgtdv_msgs/msg/Trajectory.msg)] → `/path_planning/trajectory/marker/position`, `/path_planning/trajectory/marker/speed` [[`nav_msgs/Path`](https://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Path.html)]
* `/path_tracking/cmd` [[`sgtdv_msgs/Control`](/src/sgtdv_msgs/msg/Control.msg)] → `/path_tracking/cmd/marker` [[`visualization_msgs/MarkerArray`](https://docs.ros.org/en/noetic/api/visualization_msgs/html/msg/MarkerArray.html)]


## Compilation
```sh
$ cd ${SGT_ROOT}/ros_implementation
$ catkin build data_visualization
```

### Compilation configuration
* [`data_visualization.h`](./include/data_visualization.h)
	* `THROTTLE_MARKER_BASE` : [m]; base point of throttle visualization bar
	* `THROTTLE_GAIN` : gain of throttle value in throttle visualization bar
	* `STEER_MARKER_BASE` : [m]; base point of steering visualization bar
	* `STEER_GAIN` : gain of steering value in steering visualization bar

## Launch
```sh
$ cd ${SGT_ROOT}/ros_implementation
$ source ./devel/setup.bash
$ roslaunch data_visualization <launchfile>
```
To support easy store and change of RViz configuration depending on the use case, several launchfiles are provided and can be utilized:
* `data_visualization.launch` - default
* `data_visualization_sim.launch` - FSSIM environment
* `data_visualization_rc.launch` - RC car environment
* `data_visualization_camera.launch` - camera standalone environment
* `data_visualization_lidar.launch` - lidar standalone environment