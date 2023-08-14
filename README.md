# **Visualizator package**

___

© **SGT Driverless**

**Authors:** Patrik Knaperek

**Objective:** Visualize data from nodes in RViz. 
___

The `visualizator` node subscribes to `sgtdv_msgs` topics and transforms them into `visualization_msgs`, allowing to visualize topic data in RViz. Additionally, using `SGT_VISUALIZE` macro defined in [`SGT_Macros.h`](../SGT_Macros.h), extra data is published from intermediate calculations inside nodes.

### Related packages
* [`camera_cone_detection`](../camera_cone_detection/README.md)
* [`lidar_cone_detection`](../lidar_cone_detection/README.md)
* [`fusion`](../fusion/README.md)
* `mapper`
* [`path_planning`](../path_planning/README.md)
* [`path_tracking`](../path_tracking/README.md)

### Topic conversions
* `/camera_cones [sgtdv_msgs/ConeStampedArr] → /camera_cones/marker [visualization_msgs/MarkerArray]`
* `/lidar_cones [sgtdv_msgs/Point2DStampedArr] → /lidar_cones/marker [visualization_msgs/MarkerArray]`
* `/fusion_cones [sgtdv_msgs/ConeStampedArr] → /fusion_cones/marker [visualization_msgs/MarkerArray]`
* `/slam/pose [sgtdv_msgs/CarPose] → /slam/pose/marker [visualization_msgs/Marker]`
* `/slam/map [sgtdv_msgs/ConeArr] → /slam/map/marker [visualization_msgs/Marker]`
* `/pathplanning_trajectory [sgtdv_msgs/Point2DArr] → /pathplanning_trajectory/marker [visualization_msgs/Marker]`
* `/pathtracking_commands [sgtdv_msgs/Control] → /pathtracking_commands/marker [visualization_msgs/MarkerArray]`


## Compilation
```sh
$ cd ${SGT_ROOT}/ros_implementation
$ catkin build visualizator
```

### Compilation configuration
* [`Visualizator.h`](./include/Visualizator.h)
	* `THROTLE_MARKER_BASE` : [m]; base point of throtle visualization bar
	* `THROTLE_GAIN` : gain of throtle value in throtle visualization bar
	* `STEER_MARKER_BASE` : [m]; base point of steering visualization bar
	* `STEER_GAIN` : gain of steering value in steering visualization bar

## Launch
```sh
$ cd ${SGT_ROOT}/ros_implementation
$ source ./devel/setup.bash
$ roslaunch visualizator <launchfile>
```
To support easy store and change of RViz configuration depending on the usecase, several launchfiles are provided and can be utilized:
* `visualizator.launch` - default
* `visualizator_sim.launch` - FSSIM environment
* `visualizator_rc.launch` - RC car environment
* `visualizator_camera.launch` - camera standalone environment
* `visualizator_lidar.launch` - lidar standalone environment