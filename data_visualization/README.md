# **DataVisualization package**

___

© **SGT Driverless**

**Authors:** Patrik Knaperek

**Objective:** Visualize data from nodes in RViz. 
___

The `dataVisualization` node subscribes to `sgtdv_msgs` topics and transforms them into `visualization_msgs`, allowing to visualize topic data in RViz. Additionally, using `SGT_VISUALIZE` macro defined in [`SGT_Macros.h`](../../SGT_Macros.h), extra data is published from intermediate calculations inside nodes.

### Related packages
* [`camera_driver`](../camera_driver/README.md)
* [`lidar_cone_detection`](../lidar_cone_detection/README.md)
* [`fusion`](../fusion/README.md)
* [`mapper`](../../mapper/README.md)
* [`path_planning`](../path_planning/README.md)
* [`path_tracking`](../path_tracking/README.md)

### Topic conversions
* `/camera/cones [sgtdv_msgs/ConeStampedArr] → /camera/cones/marker [visualization_msgs/MarkerArray]`
* `/lidar/cones [sgtdv_msgs/Point2DStampedArr] → /lidar/cones/marker [visualization_msgs/MarkerArray]`
* `/fusion/cones [sgtdv_msgs/ConeWithCovStampedArr] → /fusion/cones/marker [visualization_msgs/MarkerArray]`
* `/slam/pose [sgtdv_msgs/CarPose] → /slam/pose/marker [visualization_msgs/Marker]`
* `/slam/map [sgtdv_msgs/ConeArr] → /slam/map/marker [visualization_msgs/Marker]`
* `/path_planning/trajectory [sgtdv_msgs/Point2DArr] → /path_planning/trajectory/marker [visualization_msgs/Marker]`
* `/path_tracking/cmd [sgtdv_msgs/Control] → /path_tracking/cmd/marker [visualization_msgs/MarkerArray]`


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