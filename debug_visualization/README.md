# **DebugVisualization package**

___

© **SGT Driverless**

**Authors:** Juraj Krasňanský

**Objective:** Visualize node states and performance in RViz. 
___

`SGT_DEBUG_STATE` macro defined in [`SGT_Macros.h`](../../SGT_Macros.h) enables a functionality in the main nodes of `ros_implementation` workspace, allowing us to log current state of the nodes and information about their output data. The point of the `debug_visualization` node is to provide visualization of these data alongside computed node frequencies and work times. 

### Related packages
* [`camera_driver`](../camera_driver/README.md)
* [`lidar_cone_detection`](../lidar_cone_detection/README.md)
* [`fusion`](../fusion/README.md)
* [`mapper`](../../mapper/README.md)
* [`path_planning`](../path_planning/README.md)
* [`path_tracking`](../path_tracking/README.md)

## Compilation (standalone)
```sh
$ cd ${SGT_ROOT}/ros_implementation
$ catkin build debug_visualization
```

### Compilation configuration
* [`debug_visualization.cpp`](./include/debug_visualization.cpp)
	* `NODE_GEOMETRY` : customize layout in RViz

## Launch
```sh
$ cd ${SGT_ROOT}/ros_implementation
$ source ./devel/setup.bash
$ roslaunch debug_visualization debug_visualization.launch
```