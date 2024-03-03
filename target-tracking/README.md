# Target Tracking

## Running the target_tracking ros package

Make sure you have both the `vortex-vkf` and `vortex-msgs` libraries

Then simply run
```
ros2 launch target_tracking target_tracking_launch.py
```
This launches the node with the configured params in 
`params/target_tracking_params.yaml`

To reconfigure the parameters of the `target_tracking_node` during runtime, simply run

```
ros2 param set target_tracking_node <parameter_name> <parameter_value> 
```

See `params/target_tracking_params.yaml` for parameter names and corresponding types.

## Structure

The target tracker is split in two main parts, the `track_manager`, that is independent of ROS. It updates, creates and deletes tracks, and holds all the tracking logic. 

The other main part is the `target_tracking_ros`. That is the ros node, that subscribes to the `lidar/cluster` topic from the pcl detector and publishes the tracks to `target_tracking/landmarks/`. It also publishes some parameters for visualisation purpouses to the topic `target_tracker/parameters/`