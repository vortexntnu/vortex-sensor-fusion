# Target Classifier

## Running the target_tracking ros package

Make sure you have `vortex-msgs` library

Then simply run
```
ros2 launch target_tracking target_tracking_launch.py
```
This launches the node with the configured params in 
`params/target_classifier_params.yaml`

To reconfigure the parameters of the `target_classifier_node` during runtime, simply run

```
ros2 param set target_classifier_node <parameter_name> <parameter_value> 
```

See `params/target_classifier_params.yaml` for parameter names and corresponding types.

## Structure

The main part is the `target_classifier_ros`. That is the ros node, that subscribes to the `landmark/somerthing` topic from the landmark server and the `object_detection/somerthing`. Then it tries to assign the landmarks a class from the object detector before it publishes the classified landmarks to `target_classifier/landmarks`.