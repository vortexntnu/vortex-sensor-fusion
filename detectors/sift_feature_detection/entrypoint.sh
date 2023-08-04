#!/bin/bash

type="real"

source /home/vortex/auv_ws/devel/setup.bash &&
roslaunch sift_feature_detection sift_feature_detection_node.launch type:=$type
