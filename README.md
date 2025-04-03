# vortex-sensor-fusion
A collection of sensor fusion pipelines and algorithms.

## PCL Detector

`detectors/pcl_detectors/`
Contains algorithms for detecting clusters in a pointcloud generated from lidar on the topic `/ouster/points` and publishes the detected clusters on `/lidar/centoids`

See `detectors/pcl_detectors/` README.md for detailed info.

## Target Tracking

`target_tracing/`
Contains algorithms to manage tracks based on the centrids detected from the PCL detector and publishes the estimated tracks on the topic `target_tracking/landmarks/`

See `target_tracking/` README.md for detailed info.