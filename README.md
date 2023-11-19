# vortex-sensor-fusion
A collection of sensor fusion pipelines and algorithms.

## PCL Detector

`detectors/pcl_detectors/`
Contains algorithms for detecting clusters in a pointcloud generated from lidar on the topic `/ouster/points` and publishes the detected clusters on `/lidar/centoids`

See `detectors/pcl_detectors/` README.md for detailed info.