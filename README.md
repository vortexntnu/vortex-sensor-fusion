# vortex-sensor-fusion
A collection of sensor fusion pipelines and algorithms.

## PCL Detector

`detectors/pcl_detector/`
Contains algorithms for detecting clusters in a pointcloud generated from lidar on the topic `/ouster/points` and publishes the detected clusters on `/lidar/centoids`

See `detectors/pcl_detector/` README.md for detailed info.

## Aruco Detector

`detectors/aruco_detector/`
Contains functions to detect aruco markers from image stream and publish the position of the markers on the topic `/aruco_marker_poses`
Also supports detection of aruco boards. Publishes the posistion of the board centre over the topic `/aruco_board_pose`.


See `detectors/aruco_detector/` README.md for detailed info.