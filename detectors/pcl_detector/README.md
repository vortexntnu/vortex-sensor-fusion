# PCL Detector

A framework for developing detectors that take a pcl pointcloud as input and outputs a new pcl pointcloud of centroids.


# Running the pcl_detector ros package

Simply run
```
ros2 launch pcl_detector pcl_detector_launch.py
```
This launces the node with the configured params in 
`params/pcl_detector_params.yaml`

To reconfigure the parameters of the `pcl_detector_node` during runtime, simply run

```
ros2 param set pcl_detector_node <parameter_name> <parameter_value> 
```

See `params/pcl_detector_params.yaml` for parameter names and corresponding types.

# Detectors

### Euclidean Clustering

Euclidean clustering is a clustering algorithm that groups together points that are close to each other in Euclidean space. The algorithm has two parameters: min_points and cluster_tolerance

The algorithm can be abstracted into the following steps:

1. Find the points in the epsilon neighborhood of every point.
2. Create a new cluster for each point that has not been assigned to a cluster.
3. For each point in a cluster, add all points in its epsilon neighborhood to the same cluster.

### DBSCAN

DBSCAN (Density-Based Spatial Clustering of Applications with Noise) is a density-based clustering algorithm that groups together points that are closely packed together, marking as outliers points that lie alone in low-density regions. The algorithm has two hyperparameters: epsilon and min_points. Epsilon specifies the radius of a neighborhood with respect to some point, while min_points specifies the minimum number of points required to form a dense region. 

The algorithm can be abstracted into the following steps:

1. Find the points in the epsilon neighborhood of every point, and identify the core points with more than min_points neighbors.
2. Find the connected components of core points on the neighbor graph, ignoring all non-core points.

# Detector Integration
The detectors are derived from the `IPclDetector` interface in `pcl_detector/pcl_detector_interface.hpp`. All detectors implements the `get_detections` method, that takes the pointcloud as input and returns the detected clusters. These detectors are independent from ROS.

The ROS2 node utilizes the configured detector's `get_detections` method in the `topic_callback` method of the ROS2 node.

# Develop your own detector

This framework is written to make it easy to implement different types of pcl-based detectors. These will mostly be tailored for use with lidar data, but is generic enough to be adapted to any data as long as it is in pcl format.

To make a new detector, do the following:

1. Create a .hpp under `include/pcl_detector/detectors` and declare a class that looks like the following:

    ```C++
        #pragma once

        #include <pcl_detector/pcl_detector_interface.hpp>

        namespace pcl_detector {

        class YourDetector : public IPclDetector {

        public:
            YourDetector();

            pcl::PointCloud<pcl::PointXYZ> get_detections(const pcl::PointCloud<pcl::PointXYZ>& points) override;

        private:
        };

        }; // namespace pcl_detector
    ```

2. Implement the required constructor and the `get_detections` in a separate .cpp file.
3. Test your detector with ROS by including your detector in *pcl_detector_ros_node* and passing it to the `PclDetectorNode` object. 


The `PclDetectorNode` class contains a member variable of type `std::unique_ptr<pcl_detector::IPclDetector> m_detector;`. By declaring your detector to inherit from `IPclDetector` you are able to pass a pointer of type `YourDetector` to the `PclDetectorRos` class.