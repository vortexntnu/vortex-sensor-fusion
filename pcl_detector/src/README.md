# PCL Detector

A framework for developing detectors that take a pcl pointcloud as input and outputs a new pcl pointcloud of centroids.


# Running the pcl_detector ros package

Simply run
```
ros2 launch pcl_detector pcl_detector_launch.py
```
This launces the node with the configured params in 
`params/pcl_detector_params.yaml`

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