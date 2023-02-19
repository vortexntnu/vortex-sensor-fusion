# PCL Detector

A framework for developing detectors that take a pcl pointcloud as input and outputs a new pcl pointcloud of centroids.


# Running the pcl_detector ros package

Simply run
```
roslaunch pcl_detector detector.launch ip:=<sensor-ip>
```
where <sensor-ip> is the IP address of the lidar in use. This defaults to one of the two lidars we currently have. See the [lidar driver repository](https://github.com/vortexntnu/ouster-lidar-driver) to learn how to find the sensor ip.

# Develop your own detector

This framework is written to make it easy to implement different types of pcl-based detectors. These will mostly be tailored for use with lidar data, but is generic enough to be adapted to any data as long as it is in pcl format.

To make a new detector, do the following:

1. Create a .hpp under `include/pcl_detector/detectors` and declare a class that looks like the following:

    ```C++
        #ifndef YOUR_DETECTOR_H
        #define YOUR_DETECTOR_H

        #include "pcl_detector/pcl_detector.hpp"

        class YourDetector : public pcl_detector::IPclDetector {

        public:
            YourDetector();

            pcl::PointCloud<pcl::PointXYZ> get_detections(const pcl::PointCloud<pcl::PointXYZ>& points) override;

        private:
        };

        #endif // YOUR_DETECTOR_H
    ```

2. Implement the required constructor and the `get_detections` in a separate .cpp file.
3. Test your detector with ROS by including your detector in *pcl_detector_ros_node* and passing it to the `PclDetectorRos` object. 


The `PclDetectorRos` class contains a member variable of type `std::shared_ptr<pcl_detector::IPclDetector> m_detector`. By declaring your detector to inherit from `IPclDetector` you are able to pass a pointer of type `YourDetector` to the `PclDetectorRos` class.