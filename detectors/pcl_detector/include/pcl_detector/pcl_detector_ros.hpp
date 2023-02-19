#ifndef PCL_DETECTOR_ROS_H
#define PCL_DETECTOR_ROS_H

#include <ros/ros.h>

#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/PCLPointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include "pcl_detector/pcl_detector.hpp"

#include "pcl/filters/voxel_grid.h"

class PclDetectorRos {

public:
    PclDetectorRos(ros::NodeHandle nh,
        std::shared_ptr<pcl_detector::IPclDetector> detector,
        float leaf_size = 0.1);

    void spin();

private:
    void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg);

    ros::NodeHandle m_nh;
    ros::Subscriber m_pointcloud_sub;
    ros::Publisher m_centroid_pub;
    ros::Publisher m_downsample_pub;
    float m_leaf_size;

    unsigned int m_seq{ 0 };

    std::shared_ptr<pcl_detector::IPclDetector> m_detector;
};

#endif // PCL_DETECTOR_ROS_H