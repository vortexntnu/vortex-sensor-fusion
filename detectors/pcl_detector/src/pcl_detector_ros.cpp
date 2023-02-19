#include "pcl_detector/pcl_detector_ros.hpp"

PclDetectorRos::PclDetectorRos(
    ros::NodeHandle nh, std::shared_ptr<pcl_detector::IPclDetector> detector,
    float leaf_size)
    : m_nh{ nh }
    , m_detector{ std::move(detector) }
    , m_leaf_size{ leaf_size }
{

    m_pointcloud_sub = nh.subscribe<sensor_msgs::PointCloud2>("/os_cloud_node/points", 1, &PclDetectorRos::pointCloudCallback, this);

    m_centroid_pub = m_nh.advertise<sensor_msgs::PointCloud2>("/pcl_detector/centroids", 1);
    m_downsample_pub = m_nh.advertise<sensor_msgs::PointCloud2>("/pcl_detector/downsampled_input", 1);
}

void PclDetectorRos::pointCloudCallback(
    const sensor_msgs::PointCloud2::ConstPtr& cloud_msg)
{

    pcl::PointCloud<pcl::PointXYZ>::Ptr cartesian_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*cloud_msg, *cartesian_cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(cartesian_cloud->makeShared());
    sor.setLeafSize(m_leaf_size, m_leaf_size, m_leaf_size);

    // ROS_INFO("Downsampling input cloud");
    sor.filter(*downsampled_cloud);

    sensor_msgs::PointCloud2 downsample_cloud_msg;
    pcl::toROSMsg(*downsampled_cloud, downsample_cloud_msg);
    downsample_cloud_msg.header.frame_id = cloud_msg->header.frame_id;
    downsample_cloud_msg.header.stamp = ros::Time::now();
    m_downsample_pub.publish(downsample_cloud_msg);

    // ROS_INFO("Starting clustering!");
    auto detections = m_detector->get_detections(*downsampled_cloud);
    // ROS_INFO("Finished clustering!");

    if (detections.size() == 0) {
        ROS_WARN("No clusters found");
    }

    sensor_msgs::PointCloud2 centroids_cloud_msg;
    pcl::toROSMsg(detections, centroids_cloud_msg);
    centroids_cloud_msg.header.frame_id = cloud_msg->header.frame_id;
    centroids_cloud_msg.header.stamp = ros::Time::now();
    m_centroid_pub.publish(centroids_cloud_msg);
}
