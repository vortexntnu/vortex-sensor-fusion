#ifndef PCL_DETECTOR_ROS_H
#define PCL_DETECTOR_ROS_H

#include <ros/ros.h>

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/PCLPointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>

#include "pcl_detector/pcl_detector.hpp"
#include "pcl_detector/detectors/dbscan_detector.hpp"
#include "pcl_detector/detectors/euclidean_clustering.hpp"
#include "pcl_detector/detectors/gmm_detector.hpp"
#include "pcl_detector/detectors/optics_detector.hpp"

#include <unordered_map>
#include <dynamic_reconfigure/server.h>
#include <pcl_detector/PclDetectorConfig.h>

enum class DetectorType{
    DBSCAN,
    Euclidean,
    GMM,
    OPTICS
};

class PclDetectorRos {

public:
    PclDetectorRos(ros::NodeHandle nh);

    void spin();

private:
    void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg);
    void reconfigureCallback(pcl_detector::PclDetectorConfig &config, uint32_t level);

    ros::NodeHandle m_nh;
    ros::Subscriber m_pointcloud_sub;
    ros::Publisher m_centroid_pub;
    ros::Publisher m_centroid_pose_pub;
    ros::Publisher m_downsample_pub;
    dynamic_reconfigure::Server<pcl_detector::PclDetectorConfig> m_config_server;

    float m_leaf_size;
    std::string prefix = "/pcl_detector";
    unsigned int m_seq{ 0 };
    bool first_config = true;

    template<typename T>
    T get_and_set_rosparam(std::string rosparam_name);

    std::unordered_map<std::string, DetectorType> detector_type = {
        {"dbscan", DetectorType::DBSCAN},
        {"euclidean", DetectorType::Euclidean},
        {"gmm", DetectorType::GMM},
        {"optics", DetectorType::OPTICS}
    };

    void parse_dynamic_reconfigure_parameters();
    std::unique_ptr<pcl_detector::IPclDetector> initialize_detector(std::string detector);

    std::unique_ptr<pcl_detector::IPclDetector> m_detector;
};

#endif // PCL_DETECTOR_ROS_H
