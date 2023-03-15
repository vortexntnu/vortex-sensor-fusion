#include "pcl_detector/pcl_detector_ros.hpp"

std::unique_ptr<pcl_detector::IPclDetector> PclDetectorRos::initialize_detector(std::string detector)
{

    std::string detector_name = m_prefix + "/" + detector;
    std::unique_ptr<pcl_detector::IPclDetector> configured_detector = nullptr;

    switch (detector_type[detector]) {

    case DetectorType::DBSCAN: {

        float eps = get_and_set_rosparam<float>(detector_name + "/epsilon");
        int min_points = get_and_set_rosparam<int>(detector_name + "/min_points");
        m_detector = std::make_unique<pcl_detector::DBSCANDetector>(eps, min_points);
        ROS_INFO_STREAM("Configured " << detector << " with eps=" << eps << " and min_points=" << min_points << "\n");
        break;
    }

    case DetectorType::Euclidean: {
        double cluster_tolerance = get_and_set_rosparam<double>(detector_name + "/cluster_tolerance");
        int min_points = get_and_set_rosparam<int>(detector_name + "/min_points");
        m_detector = std::make_unique<pcl_detector::EuclideanClusteringDetector>(cluster_tolerance, min_points);
        ROS_INFO_STREAM("Configured " << detector << " with cluster_tolerance=" << cluster_tolerance << " and min_points=" << min_points << "\n");
        break;
    }

    case DetectorType::GMM: {
        int num_clusters = get_and_set_rosparam<int>(detector_name + "/num_clusters");
        int max_iterations = get_and_set_rosparam<int>(detector_name + "/max_iterations");
        int step_size = get_and_set_rosparam<int>(detector_name + "/step_size");

        m_detector = std::make_unique<pcl_detector::GMMDetector>(num_clusters, max_iterations, step_size);
        ROS_INFO_STREAM("Configured " << detector << " with num_clusters=" << num_clusters << ", max_iterations= " << max_iterations << " and step_size=" << step_size << "\n");
        break;
    }

    case DetectorType::OPTICS: {
        float eps = get_and_set_rosparam<float>(detector_name + "/epsilon");
        int min_points = get_and_set_rosparam<int>(detector_name + "/min_points");
        m_detector = std::make_unique<pcl_detector::OPTICSDetector>(eps, min_points);
        ROS_INFO_STREAM("Configured " << detector << " with eps=" << eps << " and min_points=" << min_points << "\n");
        break;
    }

    default: {
        ROS_FATAL("Invalid pcl detector specified! Shutting down...");
        ros::shutdown();
    }
    }

    return std::move(configured_detector);
}

PclDetectorRos::PclDetectorRos(ros::NodeHandle nh)
    : m_nh{ nh }
{

    m_pointcloud_sub = nh.subscribe<sensor_msgs::PointCloud2>("/os_cloud_node/points", 1, &PclDetectorRos::pointcloud_callback, this);

    m_centroid_pub = m_nh.advertise<sensor_msgs::PointCloud2>(m_prefix + "/centroids", 1);
    m_centroid_pose_pub = m_nh.advertise<geometry_msgs::PoseArray>("/lidar/clusters", 1);
    m_downsample_pub = m_nh.advertise<sensor_msgs::PointCloud2>(m_prefix + "/downsampled_input", 1);

    m_leaf_size = get_and_set_rosparam<float>(m_prefix + "/leaf_size");

    std::string detector = get_and_set_rosparam<std::string>(m_prefix + "/detector_type");
    std::string detector_name = m_prefix + "/" + detector;

    // Get parameters for said detector type
    m_detector = initialize_detector(detector);

    m_config_server.setCallback(boost::bind(&PclDetectorRos::reconfigure_callback, this, _1, _2));
}

void PclDetectorRos::reconfigure_callback(pcl_detector::PclDetectorConfig& config, uint32_t level)
{

    if (m_first_config) {
        m_first_config = false;
        return; // the reconfigure callback is called once by default on launch. We don't want since that overwrites the yaml params.
    }

    auto new_detector_type = DetectorType(config.detector_type);
    std::string new_detector_name = "";
    for (const auto& entry : detector_type) {
        if (entry.second == new_detector_type) {
            new_detector_name = entry.first;
        }
    }

    if (new_detector_name == "") {
        ROS_FATAL("Invalid detector type configured! Shutting down...");
        ros::shutdown();
    }

    m_nh.setParam(m_prefix + "/detector_type", new_detector_name);
    m_nh.setParam(m_prefix + "/leaf_size", config.leaf_size);
    m_nh.setParam(m_prefix + "/dbscan/epsilon", config.dbscan_epsilon);
    m_nh.setParam(m_prefix + "/dbscan/min_points", config.dbscan_min_points);
    m_nh.setParam(m_prefix + "/euclidean/cluster_tolerance", config.euclidean_cluster_tolerance);
    m_nh.setParam(m_prefix + "/euclidean/min_points", config.euclidean_min_points);
    m_nh.setParam(m_prefix + "/gmm/num_clusters", config.gmm_num_clusters);
    m_nh.setParam(m_prefix + "/gmm/max_iterations", config.gmm_max_iterations);
    m_nh.setParam(m_prefix + "/gmm/step_size", config.gmm_step_size);
    m_nh.setParam(m_prefix + "/optics/epsilon", config.optics_epsilon);
    m_nh.setParam(m_prefix + "/optics/min_points", config.optics_min_points);

    m_detector = initialize_detector(new_detector_name);
}

template <typename T>
T PclDetectorRos::get_and_set_rosparam(std::string rosparam_name)
{

    T parameter;

    if (!m_nh.getParam(rosparam_name, parameter)) {
        ROS_FATAL_STREAM("Failed to read parameter " << rosparam_name << ". Shutting down node..");
        ros::shutdown();
    }

    return parameter;
}

void PclDetectorRos::pointcloud_callback(
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

    geometry_msgs::PoseArray centroids_point_msg;
    centroids_point_msg.header.frame_id = cloud_msg->header.frame_id;
    centroids_point_msg.header.stamp = ros::Time::now();

    for (auto detection : detections) {
        geometry_msgs::Pose pose;
        pose.position.x = detection.x;
        pose.position.y = detection.y;
        pose.position.z = detection.z;
        pose.orientation.x = 0.0;
        pose.orientation.y = 0.0;
        pose.orientation.z = 0.0;
        pose.orientation.w = 1.0;
        centroids_point_msg.poses.push_back(pose);
    }

    m_centroid_pose_pub.publish(centroids_point_msg);
}
