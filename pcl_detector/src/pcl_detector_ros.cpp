#define BOOST_BIND_NO_PLACEHOLDERS

#include <pcl_detector/pcl_detector_ros.hpp>


using std::placeholders::_1;

namespace pcl_detector {

PclDetectorNode::PclDetectorNode(const rclcpp::NodeOptions& options) : Node("pcl_detector",options) 
{
  declare_parameter<std::string>("topic_pointcloud_in","ouster/points");
  declare_parameter<std::string>("topic_pointcloud_out", "lidar/centroids");
  //from params file
  declare_parameter<std::string>("prefix", "pcl_detector");
  declare_parameter<std::string>("detector", "euclidean");
  declare_parameter<float>("dbscan/epsilon", 0.5);
  declare_parameter<int>("dbscan/min_points", 10);
  declare_parameter<float>("euclidean/cluster_tolerance", 0.5);
  declare_parameter<int>("euclidean/min_points", 10);
  declare_parameter<float>("leaf_size", 0.1);
  
  param_topic_pointcloud_in_ = get_parameter("topic_pointcloud_in").as_string();
  param_topic_pointcloud_out_ = get_parameter("topic_pointcloud_out").as_string();

  // Defining quality of service profile for publisher and subscriber
  rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
  qos_profile.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
  auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 1), qos_profile);
  
  publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(param_topic_pointcloud_out_,qos);
  subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
  param_topic_pointcloud_in_, qos, std::bind(&PclDetectorNode::topic_callback, this, _1));

  on_set_callback_handle_ = add_on_set_parameters_callback(std::bind(&PclDetectorNode::parametersCallback, this, std::placeholders::_1));
  
  std::string detector;
  get_parameter<std::string>("detector",detector);
  m_detector = initialize_detector(detector);
}

rcl_interfaces::msg::SetParametersResult PclDetectorNode::parametersCallback(const std::vector<rclcpp::Parameter> &parameters) {
    
    std::string detector_name;
    rcl_interfaces::msg::SetParametersResult result;

    for (const auto &parameter : parameters) {
        if (parameter.get_name() == "detector") {
            if (detector_type.count(parameter.as_string()) == 0) {
                result.successful = false;
                return result;
            } 
        } 
    }
    parameters_changed = true;
    result.successful = true;
    result.reason = "success";
    return result;
}

std::unique_ptr<IPclDetector> PclDetectorNode::initialize_detector(std::string detector)
{

    std::string detector_name = m_prefix + "/" + detector;
    std::unique_ptr<IPclDetector> configured_detector = nullptr;

    switch (detector_type[detector]) {

        case DetectorType::Euclidean: {
            float cluster_tolerance;
            get_parameter<float>("euclidean/cluster_tolerance",cluster_tolerance);
            int min_points;
            get_parameter<int>("euclidean/min_points",min_points);
            configured_detector = std::make_unique<EuclideanClusteringDetector>(cluster_tolerance, min_points);
            RCLCPP_INFO_STREAM(this->get_logger(),"Euclidean detector initialized with cluster tolerance = " << cluster_tolerance << " and min_points = " << min_points);
            break;
        }

        case DetectorType::DBSCAN: {
            float eps; 
            get_parameter<float>("dbscan/epsilon",eps);
            int min_points;
            get_parameter<int>("dbscan/min_points", min_points);
            configured_detector = std::make_unique<DBSCANDetector>(eps, min_points);
            RCLCPP_INFO_STREAM(this->get_logger(),"DBSCAN detector initialized with eps = " << eps << " and min_points = " << min_points);
            break;
        }

        default: {
            RCLCPP_ERROR(this->get_logger(),"Invalid pcl detector specified! Shutting down...");
            rclcpp::shutdown();
        }
    }

    return std::move(configured_detector);
}

void PclDetectorNode::topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg)
{
    // Check if parameters have changed
    if (parameters_changed) {
        std::string detector;
        get_parameter<std::string>("detector",detector);
        
        m_detector = initialize_detector(detector);
        parameters_changed = false;
    }

    float m_leaf_size;
    get_parameter<float>("leaf_size",m_leaf_size);  
    pcl::PointCloud<pcl::PointXYZ>::Ptr cartesian_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*cloud_msg, *cartesian_cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(cartesian_cloud->makeShared());
    sor.setLeafSize(m_leaf_size, m_leaf_size, m_leaf_size);

    // ROS_INFO("Downsampling input cloud");
    sor.filter(*downsampled_cloud);

    auto detections = m_detector->get_detections(*downsampled_cloud);

     if (detections.size() == 0) {
        RCLCPP_WARN(this->get_logger(),"no clusters detected!");
    }



    sensor_msgs::msg::PointCloud2 centroids_cloud_msg;
    pcl::toROSMsg(detections, centroids_cloud_msg);
    centroids_cloud_msg.header.frame_id = cloud_msg->header.frame_id;
    centroids_cloud_msg.header.stamp = rclcpp::Clock().now();

    publisher_->publish(centroids_cloud_msg);
};

} // namespace pcl_detector
