#include <target_tracking/target_tracking_ros.hpp>

using std::placeholders::_1;
using namespace std::chrono_literals;


TargetTrackingNode::TargetTrackingNode(const rclcpp::NodeOptions& options)
    : Node("target_tracking_node", options)
{
    // Configure default topics for subscribing/publishing
    declare_parameter<std::string>("topic_pointcloud_in", "lidar/centroids");
    declare_parameter<std::string>("topic_pointcloud_out", "target_tracking/position_estimate");

    // Clutter rate
    declare_parameter<double>("clutter_rate", 1.0);
    // Probability of detection
    declare_parameter<double>("probability_of_detection", 0.9);
    // Gate threshold
    declare_parameter<double>("gate_threshold", 1.0);

    // Track confirmation threshold
    declare_parameter<double>("confirmation_threshold", 0.8);
    // Track deletion threshold
    declare_parameter<double>("deletion_threshold", 0.2);

    // Std velocity parameter
    declare_parameter<double>("std_velocity", 0.1);
    // Std sensor parameter
    declare_parameter<double>("std_sensor", 0.1); 

    // Update inteval ms
    declare_parameter<int>("update_interval_ms", 500);

    // World frame
    declare_parameter<std::string>("world_frame", "world_frame");

    // Read parameters for subscriber and publisher
    param_topic_pointcloud_in_ = get_parameter("topic_pointcloud_in").as_string();
    param_topic_pointcloud_out_ = get_parameter("topic_pointcloud_out").as_string();

    // Set QoS profile
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    qos_profile.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 1), qos_profile);

    // Subscribe to topic
    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        param_topic_pointcloud_in_, qos, std::bind(&TargetTrackingNode::topic_callback, this, _1));
    // Publish to topic
    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(param_topic_pointcloud_out_, qos);

    // Set timer
    int update_interval = get_parameter("update_interval_ms").as_int();
    timer_ = this->create_wall_timer(std::chrono::milliseconds(update_interval), std::bind(&TargetTrackingNode::timer_callback, this));

    // Initialize transform listener
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Initialize track manager
    double clutter_rate = get_parameter("clutter_rate").as_double();
    double probability_of_detection = get_parameter("probability_of_detection").as_double();
    double gate_threshold = get_parameter("gate_threshold").as_double();
    double std_velocity = get_parameter("std_velocity").as_double();
    double std_sensor = get_parameter("std_sensor").as_double();
    track_manager_ = TrackManager(clutter_rate, probability_of_detection, gate_threshold, std_velocity, std_sensor);
}

void TargetTrackingNode::topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr centroids)
{
    // Transform the point cloud to the world frame
    try {
        // Get world_frame parameter
        std::string world_frame = get_parameter("world_frame").as_string();

        // Lookup the transformation
        geometry_msgs::msg::TransformStamped transform_stamped = 
        tf_buffer_->lookupTransform(world_frame, centroids->header.frame_id, centroids->header.stamp, rclcpp::Duration(1, 0));

        // Clear measurements
        measurements_.clear();

        // Transform PointCloud to vector of 2d points
        sensor_msgs::PointCloud2ConstIterator<float> iter_x(*centroids, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(*centroids, "y");

        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y)
        {
            geometry_msgs::msg::PointStamped transformed_point;
            geometry_msgs::msg::PointStamped centroid_point;
            centroid_point.header = centroids->header;
            centroid_point.point.x = *iter_x;
            centroid_point.point.y = *iter_y;
            centroid_point.point.z = 0.0;
            tf2::doTransform(centroid_point, transformed_point, transform_stamped);

            Eigen::Vector2d point_2d;
            point_2d[0] = transformed_point.point.x;
            point_2d[1] = transformed_point.point.y;

            measurements_.push_back(point_2d);
        }

        // Publishes the input pointcloud for testing
        publisher_->publish(*centroids);

    } catch (tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "Could not transform point cloud: %s", ex.what());
    }
}

void TargetTrackingNode::timer_callback()
{
    // get update interval parameter
    int update_interval = get_parameter("update_interval_ms").as_int();

    // Update tracks
    track_manager_.updateTracks(measurements_, update_interval);

    measurements_.clear();
}
