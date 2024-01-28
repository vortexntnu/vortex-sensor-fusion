#include <target_tracking/target_tracking_ros.hpp>

using std::placeholders::_1;
using namespace std::chrono_literals;


TargetTrackingNode::TargetTrackingNode(const rclcpp::NodeOptions& options)
    : Node("target_tracking_node", options)
{
    // Configure default topics for subscribing/publishing
    declare_parameter<std::string>("topic_pointcloud_in", "lidar/centroids");
    declare_parameter<std::string>("topic_pointcloud_out", "target_tracking/landmarks");

 
    declare_parameter<double>("clutter_rate", 1.0);
    declare_parameter<double>("probability_of_detection", 0.9);
    declare_parameter<double>("gate_threshold", 1.0);

    declare_parameter<double>("confirmation_threshold", 0.6);
    declare_parameter<double>("deletion_threshold", 0.2);

    declare_parameter<double>("std_velocity", 0.05);
    declare_parameter<double>("std_sensor", 0.05); 

    declare_parameter<int>("update_interval_ms", 500);

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
    publisher_ = this->create_publisher<vortex_msgs::msg::LandmarkArray>(param_topic_pointcloud_out_, qos);

    // Set timer
    int update_interval = get_parameter("update_interval_ms").as_int();
    timer_ = this->create_wall_timer(std::chrono::milliseconds(update_interval), std::bind(&TargetTrackingNode::timer_callback, this));

    // Initialize transform listener
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Initialize track manager
    double std_velocity = get_parameter("std_velocity").as_double();
    double std_sensor = get_parameter("std_sensor").as_double();

    track_manager_ = TrackManager(std_velocity, std_sensor);
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
    } catch (tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "Could not transform point cloud: %s", ex.what());
    }
}

void TargetTrackingNode::timer_callback()
{
    // get parameters
    int update_interval = get_parameter("update_interval_ms").as_int();
    double confirmation_threshold = get_parameter("confirmation_threshold").as_double();
    double gate_threshold = get_parameter("gate_threshold").as_double();
    double prob_of_detection = get_parameter("probability_of_detection").as_double();
    double clutter_intensity = get_parameter("clutter_rate").as_double();
    double delete_threshold_ = get_parameter("deletion_threshold").as_double();

    // Update tracks
    track_manager_.updateTracks(measurements_, update_interval, confirmation_threshold, gate_threshold, prob_of_detection, clutter_intensity);

    measurements_.clear();

    // Publish tracks
    vortex_msgs::msg::LandmarkArray landmark_array;
    for(const auto& track : track_manager_.getTracks())
    {   
        // Skips unconfirmed tracks
        if(track.confirmed == false){
            continue;
        }

        vortex_msgs::msg::Landmark landmark;

        // Sets landmark type
        landmark.landmark_type = "boat";

        int var = track.existence_probability < delete_threshold_ ? 0 : 1;

        // creates landmark message
        landmark.id = track.id;
        landmark.action = track.existence_probability < delete_threshold_ ? 0 : 1;
        std::cout << "ID: " << track.id << "Action:" << var << std::endl;
        landmark.odom.header.frame_id = get_parameter("world_frame").as_string();
        landmark.odom.header.stamp = this->get_clock()->now();

        landmark.odom.pose.pose.position.x = track.state.mean()(0);
        landmark.odom.pose.pose.position.y = track.state.mean()(1);
        landmark.odom.pose.pose.position.z = 0.0;

        landmark.odom.pose.covariance = {track.state.cov()(0,0), track.state.cov()(0,1), 0.0, 0.0, 0.0, 0.0,
                                         track.state.cov()(1,0), track.state.cov()(1,1), 0.0, 0.0, 0.0, 0.0,
                                         0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
                                         0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
                                         0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
                                         0.0, 0.0, 0.0, 0.0, 0.0, 1.0};

        // std::cout << track.state.cov() << std::endl;

        landmark.odom.pose.pose.orientation.x = 0.0;
        landmark.odom.pose.pose.orientation.y = 0.0;
        landmark.odom.pose.pose.orientation.z = 0.0;
        landmark.odom.pose.pose.orientation.w = 1.0;

        landmark.odom.child_frame_id = get_parameter("world_frame").as_string();
        landmark_array.landmarks.push_back(landmark);
    }



    // delete tracks
    double deletion_threshold = get_parameter("deletion_threshold").as_double();
    track_manager_.deleteTracks(deletion_threshold);

    publisher_->publish(landmark_array);

    RCLCPP_INFO(this->get_logger(), "Published %lu tracks", landmark_array.landmarks.size());

}
