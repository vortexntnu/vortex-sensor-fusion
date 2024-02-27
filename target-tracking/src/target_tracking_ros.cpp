#include <target_tracking/target_tracking_ros.hpp>

using std::placeholders::_1;
using namespace std::chrono_literals;


TargetTrackingNode::TargetTrackingNode(const rclcpp::NodeOptions& options)
    : Node("target_tracking_node", options)
{
    // Configure default topics for subscribing/publishing
    declare_parameter<std::string>("topic_pointcloud_in", "lidar/centroids");
    declare_parameter<std::string>("topic_pointcloud_out", "target_tracking/landmarks");
 
    declare_parameter<double>("clutter_rate", 0.001);
    declare_parameter<double>("probability_of_detection", 0.75);
    declare_parameter<double>("probability_of_survival", 0.99);
    declare_parameter<double>("gate_threshold", 1.5);

    declare_parameter<double>("min_gate_threshold", 1.0);
    declare_parameter<double>("max_gate_threshold", 2.0);

    declare_parameter<double>("confirmation_threshold", 0.7);
    declare_parameter<double>("deletion_threshold", 0.1);

    declare_parameter<double>("std_velocity", 0.2);
    declare_parameter<double>("std_sensor", 0.5);

    declare_parameter<int>("update_interval_ms", 500);

    declare_parameter<std::string>("world_frame", "world_frame");

    // Set parameter callback
    parameter_subscriber_ = add_on_set_parameters_callback(std::bind(&TargetTrackingNode::parametersCallback, this, std::placeholders::_1));

    // Read parameters for subscriber and publisher
    param_topic_pointcloud_in_ = get_parameter("topic_pointcloud_in").as_string();
    param_topic_pointcloud_out_ = get_parameter("topic_pointcloud_out").as_string();

    // Set QoS profile
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    qos_profile.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 1), qos_profile);

    // Subscribe to topic
    subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        param_topic_pointcloud_in_, qos, std::bind(&TargetTrackingNode::topic_callback, this, _1));

    // Publish landmarks
    landmark_publisher_ = this->create_publisher<vortex_msgs::msg::LandmarkArray>(param_topic_pointcloud_out_, qos);

    // Set timer
    int update_interval = get_parameter("update_interval_ms").as_int();
    timer_ = this->create_wall_timer(std::chrono::milliseconds(update_interval), std::bind(&TargetTrackingNode::timer_callback, this));

    // Initialize transform listener
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Initialize track manager
    double std_velocity = get_parameter("std_velocity").as_double();
    double std_sensor = get_parameter("std_sensor").as_double();

    track_manager_ = TrackManager();
    track_manager_.set_dyn_model(std_velocity);
    track_manager_.set_sensor_model(std_sensor);
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
            // std::cout << track.state.cov() << std::endl;, transform_stamped);

            Eigen::Vector2d point_2d;
            point_2d[0] = transformed_point.point.x;
            point_2d[1] = transformed_point.point.y;

            measurements_.push_back(point_2d);
        }
    } catch (tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "Could not transform point cloud: %s", ex.what());
    }
}
rcl_interfaces::msg::SetParametersResult TargetTrackingNode::parametersCallback(const std::vector<rclcpp::Parameter> &parameters)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (const auto &parameter : parameters)
    {
        if (parameter.get_name() == "std_velocity")
        {
            update_dyn_model(parameter.as_double());
        }
        else if (parameter.get_name() == "std_sensor")
        {
            update_sensor_model(parameter.as_double());
        } 
        else if (parameter.get_name() == "update_interval_ms")
        {
            update_timer(parameter.as_int());
        }
    }
    return result;
}
void TargetTrackingNode::timer_callback()
{
    // get parameters
    int update_interval = get_parameter("update_interval_ms").as_int();
    double confirmation_threshold = get_parameter("confirmation_threshold").as_double();
    double gate_threshold = get_parameter("gate_threshold").as_double();
    double min_gate_threshold = get_parameter("min_gate_threshold").as_double();
    double max_gate_threshold = get_parameter("max_gate_threshold").as_double();
    double prob_of_detection = get_parameter("probability_of_detection").as_double();
    double prob_of_survival = get_parameter("probability_of_survival").as_double();
    double clutter_intensity = get_parameter("clutter_rate").as_double();
    double deletion_threshold = get_parameter("deletion_threshold").as_double();

    // Update tracks
    track_manager_.updateTracks(measurements_, update_interval, confirmation_threshold, gate_threshold, min_gate_threshold, max_gate_threshold, prob_of_detection, prob_of_survival,clutter_intensity);

    measurements_.clear();

    // Publish tracks to landmark server
    publish_landmarks(deletion_threshold);

    // Publish visualization parameters

    // delete tracks
    track_manager_.deleteTracks(deletion_threshold);
}

void TargetTrackingNode::publish_landmarks(double deletion_threshold) {
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

        // creates landmark message
        landmark.id = track.id;
        landmark.action = track.existence_probability < deletion_threshold ? 0 : 1;
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

        double yaw_angle;

        double vel_x = track.state.mean()(2);
        double vel_y = track.state.mean()(3);

        if (vel_x == 0) {
            yaw_angle = vel_y > 0 ? M_PI / 2 : -M_PI / 2;
        } else {
            yaw_angle = std::atan2(vel_y, vel_x);
        }

        landmark.odom.pose.pose.orientation.x = 0.0;
        landmark.odom.pose.pose.orientation.y = 0.0;
        landmark.odom.pose.pose.orientation.z = std::sin(yaw_angle / 2);
        landmark.odom.pose.pose.orientation.w = std::cos(yaw_angle / 2);

        landmark.odom.twist.twist.linear.x = track.state.mean()(2);
        landmark.odom.twist.twist.linear.y = track.state.mean()(3);

        landmark.odom.twist.covariance = {track.state.cov()(2,2), track.state.cov()(2,3), 0.0, 0.0, 0.0, 0.0,
                                         track.state.cov()(3,2), track.state.cov()(3,3), 0.0, 0.0, 0.0, 0.0,
                                         0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
                                         0.0, 0.0, 0.0, 1.0 , 0.0, 0.0,
                                         0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
                                         0.0, 0.0, 0.0, 0.0, 0.0, 1.0};

        landmark.odom.child_frame_id = get_parameter("world_frame").as_string();
        landmark_array.landmarks.push_back(landmark);
    }

    landmark_publisher_->publish(landmark_array);

    RCLCPP_INFO(this->get_logger(), "Published %lu tracks", landmark_array.landmarks.size());
}

void TargetTrackingNode::update_timer(int update_interval)
{
    timer_->cancel();
    timer_ = this->create_wall_timer(std::chrono::milliseconds(update_interval), std::bind(&TargetTrackingNode::timer_callback, this));
    RCLCPP_INFO(this->get_logger(), "Updated timer with %d ms update interval", update_interval);
}

void TargetTrackingNode::update_dyn_model(double std_velocity)
{
    track_manager_.set_dyn_model(std_velocity);
    RCLCPP_INFO(this->get_logger(), "Updated dynamic model");
}

void TargetTrackingNode::update_sensor_model(double std_measurement)
{
    track_manager_.set_sensor_model(std_measurement);
    RCLCPP_INFO(this->get_logger(), "Updated sensor model");
}
