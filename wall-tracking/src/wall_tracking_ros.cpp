#include <wall_tracking/wall_tracking_ros.hpp>

using std::placeholders::_1;
using namespace std::chrono_literals;


WallTrackingNode::WallTrackingNode(const rclcpp::NodeOptions& options)
    : Node("wall_tracking_node", options)
{
    // Configure default topics for subscribing/publishing
    declare_parameter<std::string>("topic_walls_in", "wall_poses");
    declare_parameter<std::string>("topic_landmarks_out", "target_tracking/landmarks");
 
    declare_parameter<double>("clutter_rate", 0.001);
    declare_parameter<double>("probability_of_detection", 0.7);
    declare_parameter<double>("probability_of_survival", 0.99);
    declare_parameter<double>("gate_threshold", 2.5);
    declare_parameter<double>("min_gate_threshold", 1.0);
    declare_parameter<double>("max_gate_threshold", 10.0);
    declare_parameter<double>("confirmation_threshold", 0.9);
    declare_parameter<double>("deletion_threshold", 0.1);
    declare_parameter<double>("initial_existence_probability", 0.4);
    declare_parameter<double>("std_position", 0.2);
    declare_parameter<double>("std_sensor", 0.5);

    declare_parameter<int>("update_interval_ms", 500);
    declare_parameter<std::string>("fixed_frame", "world_frame");

    // Set parameter callback
    parameter_subscriber_ = add_on_set_parameters_callback(std::bind(&WallTrackingNode::parametersCallback, this, std::placeholders::_1));

    // Read parameters for subscriber and publisher
    param_topic_measurements_in_ = get_parameter("topic_measurements_in").as_string();
    param_topic_landmarks_out_ = get_parameter("topic_landmarks_out").as_string();

    // Set reliable QoS profile for publisher
    rmw_qos_profile_t qos_profile_pub = rmw_qos_profile_default;
    qos_profile_pub.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
    qos_profile_pub.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
    qos_profile_pub.depth = 10; // You can adjust this depth as needed for your use case

    auto qos_pub = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile_pub.history, qos_profile_pub.depth), qos_profile_pub);

    // Set best effort QoS profile for subscriber
    rmw_qos_profile_t qos_profile_sub = rmw_qos_profile_sensor_data;
    qos_profile_sub.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
    auto qos_sub = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile_sub.history, 1), qos_profile_sub);

    // Subscribe to topic
    subscriber_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
        param_topic_measurements_in_, qos_sub, std::bind(&WallTrackingNode::topic_callback, this, _1));

    // Publish landmarks
    landmark_publisher_ = this->create_publisher<vortex_msgs::msg::LandmarkArray>(param_topic_landmarks_out_, qos_pub);

    // Set timer
    int update_interval = get_parameter("update_interval_ms").as_int();
    timer_ = this->create_wall_timer(std::chrono::milliseconds(update_interval), std::bind(&WallTrackingNode::timer_callback, this));

    // Initialize transform listener
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Initialize track manager
    double std_position = get_parameter("std_position").as_double();
    double std_sensor = get_parameter("std_sensor").as_double();

    wall_manager_ = WallManager();
    wall_manager_.set_dyn_model(std_position);
    wall_manager_.set_sensor_model(std_sensor);
}

void WallTrackingNode::topic_callback(const geometry_msgs::msg::PoseArray::SharedPtr wall_poses)
{
    // Transform the poses to the fixed frame
    try
    {
        // Get fixed_frame parameter
        std::string fixed_frame = get_parameter("fixed_frame").as_string();

        // Lookup the transformation
        geometry_msgs::msg::TransformStamped transform_stamped = 
        tf_buffer_->lookupTransform(fixed_frame, wall_poses->header.frame_id, tf2::TimePointZero);

        // Clear measurements
        measurements_= Eigen::Array<double, 4, Eigen::Dynamic>(4, wall_poses->poses.size()/2);
        Eigen::Index index = 0;

        for (size_t i = 0; i < wall_poses->poses.size()-1; i+=2)
        {
            geometry_msgs::msg::Point wall_p1 = wall_poses->poses.at(i).position;
            geometry_msgs::msg::Point wall_p2 = wall_poses->poses.at(i+1).position;
            geometry_msgs::msg::Point wall_p1_tf;
            geometry_msgs::msg::Point wall_p2_tf;

            tf2::doTransform(wall_p1, wall_p1_tf, transform_stamped);
            tf2::doTransform(wall_p2, wall_p2_tf, transform_stamped);
            if (std::hypot(wall_p1_tf.x, wall_p1_tf.y) < std::hypot(wall_p2_tf.x, wall_p2_tf.y))
            {
                std::swap(wall_p1_tf, wall_p2_tf);
            }
            Eigen::Vector4d wall;
            wall[0]=wall_p1_tf.x;
            wall[1]=wall_p1_tf.y;
            wall[2]=wall_p2_tf.x;
            wall[3]=wall_p2_tf.y;
            measurements_.col(index) = wall;
            index++;
        }
        
    } catch (tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "Could not transform point cloud: %s", ex.what());
    }
}


rcl_interfaces::msg::SetParametersResult WallTrackingNode::parametersCallback(const std::vector<rclcpp::Parameter> &parameters)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (const auto &parameter : parameters)
    {
        if (parameter.get_name() == "std_position")
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


void WallTrackingNode::timer_callback()
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
    double initial_existence_probability = get_parameter("initial_existence_probability").as_double();


    // Update tracks
    wall_manager_.updateWalls(measurements_, update_interval, confirmation_threshold, gate_threshold, min_gate_threshold, max_gate_threshold, prob_of_detection, prob_of_survival,clutter_intensity, initial_existence_probability);

    measurements_.resize(4, 0);

    // Publish tracks to landmark server
    publish_landmarks(deletion_threshold);

    // delete tracks
    wall_manager_.deleteWalls(deletion_threshold);
}

void WallTrackingNode::publish_landmarks(double deletion_threshold) {
    vortex_msgs::msg::LandmarkArray landmark_array;
    for(const auto& track : wall_manager_.getWalls())
    {   
        // Skips unconfirmed tracks
        if(track.confirmed == false){
            continue;
        }
        vortex_msgs::msg::Landmark landmark;

        // Sets landmark type
        landmark.landmark_type = vortex_msgs::msg::Landmark::WALL;

        // Sets landmark classification
        landmark.classification = vortex_msgs::msg::Landmark::UNKNOWN;

        // creates landmark message
        landmark.id = track.id;
        landmark.action = track.existence_probability < deletion_threshold ? 0 : track.action;
        landmark.odom.header.frame_id = get_parameter("fixed_frame").as_string();
        landmark.odom.header.stamp = this->get_clock()->now();

        landmark.odom.pose.pose.position.x = (track.state.mean()(0)-track.state.mean()(2))/2;
        landmark.odom.pose.pose.position.y = (track.state.mean()(1)-track.state.mean()(3))/2;
        landmark.odom.pose.pose.position.z = 0.0;

        landmark.odom.pose.covariance = {track.state.cov()(0,0), track.state.cov()(0,1), track.state.cov()(0,2), track.state.cov()(0,3), 0.0, 0.0,
                                         track.state.cov()(1,0), track.state.cov()(1,1), track.state.cov()(1,2), track.state.cov()(1,3), 0.0, 0.0,
                                         0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
                                         0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
                                         0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
                                         0.0, 0.0, 0.0, 0.0, 0.0, 1.0};

        landmark.odom.pose.pose.orientation.x = 0.0;
        landmark.odom.pose.pose.orientation.y = 0.0;
        landmark.odom.pose.pose.orientation.z = 0.0;
        landmark.odom.pose.pose.orientation.w = 1.0;

        landmark.odom.twist.twist.linear.x = 0.0;
        landmark.odom.twist.twist.linear.y = 0.0;

        landmark.odom.twist.covariance = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                         0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                         0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
                                         0.0, 0.0, 0.0, 1.0 , 0.0, 0.0,
                                         0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
                                         0.0, 0.0, 0.0, 0.0, 0.0, 1.0};

        landmark.odom.child_frame_id = get_parameter("fixed_frame").as_string();

        shape_msgs::msg::SolidPrimitive wall_shape;
        wall_shape.type = shape_msgs::msg::SolidPrimitive::PRISM;
        wall_shape.polygon.points.resize(2);
        wall_shape.polygon.points.at(0).x = track.state.mean()(0);
        wall_shape.polygon.points.at(0).y = track.state.mean()(1);
        wall_shape.polygon.points.at(1).x = track.state.mean()(2);
        wall_shape.polygon.points.at(1).y = track.state.mean()(3);
        
        landmark.shape = wall_shape;


        landmark_array.landmarks.push_back(landmark);
    }

    landmark_publisher_->publish(landmark_array);

}

void WallTrackingNode::update_timer(int update_interval)
{
    timer_->cancel();
    timer_ = this->create_wall_timer(std::chrono::milliseconds(update_interval), std::bind(&WallTrackingNode::timer_callback, this));
    RCLCPP_INFO(this->get_logger(), "Updated timer with %d ms update interval", update_interval);
}

void WallTrackingNode::update_dyn_model(double std_position)
{
    wall_manager_.set_dyn_model(std_position);
    RCLCPP_INFO(this->get_logger(), "Updated dynamic model");
}

void WallTrackingNode::update_sensor_model(double std_measurement)
{
    wall_manager_.set_sensor_model(std_measurement);
    RCLCPP_INFO(this->get_logger(), "Updated sensor model");
}
