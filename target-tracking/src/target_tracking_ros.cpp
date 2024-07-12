#include <target_tracking/target_tracking_ros.hpp>

using std::placeholders::_1;
using namespace std::chrono_literals;


TargetTrackingNode::TargetTrackingNode(const rclcpp::NodeOptions& options)
    : Node("target_tracking_node", options)
{
    // Configure default topics for subscribing/publishing
    declare_parameter<std::string>("topic_pointcloud_in", "lidar/centroids");
    declare_parameter<std::string>("topic_landmarks_out", "target_tracking/landmarks");
    declare_parameter<std::string>("topic_visualization_parameters", "target_tracking/parameters");
 
    declare_parameter<double>("clutter_rate", 0.001);
    declare_parameter<double>("probability_of_detection", 0.7);
    declare_parameter<double>("probability_of_survival", 0.99);
    declare_parameter<double>("gate_threshold", 2.5);
    declare_parameter<double>("min_gate_threshold", 1.0);
    declare_parameter<double>("max_gate_threshold", 10.0);
    declare_parameter<double>("confirmation_threshold", 0.9);
    declare_parameter<double>("deletion_threshold", 0.1);
    declare_parameter<double>("initial_existence_probability", 0.4);
    declare_parameter<double>("std_velocity", 0.2);
    declare_parameter<double>("std_sensor", 0.5);

    declare_parameter<int>("update_interval_ms", 500);
    declare_parameter<std::string>("fixed_frame", "world_frame");
    declare_parameter<bool>("publish_visualization", true);

    // Set parameter callback
    parameter_subscriber_ = add_on_set_parameters_callback(std::bind(&TargetTrackingNode::parametersCallback, this, std::placeholders::_1));

    // Read parameters for subscriber and publisher
    param_topic_pointcloud_in_ = get_parameter("topic_pointcloud_in").as_string();
    param_topic_landmarks_out_ = get_parameter("topic_landmarks_out").as_string();
    param_topic_visualization_out_ = get_parameter("topic_visualization_parameters").as_string();


    // Set QoS profile
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    qos_profile.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 1), qos_profile);

    // Subscribe to topic
    subscriber_ = this->create_subscription<vortex_msgs::msg::Clusters>(
        param_topic_pointcloud_in_, qos, std::bind(&TargetTrackingNode::topic_callback, this, _1));

    // Publish landmarks
    landmark_publisher_ = this->create_publisher<vortex_msgs::msg::LandmarkArray>(param_topic_landmarks_out_, qos);

    // Publish visualization parameters
    visualization_publisher_ = this->create_publisher<vortex_msgs::msg::ParameterArray>(param_topic_visualization_out_, qos);

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

void TargetTrackingNode::topic_callback(const vortex_msgs::msg::Clusters::SharedPtr clusters)
{
    // Transform the point cloud to the fixed frame
    RCLCPP_INFO(this->get_logger(), "Received %lu clusters", clusters->clusters.size());
    try {
        // Get fixed_frame parameter
        std::string fixed_frame = get_parameter("fixed_frame").as_string();

        // Lookup the transformation
        geometry_msgs::msg::TransformStamped transform_stamped = 
        tf_buffer_->lookupTransform(fixed_frame, clusters->header.frame_id, clusters->header.stamp, rclcpp::Duration(1, 0));

        // Clear measurements
        centroid_z_meas_.clear();
        clusters_.clear();
        measurements_= Eigen::Array<double, 2, Eigen::Dynamic>(2, 0);
        Eigen::Index size = 0;
        
        for (const auto& cluster : clusters->clusters)
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::fromROSMsg(cluster, *cloud);
            std::vector<Eigen::Vector3f> cluster_points;
            for (const auto& point : cloud->points)
            {
                geometry_msgs::msg::Point cluster_point;
                geometry_msgs::msg::Point transformed_point;
                cluster_point.x = point.x;
                cluster_point.y = point.y;
                cluster_point.z = point.z;
                tf2::doTransform(cluster_point, transformed_point, transform_stamped);
                Eigen::Vector3f point_3d;
                point_3d[0] = transformed_point.x;
                point_3d[1] = transformed_point.y;
                point_3d[2] = transformed_point.z;
                cluster_points.push_back(point_3d);
            }
            size++;
            clusters_.push_back(cluster_points);
        }
        measurements_.resize(2, size);

        // Transform PointCloud to vector of 2d points
        sensor_msgs::PointCloud2ConstIterator<float> iter_x(clusters->centroids, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(clusters->centroids, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z(clusters->centroids, "z");
        Eigen::Index i = 0;

        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
        {
            geometry_msgs::msg::PointStamped transformed_point;
            geometry_msgs::msg::PointStamped centroid_point;
            centroid_point.header = clusters->header;
            centroid_point.point.x = *iter_x;
            centroid_point.point.y = *iter_y;
            centroid_point.point.z = *iter_z;
            tf2::doTransform(centroid_point, transformed_point, transform_stamped);

            Eigen::Vector2d point_2d;
            point_2d[0] = transformed_point.point.x;
            point_2d[1] = transformed_point.point.y;

            measurements_.col(i) = point_2d;
            centroid_z_meas_.push_back(transformed_point.point.z);
            i++;
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
    double initial_existence_probability = get_parameter("initial_existence_probability").as_double();


    // Update tracks
    track_manager_.updateTracks(measurements_, centroid_z_meas_, clusters_, update_interval, confirmation_threshold, gate_threshold, min_gate_threshold, max_gate_threshold, prob_of_detection, prob_of_survival,clutter_intensity, initial_existence_probability);

    measurements_.resize(2, 0);
    centroid_z_meas_.clear();
    clusters_.clear();

    // Publish tracks to landmark server
    publish_landmarks(deletion_threshold);

    // Publish visualization parameters
    if (get_parameter("publish_visualization").as_bool())
    {
        publish_visualization_parameters(gate_threshold, min_gate_threshold, max_gate_threshold);
    }

    // delete tracks
    track_manager_.deleteTracks(deletion_threshold);
}

void TargetTrackingNode::publish_landmarks(double deletion_threshold) {
    vortex_msgs::msg::LandmarkArray landmark_array;
    RCLCPP_INFO(this->get_logger(), "Number of tracks: %lu", track_manager_.getTracks().size());
    for(const auto& track : track_manager_.getTracks())
    {   
        RCLCPP_INFO(this->get_logger(), "Track existence probability: %f", track.existence_probability);
        // Skips unconfirmed tracks
        if(track.confirmed == false){
            continue;
        }
        RCLCPP_INFO(this->get_logger(), "Track id: %d", track.id);
        vortex_msgs::msg::Landmark landmark;

        // Sets landmark type
        landmark.landmark_type = vortex_msgs::msg::Landmark::NONE;

        // Sets landmark classification
        landmark.classification = vortex_msgs::msg::Landmark::UNKNOWN;

        // creates landmark message
        landmark.id = track.id;
        landmark.action = track.existence_probability < deletion_threshold ? 0 : track.action;
        landmark.odom.header.frame_id = get_parameter("fixed_frame").as_string();
        landmark.odom.header.stamp = this->get_clock()->now();

        landmark.odom.pose.pose.position.x = track.state.mean()(0);
        landmark.odom.pose.pose.position.y = track.state.mean()(1);
        landmark.odom.pose.pose.position.z = track.centroid_z_measurement;

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

        landmark.odom.child_frame_id = get_parameter("fixed_frame").as_string();

        shape_msgs::msg::SolidPrimitive cluster_polygon;
        cluster_polygon.type = shape_msgs::msg::SolidPrimitive::PRISM;
        cluster_polygon.polygon.points.resize(track.cluster.size());
        for (size_t i = 0; i < track.cluster.size(); ++i)
        {
            cluster_polygon.polygon.points[i].x = track.cluster[i](0);
            cluster_polygon.polygon.points[i].y = track.cluster[i](1);
            cluster_polygon.polygon.points[i].z = track.cluster[i](2);
        }
        landmark.shape = cluster_polygon;


        landmark_array.landmarks.push_back(landmark);
    }

    landmark_publisher_->publish(landmark_array);

    RCLCPP_INFO(this->get_logger(), "Published %lu tracks", landmark_array.landmarks.size());
}

void TargetTrackingNode::publish_visualization_parameters(double gate_threshold, double min_gate_threshold, double max_gate_threshold)
{
    vortex_msgs::msg::Parameter gate_threshold_msg;
    gate_threshold_msg.name = "gate_threshold";
    gate_threshold_msg.value = std::to_string(gate_threshold);

    vortex_msgs::msg::Parameter min_gate_threshold_msg;
    min_gate_threshold_msg.name = "min_gate_threshold";
    min_gate_threshold_msg.value = std::to_string(min_gate_threshold);

    vortex_msgs::msg::Parameter max_gate_threshold_msg;
    max_gate_threshold_msg.name = "max_gate_threshold";
    max_gate_threshold_msg.value = std::to_string(max_gate_threshold);

    vortex_msgs::msg::ParameterArray visualization_params;
    visualization_params.parameters.push_back(gate_threshold_msg);
    visualization_params.parameters.push_back(min_gate_threshold_msg);
    visualization_params.parameters.push_back(max_gate_threshold_msg);

    visualization_publisher_->publish(visualization_params);
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
