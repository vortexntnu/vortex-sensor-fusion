#include <target_classifier/target_classifier_ros.hpp>

using std::placeholders::_1;


TargetClassifierNode::TargetClassifierNode(const rclcpp::NodeOptions& options)
    : Node("target_classifier_node", options)
{
    // Configure default topics for subscribing/publishing
    declare_parameter<std::string>("topic_landmarks_in", "landmark_server/bouys");
    declare_parameter<std::string>("topic_detections_in", "image_detections");
    declare_parameter<std::string>("topic_landmarks_out", "target_classifier/bouys");
 
    // Parameters for the intrinsic camera matrix
    declare_parameter<double>("fx", 1500.0);
    declare_parameter<double>("fy", 1500.0);
    declare_parameter<double>("u0", 640.0);
    declare_parameter<double>("v0", 512.0);

    declare_parameter<std::string>("camera_frame", "camera_frame");
    declare_parameter<std::string>("world_frame", "world_frame");

    // Read parameters for subscriber and publisher
    param_topic_landmarks_in_ = get_parameter("topic_landmarks_in").as_string();
    param_topic_detections_in_ = get_parameter("topic_detections_in").as_string();
    param_topic_landmarks_out_ = get_parameter("topic_landmarks_out").as_string();

    // Set QoS profile
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    qos_profile.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 1), qos_profile);

    // Subscribe to topic
    landmark_subscriber_ = this->create_subscription<vortex_msgs::msg::LandmarkArray>(
        param_topic_landmarks_in_, qos, std::bind(&TargetClassifierNode::landmark_callback, this, _1));

    detection_subscriber_ = this->create_subscription<vortex_msgs::msg::DetectionArray>(
        param_topic_detections_in_, qos, std::bind(&TargetClassifierNode::image_detection_callback, this, _1));

    // Publish landmarks
    landmark_publisher_ = this->create_publisher<vortex_msgs::msg::LandmarkArray>(param_topic_landmarks_out_, qos);

    // Initialize transform listener
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Update camera matrix
    update_camera_matrix();
}

void TargetClassifierNode::landmark_callback(const vortex_msgs::msg::LandmarkArray::SharedPtr landmarks)
{
    vortex_msgs::msg::LandmarkArray classified_landmarks;

    // Transform the landmarks to the camera frame
    try {
        std::string camera_frame = get_parameter("camera_frame").as_string();
        std::string world_frame = get_parameter("world_frame").as_string();

        // Lookup the transformation from world to camera frame
        geometry_msgs::msg::TransformStamped transform_stamped = 
            tf_buffer_->lookupTransform(world_frame, camera_frame, rclcpp::Time(0), rclcpp::Duration(1, 0));


        for (auto& landmark : landmarks->landmarks) {
            
            // Eigen::Vector3d landmark_camera;
            geometry_msgs::msg::PointStamped landmark_world;
            landmark_world.point.x = landmark.odom.pose.pose.position.x;
            landmark_world.point.y = landmark.odom.pose.pose.position.y;
            landmark_world.point.z = landmark.odom.pose.pose.position.z;
            
            geometry_msgs::msg::PointStamped landmark_camera;
            tf2::doTransform(landmark_world, landmark_camera, transform_stamped);

            // Normalized image coordinates
            Eigen::Vector3d normalized_landmark_camera(landmark_camera.point.x/landmark_camera.point.z, landmark_camera.point.y/landmark_camera.point.z, 1);

            // Project the landmark to the image plane
            Eigen::Vector3d landmark_pixel = camera_matrix_ * normalized_landmark_camera;

            for (auto& detection : image_detections_->detections) {
                Eigen::Vector2d detection_center(detection.bbox.center.position.x, detection.bbox.center.position.y);
                int detection_width = detection.bbox.size.x;
                int detection_height = detection.bbox.size.y;
                if (landmark_pixel(0) > detection_center(0) - detection_width / 2 &&
                    landmark_pixel(0) < detection_center(0) + detection_width / 2 &&
                    landmark_pixel(1) > detection_center(1) - detection_height / 2 &&
                    landmark_pixel(1) < detection_center(1) + detection_height / 2) {
                    landmark.classification = detection.classification;
                    landmark.action = 2;
                    classified_landmarks.landmarks.push_back(landmark);
                    break;
                }
            }
        }

    } catch (tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "Could not transform landmarks %s", ex.what());
    }
    // Publish the classified landmarks
    landmark_publisher_->publish(classified_landmarks);
}

void TargetClassifierNode::image_detection_callback(const vortex_msgs::msg::DetectionArray::SharedPtr image_detections)
{
    image_detections_ = image_detections;
}

void TargetClassifierNode::update_camera_matrix()
{
    // Read camera matrix parameters
    double fx = get_parameter("fx").as_double();
    double fy = get_parameter("fy").as_double();
    double u0 = get_parameter("u0").as_double();
    double v0 = get_parameter("v0").as_double();

    // Set camera matrix
    camera_matrix_ << fx, 0, u0,
                     0, fy, v0,
                     0, 0, 1;
}
