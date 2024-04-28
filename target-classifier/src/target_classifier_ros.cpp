#include <target_classifier/target_classifier_ros.hpp>

using std::placeholders::_1;


TargetClassifierNode::TargetClassifierNode(const rclcpp::NodeOptions& options)
    : Node("target_classifier_node", options)
{
    // Configure default topics for subscribing/publishing
    declare_parameter<std::string>("topic_landmarks_in", "target_tracking/landmarks");
    declare_parameter<std::string>("topic_detections_in", "image_detections");
    declare_parameter<std::string>("topic_camera_parameter_", "zed_camera/camera_matrix");
    declare_parameter<std::string>("topic_landmarks_out", "target_classifier/bouys");
 
    // Parameters for the intrinsic camera matrix
    declare_parameter<std::vector<double>>("camera_intrinsic",{262.0032958984375, 262.0032958984375, 316.2785949707031, 180.38784790039062});
    declare_parameter<std::string>("subs.camera_info_topic", "/zed/zed_node/left/camera_info");

    declare_parameter<std::string>("camera_frame", "zed_left_camera_frame");
    declare_parameter<std::string>("world_frame", "base_link");

    declare_parameter<int>("update_interval_ms", 200);

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
    landmark_pixel_publisher_ = this->create_publisher<foxglove_msgs::msg::ImageAnnotations>("landmark_pixel", qos);

    // Set timer
    int update_interval = get_parameter("update_interval_ms").as_int();
    timer_ = this->create_wall_timer(std::chrono::milliseconds(update_interval), std::bind(&TargetClassifierNode::timer_callback, this));

    // Initialize transform listener
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Update camera matrix
    update_camera_matrix();

    // Camera topic
    camera_info_topic_ = get_parameter("subs.camera_info_topic").as_string();
}

void TargetClassifierNode::timer_callback()
{   
    // Exit early if either pointer is nullptr
    if (image_detections_ == nullptr || landmarks_ == nullptr) {
    return;
    }
    // Check if landmarks and image detections are empty
    if (image_detections_->detections.empty() || landmarks_->landmarks.empty()) {
        return;
    }

    vortex_msgs::msg::LandmarkArray classified_landmarks;

    // Find the transformation from world to camera frame
    try {
        std::string camera_frame = get_parameter("camera_frame").as_string();
        std::string world_frame = get_parameter("world_frame").as_string();

        // Lookup the transformation from world to camera frame
        geometry_msgs::msg::TransformStamped transform_stamped = 
            tf_buffer_->lookupTransform(world_frame, camera_frame, rclcpp::Time(0), rclcpp::Duration(1, 0));
    
        std::vector<Eigen::Vector3d> pixel_coordinates = get_pixel_coordinates(transform_stamped);
        Eigen::MatrixXd reward_matrix = generate_reward_matrix(image_detections_, pixel_coordinates);

        Eigen::VectorXi assignment = auction_algorithm(reward_matrix);

        // Find the assigned landmarks and update the classification
        for (int i = 0; i < assignment.size(); i++) {
            if (assignment(i) < landmarks_->landmarks.size()){
                vortex_msgs::msg::Landmark landmark = landmarks_->landmarks[assignment(i)];
                landmark.classification = image_detections_->detections[i].class_id;
                landmark.action = 2;
                classified_landmarks.landmarks.push_back(landmark);
            }
        }

        // Publish the pixel coordinates of the landmarks
        visualize_landmark_pixels(pixel_coordinates);
    } 
    catch (tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "Could not transform landmarks %s", ex.what());
    }

    // Clear the detections and landmarks
    if (image_detections_ != nullptr) {
        image_detections_->detections.clear();
    }
    if (landmarks_ != nullptr) {
        landmarks_->landmarks.clear();
    }

    // Publish the classified landmarks
    landmark_publisher_->publish(classified_landmarks);
}   

geometry_msgs::msg::TransformStamped TargetClassifierNode::z_up_to_z_fwd(
    geometry_msgs::msg::TransformStamped transform_stamped
) 
{   
    // Change cooridnate stystem from z up xfwd to z fwd x right
    tf2::Quaternion q_y, q_z;
    q_y.setRPY(0, -M_PI/2, 0);
    q_z.setRPY(0, 0, M_PI/2);
    tf2::Quaternion q_orig;
    tf2::fromMsg(transform_stamped.transform.rotation, q_orig);
    tf2::Quaternion q_new = q_z * q_y * q_orig;
    transform_stamped.transform.rotation = tf2::toMsg(q_new);
    return transform_stamped;
}

std::vector<Eigen::Vector3d> TargetClassifierNode::get_pixel_coordinates(
    const geometry_msgs::msg::TransformStamped transform
)
{
    std::vector<Eigen::Vector3d> pixel_coordinates;

    for (auto& landmark : landmarks_->landmarks) {
        
        // Eigen::Vector3d landmark_camera;
        geometry_msgs::msg::PointStamped landmark_world;
        landmark_world.point.x = landmark.odom.pose.pose.position.x;
        landmark_world.point.y = landmark.odom.pose.pose.position.y;
        landmark_world.point.z = landmark.odom.pose.pose.position.z;
        
        geometry_msgs::msg::PointStamped landmark_camera;
        tf2::doTransform(landmark_world, landmark_camera, transform);

        // Check if the landmark is behind the camera
        if (landmark_camera.point.z <= 0) {
            continue;
        }

        // Normalized image coordinates
        Eigen::Vector3d normalized_landmark_camera(landmark_camera.point.x/landmark_camera.point.z, landmark_camera.point.y/landmark_camera.point.z, 1);

        // Project the landmark to the image plane
        Eigen::Vector3d landmark_pixel = camera_matrix_ * normalized_landmark_camera;

        pixel_coordinates.push_back(landmark_pixel);
    }
    return pixel_coordinates;
}

void TargetClassifierNode::visualize_landmark_pixels(const std::vector<Eigen::Vector3d>& landmark_pixels)
{
    foxglove_msgs::msg::ImageAnnotations image_annotations;
    foxglove_msgs::msg::PointsAnnotation point_annotations;
    point_annotations.timestamp = this->now();
    point_annotations.type = 1;
    foxglove_msgs::msg::Color color;
    color.r = 255;
    color.g = 0;
    color.b = 0;
    color.a = 1;
    point_annotations.outline_color = color;
    point_annotations.fill_color = color;
    point_annotations.thickness = 5;
    
    for (auto& landmark_pixel : landmark_pixels) {
        foxglove_msgs::msg::Point2 point;
        point.x = landmark_pixel(0);
        point.y = landmark_pixel(1);
        
        point_annotations.outline_colors.push_back(color);
        point_annotations.points.push_back(point);
    }

    image_annotations.points.push_back(point_annotations);
    landmark_pixel_publisher_->publish(image_annotations);
}

void TargetClassifierNode::landmark_callback(const vortex_msgs::msg::LandmarkArray::SharedPtr landmarks)
{
    landmarks_ = landmarks;
}

void TargetClassifierNode::image_detection_callback(const vortex_msgs::msg::DetectionArray::SharedPtr image_detections)
{
    image_detections_ = image_detections;
}

void TargetClassifierNode::subscribeToCameraTopics() {
    std::string camera_info_topic = get_parameter("subs.camera_info_topic").as_string();
    if (camera_info_topic_ != camera_info_topic) {
        camera_info_subscriber_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            camera_info_topic, 10, std::bind(&TargetClassifierNode::cameraInfoCallback, this, _1));
        camera_info_topic_ = camera_info_topic;
        RCLCPP_INFO(this->get_logger(), "Subscribed to camera info topic: %s", camera_info_topic.c_str());
    }
}

void TargetClassifierNode::cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr camera_info)
{
    if (camera_info->k[0] == 0) {
        RCLCPP_WARN(this->get_logger(), "Camera matrix not available");
        return;
    }
    // Updates the camera info parameters
    rclcpp::Parameter camera_intrinsic("camera_intrinsic", std::vector<double>{camera_info->k[0], camera_info->k[4], camera_info->k[2], camera_info->k[5]});
    this->set_parameter(camera_intrinsic);
    // this->set_parameter({camera_intrinsic});

    // Update the camera matrix
    update_camera_matrix();
}


void TargetClassifierNode::update_camera_matrix()
{
    // Read camera matrix parameters
    std::vector<double> camera_intrinsic = get_parameter("camera_intrinsic").as_double_array();

    // Set camera matrix
    camera_matrix_ << camera_intrinsic[0], 0, camera_intrinsic[2],
                      0, camera_intrinsic[1], camera_intrinsic[3],
                      0, 0, 1;
}

Eigen::MatrixXd TargetClassifierNode::generate_reward_matrix(const vortex_msgs::msg::DetectionArray::SharedPtr detections, const std::vector<Eigen::Vector3d>& landmark_pixels)
{
    // Initialize the reward matrix
    int num_detections = detections->detections.size();
    int num_landmarks = landmark_pixels.size();
    Eigen::MatrixXd reward_matrix(num_detections + num_landmarks, num_landmarks);
    reward_matrix.fill(-1000.0);

    for (int i = 0; i < num_detections; i++) {
        Eigen::Vector2d detection_center(detections->detections[i].bbox.center.position.x, detections->detections[i].bbox.center.position.y);
        int detection_width = detections->detections[i].bbox.size.x;
        int detection_height = detections->detections[i].bbox.size.y;

        for (int j = 0; j < num_landmarks; j++) {
            if (landmark_pixels[j](0) > detection_center(0) - detection_width / 2 &&
                landmark_pixels[j](0) < detection_center(0) + detection_width / 2 &&
                landmark_pixels[j](1) > detection_center(1) - detection_height / 2 &&
                landmark_pixels[j](1) < detection_center(1) + detection_height / 2) {
                    double x = landmark_pixels[j](0) - detection_center(0);
                    double y = landmark_pixels[j](1) - detection_center(1);
                    double reward = 1/sqrt(x*x + y*y);
                    reward_matrix(i, j) = reward;
            }

            if (j == i) {
                reward_matrix(i + num_landmarks, j) = -300;
            }
        }
    }
    return reward_matrix;
}