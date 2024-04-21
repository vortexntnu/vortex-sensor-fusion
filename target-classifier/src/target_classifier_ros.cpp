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

    // Initialize transform listener
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Update camera matrix
    update_camera_matrix();

    // Camera topic
    camera_info_topic_ = get_parameter("subs.camera_info_topic").as_string();
}

void TargetClassifierNode::landmark_callback(const vortex_msgs::msg::LandmarkArray::SharedPtr landmarks)
{
    // Check if landmarks and image detections are empty
    // if (image_detections_->detections.empty() || landmarks->landmarks.empty()) {
    //     return;
    // }
    std::cout << "Landmarks received" << std::endl;

    vortex_msgs::msg::LandmarkArray classified_landmarks;
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

    // Transform the landmarks to the camera frame
    try {
        std::string camera_frame = get_parameter("camera_frame").as_string();
        std::string world_frame = get_parameter("world_frame").as_string();

        // Lookup the transformation from world to camera frame
        geometry_msgs::msg::TransformStamped transform_stamped = 
            tf_buffer_->lookupTransform(world_frame, camera_frame, rclcpp::Time(0), rclcpp::Duration(1, 0));

        // Change cooridnate stystem from z up xfwd to z fwd x right
        tf2::Quaternion q_y, q_z;
        q_y.setRPY(0, -M_PI/2, 0);  // 90-degree rotation about y-axis
        q_z.setRPY(0, 0, M_PI/2); // - 90-degree rotation about z-axis
        tf2::Quaternion q_orig;
        tf2::fromMsg(transform_stamped.transform.rotation, q_orig);
        tf2::Quaternion q_new = q_z * q_y * q_orig;
        // Update the rotation component of the transform
        transform_stamped.transform.rotation = tf2::toMsg(q_new);




        for (auto& landmark : landmarks->landmarks) {
            
            // Eigen::Vector3d landmark_camera;
            geometry_msgs::msg::PointStamped landmark_world;
            landmark_world.point.x = landmark.odom.pose.pose.position.x;
            landmark_world.point.y = landmark.odom.pose.pose.position.y;
            landmark_world.point.z = landmark.odom.pose.pose.position.z - 0.5;
            
            geometry_msgs::msg::PointStamped landmark_camera;
            tf2::doTransform(landmark_world, landmark_camera, transform_stamped);

            // Check if the landmark is behind the camera
            if (landmark_camera.point.z <= 0) {
                std::cout << "Point behind camera" << std::endl;   
                std::cout << "Landmark camera: " << landmark_camera.point.x << ", " << landmark_camera.point.y << ", " << landmark_camera.point.z << std::endl;
                continue;
            }

            std::cout << "Landmark camera: " << landmark_camera.point.x << ", " << landmark_camera.point.y << ", " << landmark_camera.point.z << std::endl;

            // Normalized image coordinates
            Eigen::Vector3d normalized_landmark_camera(landmark_camera.point.x/landmark_camera.point.z, landmark_camera.point.y/landmark_camera.point.z, 1);

            // Project the landmark to the image plane
            Eigen::Vector3d landmark_pixel = camera_matrix_ * normalized_landmark_camera;

            foxglove_msgs::msg::Point2 point;
            point.x = landmark_pixel(0);
            point.y = landmark_pixel(1);
            
            std::cout << "Landmark pixel: " << point.x << ", " << point.y << std::endl;
            point_annotations.outline_colors.push_back(color);
            point_annotations.points.push_back(point);
            

            // for (auto& detection : image_detections_->detections) {
            //     Eigen::Vector2d detection_center(detection.bbox.center.position.x, detection.bbox.center.position.y);
            //     int detection_width = detection.bbox.size.x;
            //     int detection_height = detection.bbox.size.y;
            //     if (landmark_pixel(0) > detection_center(0) - detection_width / 2 &&
            //         landmark_pixel(0) < detection_center(0) + detection_width / 2 &&
            //         landmark_pixel(1) > detection_center(1) - detection_height / 2 &&
            //         landmark_pixel(1) < detection_center(1) + detection_height / 2) {
                    
            //         // Update the landmark classification
            //         landmark.classification = detection.classification;
            //         landmark.action = 2;
            //         classified_landmarks.landmarks.push_back(landmark);
            //         break;
            //     }
            // }
        }

    } catch (tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "Could not transform landmarks %s", ex.what());
    }


    // Clear the detections
    if (image_detections_ != nullptr) {
        image_detections_->detections.clear();
    }

    // Publish the classified landmarks
    landmark_publisher_->publish(classified_landmarks);

    // Publish the pixel coordinates of the landmarks
    image_annotations.points.push_back(point_annotations);
    landmark_pixel_publisher_->publish(image_annotations);
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
