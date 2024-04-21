#define BOOST_BIND_NO_PLACEHOLDERS

#include <pcl_detector/pcl_detector_ros.hpp>
#include <pcl_detector/geometry_processor.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <vector>
#include <Eigen/Dense>
#include <algorithm>

using std::placeholders::_1;

namespace pcl_detector {

// Constructor for the PclDetectorNode class
PclDetectorNode::PclDetectorNode(const rclcpp::NodeOptions& options) : Node("pcl_detector_node", options) 
{
  // Configure default topics for subscribing/publishing
  declare_parameter<std::string>("topic_pointcloud_in", "ouster/points");
  declare_parameter<std::string>("topic_pointcloud_out", "lidar/centroids");

  // Read parameters from YAML file or set default values
  declare_parameter<std::string>("detector", "euclidean");
  declare_parameter<float>("dbscan.epsilon", 2.0);
  declare_parameter<int>("dbscan.min_points", 20);
  declare_parameter<float>("euclidean.cluster_tolerance", 1.0);
  declare_parameter<int>("euclidean.min_points", 25);


  declare_parameter<float>("processor.voxel_leaf_size", 0.1);
  declare_parameter<float>("processor.model_thresh", 0.5);
  declare_parameter<int>("processor.model_iterations", 10);
  declare_parameter<float>("processor.prev_line_thresh", 0.5);
  declare_parameter<float>("processor.project_thresh", 0.5);
  declare_parameter<float>("processor.wall_neighbour_dist", 0.5);
  declare_parameter<int>("processor.wall_min_points", 50);
  declare_parameter<float>("processor.wall_min_length", 3.0);
  declare_parameter<float>("processor.wall_merge_dist", 10.0);

  declare_parameter<bool>("transform_lines", true);
  declare_parameter<int>("transform_timeout_nsec", 50000000);
  declare_parameter<int>("prev_line_min_inliers", 50);
  declare_parameter<int>("new_lines", 1);



  param_topic_pointcloud_in_ = get_parameter("topic_pointcloud_in").as_string();
  param_topic_pointcloud_out_ = get_parameter("topic_pointcloud_out").as_string();

  // Define the quality of service profile for publisher and subscriber
  rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
  qos_profile.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
  auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 1), qos_profile);
  
  publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(param_topic_pointcloud_out_, qos);
  poly_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("pcl/land_poly", qos);
  subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    param_topic_pointcloud_in_, qos, std::bind(&PclDetectorNode::topic_callback, this, _1));
    pose_array_publisher_ = this->create_publisher<geometry_msgs::msg::PoseArray>("wall_poses", qos);
    line_publisher = this->create_publisher<visualization_msgs::msg::MarkerArray>("line_marker_array", qos);
    tf_line_publisher = this->create_publisher<visualization_msgs::msg::MarkerArray>("tf_line_marker_array", qos);

    wall_publisher = this->create_publisher<visualization_msgs::msg::MarkerArray>("wall_marker_array", qos);
    wall_cone_publisher = this->create_publisher<visualization_msgs::msg::MarkerArray>("wall_marker_cone", qos);


  // Define a handle for validating parameters during runtime  
  on_set_callback_handle_ = add_on_set_parameters_callback(std::bind(&PclDetectorNode::parametersCallback, this, std::placeholders::_1));
  
  // Initialize the detector with configured parameters
  std::string detector;
  get_parameter<std::string>("detector", detector);
  detector_ = initialize_detector(detector);

  // Initialize the PclProcessor
  float voxel_leaf_size = this->get_parameter("processor.voxel_leaf_size").as_double();
  float model_thresh = this->get_parameter("processor.model_thresh").as_double();
  int model_iterations = this->get_parameter("processor.model_iterations").as_int();
  float prev_line_thresh = this->get_parameter("processor.prev_line_thresh").as_double();
  float project_thresh = this->get_parameter("processor.project_thresh").as_double();
  float wall_neighbour_dist = this->get_parameter("processor.wall_neighbour_dist").as_double();
  int wall_min_points = this->get_parameter("processor.wall_min_points").as_int();
  float wall_min_length = this->get_parameter("processor.wall_min_length").as_double();
  float wall_merge_dist = this->get_parameter("processor.wall_merge_dist").as_double();


  processor_ = std::make_unique<PclProcessor>(voxel_leaf_size, model_thresh, model_iterations, prev_line_thresh, project_thresh, wall_neighbour_dist, wall_min_points, wall_min_length, wall_merge_dist);

  // Initialize transform listener
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    land_masker_.set_polygon();

}

// Callback function for parameter changes
rcl_interfaces::msg::SetParametersResult PclDetectorNode::parametersCallback(const std::vector<rclcpp::Parameter> &parameters) {
    
    rcl_interfaces::msg::SetParametersResult result;

    // Checks if the detector parameter is valid, if result.successful is false, the parameter is not set
    for (const auto &parameter : parameters) {
        if (parameter.get_name() == "detector") {
            if (detector_type.count(parameter.as_string()) == 0) {
                result.successful = false;
                return result;
            }
        }
    }
    
    parameters_changed_ = true;
    result.successful = true;
    result.reason = "success";
    return result;
}

// Initialize the selected detector based on the parameter
std::unique_ptr<IPclDetector> PclDetectorNode::initialize_detector(std::string detector)
{
    std::unique_ptr<IPclDetector> configured_detector = nullptr;

    switch (detector_type[detector]) {
        case DetectorType::Euclidean: {
            float cluster_tolerance;
            get_parameter<float>("euclidean.cluster_tolerance", cluster_tolerance);
            int min_points;
            get_parameter<int>("euclidean.min_points", min_points);
            configured_detector = std::make_unique<EuclideanClusteringDetector>(cluster_tolerance, min_points);
            RCLCPP_INFO_STREAM(this->get_logger(), "Euclidean detector initialized with cluster tolerance = " << cluster_tolerance << " and min_points = " << min_points);
            break;
        }

        case DetectorType::DBSCAN: {
            float eps; 
            get_parameter<float>("dbscan.epsilon", eps);
            int min_points;
            get_parameter<int>("dbscan.min_points", min_points);
            configured_detector = std::make_unique<DBSCANDetector>(eps, min_points);
            RCLCPP_INFO_STREAM(this->get_logger(), "DBSCAN detector initialized with eps = " << eps << " and min_points = " << min_points);
            break;
        }

        default: {
            RCLCPP_ERROR(this->get_logger(), "Invalid pcl detector specified! Shutting down...");
            rclcpp::shutdown();
        }
    }

    return configured_detector;
}

geometry_msgs::msg::PoseArray PclDetectorNode::getWallPoses(std::vector<pcl::PointXYZ> wall_poses)
{
    geometry_msgs::msg::PoseArray pose_array;
    for(auto point : wall_poses){
        geometry_msgs::msg::Pose pose;
        pose.position.x = point.x;
        pose.position.y = point.y;
        pose.position.z = 0.0;
        pose.orientation.x = 0.0;
        pose.orientation.y = 0.0;
        pose.orientation.z = 0.0;
        pose.orientation.w = 1.0;
        pose_array.poses.push_back(pose);
    }
    return pose_array;
}

void PclDetectorNode::transformLines(const sensor_msgs::msg::PointCloud2::SharedPtr& cloud_msg, std::vector<Eigen::VectorXf>& prev_lines)
{
    // Calculate the transformation between the current and previous point cloud
    auto [translation, rotation] = calculateTransformation(cloud_msg);

    // Transform the previously detected lines to the current frame
    GeometryProcessor::transformLines(prev_lines, translation, rotation);
}

std::tuple<Eigen::Vector3f, Eigen::Quaternionf> PclDetectorNode::calculateTransformation(const sensor_msgs::msg::PointCloud2::SharedPtr& cloud_msg) {
    std::string fixed_frame = "world_frame";
    geometry_msgs::msg::TransformStamped current_transform;
    static geometry_msgs::msg::TransformStamped prev_transform;
    static bool isFirstRun = true; // Tracks the first run or successful retrieval.
    int transform_timeout_nsec = this->get_parameter("transform_timeout_nsec").as_int();

    try {
        // Attempt to retrieve the transform, timeouted at 50ms
        current_transform = tf_buffer_->lookupTransform(fixed_frame, cloud_msg->header.frame_id, cloud_msg->header.stamp, rclcpp::Duration(0, transform_timeout_nsec));
        
        // If it's not the first run and we successfully get the transform:
        if (!isFirstRun) {
            Eigen::Vector3f translation(
                current_transform.transform.translation.x - prev_transform.transform.translation.x,
                current_transform.transform.translation.y - prev_transform.transform.translation.y,
                current_transform.transform.translation.z - prev_transform.transform.translation.z);

            tf2::Quaternion q_current, q_last, q_delta;
            tf2::fromMsg(current_transform.transform.rotation, q_current);
            tf2::fromMsg(prev_transform.transform.rotation, q_last);
            q_delta = q_last.inverse() * q_current;
            Eigen::Quaternionf rotation(q_delta.w(), q_delta.x(), q_delta.y(), q_delta.z());

            prev_transform = current_transform; // Update prev_transform for next time

            return std::make_tuple(translation, rotation);
        } else {
            // It's the first successful retrieval; just update the flag and prev_transform.
            isFirstRun = false;
            prev_transform = current_transform;
            // Return a default or neutral transformation indicating no movement.
            return std::make_tuple(Eigen::Vector3f(0, 0, 0), Eigen::Quaternionf(1, 0, 0, 0));
        }
    } catch (tf2::TransformException &ex) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Could not retrieve transform for previous lines: %s", ex.what());
        // In case of failure, return a default transformation or indicate the error appropriately.
        isFirstRun = false; // Ensure we don't treat the next run as the first run.
        return std::make_tuple(Eigen::Vector3f(0, 0, 0), Eigen::Quaternionf(1, 0, 0, 0));
    }
}


void PclDetectorNode::filterLines(std::vector<LineData>& linesData) {
    // Create polygons for wall on each line
    std::vector<std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>> polygons;
    for (size_t i = 0; i < linesData.size(); ++i) {
        for (size_t j = 0; j < linesData[i].wall_poses.size(); j += 2) {
            auto p1 = linesData[i].wall_poses[j];
            auto p2 = linesData[i].wall_poses[j + 1];
            auto polygon = processor_->createPolygon(p1, p2);
            polygons.push_back(polygon);
        }
    }

    // Check if the centre of walls are inside the polygons
    size_t wall_number = 0;
    for (size_t i = 0; i < linesData.size(); ++i) {
        for (size_t j = 0; j < linesData[i].wall_poses.size(); j += 2) {
            pcl::PointXYZ wall_center;
            wall_center.x = (linesData[i].wall_poses[j].x + linesData[i].wall_poses[j + 1].x) / 2;
            wall_center.y = (linesData[i].wall_poses[j].y + linesData[i].wall_poses[j + 1].y) / 2;
            
            for (size_t k = 0; k < polygons.size(); ++k) {
                if(k == wall_number) continue; // Skip the same wall (polygon)
                if (processor_->isXYPointIn2DXYPolygon(wall_center, *polygons[k])) {
                    linesData[i].wallsActive[j / 2] = false;
                    break;
                }
            }
            wall_number++;
        }
    }
    
    
    for (auto& lineData : linesData) {
        std::vector<pcl::PointXYZ> activeWallPoses; // To store poses of active walls

        // Process each wall in the current line
        for (size_t j = 0; j < lineData.wall_poses.size(); j += 2) {
            if (!lineData.wallsActive[j / 2]) continue; // Skip walls marked as inactive

            // For active walls, add both start and end points to activeWallPoses
            activeWallPoses.push_back(lineData.wall_poses[j]);
            activeWallPoses.push_back(lineData.wall_poses[j + 1]);
        }

        // Update the line's wall poses with only the active walls
        lineData.wall_poses = std::move(activeWallPoses);
    }

    // Remove lines from linesData that have no walls left
    linesData.erase(std::remove_if(linesData.begin(), linesData.end(), 
                                   [](const LineData& ld) { return ld.wall_poses.empty(); }),
                    linesData.end());

}

void PclDetectorNode::processWalls(const std::vector<pcl::PointXYZ>& walls_poses,
    std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>& cartesian_cloud){
    auto ros_wall_poses = getWallPoses(walls_poses);
    wall_poses_.poses.insert(wall_poses_.poses.end(), ros_wall_poses.poses.begin(), ros_wall_poses.poses.end());

    
    processor_->getPointsBehindWalls(cartesian_cloud, walls_poses, indices_to_remove_);

    processor_->extractPoints(cartesian_cloud, indices_to_remove_);

}

// Callback function for processing incoming point cloud messages
void PclDetectorNode::topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg)
{
    // Check if parameters have changed
    if (parameters_changed_) {
        std::string detector;
        get_parameter<std::string>("detector", detector);
        
        detector_ = initialize_detector(detector);
        parameters_changed_ = false;

        float voxel_leaf_size = this->get_parameter("processor.voxel_leaf_size").as_double();
        float model_thresh = this->get_parameter("processor.model_thresh").as_double();
        int model_iterations = this->get_parameter("processor.model_iterations").as_int();
        float prev_line_thresh = this->get_parameter("processor.prev_line_thresh").as_double();
        float project_thresh = this->get_parameter("processor.project_thresh").as_double();
        float wall_neighbour_dist = this->get_parameter("processor.wall_neighbour_dist").as_double();
        int wall_min_points = this->get_parameter("processor.wall_min_points").as_int();
        float wall_min_length = this->get_parameter("processor.wall_min_length").as_double();
        float wall_merge_dist = this->get_parameter("processor.wall_merge_dist").as_double();

        processor_ = std::make_unique<PclProcessor>(voxel_leaf_size, model_thresh, model_iterations, prev_line_thresh, project_thresh, wall_neighbour_dist, wall_min_points, wall_min_length, wall_merge_dist);

    }
    

    // Converts incoming ros-msg to a PointCloud 
    pcl::PointCloud<pcl::PointXYZ>::Ptr cartesian_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*cloud_msg, *cartesian_cloud);
    // RCLCPP_INFO_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Received PointCloud with " << cartesian_cloud->size() << " points");
    RCLCPP_INFO(this->get_logger(), "Received PointCloud with %zu points", cartesian_cloud->size());
    processor_->applyPassThrough(cartesian_cloud, "z", -2.0, 5.0);
    processor_->applyPassThrough(cartesian_cloud, "x", -50.0, 50.0);
    processor_->applyPassThrough(cartesian_cloud, "y", -50.0, 50.0);


    pcl::PointCloud<pcl::PointXYZ> land_mask = land_masker_.get_polygon();

    RCLCPP_INFO(this->get_logger(), "Land mask size: %zu", land_mask.size());
    sensor_msgs::msg::PointCloud2 land_mask_msg_out;
    try {       

            sensor_msgs::msg::PointCloud2 land_mask_msg;
            pcl::toROSMsg(land_mask, land_mask_msg);
            land_mask_msg.header = cloud_msg->header;
            std::string fixed_frame = "world";
            // The transform to apply, for example from
            geometry_msgs::msg::TransformStamped transform_stamped = tf_buffer_->lookupTransform(fixed_frame, cloud_msg->header.frame_id, cloud_msg->header.stamp, rclcpp::Duration(1, 0));

            tf2::doTransform(land_mask_msg, land_mask_msg_out, transform_stamped);
            RCLCPP_INFO(this->get_logger(), "Transformed land mask to world frame");
            
            // std::cout << "Transformed x: " << land_mask_msg_out.data[0] << std::endl;
            // std::cout << "Transformed y: " << land_mask_msg_out.data[1] << std::endl;
            // std::cout << "Transformed z: " << land_mask_msg_out.data[2] << std::endl;
            // std::cout << "Transform roll: " << land_mask_msg_out.data[3] << std::endl;
            // std::cout << "Transform pitch: " << land_mask_msg_out.data[4] << std::endl;
            // std::cout << "Transform yaw: " << land_mask_msg_out.data[5] << std::endl;


            
        } catch (tf2::TransformException &ex) {
            RCLCPP_ERROR(this->get_logger(), "Could not transform point cloud: %s", ex.what());
        }
    land_mask_msg_out.header = cloud_msg->header;
    if(land_mask_msg_out.data.size() > 0){
    poly_pub_->publish(land_mask_msg_out);
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr land_cloud_tf(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(land_mask_msg_out, *land_cloud_tf);
    RCLCPP_INFO(this->get_logger(), "Land mask size: %zu", land_cloud_tf->size());
    std::vector<int> land;
    land_masker_.get_land(cartesian_cloud, *land_cloud_tf, land);

    RCLCPP_INFO(this->get_logger(), "Land size: %zu", land.size());
    processor_->extractPoints(cartesian_cloud, land);

    // processor_->flattenPointCloud(*cartesian_cloud, 0.0);

    // processor_->applyVoxelGrid(cartesian_cloud);

    // bool transform_lines = this->get_parameter("transform_lines").as_bool();
    // if (transform_lines) {
    //   transformLines(cloud_msg, prev_lines_);
    //   publishtfLineMarkerArray(prev_lines_, cloud_msg->header.frame_id);
    // }
    // RCLCPP_INFO(this->get_logger(), "number of prev lines: %zu", prev_lines_.size());
  

    // int min_inliers = this->get_parameter("prev_line_min_inliers").as_int();

    // std::vector<LineData> linesData;
   
    // for (const auto& line : prev_lines_) {

    //     auto inliers = processor_->findInliers(line, cartesian_cloud);
    //     // RCLCPP_INFO_STREAM(this->get_logger(), "Inliers of prev line: " << inliers.size());
    //     if(inliers.size() > u_int16_t(min_inliers)){
    //         auto [optimized_coefficients, wall_poses] = processor_->getWalls(line, indices_to_remove_, inliers, cartesian_cloud);
    //         // Check if there are any walls detected on the line
    //         if(wall_poses.size() < 1){
    //             continue;
    //         }  
    //         // current_lines_.push_back(optimized_coefficients); // should be done after checking if there are walls
    //         LineData data{optimized_coefficients, wall_poses, std::vector<bool>(wall_poses.size() / 2, true)};
    //         linesData.push_back(data);
    //     }
    // }
    // // Filter lines to remove walls that are behind other walls and remove lines with no walls
    // filterLines(linesData);
    // std::vector<pcl::PointXYZ> walls_poses;

    // for (const auto& data : linesData) {
    //     walls_poses.insert(walls_poses.end(), data.wall_poses.begin(), data.wall_poses.end());
    //     current_lines_.push_back(data.coefficients);
    // }
    // RCLCPP_INFO(this->get_logger(), "number of current lines: %zu", current_lines_.size());
   
    // processWalls(walls_poses, cartesian_cloud);

    
    // std::vector<pcl::PointXYZ> new_walls_poses;
    // // Set how many new lines to find for each callback
    // int new_lines = this->get_parameter("new_lines").as_int();
    // for (int i = 0; i < new_lines; i++) {
    //     // can't compute line with less than 2 points
    //     if(cartesian_cloud->size() < 2){
    //         break;
    //     }
    //     auto [coefficients, inliers] = processor_->findLineWithMSAC(cartesian_cloud);
    //     // RCLCPP_INFO_STREAM(this->get_logger(), "Inliers of new line: " << inliers.size());
    //     auto [optimized_coefficients, wall_poses] = processor_->getWalls(coefficients, indices_to_remove_, inliers, cartesian_cloud);
    //     if (wall_poses.size() > 0) {
    //         current_lines_.push_back(optimized_coefficients);
    //         new_walls_poses.insert(new_walls_poses.end(), wall_poses.begin(), wall_poses.end());
    //     }
    // }
    // processWalls(new_walls_poses, cartesian_cloud);

    // wall_poses_.header = cloud_msg->header;
    // pose_array_publisher_->publish(wall_poses_);

    // publishLineMarkerArray(current_lines_, cloud_msg->header.frame_id);
    // publishWallMarkerArray(wall_poses_, cloud_msg->header.frame_id);
    // publishExtendedLinesFromOrigin(wall_poses_, cloud_msg->header.frame_id);

    // prev_lines_ = current_lines_;
    // current_lines_.clear();
    // wall_poses_.poses.clear();


    RCLCPP_INFO_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Filtered PointCloud to " << cartesian_cloud->size() << " points");

    // Finds clusters with the configured detector
    // pcl::PointCloud<pcl::PointXYZ> detections = detector_->get_detections(*cartesian_cloud);

    // if (detections.size() == 0) {
    //     RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "No clusters detected!");
    // }

    // Converts the clusters-PointCloud to an appropriate msg for publishing
    sensor_msgs::msg::PointCloud2 downsampled_cloud_msg;
    pcl::toROSMsg(*cartesian_cloud, downsampled_cloud_msg);
    downsampled_cloud_msg.header = cloud_msg->header;

    publisher_->publish(downsampled_cloud_msg);

    // Converts the clusters-PointCloud to an appropriate msg for publishing
    // sensor_msgs::msg::PointCloud2 centroids_cloud_msg;
    // pcl::toROSMsg(detections, centroids_cloud_msg);
    // centroids_cloud_msg.header = cloud_msg->header;

    // publisher_->publish(centroids_cloud_msg);
}































geometry_msgs::msg::Point PclDetectorNode::ExtendLineFromOriginToLength(const geometry_msgs::msg::Point& point, double length) {
    // Calculate the magnitude of the vector from origin to point
    double magnitude = sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
    
    // Normalize the vector, then scale it to the new length
    geometry_msgs::msg::Point extended_point;
    if (magnitude > 0) { // Avoid division by zero
        extended_point.x = (point.x / magnitude) * length;
        extended_point.y = (point.y / magnitude) * length;
        extended_point.z = (point.z / magnitude) * length;
    } else {
        // In case the point is at the origin, we return the original point to avoid division by zero
        extended_point = point;
    }
    
    return extended_point;
}

void PclDetectorNode::publishLineMarkerArray(const std::vector<Eigen::VectorXf>& lines, const std::string& frame_id) {
    visualization_msgs::msg::MarkerArray marker_array;
    int id = 0; // Unique ID for each marker

    for (const auto& line : lines) {
        visualization_msgs::msg::Marker line_marker;
        line_marker.header.frame_id = frame_id;
        line_marker.header.stamp = this->get_clock()->now();
        line_marker.ns = "line_markers";
        line_marker.id = id++;
        line_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        line_marker.action = visualization_msgs::msg::Marker::ADD;

        line_marker.scale.x = 0.2; // Line width

        // Color the line
        line_marker.color.r = 1.0;
        line_marker.color.g = 0.0;
        line_marker.color.b = 0.0;
        line_marker.color.a = 1.0; // Don't forget to set the alpha!

        // Assuming each line is defined by start and end points (adapt as needed)
        geometry_msgs::msg::Point start, end;
        start.x = line[0]; // Adapt based on your representation
        start.y = line[1];
        start.z = line[2];
        end.x = line[0]+line[3]*3;
        end.y = line[1]+line[4]*3;
        end.z = line[2]+line[5]*3;
        line_marker.points.push_back(start);
        line_marker.points.push_back(end);

        // Add the current line marker to the array
        marker_array.markers.push_back(line_marker);
    }

    // Publish the MarkerArray
    line_publisher->publish(marker_array);
}

void PclDetectorNode::publishtfLineMarkerArray(const std::vector<Eigen::VectorXf>& lines, const std::string& frame_id) {
    visualization_msgs::msg::MarkerArray marker_array;
    int id = 0; // Unique ID for each marker

    for (const auto& line : lines) {
        visualization_msgs::msg::Marker line_marker;
        line_marker.header.frame_id = frame_id;
        line_marker.header.stamp = this->get_clock()->now();
        line_marker.ns = "line_markers";
        line_marker.id = id++;
        line_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        line_marker.action = visualization_msgs::msg::Marker::ADD;

        line_marker.scale.x = 0.2; // Line width

        // Color the line
        line_marker.color.r = 0.0;
        line_marker.color.g = 0.0;
        line_marker.color.b = 1.0;
        line_marker.color.a = 1.0; // Don't forget to set the alpha!

        // Assuming each line is defined by start and end points (adapt as needed)
        geometry_msgs::msg::Point start, end;
        start.x = line[0]+line[3]*-3; // Adapt based on your representation
        start.y = line[1]+line[4]*-3;
        start.z = line[2]+line[5]*-3;
        end.x = line[0]+line[3]*3;
        end.y = line[1]+line[4]*3;
        end.z = line[2]+line[5]*3;
        line_marker.points.push_back(start);
        line_marker.points.push_back(end);

        // Add the current line marker to the array
        marker_array.markers.push_back(line_marker);
    }

    // Publish the MarkerArray
    tf_line_publisher->publish(marker_array);
}

void PclDetectorNode::publishExtendedLinesFromOrigin(const geometry_msgs::msg::PoseArray& pose_array, const std::string& frame_id) {
    visualization_msgs::msg::MarkerArray marker_array;
    int id = 0; // Unique ID for each marker

    for (size_t i = 0; i < pose_array.poses.size(); i += 2) {
        if (i + 1 >= pose_array.poses.size()) {
            break; // Prevent going out of bounds if there's an odd number of poses
        }
        auto& start_pose = pose_array.poses[i];
        auto& end_pose = pose_array.poses[i + 1];

        // Process start and end points to extend lines from origin
        geometry_msgs::msg::Point extended_point_1 = ExtendLineFromOriginToLength(start_pose.position, 100.0);
        geometry_msgs::msg::Point extended_point_2 = ExtendLineFromOriginToLength(end_pose.position, 100.0);


        // Store the extended points for further use
        std::vector<geometry_msgs::msg::Point> extended_points = {extended_point_1, extended_point_2};

        for (auto& extended_point : extended_points) {
            visualization_msgs::msg::Marker line_marker;
            line_marker.header.frame_id = frame_id;
            line_marker.header.stamp = this->get_clock()->now();
            line_marker.ns = "extended_lines";
            line_marker.id = id++;
            line_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
            line_marker.action = visualization_msgs::msg::Marker::ADD;

            line_marker.scale.x = 0.1; // Adjust line width as needed

            // Set the line color
            line_marker.color.r = 0.0;
            line_marker.color.g = 0.0;
            line_marker.color.b = 1.0;
            line_marker.color.a = 1.0;

            // Origin point
            geometry_msgs::msg::Point origin;
            origin.x = 0.0;
            origin.y = 0.0;
            origin.z = 0.0;

            // Define the line from origin through the point and extending out to the specified length
            line_marker.points.push_back(origin);
            line_marker.points.push_back(extended_point); // Extended point

            // Add the current line marker to the array
            marker_array.markers.push_back(line_marker);
        }
    }

    // Publish the MarkerArray
    wall_cone_publisher->publish(marker_array);
}
void PclDetectorNode::publishWallMarkerArray(const geometry_msgs::msg::PoseArray& pose_array, const std::string& frame_id) {
    visualization_msgs::msg::MarkerArray marker_array;
    int id = 0; // Unique ID for each marker

    for (size_t i = 0; i < pose_array.poses.size(); i += 2) {
        if (i + 1 >= pose_array.poses.size()) {
            break; // Prevent going out of bounds if there's an odd number of poses
        }
        auto& start_pose = pose_array.poses[i];
        auto& end_pose = pose_array.poses[i + 1];

        visualization_msgs::msg::Marker line_marker;
        line_marker.header.frame_id = frame_id;
        line_marker.header.stamp = this->get_clock()->now();
        line_marker.ns = "wall_markers";
        line_marker.id = id++;
        line_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        line_marker.action = visualization_msgs::msg::Marker::ADD;

        line_marker.scale.x = 0.25; // Line width

        // Color the line
        line_marker.color.r = 0.0;
        line_marker.color.g = 1.0;
        line_marker.color.b = 0.0;
        line_marker.color.a = 1.0;

        // Create points from poses
        geometry_msgs::msg::Point start, end;
        start.x = start_pose.position.x;
        start.y = start_pose.position.y;
        start.z = start_pose.position.z;
        end.x = end_pose.position.x;
        end.y = end_pose.position.y;
        end.z = end_pose.position.z;

        line_marker.points.push_back(start);
        line_marker.points.push_back(end);

        // Add the current line marker to the array
        marker_array.markers.push_back(line_marker);
    }

    // Publish the MarkerArray
    wall_publisher->publish(marker_array);
}

} // namespace pcl_detector
