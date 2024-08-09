#include <pcl_detector/pcl_detector_ros.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <thread>
#include <vector>
#include <Eigen/Dense>
#include <algorithm>
#include <chrono>

using std::placeholders::_1;

namespace pcl_detector {

// Constructor for the PclDetectorNode class
PclDetectorNode::PclDetectorNode(const rclcpp::NodeOptions& options) : Node("pcl_detector_node", options) 
{
  // Configure default topics for subscribing/publishing
  declare_parameter<std::string>("topic_pointcloud_in", "ouster/points");
  declare_parameter<std::string>("topic_landmask_in", "landmask");
  declare_parameter<std::string>("topic_walls_out", "wall_poses");
  declare_parameter<std::string>("topic_clusters_out", "vortex/clusters");
  

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

  declare_parameter<bool>("apply_landmask", false);
  declare_parameter<bool>("apply_voxelgrid", false);
  declare_parameter<bool>("map_to_grid", false);
  declare_parameter<int>("cell_inc_value", 5);
  declare_parameter<bool>("detect_lines", false);
  declare_parameter<std::string>("fixed_frame", "world_frame");
  declare_parameter<bool>("transform_lines", false);
  declare_parameter<int>("transform_timeout_nsec", 50000000);
  declare_parameter<int>("prev_line_min_inliers", 50);
  declare_parameter<int>("new_lines", 1);



  param_topic_pointcloud_in_ = get_parameter("topic_pointcloud_in").as_string();
  param_topic_landmask_in_ = get_parameter("topic_landmask_in").as_string();
  param_topic_walls_out_ = get_parameter("topic_walls_out").as_string();
  param_topic_clusters_out_ = get_parameter("topic_clusters_out").as_string();

  // Define the quality of service profile for publisher and subscriber
  rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
  qos_profile.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
  auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 1), qos_profile);

  rmw_qos_profile_t qos_profile_transient_local = rmw_qos_profile_parameters;
  qos_profile_transient_local.durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
  auto qos_transient_local = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile_transient_local.history, 1), qos_profile_transient_local);
  
  subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    param_topic_pointcloud_in_, qos, std::bind(&PclDetectorNode::topic_callback, this, _1));
  poly_sub_ = this->create_subscription<geometry_msgs::msg::PolygonStamped>(
    param_topic_landmask_in_, qos_transient_local, std::bind(&PclDetectorNode::land_poly_callback, this, _1));

  vortex_cluster_publisher_ = this->create_publisher<vortex_msgs::msg::Clusters>(param_topic_clusters_out_, qos);
  pose_array_publisher_ = this->create_publisher<geometry_msgs::msg::PoseArray>(param_topic_walls_out_, qos);
  grid_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("pcl_grid", qos);

  // Rest of publishers are for debugging/testing
  centroid_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("pcl/centroids", qos);
  poly_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("pcl/land_poly", qos);
  pre_wall_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("pcl/pre_wall", qos);
  after_wall_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("pcl/after_wall", qos);
  cluster_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("pcl/clusters", qos);
  convex_hull_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("pcl/concave_hulls", qos);
  line_publisher = this->create_publisher<visualization_msgs::msg::MarkerArray>("line_marker_array", qos);
  tf_line_publisher = this->create_publisher<visualization_msgs::msg::MarkerArray>("tf_line_marker_array", qos);

  wall_publisher = this->create_publisher<visualization_msgs::msg::MarkerArray>("wall_marker_array", qos);
  wall_cone_publisher = this->create_publisher<visualization_msgs::msg::MarkerArray>("wall_marker_cone", qos);


  // Define a handle for validating parameters during runtime  
  on_set_callback_handle_ = add_on_set_parameters_callback(std::bind(&PclDetectorNode::parameter_callback, this, std::placeholders::_1));
  
  // Initialize the detector with configured parameters
  std::string detector = this->get_parameter("detector").as_string();
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

  processor_ = std::make_unique<PclProcessor>(voxel_leaf_size, model_thresh, model_iterations, prev_line_thresh, project_thresh, wall_neighbour_dist, wall_min_points, wall_min_length);

  // Initialize transform listener
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    if(this->get_parameter("map_to_grid").as_bool())
    {
    grid_client_ = create_client<nav_msgs::srv::GetMap>("get_map");
    std::thread(&PclDetectorNode::get_grid, this).detach();
    }
}

void PclDetectorNode::land_poly_callback(const geometry_msgs::msg::PolygonStamped::SharedPtr msg)
{
    if(land_mask_set_ || msg->polygon.points.size() < 3)
    {
        return;
        RCLCPP_WARN(this->get_logger(), "Error setting landmask. Received landmask of size %zu", land_mask_.polygon.points.size());
    }
    land_mask_ = *msg;
    land_mask_set_ = true;
    poly_sub_.reset();
    RCLCPP_INFO(this->get_logger(), "Received land mask of size %zu", land_mask_.polygon.points.size());
}

// Callback function for parameter changes
rcl_interfaces::msg::SetParametersResult PclDetectorNode::parameter_callback(const std::vector<rclcpp::Parameter> &parameters)
{
    
    rcl_interfaces::msg::SetParametersResult result;

    // Checks if the detector parameter is valid, if result.successful is false, the parameter is not set
    for (const auto &parameter : parameters)
    {
        if (parameter.get_name() == "detector")
        {
            if (detector_type.count(parameter.as_string()) == 0)
            {
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

    switch (detector_type[detector])
    {
        case DetectorType::Euclidean:
        {
            float cluster_tolerance;
            get_parameter<float>("euclidean.cluster_tolerance", cluster_tolerance);
            int min_points;
            get_parameter<int>("euclidean.min_points", min_points);
            configured_detector = std::make_unique<EuclideanClusteringDetector>(cluster_tolerance, min_points);
            RCLCPP_INFO_STREAM(this->get_logger(), "Euclidean detector initialized with cluster tolerance = " << cluster_tolerance << " and min_points = " << min_points);
            break;
        }

        case DetectorType::DBSCAN:
        {
            float eps; 
            get_parameter<float>("dbscan.epsilon", eps);
            int min_points;
            get_parameter<int>("dbscan.min_points", min_points);
            configured_detector = std::make_unique<DBSCANDetector>(eps, min_points);
            RCLCPP_INFO_STREAM(this->get_logger(), "DBSCAN detector initialized with eps = " << eps << " and min_points = " << min_points);
            break;
        }

        default:
        {
            RCLCPP_ERROR(this->get_logger(), "Invalid pcl detector specified! Shutting down...");
            rclcpp::shutdown();
        }
    }

    return configured_detector;
}

// Callback function for processing incoming point cloud messages
void PclDetectorNode::topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg)
{
    auto start_time = std::chrono::high_resolution_clock::now();
    // auto start_time = std::chrono::high_resolution_clock::now();
    // Converts incoming ros-msg to a PointCloud 
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*cloud_msg, *cloud);
    RCLCPP_DEBUG_STREAM(this->get_logger(), "Number of points in cloud: " << cloud->size());
    // RCLCPP_INFO_STREAM(this->get_logger(), "Number of previous lines: " << prev_lines_.size());
    // Check if parameters have changed
    if (parameters_changed_)
    {
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

        processor_ = std::make_unique<PclProcessor>(voxel_leaf_size, model_thresh, model_iterations, 
        prev_line_thresh, project_thresh, wall_neighbour_dist, wall_min_points, wall_min_length);
    }

    processor_->remove_zero_points(cloud);
    RCLCPP_DEBUG_STREAM(this->get_logger(), "Number of points after removing zero points: " << cloud->size());
    
    // Used to remove ground points from testing on land
    // processor_->applyPassThrough(cloud, "z", -0.6, 100.0);

    if(this->get_parameter("apply_landmask").as_bool() && land_mask_set_)
    {
        geometry_msgs::msg::PolygonStamped land_mask_tf;
        try
        {
            geometry_msgs::msg::TransformStamped transform_stamped = tf_buffer_->
            lookupTransform(cloud_msg->header.frame_id, land_mask_.header.frame_id, tf2::TimePointZero);

            tf2::doTransform(land_mask_, land_mask_tf, transform_stamped);
            RCLCPP_DEBUG_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "Transformed land mask to frame " << cloud_msg->header.frame_id);

            int nvert = land_mask_tf.polygon.points.size();

            std::vector<float> vertx, verty;
            for (const auto& point : land_mask_tf.polygon.points)
            {
                vertx.push_back(point.x);
                verty.push_back(point.y);
            }

            processor_->apply_landmask(cloud, nvert, vertx.data(), verty.data());

            if(cloud->size() == 0)
            {
                RCLCPP_WARN(this->get_logger(), "No points left after applying landmask");
                return;
            }
            RCLCPP_DEBUG_STREAM(this->get_logger(), "Number of points after applying landmask: " << cloud->size());

            pcl::PointCloud<pcl::PointXYZ> land_mask_cloud_tf;
            for (const auto& point : land_mask_tf.polygon.points)
            {
                pcl::PointXYZ pcl_point;
                pcl_point.x = point.x;
                pcl_point.y = point.y;
                pcl_point.z = point.z;
                land_mask_cloud_tf.push_back(pcl_point);
            }
            sensor_msgs::msg::PointCloud2 polygon_cloud_msg;
            pcl::toROSMsg(land_mask_cloud_tf, polygon_cloud_msg);
            polygon_cloud_msg.header = cloud_msg->header;
            poly_pub_->publish(polygon_cloud_msg);

        }
        catch(tf2::TransformException &ex)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to apply landmask: %s", ex.what());
        }
    }

    if(this->get_parameter("apply_voxelgrid").as_bool())
    {
        processor_->apply_voxel_grid(cloud);
        RCLCPP_DEBUG_STREAM(this->get_logger(), "Number of points after applying voxel grid: " << cloud->size());
        sensor_msgs::msg::PointCloud2 land_inlier_cloud_msg;
        pcl::toROSMsg(*cloud, land_inlier_cloud_msg);
        land_inlier_cloud_msg.header = cloud_msg->header;
        pre_wall_pub_->publish(land_inlier_cloud_msg);
    }


    if (this->get_parameter("map_to_grid").as_bool())
    {
        if(grid_info_received_){
            nav_msgs::msg::OccupancyGrid grid = map_to_grid(cloud_msg);
            grid_publisher_->publish(grid);
        }
    }
    if (this->get_parameter("detect_lines").as_bool())
    {

        // Porcess the previous lines
        if(prev_lines_.size() > 0)
        {
            if (this->get_parameter("transform_lines").as_bool()) 
            {
                transform_lines(cloud_msg->header, prev_lines_);
                publish_tf_line_marker_array(prev_lines_, cloud_msg->header.frame_id);
            }
            RCLCPP_DEBUG_STREAM(this->get_logger(), "Number of previous lines: " << prev_lines_.size());
        

            int min_inliers = this->get_parameter("prev_line_min_inliers").as_int();

            std::vector<LineData> lines_data;
        
            for (const auto& coefficients : prev_lines_)
            {
                // RCLCPP_INFO_STREAM(this->get_logger(), "previous lines: " << prev_lines_.size());
                // RCLCPP_INFO_STREAM(this->get_logger(), "Line: " << line);
                auto inliers = processor_->get_inliers(coefficients, cloud);
                // RCLCPP_INFO_STREAM(this->get_logger(), "Inliers of prev line: " << inliers.size());
                if(inliers.size() > u_int16_t(min_inliers)){
                    auto [optimized_coefficients, wall_poses, wall_indices] = processor_->get_walls(coefficients, inliers, cloud);
                    // Check if there are any walls detected on the line
                    if(wall_poses.size() < 1){
                        continue;
                    }  
                    LineData line_data{optimized_coefficients, wall_poses, wall_indices, std::vector<bool>(wall_poses.size(), true)};
                    lines_data.push_back(line_data);
                }
            }
            // Filter lines to remove walls that are behind other walls and remove lines with no remaining walls after filter
            processor_->filter_lines(lines_data);
            std::vector<WallPose> filtered_wall_poses;
            std::vector<int> filtered_wall_indices;

            for (const auto& data : lines_data) {
                for (size_t i = 0; i < data.wall_poses.size(); i++)
                {
                    if (data.walls_active[i])
                    {
                        filtered_wall_poses.push_back(data.wall_poses[i]);
                        filtered_wall_indices.insert(filtered_wall_indices.end(), data.wall_indices[i].begin(), data.wall_indices[i].end());
                    }
                }
                current_lines_.push_back(data.coefficients);
            }
            RCLCPP_DEBUG_STREAM(this->get_logger(), "Number of current lines: " << current_lines_.size());

            add_ros_wall_poses(filtered_wall_poses);
            processor_->get_points_behind_walls(cloud, filtered_wall_poses, filtered_wall_indices);
            processor_->extract_points(cloud, filtered_wall_indices);
        }

        // Find new lines
        
        std::vector<WallPose> new_walls_poses;
        std::vector<int> new_wall_indices;
        // Set how many new lines to find for each callback
        int new_lines = this->get_parameter("new_lines").as_int();
        for (int i = 0; i < new_lines; i++) {
            // can't compute line with less than 2 points
            if(cloud->size() < 2){
                break;
            }
            auto [coefficients, inliers] = processor_->find_line_with_MSAC(cloud);

            auto [optimized_coefficients, wall_poses, wall_indices] = processor_->get_walls(coefficients, inliers, cloud);

            if (wall_poses.size() > 0)
            {
                current_lines_.push_back(optimized_coefficients);
                for (size_t i = 0; i < wall_poses.size(); i++)
                {
                    new_walls_poses.push_back(wall_poses[i]);
                    new_wall_indices.insert(new_wall_indices.end(), wall_indices[i].begin(), wall_indices[i].end());
                }

            add_ros_wall_poses(new_walls_poses);
            processor_->get_points_behind_walls(cloud, new_walls_poses, new_wall_indices);
            processor_->extract_points(cloud, new_wall_indices);
            }

        }
        ros_wall_poses_.header = cloud_msg->header;
        pose_array_publisher_->publish(ros_wall_poses_);

        publish_line_marker_array(current_lines_, cloud_msg->header.frame_id);
        publish_wall_marker_array(ros_wall_poses_, cloud_msg->header.frame_id);
        publish_extended_lines_from_origin(ros_wall_poses_, cloud_msg->header.frame_id);

        prev_lines_ = current_lines_;
        current_lines_.clear();
        ros_wall_poses_.poses.clear();

        RCLCPP_DEBUG_STREAM(this->get_logger(), "Number of points after detecting lines: " << cloud->size());
        // RCLCPP_INFO_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Filtered PointCloud to " << cloud->size() << " points");
    }
    
    sensor_msgs::msg::PointCloud2 downsampled_cloud_msg;
    pcl::toROSMsg(*cloud, downsampled_cloud_msg);
    downsampled_cloud_msg.header = cloud_msg->header;

    after_wall_pub_->publish(downsampled_cloud_msg);
    for (auto& point : cloud->points)
    {
        point.z = 0.0;
    }

    // Finds clusters with the configured detector
    // pcl::PointCloud<pcl::PointXYZ> detections = detector_->get_detections(*cloud);
    std::vector<pcl::PointCloud<pcl::PointXYZ>> clusters = detector_->get_clusters(*cloud);
    pcl::PointCloud<pcl::PointXYZ> centroids = detector_->get_centroids(clusters);

    if (centroids.size() == 0) {
        RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "No clusters detected!");
    }
    RCLCPP_DEBUG_STREAM(this->get_logger(), "Number of clusters detected: " << centroids.size());

    // Converts the clusters-PointCloud to an appropriate msg for publishing
    pcl::PointCloud<pcl::PointXYZ> cluster_pcl;
    sensor_msgs::msg::PointCloud2 cluster_pcl_msg;
    for (const auto& cluster : clusters) {
        cluster_pcl.points.insert(cluster_pcl.points.end(), cluster.points.begin(), cluster.points.end());
    }
    pcl::toROSMsg(cluster_pcl, cluster_pcl_msg);
    cluster_pcl_msg.header = cloud_msg->header;
    cluster_publisher_->publish(cluster_pcl_msg);

    // Converts the clusters-PointCloud to an appropriate msg for publishing
    sensor_msgs::msg::PointCloud2 centroids_cloud_msg;
    pcl::toROSMsg(centroids, centroids_cloud_msg);
    centroids_cloud_msg.header = cloud_msg->header;

    centroid_publisher_->publish(centroids_cloud_msg);

    // vortex_msgs::msg::Clusters vortex_clusters;
    // pcl::PointCloud<pcl::PointXYZ> convex_hull_pcl;
    // vortex_clusters.centroids=centroids_cloud_msg;
    // vortex_clusters.header = cloud_msg->header;
    // for ( auto& cluster : clusters)
    // {
    //     pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_convex_hull (new pcl::PointCloud<pcl::PointXYZ>);
    //     processor_->compute_convex_hull(cluster, cluster_convex_hull);
    //     sensor_msgs::msg::PointCloud2 convex_hull_msg;
    //      pcl::toROSMsg(*cluster_convex_hull, convex_hull_msg);
    //     //  RCLCPP_DEBUG_STREAM(this->get_logger(), "Point diff between concave hull and cluster: " << cluster.size() - cluster_convex_hull->size());
    //     vortex_clusters.clusters.push_back(convex_hull_msg);
    //     convex_hull_pcl.points.insert(convex_hull_pcl.points.end(), cluster_convex_hull->points.begin(), cluster_convex_hull->points.end());
    // }
    
    // sensor_msgs::msg::PointCloud2 convex_hull_msg;
    // pcl::toROSMsg(convex_hull_pcl, convex_hull_msg);
    // convex_hull_msg.header = cloud_msg->header;
    // convex_hull_publisher_->publish(convex_hull_msg);

    vortex_msgs::msg::Clusters vortex_clusters;
    vortex_clusters.header = cloud_msg->header;
    vortex_clusters.centroids = centroids_cloud_msg;
    for (const auto& cluster : clusters)
    {
        sensor_msgs::msg::PointCloud2 cluster_msg;
        pcl::toROSMsg(cluster, cluster_msg);
        vortex_clusters.clusters.push_back(cluster_msg);
    }

    vortex_cluster_publisher_->publish(vortex_clusters);
    auto end_time = std::chrono::high_resolution_clock::now();

    std::chrono::duration<double> duration = end_time - start_time;
    RCLCPP_DEBUG_STREAM(this->get_logger(), "Processing time for topic_callback: " << duration.count() << " seconds");
}

void PclDetectorNode::get_grid() {
  while (true) {
    rclcpp::sleep_for(std::chrono::seconds(1));
    if (!grid_client_->wait_for_service(std::chrono::seconds(5))) {
      RCLCPP_ERROR(this->get_logger(), "Service not available after waiting");
      continue;
    }

    auto request = std::make_shared<nav_msgs::srv::GetMap::Request>();
    auto result_future = grid_client_->async_send_request(request);

    // Wait for the result within a specified timeout period
    auto status = result_future.wait_for(std::chrono::seconds(5));
    if (status == std::future_status::ready) {
      try {
        auto result = result_future.get();
        if (result->map.data.empty()) {
          RCLCPP_ERROR(this->get_logger(),
                       "Received empty map from grid client");
          continue;
        }
        height_ = result->map.info.height;
        width_ = result->map.info.width;
        resolution_ = result->map.info.resolution;
        grid_info_received_ = true;

        RCLCPP_INFO(this->get_logger(),
                    "Successfully received map from grid client");
        return;
      } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(),
                     "Exception while getting result from future: %s",
                     e.what());
      }
    } else {
      RCLCPP_ERROR(this->get_logger(),
                   "Failed to get map from grid client within timeout period");
      continue;
    }
  }
}

nav_msgs::msg::OccupancyGrid PclDetectorNode::map_to_grid(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    nav_msgs::msg::OccupancyGrid grid;
    grid.header.frame_id = this->get_parameter("fixed_frame").as_string();
    grid.header.stamp = msg->header.stamp;
    grid.info.resolution = resolution_;
    grid.info.width = width_;
    grid.info.height = height_;
    double half_width_meters = -(width_ *
                                resolution_) /
                                2.0;
    double half_height_meters = -(height_ *
                                    resolution_) /
                                2.0;
    // auto [lat, lon] = flat2lla(half_width_meters, half_height_meters);
    geometry_msgs::msg::Pose map_origin;
    map_origin.position.x = half_width_meters;
    map_origin.position.y = half_height_meters;
    map_origin.position.z = 0.0;
    map_origin.orientation.x = 0.0;
    map_origin.orientation.y = 0.0;
    map_origin.orientation.z = 0.0;
    map_origin.orientation.w = 1.0;
    grid.info.origin = map_origin;
    geometry_msgs::msg::TransformStamped transform_stamped;
    bool transform_success = false;
    while (!transform_success) {
        try {
            transform_stamped = tf_buffer_->lookupTransform(grid.header.frame_id, msg->header.frame_id, tf2::TimePointZero);
            transform_success = true;
        } catch (tf2::TransformException &ex) {
            RCLCPP_ERROR(this->get_logger(), "Failed to lookup transform: %s", ex.what());
            rclcpp::sleep_for(std::chrono::seconds(1));
        }
    }
    sensor_msgs::msg::PointCloud2 transformed_msg;
    tf2::doTransform(*msg, transformed_msg, transform_stamped);

     // Iterate through the point cloud and fill the grid
    grid.data.resize(grid.info.width * grid.info.height, 0);
    for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(transformed_msg, "x"), iter_y(transformed_msg, "y"); 
         iter_x != iter_x.end(); ++iter_x, ++iter_y) {
        
        // Get the x and y coordinates of the point
        float x = *iter_x;
        float y = *iter_y;

        // Calculate the grid cell index
        int grid_x = static_cast<int>((x) / grid.info.resolution + grid.info.width / 2);
        int grid_y = static_cast<int>((y) / grid.info.resolution + grid.info.height / 2);

        // Ensure the point is within the grid bounds
        if (grid_x >= 0 && grid_x < static_cast<int>(grid.info.width) && grid_y >= 0 && grid_y < static_cast<int>(grid.info.height)) {
            int index = grid_y * grid.info.width + grid_x;
            // Mark the cell as occupied (100)
            if (grid.data[index] < std::numeric_limits<int8_t>::max() - this->get_parameter("cell_inc_value").as_int()) {
            grid.data[index] += this->get_parameter("cell_inc_value").as_int();
            }
        }
    }
    return grid;
}

void PclDetectorNode::transform_lines(const std_msgs::msg::Header& cloud_header, std::vector<Eigen::VectorXf>& prev_lines)
{
    // Calculate the transformation between the current and previous point cloud
    std::string fixed_frame = this->get_parameter("fixed_frame").as_string();
    geometry_msgs::msg::TransformStamped current_transform;
    static geometry_msgs::msg::TransformStamped prev_transform;
    static bool isFirstRun = true; // Tracks the first run or successful retrieval.

    try
    {
        current_transform = tf_buffer_->lookupTransform(fixed_frame, cloud_header.frame_id, cloud_header.stamp,
        rclcpp::Duration(0, this->get_parameter("transform_timeout_nsec").as_int()));
        
        // If it's not the first run and we successfully get the transform:
        if (!isFirstRun)
        {
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

            for (auto& line : prev_lines)
            { 
                // Use 3f to be compatible with the Eigen::Quaternionf * operator
                Eigen::Vector3f point(line[0], line[1], 0.0);
                Eigen::Vector3f direction(line[2], line[3], 0.0);

                // Apply the rotation
                Eigen::Vector3f rotated_point = rotation * point;
                Eigen::Vector3f rotated_direction = rotation * direction;

                // Apply the translation to the point
                Eigen::Vector3f translated_point = rotated_point + translation;

                line[0] = translated_point[0];
                line[1] = translated_point[1];
                line[2] = rotated_direction[0];
                line[3] = rotated_direction[1];
            }

        } else {
            // It's the first successful retrieval; just update the flag and prev_transform.
            isFirstRun = false;
            prev_transform = current_transform;
            return;
        }

    } catch (tf2::TransformException &ex) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Could not retrieve transform for previous lines: %s", ex.what());
        isFirstRun = true;
        return;
    }
}

void PclDetectorNode::add_ros_wall_poses(const std::vector<std::pair<pcl::PointXYZ,pcl::PointXYZ>>& wall_poses)
{
    for(auto wall : wall_poses)
    {
        geometry_msgs::msg::Pose pose_1;
        geometry_msgs::msg::Pose pose_2;

        pose_1.position.x = wall.first.x;
        pose_1.position.y = wall.first.y;
        pose_1.position.z = 0.0;
        pose_1.orientation.x = 0.0;
        pose_1.orientation.y = 0.0;
        pose_1.orientation.z = 0.0;
        pose_1.orientation.w = 1.0;

        pose_2.position.x = wall.second.x;
        pose_2.position.y = wall.second.y;
        pose_2.position.z = 0.0;
        pose_2.orientation.x = 0.0;
        pose_2.orientation.y = 0.0;
        pose_2.orientation.z = 0.0;
        pose_2.orientation.w = 1.0;
        
        ros_wall_poses_.poses.push_back(pose_1);
        ros_wall_poses_.poses.push_back(pose_2);
    }
}

geometry_msgs::msg::Point PclDetectorNode::extend_line_from_origin_to_length(const geometry_msgs::msg::Point& point, double length) {
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

void PclDetectorNode::publish_line_marker_array(const std::vector<Eigen::VectorXf>& lines, const std::string& frame_id) {
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
        start.z = 0.0;
        end.x = line[0]+line[2]*3;
        end.y = line[1]+line[3]*3;
        end.z = 0.0;
        line_marker.points.push_back(start);
        line_marker.points.push_back(end);

        // Add the current line marker to the array
        marker_array.markers.push_back(line_marker);
    }

    // Publish the MarkerArray
    line_publisher->publish(marker_array);
}

void PclDetectorNode::publish_tf_line_marker_array(const std::vector<Eigen::VectorXf>& lines, const std::string& frame_id) {
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
        start.x = line[0]+line[2]*-3; // Adapt based on your representation
        start.y = line[1]+line[3]*-3;
        start.z = 0;
        end.x = line[0]+line[2]*3;
        end.y = line[1]+line[3]*3;
        end.z = 0;
        line_marker.points.push_back(start);
        line_marker.points.push_back(end);

        // Add the current line marker to the array
        marker_array.markers.push_back(line_marker);
    }

    // Publish the MarkerArray
    tf_line_publisher->publish(marker_array);
}

void PclDetectorNode::publish_extended_lines_from_origin(const geometry_msgs::msg::PoseArray& pose_array, const std::string& frame_id) {
    visualization_msgs::msg::MarkerArray marker_array;
    int id = 0; // Unique ID for each marker

    for (size_t i = 0; i < pose_array.poses.size(); i += 2) {
        if (i + 1 >= pose_array.poses.size()) {
            break; // Prevent going out of bounds if there's an odd number of poses
        }
        auto& start_pose = pose_array.poses[i];
        auto& end_pose = pose_array.poses[i + 1];

        // Process start and end points to extend lines from origin
        geometry_msgs::msg::Point extended_point_1 = extend_line_from_origin_to_length(start_pose.position, 100.0);
        geometry_msgs::msg::Point extended_point_2 = extend_line_from_origin_to_length(end_pose.position, 100.0);


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
void PclDetectorNode::publish_wall_marker_array(const geometry_msgs::msg::PoseArray& pose_array, const std::string& frame_id) {
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
