#include <image_filters/image_filtering_ros.hpp>

using std::placeholders::_1;

namespace vortex::image_processing {


ImageFilteringNode::ImageFilteringNode() : Node("image_filtering_node") 
{
    this->declare_parameter<std::string>("image_topic", "/flir_camera/image_raw");
    this->declare_parameter<std::string>("filter_params.filter_type", "ebus");
    this->declare_parameter<int>("filter_params.unsharpening.blur_size", 8);
    this->declare_parameter<int>("filter_params.erosion.size", 1);
    this->declare_parameter<int>("filter_params.dilation.size", 1);
    this->declare_parameter<double>("filter_params.white_balancing.contrast_percentage", 0.8);
    this->declare_parameter<int>("filter_params.ebus.erosion_size", 2);
    this->declare_parameter<int>("filter_params.ebus.blur_size", 30);
    this->declare_parameter<int>("filter_params.ebus.mask_weight", 5);

    // Set up the QoS profile for the image subscriber
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos_sensor_data = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 1), qos_profile);

    check_and_subscribe_to_image_topic();
    set_filter_params();

    initialize_parameter_handler();

    image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/filtered_image", qos_sensor_data);
}

void ImageFilteringNode::set_filter_params(){
    FilterParams params;
    std::string filter = this->get_parameter("filter_params.filter_type").as_string();
    if(!filter_functions.contains(filter)){
        RCLCPP_ERROR_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Invalid filter type: " << filter << " Setting to no_filter.");
        filter_ = "no_filter";
    } else{
        filter_ = filter;
    }
    params.unsharpening.blur_size = this->get_parameter("filter_params.unsharpening.blur_size").as_int();
    params.eroding.size = this->get_parameter("filter_params.erosion.size").as_int();
    params.dilating.size = this->get_parameter("filter_params.dilation.size").as_int();
    params.white_balancing.contrast_percentage = this->get_parameter("filter_params.white_balancing.contrast_percentage").as_double();
    params.ebus.erosion_size = this->get_parameter("filter_params.ebus.erosion_size").as_int();
    params.ebus.blur_size = this->get_parameter("filter_params.ebus.blur_size").as_int();
    params.ebus.mask_weight = this->get_parameter("filter_params.ebus.mask_weight").as_int();
    filter_params_ = params;
    RCLCPP_INFO(this->get_logger(), "Filter parameters updated.");
}

void ImageFilteringNode::check_and_subscribe_to_image_topic() {
    std::string image_topic = this->get_parameter("image_topic").as_string();
    if (image_topic_ != image_topic) {
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            image_topic, 10, std::bind(&ImageFilteringNode::image_callback, this, _1));
        image_topic_ = image_topic;
        RCLCPP_INFO(this->get_logger(), "Subscribed to image topic: %s", image_topic.c_str());
    }
}

void ImageFilteringNode::initialize_parameter_handler() {
    param_handler_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
    
    // Register the parameter event callback with the correct signature
    auto parameter_event_callback =
        [this](const rcl_interfaces::msg::ParameterEvent & event) -> void {
            this->on_parameter_event(event);
        };

    // Register the callback with the parameter event handler
    param_cb_handle_ = param_handler_->add_parameter_event_callback(parameter_event_callback);
}

void ImageFilteringNode::on_parameter_event(const rcl_interfaces::msg::ParameterEvent & event) {
     // Get the fully qualified name of the current node
    auto node_name = this->get_fully_qualified_name();

    // Filter out events not related to this node
    if (event.node != node_name) {
        return; // Early return if the event is not from this node
    }
    RCLCPP_INFO(this->get_logger(), "Received parameter event");
    std::unique_lock<std::mutex> guard(callback_mutex_);
    for (const auto& changed_parameter : event.changed_parameters) {
        if (changed_parameter.name.find("image_topic") == 0) check_and_subscribe_to_image_topic();
        if (changed_parameter.name.find("filter_params") == 0) set_filter_params(); 
    } 
}

void ImageFilteringNode::image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    RCLCPP_INFO_ONCE(this->get_logger(), "Received image message.");
    std::unique_lock<std::mutex> guard(callback_mutex_);
    cv_bridge::CvImagePtr cv_ptr;

    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

        if (cv_ptr->image.empty()) {
        RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Empty image received, skipping processing.");
        return;
        }

    }
    catch (cv_bridge::Exception &e)
    {
        RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "cv_bridge exception: " << e.what());
        return;
    }
    
    cv::Mat input_image = cv_ptr->image;
    cv::Mat filtered_image;
    apply_filter(filter_, filter_params_, input_image, filtered_image);
    auto message = cv_bridge::CvImage(msg->header, "bgr8", filtered_image).toImageMsg();
    
    image_pub_->publish(*message);
}
} // namespace vortex::image_processing
