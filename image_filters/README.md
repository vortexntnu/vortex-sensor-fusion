# Image Filtering Node

The `image_filtering_node` is a ROS 2 node developed in the `vortex::image_filters` namespace. It is designed to subscribe to image topics, apply various image filters using OpenCV, and publish the filtered images back to ROS.

## Features

- **Multiple Filters**: Supports sharpening, unsharpening, eroding, dilating, white balancing, and a custom "ebus" filter.
- **Dynamic Reconfiguration**: Allows for runtime changes to filter parameters and subscribed image topics via ROS 2 parameters.
- **Parameter-Driven Configuration**: Configures filter types and specific attributes through ROS 2 parameters.

## Configuration

Configure the node using ROS 2 parameters:

- **`image_topic`**: Topic from which the node will subscribe to image messages.
- **`filter_params.filter_type`**: Specifies the filter to apply. Current options include:
  - `nofilter`
  - `sharpening`
  - `unsharpening`
  - `eroding`
  - `dilating`
  - `white_balancing`
  - `ebus`
- **Other filter-specific parameters** such as `blur_size`, `size`, `contrast_percentage`, can be configured through parameter `filter_params`.{filter_name}.{param_name}

Parameters can be set through a YAML file or dynamically adjusted at runtime.

## Additional filters

## Implementing New Filters

To extend the functionality of the `image_filtering_node` by adding new filters, follow these steps to ensure compatibility and integration with the existing codebase:

### Step 1: Define Filter Parameters

Each filter should have its own set of parameters encapsulated in a structure. Define this structure within the `vortex::image_filters` namespace.

```cpp
struct YourFilterParams {
    // Add necessary parameters here
    int example_param;
};
```

### Step 2: Add to FilterParams Structure

Integrate your new filter parameters structure into the existing `FilterParams` structure. This allows the `apply_filter` function to access the parameters specific to your filter.

```cpp
struct FilterParams {
    std::string filter_type;
    UnsharpeningFilterParams unsharpening;
    ErodingFilterParams eroding;
    DilatingFilterParams dilating;
    WhiteBalancingFilterParams white_balancing;
    EbusFilterParams ebus;
    YourFilterParams your_filter; // Add your filter params here
};
```

### Step 3: Create the Filter Function

Implement your filter function. This function should take the `cv::Mat` objects for the input and output images and a `const FilterParams&` which includes your specific filter parameters. Make sure to use your parameter structure within this function.

```cpp
void your_filter_function(const cv::Mat &original, cv::Mat &filtered, const FilterParams& filter_params) {
    // Access your filter-specific parameters like this:
    int example_param = filter_params.your_filter.example_param;
    
    // Implement your filtering logic here
}
```

### Step 4: Register the Filter Function

Add an entry to the `filter_functions` map for your new filter. This step is crucial as it links the filter name (as a string) to the corresponding filter function pointer.

```cpp
std::map<std::string, FilterFunction> filter_functions = {
    {"no_filter", no_filter},
    {"sharpening", sharpening_filter},
    {"unsharpening", unsharpening_filter},
    {"eroding", eroding_filter},
    {"dilating", dilating_filter},
    {"white_balancing", white_balance_filter},
    {"ebus", ebus_filter},
    {"your_filter", your_filter_function} // Add your filter here
};
```

### Step 5: Declare and Assign Parameters

Declare the new filter parameters in the ROS 2 node constructor and assign these parameters to the `FilterParams` structure within the `set_filter_params` function.

#### In the Node Constructor

In the constructor of your ROS 2 node, declare each of the new filter parameters using the `declare_parameter` function. This sets the default values and prepares the node to accept these parameters at runtime through command line or a YAML configuration file.

```cpp
ImageFilteringNode::ImageFilteringNode() : Node("image_filtering_node")
{
    this->declare_parameter<std::string>("filter_params.your_filter.example_param", "default_value");
    ...
    // Other parameters declarations
}
```


#### In the set_filter_params Function

In the set_filter_params function, retrieve and assign the parameters to the corresponding fields in the FilterParams structure. Ensure to handle cases where the parameter might not be set or provided.

```cpp

void ImageFilteringNode::set_filter_params(){
    FilterParams params = filter_params_; // assuming filter_params_ is already defined in your class

    params.your_filter.example_param = this->get_parameter("filter_params.your_filter.example_param").as_string();
    ...
    // Retrieve other parameters and handle cases where parameters might not be provided
    filter_params_ = params; // Update the filter parameters structure
    RCLCPP_INFO(this->get_logger(), "Filter parameters updated for your_filter.");
}
```