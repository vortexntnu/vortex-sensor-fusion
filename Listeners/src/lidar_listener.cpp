#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>

#include <inttypes.h>

using std::placeholders::_1;

class LidarSubscriber : public rclcpp::Node
{
public:
  LidarSubscriber()
  : Node("lider_pcl")
  {
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    qos_profile.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 1), qos_profile);

    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "ouster/points", qos, std::bind(&LidarSubscriber::topic_callback, this, _1));
  }

private:
  void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) const
  {
    // RCLCPP_INFO_STREAM(this->get_logger(), ("I heard: '%s'", msg->fields.));
    //pcl::PointCloud<pcl::PointXYZ> test = pcl::PointCloud<pcl::PointXYZ>>::;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    auto a = *msg;
    // RCLCPP_INFO(this->get_logger(), "Hei");
    pcl::fromROSMsg(*msg, *cloud);

    
    // for(pcl::PointXYZ point : cloud->points){
    //   std::ostringstream stream;
    //   stream << "X: " << point.x << " | Y: " << point.y << " | Z: " << point.z;
    //   RCLCPP_INFO_STREAM(this->get_logger(), stream.str());
    // }
    

    RCLCPP_INFO_STREAM(this->get_logger(), *cloud);
    
  }
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarSubscriber>());
  rclcpp::shutdown();
  return 0;
}