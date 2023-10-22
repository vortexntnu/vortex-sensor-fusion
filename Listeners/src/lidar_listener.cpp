#include <memory>

#include "rclcpp/rclcpp.hpp"                 //For creating ros2 Nodes
#include "std_msgs/msg/string.hpp"           // Ros2 message type for trings
#include <sensor_msgs/msg/point_cloud2.hpp>  // Ros2 messgae type for PointCLoud2
#include <pcl_conversions/pcl_conversions.h> // For converting Ros message and pcl types
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_clusters.h>

#include <inttypes.h>

using std::placeholders::_1;

class LidarSubscriber : public rclcpp::Node
{
public:
  LidarSubscriber()
      : Node("lidar_pcl")
  {
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    qos_profile.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 1), qos_profile);

    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "ouster/points", qos, std::bind(&LidarSubscriber::topic_callback, this, _1));

    centroids_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("centroids", qos);
  }

private:
  void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) const
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pcl::fromROSMsg(*msg, *cloud);

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(0.1);
    ec.setMinClusterSize(cloud->size() / 100);
    ec.setMaxClusterSize(cloud->size() / 2);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    pcl::PointCloud<pcl::PointXYZ> centroids;

    for (const auto &indices : cluster_indices)
    {
      pcl::PointXYZ centroid;
      centroid.getArray3fMap() = Eigen::Array3f::Zero();
      for (const auto &index : indices.indices)
      {
        const auto &point = cloud->at(index);
        centroid.x += point.x;
        centroid.y += point.y;
        centroid.z += point.z;
      }
      centroid.x /= indices.indices.size();
      centroid.y /= indices.indices.size();
      centroid.z /= indices.indices.size();
      centroids.push_back(centroid);
    }

    RCLCPP_INFO(this->get_logger(), "Centroids:");

    for (size_t i = 0; i < centroids.size(); ++i)
    {
      const pcl::PointXYZ &point = centroids.at(i);
      RCLCPP_INFO_STREAM(this->get_logger(),
                         "Point " << i << ": X = " << point.x << ", Y = " << point.y << ", Z = " << point.z);
    }

    // Publisher
    sensor_msgs::msg::PointCloud2 centroids_pcl2;
    pcl::toROSMsg(centroids, centroids_pcl2);
    centroids_pcl2.header.frame_id = msg->header.frame_id;

    centroids_publisher_->publish(centroids_pcl2);
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr centroids_publisher_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarSubscriber>());
  rclcpp::shutdown();
  return 0;
}