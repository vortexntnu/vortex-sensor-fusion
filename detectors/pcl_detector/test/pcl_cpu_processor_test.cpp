#include <gtest/gtest.h>
#include <pcl_detector/pcl_processor.hpp>
#include <pcl/io/pcd_io.h>

class PointCloudTest : public ::testing::Test {
protected:
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2;

    void SetUp() override {
        cloud1.reset(new pcl::PointCloud<pcl::PointXYZ>());
        cloud2.reset(new pcl::PointCloud<pcl::PointXYZ>());
        
        // Attempt to load the first point cloud
        if (pcl::io::loadPCDFile<pcl::PointXYZ>("/home/jorgen/ros2_ws/saved_cloud_1.pcd", *cloud1) == -1) {
            FAIL() << "Failed to load cloud1 from file";
        }

        // Attempt to load the second point cloud
        if (pcl::io::loadPCDFile<pcl::PointXYZ>("/home/jorgen/ros2_ws/saved_cloud_2.pcd", *cloud2) == -1) {
            FAIL() << "Failed to load cloud2 from file";
        }
    }
};





TEST(FilterTest, RemoveNaNPoints) {
    

}

