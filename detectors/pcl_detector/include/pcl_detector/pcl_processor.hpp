#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PointIndices.h>

#include <utility>


namespace pcl_detector {


class PclProcessor{
public:

    std::vector<int> removeNanPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
    void flattenPointCloud(pcl::PointCloud<pcl::PointXYZ>& cloud, float new_z_value);
    void applyVoxelGrid(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, float leaf_size);
    void applyPassThrough(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const std::string& field_name, float limit_min, float limit_max);
    std::tuple<Eigen::VectorXf, std::vector<int>> findLineWithMSAC(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
    std::vector<int> findInliers(const Eigen::VectorXf& coefficients, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
    Eigen::VectorXf optimizeLine(Eigen::VectorXf model, std::vector<int> inliers, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
    std::tuple<pcl::PointCloud<pcl::PointXYZ>::Ptr, std::vector<int>> projectInliersOnLine(const Eigen::VectorXf& coefficients, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
    std::tuple<Eigen::VectorXf,std::vector<std::vector<int>>, std::vector<pcl::PointXYZ>> handleLine(
                    const Eigen::VectorXf& coefficients, std::vector<int> inliers, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
    void sortProjectedPoints(const std::vector<int>& inliers, pcl::PointCloud<pcl::PointXYZ>::Ptr& projection);
    std::tuple<std::vector<std::vector<int>>, std::vector<pcl::PointXYZ>> findWalls(pcl::PointCloud<pcl::PointXYZ>::Ptr& projection);
    void extractWalls(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, std::vector<std::vector<int>>);
    pcl::PointIndices::Ptr getPointsBehindWall(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const Eigen::Vector3f& P1, const Eigen::Vector3f& P2);
    void extractPointsBehindWall(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointIndices::Ptr indices_to_remove);



    

    // std::vector<pcl::PointIndices> findLineClusters(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, float cluster_tolerance, int min_cluster_size, int max_cluster_size);



};
}; // namespace pcl_detector