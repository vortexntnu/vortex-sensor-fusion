#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PointIndices.h>

#include <utility>


namespace pcl_detector {


class PclProcessor{
public:
    

    PclProcessor(float voxel_leaf_size, float model_thresh, int model_iterations,
                 float prev_line_thresh, float project_thresh, float wall_min_dist, int wall_min_points);

    std::vector<int> removeNanPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
    void flattenPointCloud(pcl::PointCloud<pcl::PointXYZ>& cloud, float new_z_value);
    void applyVoxelGrid(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
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
    pcl::PointIndices::Ptr getPointsBehindWall(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointXYZ& wall_start, pcl::PointXYZ& wall_end);
    void extractPointsBehindWall(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointIndices::Ptr indices_to_remove);

private:
    float voxel_leaf_size_;
    float model_thresh_;
    int model_iterations_;
    float prev_line_thresh_;
    float project_thresh_;
    float wall_min_dist_;
    int wall_min_points_;

    // std::vector<pcl::PointIndices> findLineClusters(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, float cluster_tolerance, int min_cluster_size, int max_cluster_size);



};
}; // namespace pcl_detector