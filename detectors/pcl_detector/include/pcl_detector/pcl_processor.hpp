#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PointIndices.h>

#include <utility>


namespace pcl_detector {


class PclProcessor{
public:
    

    PclProcessor(float voxel_leaf_size, float model_thresh, int model_iterations,
                 float prev_line_thresh, float project_thresh, float wall_min_dist, int wall_min_points, float wall_merge_dist);

    std::vector<int> removeNanPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
    void flattenPointCloud(pcl::PointCloud<pcl::PointXYZ>& cloud, float new_z_value);
    void applyVoxelGrid(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
    void applyPassThrough(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const std::string& field_name, float limit_min, float limit_max);
    std::tuple<Eigen::VectorXf, std::vector<int>> findLineWithMSAC(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
    std::vector<int> findInliers(const Eigen::VectorXf& coefficients, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
    Eigen::VectorXf optimizeLine(Eigen::VectorXf model, std::vector<int> inliers, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
    std::tuple<pcl::PointCloud<pcl::PointXYZ>::Ptr, std::vector<int>> projectInliersOnLine(const Eigen::VectorXf& coefficients, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
    std::tuple<Eigen::VectorXf, std::vector<pcl::PointXYZ>> handleLine(
                    const Eigen::VectorXf& coefficients, std::vector<int>& wall_indices, std::vector<int> inliers, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
    void sortProjectedPoints(const std::vector<int>& inliers, pcl::PointCloud<pcl::PointXYZ>::Ptr& projection);
    std::vector<pcl::PointXYZ> findWalls(pcl::PointCloud<pcl::PointXYZ>::Ptr& projectedCloud, std::vector<int>& wall_indices);
    void addOrMergeWall(std::vector<int>& wall_indices, std::vector<pcl::PointXYZ>& wall_poses, const std::vector<int>& current_wall_indices, 
    const pcl::PointXYZ& start_point, const pcl::PointXYZ& end_point);
    // void extractWalls(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, std::vector<std::vector<int>>);
    // pcl::PointIndices::Ptr getPointsBehindWall(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointXYZ& wall_start, pcl::PointXYZ& wall_end);
    pcl::PointIndices::Ptr getPointsBehindWalls(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const std::vector<pcl::PointXYZ>& wall_poses);
    // void extractPointsBehindWall(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointIndices::Ptr indices_to_remove);
    pcl::PointCloud<pcl::PointXYZ>::Ptr createPolygon(const pcl::PointXYZ& p1, const pcl::PointXYZ& p2);
    bool isXYPointIn2DXYPolygon (const pcl::PointXYZ& point, const pcl::PointCloud<pcl::PointXYZ> &polygon);
    void extractPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::vector<int>& indices_to_remove);



private:
    float voxel_leaf_size_;
    float model_thresh_;
    int model_iterations_;
    float prev_line_thresh_;
    float project_thresh_;
    float wall_min_dist_;
    int wall_min_points_;
    float wall_merge_dist_;

    // std::vector<pcl::PointIndices> findLineClusters(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, float cluster_tolerance, int min_cluster_size, int max_cluster_size);



};
}; // namespace pcl_detector