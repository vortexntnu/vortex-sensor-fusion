#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PointIndices.h>

#include <utility>



namespace pcl_detector {

struct LineData {
    Eigen::VectorXf coefficients;
    std::vector<std::pair<pcl::PointXYZ,pcl::PointXYZ>> wall_poses;
    std::vector<bool> walls_active;
    };

class PclProcessor{
public:
    

    PclProcessor(float voxel_leaf_size, float model_thresh, int model_iterations,
                 float prev_line_thresh, float project_thresh, float wall_neighbour_dist, int wall_min_points, float wall_min_length, float wall_merge_dist);

    // void flattenPointCloud(pcl::PointCloud<pcl::PointXYZ>& cloud, float new_z_value);
    // Invalied points from the lidar are included as {0.0,0.0,0.0} points in the point cloud. Remove them here.
    void remove_zero_points(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
    void apply_voxel_grid(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
    void applyPassThrough(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const std::string& field_name, float limit_min, float limit_max);
    std::tuple<Eigen::VectorXf, std::vector<int>> find_line_with_MSAC(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
    // Returns the indices of the inliers
    std::vector<int> get_inliers(const Eigen::VectorXf& coefficients, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
    Eigen::VectorXf optimize_line(Eigen::VectorXf model, std::vector<int> inliers, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
    std::tuple<pcl::PointCloud<pcl::PointXYZ>::Ptr, std::vector<int>> project_inliers_on_line(const Eigen::VectorXf& coefficients, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
    std::tuple<Eigen::VectorXf, std::vector<std::pair<pcl::PointXYZ,pcl::PointXYZ>>> get_walls(
                    const Eigen::VectorXf& coefficients, std::vector<int>& wall_indices, std::vector<int> inliers, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
    // void sort_projected_points(const std::vector<int>& inliers, pcl::PointCloud<pcl::PointXYZ>::Ptr& projection);
    std::vector<std::tuple<float, int, int>> sort_projected_points(
        const std::vector<int>& inliers, const pcl::PointCloud<pcl::PointXYZ>::Ptr& projectedCloud);
    // std::vector<pcl::PointXYZ> findWalls(pcl::PointCloud<pcl::PointXYZ>::Ptr& projectedCloud, std::vector<int>& wall_indices);
    // std::vector<pcl::PointXYZ> find_walls_on_line(const std::vector<std::pair<float, int>>& sqr_distances_and_indices, std::vector<int>& wall_indices);
    std::vector<std::pair<pcl::PointXYZ,pcl::PointXYZ>> find_walls_on_line(const std::vector<std::tuple<float, int, int>>& sqr_distances_and_indices, std::vector<int>& wall_indices, const pcl::PointCloud<pcl::PointXYZ>::Ptr& projectedCloud);


    void add_or_merge_wall(std::vector<int>& wall_indices, std::vector<std::pair<pcl::PointXYZ,pcl::PointXYZ>>& wall_poses, const std::vector<int>& current_wall_indices, 
    const pcl::PointXYZ& wall_start_point, const pcl::PointXYZ& wall_end_point);
    void get_points_behind_walls(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const std::vector<std::pair<pcl::PointXYZ,pcl::PointXYZ>>& wall_poses, std::vector<int>& indices_to_remove);
    // pcl::PointCloud<pcl::PointXYZ>::Ptr create_extended_point(const pcl::PointXYZ& p1, const pcl::PointXYZ& p2);
    std::pair<float,float> create_extended_point(const pcl::PointXYZ& p);
    void filter_lines(std::vector<LineData>& linesData);


    // bool isXYPointIn2DXYPolygon (const pcl::PointXYZ& point, const pcl::PointCloud<pcl::PointXYZ> &polygon);
    void extract_points(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::vector<int>& indices_to_remove);
    void apply_landmask(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, int nvert, float* vertx, float* verty);
    int pnpoly(int nvert, float *vertx, float *verty, float testx, float testy);





private:
    float voxel_leaf_size_;
    float model_thresh_;
    int model_iterations_;
    float prev_line_thresh_;
    float project_thresh_;
    float wall_neighbour_dist_;
    int wall_min_points_;
    float wall_min_length_;
    float wall_merge_dist_;

    // std::vector<pcl::PointIndices> findLineClusters(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, float cluster_tolerance, int min_cluster_size, int max_cluster_size);



};
}; // namespace pcl_detector