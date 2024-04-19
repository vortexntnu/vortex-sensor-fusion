#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PointIndices.h>

#include <utility>


namespace pcl_detector {


class LandMasker{
public:
    

    LandMasker(double origin_lat, double origin_lon);

    constexpr double deg2rad(double degrees) const;
    std::array<double, 2> lla2flat(double lat, double lon) const;
    void processCoordinates(const std::vector<std::array<double, 2>>& coordinates);
    void setOrigin();
    void set_polygon();
    pcl::PointCloud<pcl::PointXYZ> get_polygon();
    void get_land(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const pcl::PointCloud<pcl::PointXYZ>& polygon, std::vector<int>& indices_to_remove);
    bool isXYPointIn2DXYPolygon (const pcl::PointXYZ& point, const pcl::PointCloud<pcl::PointXYZ> &polygon);





private:
    double origin_lat_;
    double origin_lon_;
    pcl::PointCloud<pcl::PointXYZ> polygon_;
    // std::vector<pcl::PointIndices> findLineClusters(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, float cluster_tolerance, int min_cluster_size, int max_cluster_size);



};
}; // namespace pcl_detector