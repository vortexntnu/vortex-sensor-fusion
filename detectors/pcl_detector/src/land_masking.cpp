#include <pcl_detector/land_masking.hpp>
#include <pcl_detector/geometry_processor.hpp>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>

#include <pcl/sample_consensus/msac.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <algorithm> 
#include <vector>
#include <cmath>



namespace pcl_detector {

LandMasker::LandMasker(double origin_lat, double origin_lon) : origin_lat_(origin_lat), origin_lon_(origin_lon)
{}

constexpr double LandMasker::deg2rad(double degrees) const
    {
        return degrees * (M_PI / 180.0);
    }

    std::array<double, 2> LandMasker::lla2flat(double lat, double lon) const
    {

        const double R = 6378137.0; // WGS-84 Earth semimajor axis (meters)
        const double f = 1.0 / 298.257223563; // Flattening of the earth
        const double psi_rad = 0.0; // Angular direction of the flat Earth x-axis, specified as a scalar. 
            // The angular direction is the degrees clockwise from north, 
            // which is the angle in degrees used for converting flat Earth x and y coordinates to the north and east coordinates

        // Convert angles from degrees to radians
        const double lat_rad = deg2rad(lat);
        const double lon_rad = deg2rad(lon);
        const double origin_lat_rad = deg2rad(origin_lat_);
        const double origin_lon_rad = deg2rad(origin_lon_);

        // Calculate delta latitude and delta longitude in radians
        const double dlat = lat_rad - origin_lat_rad;
        const double dlon = lon_rad - origin_lon_rad;

        // Radius of curvature in the vertical prime (RN)
        const double RN = R / sqrt(1.0 - (2.0 * f - f * f) * pow(sin(origin_lat_rad), 2));
        
        // Radius of curvature in the meridian (RM)
        const double RM = RN * (1.0 - (2.0 * f - f * f)) / (1.0 - (2.0 * f - f * f) * pow(sin(origin_lat_rad), 2));

        // Changes in the north (dN) and east (dE) positions
        const double dN = RM * dlat;
        const double dE = RN * cos(origin_lat_rad) * dlon;

        // Transformation from North-East to flat x-y coordinates
        const double px = cos(psi_rad) * dN - sin(psi_rad) * dE;
        const double py = sin(psi_rad) * dN + cos(psi_rad) * dE;

        return {px, py};
    }

    void LandMasker::processCoordinates(const std::vector<std::array<double, 2>>& coordinates) {
    for (const auto& coord : coordinates) {
        std::array<double, 2> flat = lla2flat(coord[0], coord[1]);
        pcl::PointXYZ point(flat[0], flat[1], 0); // Assuming z-coordinate is 0
        polygon_.push_back(point);
    }
    }

    void LandMasker::setOrigin() {
        //lidar ori
        origin_lat_ = 63.414614704428324;
        origin_lon_ = 10.398600798333675;
        //lidar pos
        origin_lat_ = 63.414614704428324;
        origin_lon_ = 10.398600798333675;
    }

    void LandMasker::set_polygon() {
        std::vector<std::array<double, 2>> coords = {
        {63.41489120183045}, {10.39705552070149},
        {63.41497265487507}, {10.39783732516908},
        {63.41538547016246}, {10.397651181248118},
        {63.415431749532054}, {10.398250978326427},
        {63.415048554100395}, {10.398449531842004},
        {63.41512075073269}, {10.399185834462275},
        {63.41495229164123}, {10.399305793877936},
        {63.41486158248972}, {10.398540535536645},
        {63.41453021396673}, {10.398747362115373},
        {63.4145246602717}, {10.398631539231284},
        {63.414280296625655}, {10.39877218130482},
        {63.41424142039905}, {10.398197203415958},
        {63.4144598670836}, {10.398110336252891},
        {63.41444505719146}, {10.397990376837232},
        {63.41478938520484}, {10.397841461700548},
        {63.41472089017681}, {10.397167207053899}, };

        processCoordinates(coords);}

    pcl::PointCloud<pcl::PointXYZ> LandMasker::get_polygon() {
        return polygon_;
    }


    void LandMasker::get_land(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const pcl::PointCloud<pcl::PointXYZ>& polygon, std::vector<int>& indices_to_remove)
    {
    
    for (size_t i = 0; i < cloud->points.size(); ++i) {
        const auto& point = cloud->points[i];
        for(size_t j = 0; j < polygon.size(); j++){
            if (isXYPointIn2DXYPolygon(point, polygon)) {
                indices_to_remove.push_back(i);
                break;
            }
        }
    }
    }
    
    bool LandMasker::isXYPointIn2DXYPolygon (const pcl::PointXYZ& point, const pcl::PointCloud<pcl::PointXYZ> &polygon)
{
  bool in_poly = false;
  double x1, x2, y1, y2;

  const auto nr_poly_points = polygon.size ();
  // start with the last point to make the check last point<->first point the first one
  double xold = polygon[nr_poly_points - 1].x;
  double yold = polygon[nr_poly_points - 1].y;
  for (std::size_t i = 0; i < nr_poly_points; i++)
  {
    double xnew = polygon[i].x;
    double ynew = polygon[i].y;
    if (xnew > xold)
    {
      x1 = xold;
      x2 = xnew;
      y1 = yold;
      y2 = ynew;
    }
    else
    {
      x1 = xnew;
      x2 = xold;
      y1 = ynew;
      y2 = yold;
    }

    if ( (xnew < point.x) == (point.x <= xold) && (point.y - y1) * (x2 - x1) < (y2 - y1) * (point.x - x1) )
    {
      in_poly = !in_poly;
    }
    xold = xnew;
    yold = ynew;
  }

  return (in_poly);
}
    

}; // namespace pcl_detector