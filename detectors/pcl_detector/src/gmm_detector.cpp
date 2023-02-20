#include "pcl_detector/detectors/gmm_detector.hpp"


namespace pcl_detector {

GMMDetector::GMMDetector(int num_clusters, int max_iterations)
    : num_clusters_(num_clusters), max_iterations_(max_iterations) {}

pcl::PointCloud<pcl::PointXYZ> GMMDetector::get_detections(const pcl::PointCloud<pcl::PointXYZ>& points) {
    // Compute surface normals
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
    normal_estimator.setInputCloud(points.makeShared());
    normal_estimator.setRadiusSearch(0.03);
    normal_estimator.compute(*normals);

    // Compute smoothed surface normals
    pcl::PointCloud<pcl::PointXYZ>::Ptr smoothed_points(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(points.makeShared());
    sor.setMeanK(50);
    sor.setStddevMulThresh(1.0);
    sor.filter(*smoothed_points);

    pcl::PointCloud<pcl::Normal>::Ptr smoothed_normals(new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> smoothed_normal_estimator;
    smoothed_normal_estimator.setInputCloud(smoothed_points);
    smoothed_normal_estimator.setSearchMethod(pcl::search::KdTree<pcl::PointXYZ>::Ptr(new pcl::search::KdTree<pcl::PointXYZ>));
    smoothed_normal_estimator.setRadiusSearch(0.05);
    smoothed_normal_estimator.compute(*smoothed_normals);

    // Compute GMM clusters
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_points(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::copyPointCloud(points, *colored_points);

    pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
    reg.setInputCloud(points.makeShared());
    reg.setInputNormals(normals);
    reg.setSmoothModeFlag(true);
    reg.setCurvatureTestFlag(true);
    reg.setCurvatureThreshold(0.1);
    reg.setSmoothnessThreshold(2.0 / 180.0 * M_PI);
    reg.setResidualThreshold(0.05);
    reg.setNumberOfNeighbours(10);
    reg.setSearchMethod(pcl::search::KdTree<pcl::PointXYZ>::Ptr(new pcl::search::KdTree<pcl::PointXYZ>));
    reg.extract(*colored_points);

    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr kdtree(new pcl::search::KdTreepcl::PointXYZRGB);
    kdtree->setInputCloud(colored_points);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
    ec.setClusterTolerance(0.05);
    ec.setMinClusterSize(100);
    ec.setMaxClusterSize(25000);
    ec.setSearchMethod(kdtree);
    ec.setInputCloud(colored_points);
    ec.extract(cluster_indices);

    pcl::PointCloud<pcl::PointXYZ> detections;
    detections.reserve(cluster_indices.size() * num_clusters_);

    for (const auto& indices : cluster_indices) {
        if (indices.indices.size() < num_clusters_) {
            continue;
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_points(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::copyPointCloud(*colored_points, indices.indices, *cluster_points);

        // Compute GMM for the cluster
        pcl::GaussianMixtureModel<pcl::PointXYZ> gmm(num_clusters_);
        gmm.setInputCloud(cluster_points);
        gmm.setMaxIterations(max_iterations_);
        gmm.setCovarianceType(pcl::GaussianMixtureModel<pcl::PointXYZ>::COVARIANCE_DIAGONAL);
        gmm.compute();

        // Get the GMM centroids
        std::vector<pcl::PointXYZ> centroids(num_clusters_);
        for (int i = 0; i < num_clusters_; ++i) {
            centroids[i] = gmm.getCentroid(i);
        }

        // Add the centroids to the detections
        for (const auto& centroid : centroids) {
            detections.push_back(centroid);
        }
    }

    return detections;
}

} // namespace pcl_detector
