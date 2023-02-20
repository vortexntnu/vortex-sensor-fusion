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

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
    ec.setClusterTolerance(0.05);
    ec.setMinClusterSize(100);
    ec.setMaxClusterSize(25000);
    ec.setInputCloud(colored_points);
    ec.extract(cluster_indices);

    pcl::PointCloud<pcl::PointXYZ> detections;
    detections.reserve(cluster_indices.size() * num_clusters_);

    for (const auto& indices : cluster_indices) {
        if (indices.indices.size() < num_clusters_) {
            continue;
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_points(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::copyPointCloud(*colored_points, indices.indices, *cluster _points);
 
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

        // Store the individual clusters
        for (const auto& index : indices.indices) {
            pcl::PointXYZ point = points[index];
            double distance = std::numeric_limits<double>::infinity();
            int nearest_cluster = -1;
            for (size_t i = 0; i < centroids.size(); ++i) {
                double d = pcl::euclideanDistance(point, centroids[i]);
                if (d < distance) {
                    distance = d;
                    nearest_cluster = i;
                }
            }
            if (nearest_cluster >= 0) {
                while (nearest_cluster >= clusters.size()) {
                    clusters.emplace_back();
                }
                clusters[nearest_cluster].push_back(point);
            }
        }
    }

    // Output the individual clusters
    std::vector<pcl::PointCloud<pcl::PointXYZ>> output_clusters;
    for (const auto& cluster : clusters) {
        if (cluster.empty()) {
            continue;
        }
        output_clusters.emplace_back();
        output_clusters.back().reserve(cluster.size());
        for (const auto& point : cluster) {
            output_clusters.back().push_back(point);
        }
    }

    return detections;
}

} // namespace pcl_detector
