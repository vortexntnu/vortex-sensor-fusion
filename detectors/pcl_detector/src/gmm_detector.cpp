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
    for (const auto& point : points) {
        pcl::PointXYZRGB colored_point;
        colored_point.x = point.x;
        colored_point.y = point.y;
        colored_point.z = point.z;
        colored_points->push_back(colored_point);
    }

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
        for (const auto& index : indices.indices) {
            const auto& point = points[index];
            cluster_points->push_back(point);
        }

        // Compute GMM for the cluster
        Eigen::MatrixXf data(cluster_points->size(), 3);
        for (size_t i = 0; i < cluster_points->size(); ++i) {
            const auto& point = (*cluster_points)[i];
            data(i, 0) = point.x;
            data(i, 1) = point.y;
            data(i, 2) = point.z;
        }

        Eigen::Vector3f mean = data.colwise().mean();
        Eigen::Matrix3f cov = (data.rowwise() - mean.transpose()).transpose() * (data.rowwise() - mean.transpose());
        cov /= cluster_points->size() - 1;

        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(cov);
        Eigen::Vector3f eigenvalues = solver.eigenvalues();
        Eigen::Matrix3f eigenvectors = solver.eigenvectors();

        // Get the GMM centroids
        std::vector<pcl::PointXYZ> centroids(num_clusters_);
        for (int i = 0; i < num_clusters_; ++i) {
            Eigen::Vector3f direction = eigenvectors.col(i);
            pcl::PointXYZ centroid(mean(0), mean(1), mean(2));
            for (float distance = 0.0; distance < 2.0 * eps_; distance += eps_ / 5.0) {
                centroid.x += direction(0) * distance;
                centroid.y += direction(1) * distance;
                centroid.z += direction(2) * distance;
                centroids[i] = centroid;
            }
        }

        // Add the centroids to the detections
        for (const auto& centroid : centroids) {
            detections.push_back(centroid);
        }
    }

    return detections;

}

} // namespace pcl_detector
