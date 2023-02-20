#include "pcl_detector/detectors/gmm_detector.hpp"

namespace pcl_detector {


pcl::PointCloud<pcl::PointXYZ> GMMDetector::get_detections(const pcl::PointCloud<pcl::PointXYZ>& points) {
    // Create a search tree for fast nearest neighbor searches
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(points.makeShared());

    // Initialize the GMM parameters
    int num_points = points.size();
    int num_dims = 3;
    int num_components = num_gaussians_;
    int max_iterations = max_iterations_;
    double convergence_threshold = convergence_threshold_;
    
    std::vector<Eigen::Vector3f> means(num_components);
    std::vector<Eigen::Matrix3f> covariances(num_components);
    std::vector<double> priors(num_components, 1.0 / num_components);

    // Initialize the means to random points in the input cloud
    for (int i = 0; i < num_components; i++) {
        int idx = rand() % num_points;
        means[i] = points[idx].getVector3fMap();
    }

    // Initialize the covariances to the identity matrix
    for (int i = 0; i < num_components; i++) {
        covariances[i] = Eigen::Matrix3f::Identity();
    }

    // Run the GMM algorithm to estimate the means, covariances, and priors
    for (int iter = 0; iter < max_iterations; iter++) {
        // E-step: Compute the responsibilities for each component and point
        std::vector<std::vector<double>> responsibilities(num_points, std::vector<double>(num_components));
        for (int i = 0; i < num_points; i++) {
            for (int j = 0; j < num_components; j++) {
                Eigen::Vector3f diff = points[i].getVector3fMap() - means[j];
                Eigen::Matrix3f cov_inv = covariances[j].inverse();
                double exponent = -0.5 * diff.transpose() * cov_inv * diff;
                double denominator = pow(2 * M_PI, num_dims / 2.0) * sqrt(covariances[j].determinant());
                responsibilities[i][j] = priors[j] * exp(exponent) / denominator;
            }
            double sum_responsibilities = std::accumulate(responsibilities[i].begin(), responsibilities[i].end(), 0.0);
            for (int j = 0; j < num_components; j++) {
                responsibilities[i][j] /= sum_responsibilities;
            }
        }

        // M-step: Update the means, covariances, and priors based on the responsibilities
        std::vector<Eigen::Vector3f> new_means(num_components);
        std::vector<Eigen::Matrix3f> new_covariances(num_components);
        std::vector<double> new_priors(num_components);
        for (int j = 0; j < num_components; j++) {
            double sum_responsibilities = 0.0;
            Eigen::Vector3f sum_points = Eigen::Vector3f::Zero();
            Eigen::Matrix3f sum_covariances = Eigen::Matrix3f::Zero();
            for (int i = 0; i < num_points; i++) {
                sum_points += responsibilities[i][j] * points[i].getVector3fMap();
            }
            new_means[j] = sum_points / sum_responsibilities;

            for (int i = 0; i < num_points; i++) {
                Eigen::Vector3f diff = points[i].getVector3fMap() - new_means[j];
                sum_covariances += responsibilities[i][j] * diff * diff.transpose();
            }
            new_covariances[j] = sum_covariances / sum_responsibilities;

            new_priors[j] = sum_responsibilities / num_points;
        }

        // Check for convergence of the means and covariances
        double mean_change = 0.0;
        double covar_change = 0.0;
        for (int j = 0; j < num_components; j++) {
            mean_change += (new_means[j] - means[j]).norm();
            covar_change += (new_covariances[j] - covariances[j]).norm();
        }
        if (mean_change < convergence_threshold && covar_change < convergence_threshold) {
            break;
        }

        // Update the parameters for the next iteration
        means = new_means;
        covariances = new_covariances;
        priors = new_priors;
    }

    // Assign each point to the cluster with the highest probability
    std::vector<int> cluster_labels(num_points, -1);
    for (int i = 0; i < num_points; i++) {
        int max_label = 0;
        double max_prob = 0.0;
        for (int j = 0; j < num_components; j++) {
            Eigen::Vector3f diff = points[i].getVector3fMap() - means[j];
            Eigen::Matrix3f cov_inv = covariances[j].inverse();
            double exponent = -0.5 * diff.transpose() * cov_inv * diff;
            double denominator = pow(2 * M_PI, num_dims / 2.0) * sqrt(covariances[j].determinant());
            double prob = priors[j] * exp(exponent) / denominator;
            if (prob > max_prob) {
                max_prob = prob;
                max_label = j;
            }
        }
        cluster_labels[i] = max_label;
    }

    // Extract the centroids of the clusters as the detections
    pcl::PointCloud<pcl::PointXYZ> detections;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> cluster_extraction;
    cluster_extraction.setClusterTolerance(cluster_tolerance_);
    cluster_extraction.setMinClusterSize(min_cluster_size_);
    cluster_extraction.setMaxClusterSize(points.size());
    cluster_extraction.setSearchMethod(tree);
    cluster_extraction.setInputCloud(points.makeShared());
    cluster_extraction.extract(cluster_labels, cluster_indices_);

    for (const auto& indices : cluster_indices_) {
        Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
        for (const auto& index : indices) {
            centroid += points[index].getVector3fMap();
        }
        centroid /= indices.size();
        detections.push_back(pcl::PointXYZ(centroid[0], centroid[1], centroid[2]));
    }

    return detections;


}; // namespace pcl_detector
