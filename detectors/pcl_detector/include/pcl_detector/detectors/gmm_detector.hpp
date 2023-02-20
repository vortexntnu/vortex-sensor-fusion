/**
 * @file gmm_detector.hpp
 * @brief This file defines a GMM detector that inherits from the IPclDetector interface.
 */

#ifndef GMM_DETECTOR_H
#define GMM_DETECTOR_H

#include <pcl/segmentation/gmm.h>

#include "pcl_detector.hpp"

namespace pcl_detector {

/**
 * @class GMMDetector
 * @brief A GMM detector that implements the IPclDetector interface.
 */
class GMMDetector : public IPclDetector {

public:
    /**
     * @brief Constructs a GMM detector with the given parameters.
     * @param num_gaussians The number of Gaussian components to fit to the data.
     * @param max_iterations The maximum number of iterations to run the GMM algorithm.
     * @param convergence_threshold The convergence threshold for the GMM algorithm.
     */
    GMMDetector(int num_gaussians, int max_iterations, double convergence_threshold)
        : num_gaussians_(num_gaussians), max_iterations_(max_iterations), convergence_threshold_(convergence_threshold) {}

    /**
     * @brief Detects clusters in the given point cloud using GMM.
     * @param points The input point cloud.
     * @return The detected clusters as a point cloud.
     */
    pcl::PointCloud<pcl::PointXYZ> get_detections(const pcl::PointCloud<pcl::PointXYZ>& points) override;

private:
    int num_gaussians_; /**< The number of Gaussian components to fit to the data. */
    int max_iterations_; /**< The maximum number of iterations to run the GMM algorithm. */
    double convergence_threshold_; /**< The convergence threshold for the GMM algorithm. */

};

}; // namespace pcl_detector

#endif // GMM_DETECTOR_H
