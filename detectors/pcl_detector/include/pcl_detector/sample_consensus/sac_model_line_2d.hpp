/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2009, Willow Garage, Inc.
 *  Copyright (c) 2012-, Open Perception, Inc.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id$
 *
 */

// #ifndef PCL_SAMPLE_CONSENSUS_IMPL_SAC_MODEL_LINE_2D_H_
// #define PCL_SAMPLE_CONSENSUS_IMPL_SAC_MODEL_LINE_2D_H_

#include <pcl/common/centroid.h>
#include <pcl/common/concatenate.h>
// #include <pcl/common/eigen.h> // for eigen33
#include <Eigen/Eigenvalues>
#include "sac_model_line_2d.h"


//////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
pcl::SampleConsensusModelLine2D<PointT>::isSampleGood (const Indices &samples) const
{
  if (samples.size () != sample_size_)
  {
    PCL_ERROR ("[pcl::SampleConsensusModelLine2D::isSampleGood] Wrong number of samples (is %lu, should be %lu)!\n", samples.size (), sample_size_);
    return (false);
  }

  // Make sure that the two sample points are not identical
  if (
      std::abs ((*input_)[samples[0]].x - (*input_)[samples[1]].x) <= std::numeric_limits<float>::epsilon ()
    &&
      std::abs ((*input_)[samples[0]].y - (*input_)[samples[1]].y) <= std::numeric_limits<float>::epsilon ())
  {
    PCL_ERROR ("[pcl::SampleConsensusModelLine2D::isSampleGood] The two sample points are (almost) identical!\n");
    return (false);
  }

  return (true);
}

//////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
pcl::SampleConsensusModelLine2D<PointT>::computeModelCoefficients (
      const Indices &samples, Eigen::VectorXf &model_coefficients) const
{
  // Make sure that the samples are valid
  if (!isSampleGood (samples))
  {
    PCL_ERROR ("[pcl::SampleConsensusModelLine2D::computeModelCoefficients] Invalid set of samples given!\n");
    return (false);
  }

  model_coefficients.resize (model_size_);
  model_coefficients[0] = (*input_)[samples[0]].x;
  model_coefficients[1] = (*input_)[samples[0]].y;

  model_coefficients[2] = (*input_)[samples[1]].x - model_coefficients[0];
  model_coefficients[3] = (*input_)[samples[1]].y - model_coefficients[1];

  // This precondition should hold if the samples have been found to be good
  assert (model_coefficients.template tail<2> ().squaredNorm () > 0.0f);

  model_coefficients.template tail<2> ().normalize ();
  PCL_DEBUG ("[pcl::SampleConsensusModelLine2D::computeModelCoefficients] Model is (%g,%g,%g,%g).\n",
             model_coefficients[0], model_coefficients[1], model_coefficients[2],
             model_coefficients[3]);
  return (true);
}

//////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::SampleConsensusModelLine2D<PointT>::getDistancesToModel (
      const Eigen::VectorXf &model_coefficients, std::vector<double> &distances) const
{
  // Needs a valid set of model coefficients
  if (!isModelValid (model_coefficients))
  {
    return;
  }

  distances.resize (indices_->size ());

  // Obtain the line point and direction
  Eigen::Vector2f line_pt  (model_coefficients[0], model_coefficients[1]);
  Eigen::Vector2f line_dir (model_coefficients[2], model_coefficients[3]);
  line_dir.normalize ();

  // Iterate through the points points and calculate the distances from them to the line ignoring z component
  for (std::size_t i = 0; i < indices_->size (); ++i)
  {
     // Extract the 2D point from the input point cloud
    Eigen::Vector2f point((*input_)[(*indices_)[i]].x, (*input_)[(*indices_)[i]].y);

    // Calculate the distance from the point to the line
    // Distance formula for point to line in 2D: |(point - line_pt) x line_dir| / |line_dir|
    // Cross product in 2D is a scalar value: (x1 * y2 - y1 * x2)
    distances[i] = std::abs((point.x() - line_pt.x()) * line_dir.y() - (point.y() - line_pt.y()) * line_dir.x());
  }
}

//////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::SampleConsensusModelLine2D<PointT>::selectWithinDistance (
      const Eigen::VectorXf &model_coefficients, const double threshold, Indices &inliers)
{
  // Needs a valid set of model coefficients
  if (!isModelValid (model_coefficients))
    return;

  double sqr_threshold = threshold * threshold;

  inliers.clear ();
  error_sqr_dists_.clear ();
  inliers.reserve (indices_->size ());
  error_sqr_dists_.reserve (indices_->size ());

  // Obtain the line point and direction
  Eigen::Vector2f line_pt  (model_coefficients[0], model_coefficients[1]);
  Eigen::Vector2f line_dir (model_coefficients[2], model_coefficients[3]);
  line_dir.normalize ();

  // Iterate through the 3d points and calculate the distances from them to the line
  for (std::size_t i = 0; i < indices_->size (); ++i)
  {
     // Extract the 2D point from the input point cloud
    Eigen::Vector2f point((*input_)[(*indices_)[i]].x, (*input_)[(*indices_)[i]].y);

    // Calculate the squared distance from the point to the line
    // We use squared difference to be compatible with functions defined in SampleConsensusModel
    Eigen::Vector2f diff = point - line_pt;
    double cross_product = diff.x() * line_dir.y() - diff.y() * line_dir.x();
    double sqr_distance = cross_product * cross_product; // Use squared distance

    if (sqr_distance < sqr_threshold)
    {
      // Returns the indices of the points whose squared distances are smaller than the threshold
      inliers.push_back ((*indices_)[i]);
      error_sqr_dists_.push_back (sqr_distance);
    }
  }
}

//////////////////////////////////////////////////////////////////////////
template <typename PointT> std::size_t
pcl::SampleConsensusModelLine2D<PointT>::countWithinDistance (
      const Eigen::VectorXf &model_coefficients, const double threshold) const
{
  // Needs a valid set of model coefficients
  if (!isModelValid (model_coefficients))
    return (0);

  std::size_t nr_p = 0;

  // Obtain the line point and direction
  Eigen::Vector2f line_pt  (model_coefficients[0], model_coefficients[1]);
  Eigen::Vector2f line_dir (model_coefficients[2], model_coefficients[3]);
  line_dir.normalize ();

  // Iterate through the points points and calculate the distances from them to the line ignoring z component
  for (std::size_t i = 0; i < indices_->size (); ++i)
  {
     // Extract the 2D point from the input point cloud
    Eigen::Vector2f point((*input_)[(*indices_)[i]].x, (*input_)[(*indices_)[i]].y);

    // Calculate the distance from the point to the line
    // Distance formula for point to line in 2D: |(point - line_pt) x line_dir| / |line_dir|
    // Cross product in 2D is a scalar value: (x1 * y2 - y1 * x2)
    double distance = std::abs((point.x() - line_pt.x()) * line_dir.y() - (point.y() - line_pt.y()) * line_dir.x());
  
    if (distance < threshold)
      nr_p++;
  }
  return (nr_p);
}

template <typename PointT> inline unsigned int
compute2DCentroid (const pcl::PointCloud<PointT> &cloud,
                   const pcl::Indices &indices,
                   Eigen::Vector2f &centroid)
{
  if (indices.empty ())
    return (0);

  // Initialize to 0
  centroid.setZero ();
  // If the data is dense, we don't need to check for NaN
  if (cloud.is_dense)
  {
    for (const auto& index : indices)
    {
      centroid[0] += cloud[index].x;
      centroid[1] += cloud[index].y;
    }
    centroid /= static_cast<float> (indices.size ());
    return (static_cast<unsigned int> (indices.size ()));
  }
  // NaN or Inf values could exist => check for them
    unsigned cp = 0;
  for (const auto& index : indices)
  {
    // Check if the point is invalid
    if (!isFinite (cloud [index]))
      continue;

    centroid[0] += cloud[index].x;
    centroid[1] += cloud[index].y;
    ++cp;
  }
  centroid /= static_cast<float> (cp);
  return (cp);
}

template <typename PointT> inline unsigned int
computeCovarianceMatrix2D (const pcl::PointCloud<PointT> &cloud,
                         const pcl::Indices &indices,
                         const Eigen::Vector2f &centroid,
                         Eigen::Matrix<float, 2, 2> &covariance_matrix)
{
  if (indices.empty ())
    return (0);

  // Initialize to 0
  covariance_matrix.setZero ();

  std::size_t point_count;
  // If the data is dense, we don't need to check for NaN
  if (cloud.is_dense)
  {
    point_count = indices.size ();
    // For each point in the cloud
    for (const auto& idx: indices)
    {
      Eigen::Matrix<float, 2, 1> pt;
      pt[0] = cloud[idx].x - centroid[0];
      pt[1] = cloud[idx].y - centroid[1];

      covariance_matrix(0, 0) += pt.x() * pt.x();
      covariance_matrix(0, 1) += pt.x() * pt.y();
      covariance_matrix(1, 1) += pt.y() * pt.y();

    }
  }
  // NaN or Inf values could exist => check for them
  else
  {
    point_count = 0;
    // For each point in the cloud
    for (const auto &index : indices)
    {
      // Check if the point is invalid
      if (!isFinite (cloud[index]))
        continue;

      Eigen::Matrix<float, 2, 1> pt;
      pt[0] = cloud[index].x - centroid[0];
      pt[1] = cloud[index].y - centroid[1];

      covariance_matrix(0, 0) += pt.x() * pt.x();
      covariance_matrix(0, 1) += pt.x() * pt.y();
      covariance_matrix(1, 1) += pt.y() * pt.y();
      ++point_count;
    }
  }
  covariance_matrix (1, 0) = covariance_matrix (0, 1);
  return (static_cast<unsigned int> (point_count));
}

template <typename PointT> void
pcl::SampleConsensusModelLine2D<PointT>::optimizeModelCoefficients(
      const Indices &inliers, const Eigen::VectorXf &model_coefficients, Eigen::VectorXf &optimized_coefficients) const
{
  // Needs a valid set of model coefficients
  if (!isModelValid(model_coefficients))
  {
    optimized_coefficients = model_coefficients;
    return;
  }

  // Need more than the minimum sample size to make a difference
  if (inliers.size() <= sample_size_)
  {
    PCL_ERROR("[pcl::SampleConsensusModelLine2D::optimizeModelCoefficients] Not enough inliers to refine/optimize the model's coefficients (%lu)! Returning the same coefficients.\n", inliers.size());
    optimized_coefficients = model_coefficients;
    return;
  }

  optimized_coefficients.resize(model_size_);

  // Compute the 2D centroid
  Eigen::Vector2f centroid;
  if (0 == compute2DCentroid (*input_, inliers, centroid))
  {
    PCL_WARN ("[pcl::SampleConsensusModelLine2D::optimizeModelCoefficients] compute2DCentroid failed (returned 0) because there are no valid inliers.\n");
    optimized_coefficients = model_coefficients;
    return;
  }

  Eigen::Matrix2f covariance_matrix;
  computeCovarianceMatrix2D(*input_, inliers, centroid, covariance_matrix);
  optimized_coefficients[0] = centroid[0];
  optimized_coefficients[1] = centroid[1];

  // Eigen::Vector2f eigen_values;
  // Eigen::Vector2f eigen_vector;
  // pcl::eigen22(covariance_matrix, eigen_values);
  // pcl::computeCorrespondingEigenVector(covariance_matrix, eigen_values[1], eigen_vector)
 
  // Extract the eigenvalues and eigenvectors
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f> solver(covariance_matrix);

  if (solver.info() != Eigen::Success)
  {
    PCL_WARN ("[pcl::SampleConsensusModelLine2D::optimizeModelCoefficients] EigenSolver failed.\n");
    optimized_coefficients = model_coefficients;
    return;
  }

  // The direction of the line is given by the eigenvector corresponding to the largest eigenvalue of the covariance matrix.
  Eigen::Vector2f eigen_vector = solver.eigenvectors().col(1);

  optimized_coefficients[2] = eigen_vector[0];
  optimized_coefficients[3] = eigen_vector[1];
}


//////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::SampleConsensusModelLine2D<PointT>::projectPoints (
      const Indices &inliers, const Eigen::VectorXf &model_coefficients, PointCloud &projected_points, bool copy_data_fields) const
{
  // Needs a valid model coefficients
  if (!isModelValid (model_coefficients))
  {
    PCL_ERROR ("[pcl::SampleConsensusModelLine2D::projectPoints] Given model is invalid!\n");
    return;
  }

  // Obtain the line point and direction
  Eigen::Vector2f line_pt  (model_coefficients[0], model_coefficients[1]);
  Eigen::Vector2f line_dir (model_coefficients[2], model_coefficients[3]);
  line_dir.normalize();  // Ensure the direction vector is normalized

  projected_points.header = input_->header;
  projected_points.is_dense = input_->is_dense;

  // Copy all the data fields from the input cloud to the projected one?
  if (copy_data_fields)
  {
    // Allocate enough space and copy the basics
    projected_points.resize (input_->size ());
    projected_points.width    = input_->width;
    projected_points.height   = input_->height;

    using FieldList = typename pcl::traits::fieldList<PointT>::type;
    // Iterate over each point
    for (std::size_t i = 0; i < projected_points.size (); ++i)
      // Iterate over each dimension and copy data
      pcl::for_each_type <FieldList> (NdConcatenateFunctor <PointT, PointT> ((*input_)[i], projected_points[i]));

    // Iterate through the 3d points and project the xy coordinate on the line.
    for (const auto &inlier : inliers)
    {
      Eigen::Vector2f pt ((*input_)[inlier].x, (*input_)[inlier].y);
      float k = (pt - line_pt).dot (line_dir);

      Eigen::Vector2f pp = line_pt + k * line_dir;
      // Calculate the projection of the point on the line (pointProj = A + k * B)
      projected_points[inlier].x = pp[0];
      projected_points[inlier].y = pp[1];
    }
  }
  else
  {
    // Allocate enough space and copy the basics
    projected_points.resize (inliers.size ());
    projected_points.width    = inliers.size ();
    projected_points.height   = 1;

    using FieldList = typename pcl::traits::fieldList<PointT>::type;
    // Iterate over each point
    for (std::size_t i = 0; i < inliers.size (); ++i)
      // Iterate over each dimension
      pcl::for_each_type <FieldList> (NdConcatenateFunctor <PointT, PointT> ((*input_)[inliers[i]], projected_points[i]));

    // Iterate through the 3d points and calculate the distances from them to the line
    for (std::size_t i = 0; i < inliers.size (); ++i)
    {
      Eigen::Vector2f pt ((*input_)[inliers[i]].x, (*input_)[inliers[i]].y);
      float k = (pt - line_pt).dot (line_dir);

      Eigen::Vector2f pp = line_pt + k * line_dir;
      // Calculate the projection of the point on the line (pointProj = A + k * B)
      projected_points[i].x = pp[0];
      projected_points[i].y = pp[1];
    }
  }
}

//////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
pcl::SampleConsensusModelLine2D<PointT>::doSamplesVerifyModel (
      const std::set<index_t> &indices, const Eigen::VectorXf &model_coefficients, const double threshold) const
{
  // Needs a valid set of model coefficients
  if (!isModelValid (model_coefficients))
    return (false);

  // Obtain the line point and direction
  Eigen::Vector2f line_pt  (model_coefficients[0], model_coefficients[1]);
  Eigen::Vector2f line_dir (model_coefficients[2], model_coefficients[3]);
  line_dir.normalize ();

  // Iterate through the 3d points and calculate the distances from them to the line ignoring z component
  for (const auto &index : indices)
  {

     // Extract the 2D point from the input point cloud
    Eigen::Vector2f point((*input_)[index].x, (*input_)[index].y);

    // Calculate the distance from the point to the line
    // Distance formula for point to line in 2D: |(point - line_pt) x line_dir| / |line_dir|
    // Cross product in 2D is a scalar value: (x1 * y2 - y1 * x2)
    if (std::abs((point.x() - line_pt.x()) * line_dir.y() - (point.y() - line_pt.y()) * line_dir.x()) > threshold)
      return (false);
  }

  return (true);
}

#define PCL_INSTANTIATE_SampleConsensusModelLine2D(T) template class PCL_EXPORTS pcl::SampleConsensusModelLine2D<T>;

// #endif    // PCL_SAMPLE_CONSENSUS_IMPL_SAC_MODEL_LINE_2D_H_