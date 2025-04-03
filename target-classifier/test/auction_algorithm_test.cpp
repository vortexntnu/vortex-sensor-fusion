#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <target_classifier/auction_algorithm.hpp>

TEST(AuctionAlgorithmTest, TestAssignment) {
  // Create a cost matrix
  Eigen::MatrixXd reward_matrix(6, 3);
  reward_matrix << 1.0, -1000.0, -1000.0,
                -1000.0, 0.5, 0.3,
                -1000.0, 0.2, 0.6,
                -300.0, -1000.0, -1000.0,
                -1000.0, -300.0, -1000.0,
                -1000.0, -1000.0, -300.0;

  // Call the auction algorithm
  Eigen::VectorXi assignment = auction_algorithm(reward_matrix);

  // Check the assignment
  EXPECT_EQ(assignment[0], 0);
  EXPECT_EQ(assignment[1], 1);
  EXPECT_EQ(assignment[2], 2);
}

TEST(AuctionAlgorithmTest, TestAssignment2) {
  // Create a cost matrix
  Eigen::MatrixXd reward_matrix(3, 3);
  reward_matrix << 1.0, -1000.0, -1000.0,
                -1000.0, 0.5, 0.3,
                -1000.0, 0.2, 0.6;

  std::cout << reward_matrix << std::endl;

  // Call the auction algorithm
  Eigen::VectorXi assignment = auction_algorithm(reward_matrix);

  // Check the assignment
  EXPECT_EQ(assignment[0], 0);
  EXPECT_EQ(assignment[1], 1);
  EXPECT_EQ(assignment[2], 2);
}

TEST(AuctionAlgorithmTest, TestAssignment3) {
  // Create a cost matrix
  Eigen::MatrixXd reward_matrix(7, 4);
  reward_matrix << 1.0, -1000.0, -1000.0, -1000.0,
                -1000.0, 0.5, 0.3, -1000.0,
                -1000.0, 0.2, 0.6, -1000.0,
                -300.0, -1000.0, -1000.0, -1000.0,
                -1000.0, -300.0, -1000.0, -1000.0,
                -1000.0, -1000.0, -300.0, -1000.0,
                -1000.0, -1000.0, -1000.0, -300.0; 

  std::cout << reward_matrix << std::endl;

  // Call the auction algorithm
  Eigen::VectorXi assignment = auction_algorithm(reward_matrix);

  // Check the assignment
  EXPECT_EQ(assignment[0], 0);
  EXPECT_EQ(assignment[1], 1);
  EXPECT_EQ(assignment[2], 2);
  EXPECT_EQ(assignment[3], 6);
}