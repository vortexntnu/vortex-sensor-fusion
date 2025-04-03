#pragma once

#include <limits>
#include <vector>
#include <Eigen/Dense>


Eigen::VectorXi auction_algorithm(const Eigen::MatrixXd &reward_matrix);

