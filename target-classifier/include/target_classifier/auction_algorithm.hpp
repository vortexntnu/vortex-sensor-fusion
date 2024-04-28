#pragma once

#include <limits>
#include <vector>
#include <Eigen/Dense>


Eigen::VectorXi auction_algorithm(const Eigen::MatrixXd &cost_matrix);

