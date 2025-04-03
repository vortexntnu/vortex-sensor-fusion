#include <target_classifier/auction_algorithm.hpp>

Eigen::VectorXi auction_algorithm(const Eigen::MatrixXd &reward_matrix)
{
  
  int num_items              = reward_matrix.rows();
  int num_customers          = reward_matrix.cols();
  Eigen::VectorXi assignment = Eigen::VectorXi::Constant(num_customers, -1);
  Eigen::VectorXd prices     = Eigen::VectorXd::Zero(num_items);

  std::vector<int> unassigned;
  for (int i = 0; i < num_customers; ++i) {
    unassigned.push_back(i);
  }

  double epsilon = 1.0 / (num_items + 1);

  while (!unassigned.empty()) {
    int customer = unassigned.back();
    unassigned.pop_back();

    double max_value = std::numeric_limits<double>::lowest();
    int max_item = -1;
    for (int item = 0; item < reward_matrix.rows(); ++item) {
      double value = reward_matrix.coeff(item, customer) - prices[item];
      if (value > max_value) {
        max_value = value;
        max_item  = item;
      }
    }

    // Find the current owner of max item
    int current_owner = -1;
    for (int i = 0; i < num_customers; ++i) {
      if (assignment[i] == max_item) {
        current_owner = i;
        break;
      }
    }
    if (current_owner != -1) {
      unassigned.push_back(current_owner);
    }

    assignment[customer] = max_item;
    prices[max_item] += max_value + epsilon;
  }

  return assignment;
}
