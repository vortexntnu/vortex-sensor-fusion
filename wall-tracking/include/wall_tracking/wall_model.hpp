#include <vortex_filtering/models/dynamic_model_interfaces.hpp>

namespace vortex::models {

// constexpr int UNUSED = 1; // For when a template parameter is required but not used.

/** Wall Model that describes the position of the end points of a wall.
 * State x = [x1, y1, x2, y2]
 * @tparam n_spatial_dim Number of spatial dimensions
 */
class Wall : public interface::DynamicModelLTV<4> {
  using Parent = interface::DynamicModelLTV<4>;

public:
  static constexpr int N_DIM_x = Parent::N_DIM_x;
  static constexpr int N_DIM_u = Parent::N_DIM_u;
  static constexpr int N_DIM_v = Parent::N_DIM_v;

  using T = vortex::Types_xuv<N_DIM_x, N_DIM_u, N_DIM_v>;

  /** Wall Model
   * x = [x1, y1, x2, y2]
   * @param std_pos Standard deviation of position
   */
  Wall(double std_pos)
      : std_pos_(std_pos)
  {
  }

  /** Get the Jacobian of the continuous state transition model with respect to the state.
   * @param dt Time step
   * @param x State (unused)
   * @return T::Mat_xx
   * @note Overriding DynamicModelLTV::A_d
   */
  T::Mat_xx A_d(double /*dt*/, const T::Vec_x /*x*/ & = T::Vec_x::Zero()) const override { return T::Mat_xx::Identity(); }

  /** Get the Jacobian of the continuous state transition model with respect to the process noise.
   * @param dt Time step
   * @param x State (unused)
   * @return T::Mat_xv
   * @note Overriding DynamicModelLTV::G_d
   */
  T::Mat_xv G_d(double dt, const T::Vec_x /*x*/ & = T::Vec_x::Zero()) const override
  {
    T::Mat_xx I = T::Mat_xx::Identity();
    return 0.5 * dt * I;
  }

  /** Get the continuous time process noise covariance matrix.
   * @param dt Time step (unused)
   * @param x State (unused)
   * @return T::Mat_xx Process noise covariance
   * @note Overriding DynamicModelLTV::Q_d
   */
  T::Mat_vv Q_d(double /*dt*/ = 0.0, const T::Vec_x /*x*/ & = T::Vec_x::Zero()) const override { return T::Mat_vv::Identity() * std_pos_ * std_pos_; }

private:
  double std_pos_;
};

} // namespace vortex::models