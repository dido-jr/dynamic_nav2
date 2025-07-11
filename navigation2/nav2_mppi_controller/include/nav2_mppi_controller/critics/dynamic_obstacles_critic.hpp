#ifndef NAV2_MPPI_CONTROLLER__CRITICS__DYNAMIC_OBSTACLES_CRITIC_HPP_
#define NAV2_MPPI_CONTROLLER__CRITICS__DYNAMIC_OBSTACLES_CRITIC_HPP_

// #include <rclcpp/rclcpp.hpp>
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "nav2_mppi_controller/critic_function.hpp"
#include "nav2_mppi_controller/models/state.hpp"
#include "nav2_mppi_controller/tools/utils.hpp"

#include "messages/msg/pose_with_covariances_array.hpp"

#include <mutex>
#include <xtensor/xtensor.hpp>

namespace mppi::critics
{

/**
 * @class mppi::critics::ConstraintCritic
 * @brief Critic objective function for driving towards goal
 */
class DynamicObstaclesCritic : public CriticFunction
{
public:
  /**
    * @brief Initialize critic
    */
  void initialize() override;

  /**
   * @brief Evaluate cost related to goal following
   *
   * @param costs [out] add reference cost values to this tensor
   */
  void score(CriticData & data) override;

  void GetDynamicObstacles();

protected:
  float vr_{0.0f};
  xt::xtensor<float, 0> Vr_ = {0.0f};  // Robot volume
  float collision_cost_{0};

  float loc_{0.0f};  // Level of Certainty
  xt::xtensor<float, 0> LoC_ = {0.0f};
  float near_goal_distance_;

  unsigned int power_{0};
  float repulsion_weight_, critical_weight_{0};
  float k1_{0.3f}, k2_{0.2f};  // Constants for covariance calculation

  std::mutex my_mutex_;

  rclcpp::Subscription<messages::msg::PoseWithCovariancesArray>::SharedPtr dynamic_obstacles_sub_;
  messages::msg::PoseWithCovariancesArray dynamic_obstacles_;
};

}  // namespace mppi::critics

#endif  // NAV2_MPPI_CONTROLLER__CRITICS__DYNAMIC_OBSTACLES_CRITIC_HPP_
