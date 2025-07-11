#ifndef NAV2_MPPI_CONTROLLER__CRITICS__TRAJECTORY_SUBSCRIBER_HPP_
#define NAV2_MPPI_CONTROLLER__CRITICS__TRAJECTORY_SUBSCRIBER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "messages/msg/pose_with_covariances_array.hpp"
#include "geometry_msgs/msg/pose_with_covariance.hpp"


class TrajectorySubscriber : public rclcpp::Node
{
public:
  TrajectorySubscriber();

private:
  void pos_cov_CB(const messages::msg::PoseWithCovariancesArray::SharedPtr msg);

  rclcpp::Subscription<messages::msg::PoseWithCovariancesArray>::SharedPtr pos_cov_sub_;
};


#endif  // NAV2_MPPI_CONTROLLER__CRITICS__TRAJECTORY_SUBSCRIBER_HPP_
