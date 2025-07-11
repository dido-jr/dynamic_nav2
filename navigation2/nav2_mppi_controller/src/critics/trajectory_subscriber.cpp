#include "nav2_mppi_controller/critics/trajectory_subscriber.hpp"

using std::placeholders::_1;

TrajectorySubscriber::TrajectorySubscriber()
: Node("trajectory_subscriber")
{
  pos_cov_sub_ = this->create_subscription<messages::msg::PoseWithCovariancesArray>(
    "predicted_trajectory", 10, std::bind(&TrajectorySubscriber::pos_cov_CB, this, _1));
}

void TrajectorySubscriber::pos_cov_CB(const messages::msg::PoseWithCovariancesArray::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "Ricevuti %zu pose con covarianza", msg->poses.size());

  for (size_t i = 0; i < msg->poses.size(); ++i) {
    const auto & pose_cov = msg->poses[i];
    const auto & pos = pose_cov.pose.position;
    const auto & cov = pose_cov.covariance;

    RCLCPP_INFO(this->get_logger(), "Punto %ld: x=%.2f, y=%.2f", i, pos.x, pos.y);
    RCLCPP_INFO(this->get_logger(), "  Covarianza: [[%.3f, %.3f], [%.3f, %.3f]]",
                cov[0], cov[1], cov[6], cov[7]);
  }
}

// main
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrajectorySubscriber>());
  rclcpp::shutdown();
  return 0;
}
