#include "nav2_mppi_controller/critics/dynamic_obstacles_critic.hpp"
#include <xtensor/xmath.hpp>
#include <xtensor-blas/xlinalg.hpp>
#include <cmath>
#include <Eigen/Core>

namespace mppi::critics
{

using xt::evaluation_strategy::immediate;

void DynamicObstaclesCritic::initialize()
{
  auto getParam = parameters_handler_->getParamGetter(name_);

  getParam(vr_, "robot_volume", 0.0);
  getParam(power_, "cost_power", 1);
  getParam(repulsion_weight_, "repulsion_weight", 1.5);
  getParam(critical_weight_, "critical_weight", 20.0);
  getParam(collision_cost_, "collision_cost", 10000.0);
  getParam(loc_, "level_of_certainty", 0.05);
  getParam(near_goal_distance_, "near_goal_distance", 0.5);

  LoC_() = loc_;
  Vr_() = vr_;

  this->GetDynamicObstacles();

  RCLCPP_INFO(
    logger_,
    "DynamicObstaclesCritic instantiated with %d power and %f / %f weights. ",
    power_, critical_weight_, repulsion_weight_);
}

void DynamicObstaclesCritic::GetDynamicObstacles()
{
    auto node = parent_.lock();
    dynamic_obstacles_sub_ = node->create_subscription<messages::msg::PoseWithCovariancesArray>(
        "/predicted_trajectory",
        rclcpp::QoS(10),
        [this](const messages::msg::PoseWithCovariancesArray::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(my_mutex_);
            dynamic_obstacles_ = *msg;
        });
    // RCLCPP_INFO(logger_, "DynamicObstaclesCritic subscribed to dynamic obstacles topic.");
}

void DynamicObstaclesCritic::score(CriticData & data)
{
    // if (!enabled_)
    // {
    //     return;
    // }

    // const size_t traj_len = data.trajectories.x.shape(1);
    // const size_t traj_obs_len = dynamic_obstacles_.poses.size();
    // int colliding_count = 0;

    // if (traj_obs_len > 0) {
    //     bool all_trajectories_collide = true;
    //     auto && raw_cost = xt::xtensor<float, 1>::from_shape({data.costs.shape(0)});
    //     raw_cost.fill(0.0f);
    //     auto && repulsive_cost = xt::xtensor<float, 1>::from_shape({data.costs.shape(0)});
    //     repulsive_cost.fill(0.0f);

    //     for (size_t i = 0; i < data.trajectories.x.shape(0); ++i){
    //         bool trajectory_collide = false;
    //         xt::xtensor<float,2> sigma_inv = xt::zeros<float>({2, 2});
    //         xt::xtensor<float,2> robot_pos = xt::zeros<float>({2,1});
    //         xt::xtensor<float,2> obs_pos = xt::zeros<float>({2,1});
    //         const auto & traj = data.trajectories;

    //         for (size_t j = 0; j < traj_obs_len; j++) {
    //             // Robot position and velocity
    //             float robot_x = traj.x(i,j);
    //             float robot_y = traj.y(i,j);
    //             robot_pos[0] = robot_x;
    //             robot_pos[1] = robot_y;
    //             float robot_cvx = data.state.cvx(i,j);
    //             float robot_cvy = data.state.cvy(i,j);
    //             // Compute robot covariance
    //             float robot_covx = k1_ * j + k2_ * robot_cvx;
    //             float robot_covy = k1_ * j + k2_ * robot_cvy;

    //             // Obstacle position and covariance
    //             {
    //                 std::lock_guard<std::mutex> lock(my_mutex_);
    //                 auto pose_cov = dynamic_obstacles_.poses[j];
    //                 float obs_x = pose_cov.pose.position.x;
    //                 float obs_y = pose_cov.pose.position.y;
    //                 obs_pos[0] = obs_x;
    //                 obs_pos[1] = obs_y;
    //                 float obs_covx = pose_cov.covariance[0];
    //                 float obs_covy = pose_cov.covariance[7];
                
    //                 // Compute costs
    //                 float covx = std::max(1e-3f, robot_covx + obs_covx); // Avoid division by zero
    //                 float covy = std::max(1e-3f, robot_covy + obs_covy);
    //                 sigma_inv = {{1.0f / covx,          0.0f},  
    //                              {0.0f,           1.0f / covy}};
    //                 auto distance = robot_pos - obs_pos;
    //                 auto distance_T = xt::transpose(distance); //xt::transpose(xt::reshape_view(distance, {1, 2}));
    //                 float dx = robot_x - obs_x;
    //                 float dy = robot_y - obs_y;
    //                 float norm = std::sqrt(dx*dx + dy*dy);
    //                 // Distance
    //                 // auto norm = xt::linalg::norm(distance);
    //                 if (norm < 5) {
    //                     // raw_cost[i] += 1/norm; //1 / (xt::linalg::norm(distance) + 5e-2f); // Avoid division by zer
    //                     // Probabilistic distance
    //                     // float prob_distance = (distance_T * sigma_inv * distance)(0, 0);
    //                     float prob_distance = xt::linalg::dot(xt::linalg::dot(distance_T, sigma_inv), distance)(0,0);
    //                     // repulsive_cost[i] += 0; //1 / prob_distance;
    //                     auto det = xt::eval(xt::xtensor<float, 0>::from_shape({}));
    //                     det = std::max(1e-3f, (2.0f*3.1415f*covx)*(2.0f*3.1415f*covy)); 
    //                     float k = xt::eval(-2.0f * xt::log(xt::sqrt(det) * LoC_/Vr_))();
    //                     // if (j% 20 == 0) {
    //                         //RCLCPP_INFO(logger_, "distance = %f, prob_distance = %f, k = %f", norm, prob_distance, k);
    //                         // RCLCPP_INFO(logger_, "obstacle position: (%f, %f)", obs_x, obs_y);
    //                         // RCLCPP_INFO(logger_, "robot position: (%f, %f)", robot_x, robot_y);
    //                     // }
    //                     // if (prob_distance - norm > 1.0f){
    //                     //     RCLCPP_INFO(logger_, "prob_distance, norm, k = %f, %f, %f", prob_distance, norm, k);
    //                     // }
    //                     // float k = 0.7f;
    //                     // float prob_distance = norm;
    //                     if (prob_distance < k) {
    //                         // RCLCPP_INFO(logger_, "k = %f, repulsive_cost = %f", k, prob_distance);
    //                         // RCLCPP_INFO(logger_, "obstacle position: (%f, %f)", obs_x, obs_y);
    //                         // RCLCPP_INFO(logger_, "robot position: (%f, %f)", robot_x, robot_y);
    //                         trajectory_collide = true;
    //                         repulsive_cost[i] = collision_cost_;
    //                         prob_distance = std::max(prob_distance, 1e-3f); // Avoid division by zero
    //                         raw_cost[i] = collision_cost_ + std::log(1/prob_distance);
    //                         colliding_count++;
    //                         break;
    //                     }
    //                     else{
    //                         raw_cost[i] += 1/norm;
    //                     }
    //                 }
    //             }
    //         }
    //         if (!trajectory_collide) {all_trajectories_collide = false;}
    //     }

    //     RCLCPP_INFO(logger_, "Traiettorie in collisione: %d su %lu", colliding_count, data.trajectories.x.shape(0));

    //     data.costs += xt::pow(
    //         (repulsion_weight_ * raw_cost) +
    //         (critical_weight_ * repulsive_cost / traj_len),
    //         power_);
    //     //data.fail_flag = all_trajectories_collide;
    //     if(all_trajectories_collide) {
    //         RCLCPP_WARN(logger_, "All trajectories collide with dynamic obstacles.");
    //     }
    // }

  if (!enabled_ || dynamic_obstacles_.poses.empty()) {
    return;
  }

  const size_t n_traj = data.trajectories.x.shape(0);
  const size_t n_steps = data.trajectories.x.shape(1);
  const size_t n_obs = dynamic_obstacles_.poses.size();

  bool all_trajectories_collide = true;
  int colliding_count = 0;

  auto& traj = data.trajectories;
  auto& state = data.state;

  // Prealloca costi
  xt::xtensor<float, 1> raw_cost = xt::zeros<float>({n_traj});
  xt::xtensor<float, 1> repulsive_cost = xt::zeros<float>({n_traj});

  // Copia gli ostacoli una sola volta
  std::vector<std::pair<Eigen::Vector2f, Eigen::Vector2f>> obstacles;
  {
    std::lock_guard<std::mutex> lock(my_mutex_);
    for (const auto& pose_cov : dynamic_obstacles_.poses) {
      Eigen::Vector2f pos(pose_cov.pose.position.x, pose_cov.pose.position.y);
      Eigen::Vector2f cov(pose_cov.covariance[0], pose_cov.covariance[7]);
      obstacles.emplace_back(pos, cov);
    }
  }

  for (size_t i = 0; i < n_traj; ++i) {
    bool trajectory_collide = false;

    for (size_t j = 0; j < std::min(n_obs, n_steps); ++j) {
      float robot_x = traj.x(i, j);
      float robot_y = traj.y(i, j);
      float dx = robot_x - obstacles[j].first[0];
      float dy = robot_y - obstacles[j].first[1];
      float dist_sq = dx * dx + dy * dy;

      if (dist_sq < 25.0f) {  // entro 5 metri
        float robot_covx = k1_ * j + k2_ * state.cvx(i, j);
        float robot_covy = k1_ * j + k2_ * state.cvy(i, j);

        float total_covx = std::max(1e-3f, robot_covx + obstacles[j].second[0]);
        float total_covy = std::max(1e-3f, robot_covy + obstacles[j].second[1]);

        float prob_dist = (dx * dx) / total_covx + (dy * dy) / total_covy;

        float det = std::max(1e-6f, (2.0f * static_cast<float>(M_PI) * total_covx) * (2.0f * static_cast<float>(M_PI) * total_covy));
        float k = -2.0f * std::log(std::sqrt(det) * loc_ / vr_);


        if (prob_dist < k) {
          trajectory_collide = true;
          repulsive_cost[i] = collision_cost_;
          raw_cost[i] = collision_cost_ + std::log(1.0f / std::sqrt(std::max(1e-3f, dist_sq)));
          ++colliding_count;
          break;
        } else {
          // raw_cost[i] += 1.0f / std::sqrt(dist_sq);
          raw_cost[i] += std::sqrt(dist_sq);
        }
      }
    }

    if (!trajectory_collide) {
      all_trajectories_collide = false;
    }
  }

  // Somma costi finali
  data.costs += xt::pow(repulsion_weight_ * (1/raw_cost) + (critical_weight_ * repulsive_cost / n_steps), power_);

  RCLCPP_INFO(logger_, "Traiettorie in collisione: %d su %lu", colliding_count, n_traj);

  if (all_trajectories_collide) {
    RCLCPP_WARN(logger_, "All trajectories collide with dynamic obstacles.");
    // data.fail_flag = true; // opzionale
  }

}

}  // namespace mppi::critics

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(mppi::critics::DynamicObstaclesCritic, mppi::critics::CriticFunction)