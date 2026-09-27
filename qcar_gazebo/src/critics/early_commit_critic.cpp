#include "qcar_gazebo/critics/early_commit_critic.hpp"

#include <algorithm>
#include <cmath>

#include <xtensor/xbuilder.hpp>
#include <xtensor/xmath.hpp>
#include <xtensor/xview.hpp>

namespace mppi::critics
{

void EarlyCommitCritic::initialize()
{
  auto getParam = parameters_handler_->getParamGetter(name_);
  getParam(offset_from_furthest_, "offset_from_furthest", 3);
  getParam(early_time_steps_, "early_time_steps", 10);
  getParam(max_lead_distance_, "max_lead_distance", 0.3f);
  getParam(forward_preference_, "forward_preference", false);
  getParam(power_, "cost_power", 1);
  getParam(weight_, "cost_weight", 10.0);

  RCLCPP_INFO(
    logger_,
    "EarlyCommitCritic instantiated with %u power, %f weight, offset_from_furthest %zu, "
    "early_time_steps %zu, max_lead_distance %f, forward_preference %s",
    power_, weight_, offset_from_furthest_, early_time_steps_, max_lead_distance_,
    forward_preference_ ? "true" : "false");
}

void EarlyCommitCritic::score(CriticData & data)
{
  if (!enabled_ || data.path.x.shape(0) < 2) {
    return;
  }

  utils::setPathFurthestPointIfNotSet(data);

  const size_t path_size = data.path.x.shape(0) - 1;
  const size_t offseted_idx = std::min(
    *data.furthest_reached_path_point + offset_from_furthest_, path_size);

  const float target_x = data.path.x(offseted_idx);
  const float target_y = data.path.y(offseted_idx);

  const size_t batch_size = data.trajectories.x.shape(0);
  const size_t time_steps = data.trajectories.x.shape(1);
  const size_t early_steps = std::min(early_time_steps_, time_steps);
  if (early_steps == 0) {
    return;
  }

  const xt::xtensor<float, 1> x0 = xt::view(data.trajectories.x, xt::all(), 0);
  const xt::xtensor<float, 1> y0 = xt::view(data.trajectories.y, xt::all(), 0);

  const float dx = target_x - x0(0);
  const float dy = target_y - y0(0);
  if (std::sqrt(dx * dx + dy * dy) > max_lead_distance_) {
    return;
  }

  const xt::xtensor<float, 1> bearing = xt::atan2(target_y - y0, target_x - x0);
  const xt::xtensor<float, 1> reverse_bearing = bearing + static_cast<float>(M_PI);

  xt::xtensor<float, 1> sum_diff = xt::zeros<float>({batch_size});
  for (size_t t = 0; t < early_steps; ++t) {
    const xt::xtensor<float, 1> yaw_t = xt::view(data.trajectories.yaws, xt::all(), t);
    const xt::xtensor<float, 1> fwd_diff =
      xt::abs(utils::shortest_angular_distance(yaw_t, bearing));
    if (forward_preference_) {
      sum_diff += fwd_diff;
    } else {
      const xt::xtensor<float, 1> rev_diff =
        xt::abs(utils::shortest_angular_distance(yaw_t, reverse_bearing));
      sum_diff += xt::minimum(fwd_diff, rev_diff);
    }
  }
  const xt::xtensor<float, 1> cost = sum_diff / static_cast<float>(early_steps);

  data.costs += xt::pow(cost * weight_, power_);
}

}  // namespace mppi::critics

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  mppi::critics::EarlyCommitCritic,
  mppi::critics::CriticFunction)
