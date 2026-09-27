#include "qcar_gazebo/smoothers/cusp_straightener_smoother.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace qcar_gazebo::smoothers
{

void CuspStraightenerSmoother::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer>/*tf*/,
  std::shared_ptr<nav2_costmap_2d::CostmapSubscriber>/*costmap_sub*/,
  std::shared_ptr<nav2_costmap_2d::FootprintSubscriber>/*footprint_sub*/)
{
  name_ = name;
  auto node = parent.lock();
  logger_ = node->get_logger();
  node->declare_parameter(name_ + ".straight_distance", straight_distance_);
  node->get_parameter(name_ + ".straight_distance", straight_distance_);
  node->declare_parameter(name_ + ".extend_distance", extend_distance_);
  node->get_parameter(name_ + ".extend_distance", extend_distance_);
  node->declare_parameter(name_ + ".min_lead_distance", min_lead_distance_);
  node->get_parameter(name_ + ".min_lead_distance", min_lead_distance_);
  RCLCPP_INFO(
    logger_, "CuspStraightenerSmoother instantiated with straight_distance %.3f, extend_distance %.3f, min_lead_distance %.3f",
    straight_distance_, extend_distance_, min_lead_distance_);
}

bool CuspStraightenerSmoother::smooth(nav_msgs::msg::Path & path, const rclcpp::Duration &)
{
  const size_t n = path.poses.size();
  if (n < 3) {
    return true;
  }

  const std::vector<geometry_msgs::msg::PoseStamped> original = path.poses;

  // Find cusps: consecutive displacement vectors whose dot product is negative (direction of
  // travel reverses by more than 90 degrees).
  std::vector<size_t> cusp_indices;
  for (size_t i = 1; i + 1 < n; ++i) {
    const double d1x = original[i].pose.position.x - original[i - 1].pose.position.x;
    const double d1y = original[i].pose.position.y - original[i - 1].pose.position.y;
    const double d2x = original[i + 1].pose.position.x - original[i].pose.position.x;
    const double d2y = original[i + 1].pose.position.y - original[i].pose.position.y;
    const double len1 = std::hypot(d1x, d1y);
    const double len2 = std::hypot(d2x, d2y);
    if (len1 < 1e-6 || len2 < 1e-6) {
      continue;
    }
    const double dot = (d1x * d2x + d1y * d2y) / (len1 * len2);
    if (dot < 0.0) {
      cusp_indices.push_back(i);
    }
  }
  RCLCPP_INFO(logger_, "smooth(): path has %zu points, %zu cusp(s) detected", n, cusp_indices.size());

  std::vector<double> lead_dist(n, 0.0);
  for (size_t i = 1; i < n; ++i) {
    lead_dist[i] = lead_dist[i - 1] + std::hypot(
      original[i].pose.position.x - original[i - 1].pose.position.x,
      original[i].pose.position.y - original[i - 1].pose.position.y);
  }

  struct CuspInfo
  {
    size_t idx;
    bool reverse;
    double along;
    double lead_distance;
  };
  std::vector<CuspInfo> cusps;
  cusps.reserve(cusp_indices.size());
  for (const size_t cusp_i : cusp_indices) {
    const double cx = original[cusp_i].pose.position.x;
    const double cy = original[cusp_i].pose.position.y;
    const double cusp_yaw = tf2::getYaw(original[cusp_i].pose.orientation);
    const double dx = original[cusp_i + 1].pose.position.x - cx;
    const double dy = original[cusp_i + 1].pose.position.y - cy;
    const double along = dx * std::cos(cusp_yaw) + dy * std::sin(cusp_yaw);
    cusps.push_back({cusp_i, along < 0.0, along, lead_dist[cusp_i]});
  }

  std::vector<geometry_msgs::msg::PoseStamped> output;
  output.reserve(n + cusps.size() * 6);

  size_t next_cusp = 0;
  for (size_t i = 0; i < n; ++i) {
    output.push_back(original[i]);

    if (next_cusp < cusps.size() && cusps[next_cusp].idx == i) {
      const CuspInfo & c = cusps[next_cusp];
      if (!c.reverse) {
        RCLCPP_INFO(logger_, "  cusp at idx=%zu: forward-entering, skipped (along=%.4f)", i, c.along);
        ++next_cusp;
        continue;
      }
      if (c.lead_distance < min_lead_distance_) {
        RCLCPP_INFO(
          logger_, "  cusp at idx=%zu: REVERSE-entering but only %.3fm ahead (< %.3fm) - robot already at/past it, skipped (along=%.4f)",
          i, c.lead_distance, min_lead_distance_, c.along);
        ++next_cusp;
        continue;
      }
      RCLCPP_INFO(
        logger_, "  cusp at idx=%zu: REVERSE-entering, extending corner by %.3f (curve-continuing) then straightening (along=%.4f)",
        i, extend_distance_, c.along);

      const double cusp_yaw = tf2::getYaw(original[i].pose.orientation);
      const double cx = original[i].pose.position.x;
      const double cy = original[i].pose.position.y;

      double kappa = 0.0;
      {
        constexpr double kCurvatureLookback = 0.15;
        size_t lookback_floor = 0;
        for (auto it = cusp_indices.rbegin(); it != cusp_indices.rend(); ++it) {
          if (*it < i) {
            lookback_floor = *it + 1;
            break;
          }
        }
        size_t k = i;
        double acc = 0.0;
        while (k > lookback_floor && acc < kCurvatureLookback) {
          acc += std::hypot(
            original[k].pose.position.x - original[k - 1].pose.position.x,
            original[k].pose.position.y - original[k - 1].pose.position.y);
          --k;
        }
        constexpr double kMinCurvatureBaseline = 0.05;
        if (acc > kMinCurvatureBaseline) {
          const double back_yaw = tf2::getYaw(original[k].pose.orientation);
          const double dyaw = std::atan2(std::sin(cusp_yaw - back_yaw), std::cos(cusp_yaw - back_yaw));
          kappa = dyaw / acc;
        }
      }

      constexpr double kExtensionStep = 0.03;
      double ex = cx, ey = cy, ext_yaw = cusp_yaw;
      if (extend_distance_ > 1e-6) {
        const int extension_steps = std::max(1, static_cast<int>(std::round(extend_distance_ / kExtensionStep)));
        const double step = extend_distance_ / extension_steps;
        for (int s = 1; s <= extension_steps; ++s) {
          const double arc = step * s;
          double px, py;
          if (std::abs(kappa) > 1e-6) {
            const double radius = 1.0 / kappa;
            px = cx + radius * (std::sin(cusp_yaw + kappa * arc) - std::sin(cusp_yaw));
            py = cy - radius * (std::cos(cusp_yaw + kappa * arc) - std::cos(cusp_yaw));
          } else {
            px = cx + std::cos(cusp_yaw) * arc;
            py = cy + std::sin(cusp_yaw) * arc;
          }
          ext_yaw = cusp_yaw + kappa * arc;

          geometry_msgs::msg::PoseStamped ext_pose = original[i];
          ext_pose.pose.position.x = px;
          ext_pose.pose.position.y = py;
          tf2::Quaternion ext_q;
          ext_q.setRPY(0, 0, ext_yaw);
          ext_pose.pose.orientation = tf2::toMsg(ext_q);
          output.push_back(ext_pose);
          ex = px;
          ey = py;
        }
      }

      const double ux = -std::cos(ext_yaw);
      const double uy = -std::sin(ext_yaw);
      double cumulative = 0.0;
      double prev_x = ex;
      double prev_y = ey;
      size_t j = i + 1;
      while (j < n && cumulative < straight_distance_) {
        if (next_cusp + 1 < cusps.size() && cusps[next_cusp + 1].idx == j) {
          break;
        }
        const double orig_x = original[j].pose.position.x;
        const double orig_y = original[j].pose.position.y;
        const double orig_yaw = tf2::getYaw(original[j].pose.orientation);
        const double seg_dx = orig_x - prev_x;
        const double seg_dy = orig_y - prev_y;
        prev_x = orig_x;
        prev_y = orig_y;
        cumulative += std::hypot(seg_dx, seg_dy);

        const double weight = std::clamp(1.0 - cumulative / straight_distance_, 0.0, 1.0);
        const double straight_x = ex + ux * cumulative;
        const double straight_y = ey + uy * cumulative;
        const double yaw_diff = std::atan2(
          std::sin(orig_yaw - ext_yaw), std::cos(orig_yaw - ext_yaw));
        const double blended_yaw = ext_yaw + (1.0 - weight) * yaw_diff;

        geometry_msgs::msg::PoseStamped blended = original[j];
        blended.pose.position.x = weight * straight_x + (1.0 - weight) * orig_x;
        blended.pose.position.y = weight * straight_y + (1.0 - weight) * orig_y;
        tf2::Quaternion blended_q;
        blended_q.setRPY(0, 0, blended_yaw);
        blended.pose.orientation = tf2::toMsg(blended_q);
        output.push_back(blended);
        ++j;
      }

      i = j - 1;  // outer loop's ++i resumes at j next iteration
      ++next_cusp;
    }
  }

  path.poses = std::move(output);
  return true;
}

}

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  qcar_gazebo::smoothers::CuspStraightenerSmoother,
  nav2_core::Smoother)
