#ifndef QCAR_GAZEBO__SMOOTHERS__CUSP_STRAIGHTENER_SMOOTHER_HPP_
#define QCAR_GAZEBO__SMOOTHERS__CUSP_STRAIGHTENER_SMOOTHER_HPP_

#include <memory>
#include <string>

#include "nav2_core/smoother.hpp"

namespace qcar_gazebo::smoothers
{

class CuspStraightenerSmoother : public nav2_core::Smoother
{
public:
  CuspStraightenerSmoother() = default;
  ~CuspStraightenerSmoother() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_sub,
    std::shared_ptr<nav2_costmap_2d::FootprintSubscriber> footprint_sub) override;

  void cleanup() override {}
  void activate() override {}
  void deactivate() override {}

  bool smooth(nav_msgs::msg::Path & path, const rclcpp::Duration & max_time) override;

protected:
  std::string name_;
  rclcpp::Logger logger_{rclcpp::get_logger("CuspStraightenerSmoother")};
  double straight_distance_{0.21};
  double extend_distance_{0.15};
  double min_lead_distance_{0.3};
};

}  // namespace qcar_gazebo::smoothers

#endif  // QCAR_GAZEBO__SMOOTHERS__CUSP_STRAIGHTENER_SMOOTHER_HPP_
