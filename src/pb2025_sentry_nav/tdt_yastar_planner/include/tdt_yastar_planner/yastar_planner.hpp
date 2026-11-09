#pragma once

#include <nav2_core/global_planner.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <tdt_yastar_planner/tdt_yastar.hpp>
#include <tdt_yastar_planner/tdt_trajectory_optimizer.hpp>
#include <tdt_yastar_planner/cancelable_planner.hpp>

#include <array>
#include <mutex>
#include <vector>
#include <string>

namespace tdt_yastar_planner
{
class YastarPlanner final : public nav2_core::GlobalPlanner, public CancelablePlanner
{
public:
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf, std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
  void cleanup() override;
  void activate() override;
  void deactivate() override;
  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;
  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal, const CancelChecker & cancel_checker) override;

private:
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_;
  std::string name_;
  std::mutex mutex_;
  double cost_weight_{1.0};
  int max_expansions_{YAstar::DEFAULT_MAX_EXPANSIONS};
  double max_planning_time_{YAstar::DEFAULT_MAX_SEARCH_SECONDS};
  bool allow_unknown_{false};
  bool tunnel_cost_enabled_{false};
  double tunnel_cost_scale_{0.1};
  // Flattened parameter rectangles use [min_x, min_y, max_x, max_y] in the global frame.
  std::vector<std::array<double, 4>> tunnel_cost_regions_;
  TrajectoryOptimizationOptions trajectory_options_;
  std::size_t trajectory_applied_{0};
  std::size_t trajectory_fallback_{0};
  bool configured_{false};
  bool active_{false};
};
}
