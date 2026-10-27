#include "tdt_yastar_planner/yastar_planner.hpp"

#include <nav2_costmap_2d/cost_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace tdt_yastar_planner
{
void YastarPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf, std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  parent_ = parent; name_ = std::move(name); tf_ = std::move(tf); costmap_ros_ = std::move(costmap_ros);
  auto node = parent_.lock();
  if (!node || !costmap_ros_) throw std::runtime_error("TDT YAstar requires parent and costmap");
  const auto declare = [&node](const std::string & key, const auto & default_value) {
    if (!node->has_parameter(key)) {
      node->declare_parameter(key, default_value);
    }
    return node->get_parameter(key).template get_value<typename std::decay_t<decltype(default_value)>>();
  };
  cost_weight_ = declare(name_ + ".cost_weight", 1.0);
  max_expansions_ = declare(name_ + ".max_expansions", YAstar::DEFAULT_MAX_EXPANSIONS);
  max_planning_time_ = declare(name_ + ".max_planning_time", YAstar::DEFAULT_MAX_SEARCH_SECONDS);
  allow_unknown_ = declare(name_ + ".allow_unknown", false);
  if (!std::isfinite(cost_weight_) || !(cost_weight_ > 0.0) || max_expansions_ < 0 ||
      !std::isfinite(max_planning_time_) || max_planning_time_ < 0.0) {
    throw std::invalid_argument("invalid TDT YAstar planner parameters");
  }
  configured_ = true;
}

void YastarPlanner::cleanup() { std::scoped_lock lock(mutex_); costmap_ros_.reset(); configured_ = active_ = false; }
void YastarPlanner::activate() { std::scoped_lock lock(mutex_); if (!configured_) throw std::logic_error("planner not configured"); active_ = true; }
void YastarPlanner::deactivate() { std::scoped_lock lock(mutex_); active_ = false; }

nav_msgs::msg::Path YastarPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start, const geometry_msgs::msg::PoseStamped & goal)
{
  std::scoped_lock lock(mutex_);
  if (!configured_ || !active_) throw std::logic_error("TDT YAstar planner is not active");
  auto * cm = costmap_ros_->getCostmap();
  if (!cm) throw std::runtime_error("costmap is unavailable");
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> map_lock(*(cm->getMutex()));
  const auto w = cm->getSizeInCellsX(), h = cm->getSizeInCellsY();
  if (w == 0 || h == 0 || cm->getResolution() <= 0.0) throw std::runtime_error("invalid costmap geometry");
  const auto finite_pose = [](const geometry_msgs::msg::PoseStamped & pose) {
    return std::isfinite(pose.pose.position.x) && std::isfinite(pose.pose.position.y) &&
           std::isfinite(pose.pose.position.z) &&
           std::isfinite(pose.pose.orientation.x) && std::isfinite(pose.pose.orientation.y) &&
           std::isfinite(pose.pose.orientation.z) && std::isfinite(pose.pose.orientation.w);
  };
  if (!finite_pose(start) || !finite_pose(goal)) throw std::invalid_argument("YAstar start/goal is not finite");
  const auto in_world_bounds = [cm](const geometry_msgs::msg::PoseStamped & pose) {
    return pose.pose.position.x >= cm->getOriginX() && pose.pose.position.y >= cm->getOriginY() &&
           pose.pose.position.x < cm->getOriginX() + cm->getSizeInCellsX() * cm->getResolution() &&
           pose.pose.position.y < cm->getOriginY() + cm->getSizeInCellsY() * cm->getResolution();
  };
  if (!in_world_bounds(start) || !in_world_bounds(goal)) {
    throw std::invalid_argument("YAstar start/goal is outside costmap bounds");
  }
  YAstar planner(static_cast<int>(w), static_cast<int>(h), static_cast<float>(cm->getResolution()),
    static_cast<float>(cm->getOriginX()), static_cast<float>(cm->getOriginY()));
  planner.setCostWeight(static_cast<float>(cost_weight_));
  std::vector<int8_t> map(static_cast<size_t>(w) * h, 0);
  std::vector<float> costs(static_cast<size_t>(w) * h, 0.0F);
  const auto * data = cm->getCharMap();
  for (size_t i = 0; i < map.size(); ++i) {
    const auto c = data[i];
    const bool blocked = c == nav2_costmap_2d::NO_INFORMATION ? !allow_unknown_ :
      c >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
    map[i] = blocked ? 100 : 0;
    costs[i] = blocked ? std::numeric_limits<float>::infinity() : std::max(1.0F, static_cast<float>(c) / 10.0F);
  }
  planner.setMap(static_cast<int>(w), static_cast<int>(h), static_cast<float>(cm->getResolution()),
    static_cast<float>(cm->getOriginX()), static_cast<float>(cm->getOriginY()), map);
  planner.setCostMapSnapshot(costs);
  planner.setSearchLimits(max_expansions_, max_planning_time_);
  map_lock.unlock();
  const Eigen::Vector2f start_xy{static_cast<float>(start.pose.position.x), static_cast<float>(start.pose.position.y)};
  const Eigen::Vector2f goal_xy{static_cast<float>(goal.pose.position.x), static_cast<float>(goal.pose.position.y)};
  const auto path = planner.search(start_xy, goal_xy);
  if (path.empty()) throw std::runtime_error("YAstar failed: no collision-free path");
  auto node = parent_.lock();
  if (!node) throw std::runtime_error("YAstar parent node expired");
  const auto global_frame = costmap_ros_->getGlobalFrameID();
  if (global_frame.empty() || (!start.header.frame_id.empty() && start.header.frame_id != global_frame) ||
      (!goal.header.frame_id.empty() && goal.header.frame_id != global_frame)) {
    throw std::invalid_argument("YAstar start/goal frame does not match costmap global frame");
  }
  nav_msgs::msg::Path result;
  result.header.frame_id = global_frame;
  result.header.stamp = node->now();
  std::vector<Eigen::Vector2f> points;
  points.reserve(path.size() + 2);
  points.push_back(start_xy);
  for (const auto & point : path) {
    if ((point - points.back()).norm() > 1.0e-5F) points.push_back(point);
  }
  if ((goal_xy - points.back()).norm() > 1.0e-5F) points.push_back(goal_xy);
  result.poses.reserve(points.size());
  for (size_t i = 0; i < points.size(); ++i) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = result.header;
    pose.pose.position.x = points[i].x();
    pose.pose.position.y = points[i].y();
    pose.pose.position.z = 0.0;
    if (i + 1 == points.size()) {
      pose.pose.position.x = goal.pose.position.x;
      pose.pose.position.y = goal.pose.position.y;
      pose.pose.orientation = goal.pose.orientation;
    } else {
      tf2::Quaternion orientation;
      orientation.setRPY(0.0, 0.0,
        std::atan2(points[i + 1].y() - points[i].y(), points[i + 1].x() - points[i].x()));
      pose.pose.orientation = tf2::toMsg(orientation);
    }
    result.poses.push_back(pose);
  }
  result.poses.front().pose.position.x = start.pose.position.x;
  result.poses.front().pose.position.y = start.pose.position.y;
  return result;
}
}

PLUGINLIB_EXPORT_CLASS(tdt_yastar_planner::YastarPlanner, nav2_core::GlobalPlanner)
