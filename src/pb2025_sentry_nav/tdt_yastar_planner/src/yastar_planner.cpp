#include "tdt_yastar_planner/yastar_planner.hpp"

#include <nav2_costmap_2d/cost_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <pb_omni_pid_pursuit_controller/footprint_collision.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <thread>
#include <unordered_map>
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
  if (allow_unknown_) {
    throw std::invalid_argument("YAstar full-body checking requires allow_unknown=false, like the controller");
  }
  if (!std::isfinite(cost_weight_) || !(cost_weight_ > 0.0) || max_expansions_ < 0 ||
      !std::isfinite(max_planning_time_) || max_planning_time_ < 0.0) {
    throw std::invalid_argument("invalid TDT YAstar planner parameters");
  }
  tunnel_cost_enabled_ = declare(name_ + ".tunnel_cost_enabled", false);
  tunnel_cost_scale_ = declare(name_ + ".tunnel_cost_scale", 0.1);
  const auto regions = declare(name_ + ".tunnel_cost_regions", std::vector<double>{});
  if (!std::isfinite(tunnel_cost_scale_) || tunnel_cost_scale_ <= 0.0 ||
      tunnel_cost_scale_ > 1.0 || regions.size() % 4 != 0 ||
      (tunnel_cost_enabled_ && regions.empty())) {
    throw std::invalid_argument("invalid YAstar tunnel cost scale or regions");
  }
  tunnel_cost_regions_.clear();
  for (size_t i = 0; i < regions.size(); i += 4) {
    if (!std::all_of(regions.begin() + i, regions.begin() + i + 4,
        [](double v) {return std::isfinite(v);}) ||
        regions[i] >= regions[i + 2] || regions[i + 1] >= regions[i + 3]) {
      throw std::invalid_argument("YAstar tunnel regions require finite ordered bounds");
    }
    tunnel_cost_regions_.push_back({regions[i], regions[i + 1], regions[i + 2], regions[i + 3]});
  }
  if (tunnel_cost_enabled_) {
    RCLCPP_INFO(node->get_logger(),
      "YAstar tunnel soft-cost compensation enabled: %zu regions in %s, scale=%.3f",
      tunnel_cost_regions_.size(), costmap_ros_->getGlobalFrameID().c_str(), tunnel_cost_scale_);
  }
  const std::string trajectory_prefix = name_ + ".trajectory_optimization.";
  trajectory_options_.enabled = declare(trajectory_prefix + "enabled", false);
  trajectory_options_.max_time_seconds = declare(trajectory_prefix + "max_time_seconds", 0.03);
  const int control_limit = declare(trajectory_prefix + "max_control_points", 12);
  const int output_limit = declare(trajectory_prefix + "max_output_points", 1200);
  if (control_limit < 0 || output_limit < 0) {
    throw std::invalid_argument("invalid TDT trajectory point limits");
  }
  trajectory_options_.max_control_points = static_cast<std::size_t>(control_limit);
  trajectory_options_.max_output_points = static_cast<std::size_t>(output_limit);
  trajectory_options_.control_spacing = declare(trajectory_prefix + "control_spacing", 0.6);
  trajectory_options_.sample_spacing = declare(trajectory_prefix + "sample_spacing", 0.025);
  trajectory_options_.max_deviation = declare(trajectory_prefix + "max_deviation", 0.25);
  trajectory_options_.max_length_ratio = declare(trajectory_prefix + "max_length_ratio", 1.1);
  validateTrajectoryOptions(trajectory_options_);
  trajectory_applied_ = trajectory_fallback_ = 0;
  configured_ = true;
}

void YastarPlanner::cleanup() { std::scoped_lock lock(mutex_); costmap_ros_.reset(); configured_ = active_ = false; }
void YastarPlanner::activate() { std::scoped_lock lock(mutex_); if (!configured_) throw std::logic_error("planner not configured"); active_ = true; }
void YastarPlanner::deactivate() { std::scoped_lock lock(mutex_); active_ = false; }

nav_msgs::msg::Path YastarPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start, const geometry_msgs::msg::PoseStamped & goal)
{
  return createPlan(start, goal, {});
}

nav_msgs::msg::Path YastarPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start, const geometry_msgs::msg::PoseStamped & goal,
  const CancelChecker & cancel_checker)
{
  const auto check_cancelled = [&]() {
    if (cancel_checker && cancel_checker()) throw PlannerCancelled();
  };
  std::unique_lock<std::mutex> lock(mutex_, std::defer_lock);
  while (!lock.try_lock()) {
    check_cancelled();
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  check_cancelled();
  if (!configured_ || !active_) throw std::logic_error("TDT YAstar planner is not active");
  auto * cm = costmap_ros_->getCostmap();
  if (!cm) throw std::runtime_error("costmap is unavailable");
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> map_lock(*(cm->getMutex()), std::defer_lock);
  while (!map_lock.try_lock()) {
    check_cancelled();
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  check_cancelled();
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
  const auto global_frame = costmap_ros_->getGlobalFrameID();
  if (global_frame.empty() || (!start.header.frame_id.empty() && start.header.frame_id != global_frame) ||
      (!goal.header.frame_id.empty() && goal.header.frame_id != global_frame)) {
    throw std::invalid_argument("YAstar start/goal frame does not match costmap global frame");
  }
  tf2::Quaternion body_orientation;
  tf2::fromMsg(start.pose.orientation, body_orientation);
  if (!std::isfinite(body_orientation.length2()) || !(body_orientation.length2() > 1.0e-12)) {
    throw std::invalid_argument("YAstar start body orientation is invalid");
  }
  body_orientation.normalize();
  // Nav2 supplies the costmap robot-base pose. The omnidirectional chassis
  // translates at this yaw; path-tangent headings are not body orientation.
  const double body_yaw = tf2::getYaw(body_orientation);
  const auto footprint = costmap_ros_->getRobotFootprint();
  if (footprint.size() < 3) throw std::runtime_error("YAstar requires the padded body footprint");
  nav2_costmap_2d::Costmap2D snapshot(*cm);
  map_lock.unlock();
  const auto collision = [&](double ax, double ay, double bx, double by) {
    check_cancelled();
    return pb_omni_pid_pursuit_controller::sweptFootprintCollision(
      snapshot, footprint, ax, ay, bx, by, body_yaw);
  };
  if (collision(start.pose.position.x, start.pose.position.y,
      start.pose.position.x, start.pose.position.y)) {
    throw std::runtime_error("YAstar start body footprint collides or leaves the costmap");
  }
  if (collision(goal.pose.position.x, goal.pose.position.y,
      goal.pose.position.x, goal.pose.position.y)) {
    throw std::runtime_error("YAstar goal body footprint collides or leaves the costmap");
  }
  YAstar planner(static_cast<int>(w), static_cast<int>(h), static_cast<float>(snapshot.getResolution()),
    static_cast<float>(snapshot.getOriginX()), static_cast<float>(snapshot.getOriginY()));
  planner.setCostWeight(static_cast<float>(cost_weight_));
  std::vector<int8_t> map(static_cast<size_t>(w) * h, 0);
  std::vector<float> costs(static_cast<size_t>(w) * h, 0.0F);
  const auto * data = snapshot.getCharMap();
  for (size_t i = 0; i < map.size(); ++i) {
    if ((i & 4095U) == 0) check_cancelled();
    const auto c = data[i];
    // Match the controller: full-body intersections with lethal/unknown cells
    // block motion. Inflation is a cost field, not a second body enlargement.
    const bool blocked = c >= nav2_costmap_2d::LETHAL_OBSTACLE;
    map[i] = blocked ? 100 : 0;
    if (blocked) {
      costs[i] = std::numeric_limits<float>::infinity();
      continue;
    }
    const float base_cost = std::max(1.0F, static_cast<float>(c) / 10.0F);
    costs[i] = base_cost;
    if (tunnel_cost_enabled_) {
      double wx, wy;
      snapshot.mapToWorld(i % w, i / w, wx, wy);
      for (const auto & region : tunnel_cost_regions_) {
        if (wx >= region[0] && wy >= region[1] && wx <= region[2] && wy <= region[3]) {
          // Discount only excess soft cost; traversal weight stays at least 1.
          // Never edit the costmap
          // used for full-body or controller collision checks, even in this ROI.
          costs[i] = 1.0F + static_cast<float>(tunnel_cost_scale_) * (base_cost - 1.0F);
          break;
        }
      }
    }
  }
  planner.setMap(static_cast<int>(w), static_cast<int>(h), static_cast<float>(snapshot.getResolution()),
    static_cast<float>(snapshot.getOriginX()), static_cast<float>(snapshot.getOriginY()), map);
  planner.setCostMapSnapshot(costs);
  planner.setSearchLimits(max_expansions_, max_planning_time_, [&]() {
    check_cancelled();
    return false;
  });
  std::vector<int8_t> clear(map.size(), -1);
  std::unordered_map<uint64_t, bool> motions;
  planner.setMotionValidator([&](const Eigen::Vector2f & a, const Eigen::Vector2f & b) {
    unsigned int ax, ay, bx, by;
    if (!snapshot.worldToMap(a.x(), a.y(), ax, ay) ||
        !snapshot.worldToMap(b.x(), b.y(), bx, by)) return false;
    const auto ai = ay * w + ax, bi = by * w + bx;
    if (ai == bi) return !collision(a.x(), a.y(), b.x(), b.y());
    for (const auto & item : {std::make_pair(ai, a), std::make_pair(bi, b)}) {
      auto & cached = clear[item.first];
      if (cached < 0) cached = !collision(item.second.x(), item.second.y(),
          item.second.x(), item.second.y());
      if (!cached) return false;
    }
    const auto key = (static_cast<uint64_t>(std::min(ai, bi)) << 32) | std::max(ai, bi);
    const auto cached = motions.find(key);
    if (cached != motions.end()) return cached->second;
    const bool valid = !collision(a.x(), a.y(), b.x(), b.y());
    motions.emplace(key, valid);
    return valid;
  });
  const Eigen::Vector2f start_xy{static_cast<float>(start.pose.position.x), static_cast<float>(start.pose.position.y)};
  const Eigen::Vector2f goal_xy{static_cast<float>(goal.pose.position.x), static_cast<float>(goal.pose.position.y)};
  const auto path = planner.search(start_xy, goal_xy);
  if (path.empty()) throw std::runtime_error("YAstar failed: no collision-free path");
  auto node = parent_.lock();
  if (!node) throw std::runtime_error("YAstar parent node expired");
  nav_msgs::msg::Path result;
  result.header.frame_id = global_frame;
  result.header.stamp = node->now();
  GeometricPath points;
  points.reserve(path.size() + 2);
  points.emplace_back(start.pose.position.x, start.pose.position.y);
  for (const auto & point : path) {
    if ((point.cast<double>() - points.back()).norm() > 1.0e-5) {
      points.push_back(point.cast<double>());
    }
  }
  const Eigen::Vector2d exact_goal(goal.pose.position.x, goal.pose.position.y);
  if ((exact_goal - points.back()).norm() > 1.0e-9) points.push_back(exact_goal);
  else points.back() = exact_goal;
  // Validate exact YAstar endpoints before allowing any optimization fallback.
  for (std::size_t i = 0; i < points.size(); ++i) {
    const auto & a = points[i == 0 ? 0 : i - 1];
    const auto & b = points[i];
    if (collision(a.x(), a.y(), b.x(), b.y())) {
      throw std::runtime_error("YAstar exact path fails full-body collision checking");
    }
  }
  if (trajectory_options_.enabled) {
    const auto optimized = optimizeTdtTrajectory(points,
      [&](const Eigen::Vector2d & a, const Eigen::Vector2d & b) {
        return !collision(a.x(), a.y(), b.x(), b.y());
      }, trajectory_options_, cancel_checker);
    if (optimized.reason == "cancelled") throw PlannerCancelled();
    check_cancelled();
    if (optimized.applied) ++trajectory_applied_;
    else ++trajectory_fallback_;
    RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 2000,
      "TDT trajectory: %s (%zu -> %zu points, %zu controls, %.3f ms); applied=%zu fallback=%zu",
      optimized.reason.c_str(), points.size(), optimized.points.size(), optimized.control_points,
      optimized.elapsed_seconds * 1000.0, trajectory_applied_, trajectory_fallback_);
    points = optimized.points;
  }
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
  // Check the final double-precision endpoints and every emitted segment too.
  for (size_t i = 0; i < result.poses.size(); ++i) {
    const auto & a = result.poses[i == 0 ? 0 : i - 1].pose.position;
    const auto & b = result.poses[i].pose.position;
    if (collision(a.x, a.y, b.x, b.y)) {
      throw std::runtime_error("YAstar final path fails full-body collision checking");
    }
  }
  check_cancelled();
  return result;
}
}

PLUGINLIB_EXPORT_CLASS(tdt_yastar_planner::YastarPlanner, nav2_core::GlobalPlanner)
