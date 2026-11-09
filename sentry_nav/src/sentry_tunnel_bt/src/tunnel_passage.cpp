// Copyright 2026 diting
// SPDX-License-Identifier: Apache-2.0
#include "action_context.hpp"
#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/condition_node.h"
#include "example_interfaces/msg/float32.hpp"
#include "nav2_behavior_tree/bt_conversions.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include "nav2_msgs/action/follow_path.hpp"
#include "nav2_msgs/msg/speed_limit.hpp"
#include "nav2_msgs/srv/get_costmap.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "pb_omni_pid_pursuit_controller/footprint_collision.hpp"
#include "rcl_interfaces/srv/get_parameters.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sentry_tunnel_bt/passage_diagnostics.hpp"
#include "sentry_tunnel_bt/tunnel_geometry.hpp"
#include "sentry_tunnel_bt/continuous_route.hpp"
#include "sentry_tunnel_bt/exit_spin_safety.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <future>
#include <memory>
#include <limits>
#include <mutex>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace sentry_tunnel_bt {
using Clock = std::chrono::steady_clock;
using Compute = nav2_msgs::action::ComputePathToPose;
using Follow = nav2_msgs::action::FollowPath;
using GetMap = nav2_msgs::srv::GetCostmap;
using GetParameters = rcl_interfaces::srv::GetParameters;
using namespace std::chrono_literals;

// A new global plan can select a tunnel after normal following has already
// started. Failing this condition halts that pipeline before the new path is
// given to FollowPath; the existing bounded recovery re-enters preparation.
class TunnelRouteGuard : public BT::ConditionNode {
public:
  TunnelRouteGuard(const std::string &name, const BT::NodeConfiguration &config)
      : BT::ConditionNode(name, config) {}
  static BT::PortsList providedPorts() {
    return {BT::InputPort<nav_msgs::msg::Path>("path"),
            BT::InputPort<std::string>("tunnels",
                                       "-3.5;-1.5;3.475;1.5;3.5;-3.475", ""),
            BT::InputPort<double>("tunnel_half_width", 0.4, "")};
  }
  BT::NodeStatus tick() override {
    try {
      nav_msgs::msg::Path path;
      std::string specification;
      if (!getInput("path", path) || !getInput("tunnels", specification) ||
          path.poses.empty()) {
        return BT::NodeStatus::FAILURE;
      }
      for (const auto &pose : path.poses) {
        if (!std::isfinite(pose.pose.position.x + pose.pose.position.y)) {
          return BT::NodeStatus::FAILURE;
        }
      }
      const auto fields = BT::splitString(specification, ';');
      if (fields.empty() || fields.size() % 3 != 0) {
        return BT::NodeStatus::FAILURE;
      }
      double half_width = 0.0;
      if (!getInput("tunnel_half_width", half_width) ||
          !std::isfinite(half_width) || half_width <= 0.0) {
        return BT::NodeStatus::FAILURE;
      }
      std::vector<Tunnel> tunnels;
      for (size_t i = 0; i < fields.size(); i += 3) {
        Tunnel region{BT::convertFromString<double>(fields[i]),
                      BT::convertFromString<double>(fields[i + 1]),
                      BT::convertFromString<double>(fields[i + 2]), half_width};
        if (!(region.x_min < region.x_max) ||
            !std::isfinite(region.x_min + region.x_max + region.y)) {
          return BT::NodeStatus::FAILURE;
        }
        tunnels.push_back(region);
      }
      return pathTouchesTunnel(path, tunnels) ? BT::NodeStatus::FAILURE
                                              : BT::NodeStatus::SUCCESS;
    } catch (const std::exception &) {
      return BT::NodeStatus::FAILURE;
    }
  }
};

// Preserve the actual final goal while the standard NavigateThroughPoses
// blackboard list is shortened after verified passage of intermediate points.
class TunnelFinalGoal : public BT::SyncActionNode {
public:
  TunnelFinalGoal(const std::string &name, const BT::NodeConfiguration &config)
      : BT::SyncActionNode(name, config) {}
  static BT::PortsList providedPorts() {
    return {
        BT::InputPort<std::vector<geometry_msgs::msg::PoseStamped>>("goals"),
        BT::OutputPort<geometry_msgs::msg::PoseStamped>("goal")};
  }
  BT::NodeStatus tick() override {
    std::vector<geometry_msgs::msg::PoseStamped> goals;
    if (!getInput("goals", goals) || goals.empty()) {
      return BT::NodeStatus::FAILURE;
    }
    setOutput("goal", goals.back());
    return BT::NodeStatus::SUCCESS;
  }
};

// A completed continuous route must not be followed by another outer action.
class TunnelTaskCompleted : public BT::ConditionNode {
public:
  TunnelTaskCompleted(const std::string &name, const BT::NodeConfiguration &config)
      : BT::ConditionNode(name, config) {}
  static BT::PortsList providedPorts() {
    return {BT::InputPort<bool>("completed", false, "")};
  }
  BT::NodeStatus tick() override {
    bool completed = false;
    return getInput("completed", completed).has_value() && completed
               ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
};

class TunnelPassage : public BT::StatefulActionNode {
public:
  TunnelPassage(const std::string &name, const BT::NodeConfiguration &config)
      : BT::StatefulActionNode(name, config) {
    node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
    // Humble's private navigator BT client node can retain the predeclared
    // false default even while the main navigator uses simulation time. Honor
    // the explicit launch mode before creating any clients or subscriptions.
    const bool use_sim_time = value<bool>("use_sim_time");
    if (!node_->has_parameter("use_sim_time")) {
      node_->declare_parameter<bool>("use_sim_time", use_sim_time);
    }
    const auto clock_result =
        node_->set_parameter(rclcpp::Parameter("use_sim_time", use_sim_time));
    if (!clock_result.successful) {
      throw std::runtime_error("Cannot synchronize BT client use_sim_time: " +
                               clock_result.reason);
    }
    RCLCPP_INFO(node_->get_logger(),
                "Tunnel BT client clock synchronized: use_sim_time=%s",
                use_sim_time ? "true" : "false");
    tf_ = config.blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
    group_ = node_->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive, false);
    executor_.add_callback_group(group_, node_->get_node_base_interface());
    for (size_t i = 0; i < maps_.size(); ++i) {
      const std::string key =
          i == 0 ? "local_costmap_service" : "global_costmap_service";
      maps_[i].client = node_->create_client<GetMap>(
          value<std::string>(key), rmw_qos_profile_services_default, group_);
      maps_[i].footprint_client = node_->create_client<GetParameters>(
          i == 0 ? "local_costmap/local_costmap/get_parameters"
                 : "global_costmap/global_costmap/get_parameters",
          rmw_qos_profile_services_default, group_);
    }
    spin_pub_ = node_->create_publisher<example_interfaces::msg::Float32>(
        value<std::string>("spin_topic"), 1);
    continuation_config_client_ = node_->create_client<GetParameters>(
        "controller_server/get_parameters", rmw_qos_profile_services_default,
        group_);
    spin_client_ = node_->create_client<GetParameters>(
        "fake_vel_transform/get_parameters", rmw_qos_profile_services_default,
        group_);
    cmd_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>(
        value<std::string>("cmd_vel_topic"), 1);
    limit_pub_ = node_->create_publisher<nav2_msgs::msg::SpeedLimit>(
        value<std::string>("speed_limit_topic"), 1);
    state_pub_ = node_->create_publisher<std_msgs::msg::String>(
        value<std::string>("state_topic"), rclcpp::QoS(1).transient_local());
    deviation_pub_ = node_->create_publisher<std_msgs::msg::String>(
        value<std::string>("deviation_diagnostic_topic"),
        rclcpp::QoS(1).reliable().transient_local());
    rclcpp::SubscriptionOptions options;
    options.callback_group = group_;
    odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        value<std::string>("odom_topic"), rclcpp::SensorDataQoS(),
        [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
          odom_ = msg;
          odom_received_ = Clock::now();
        },
        options);
    scan_sub_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
        value<std::string>("scan_topic"), rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::LaserScan::ConstSharedPtr msg) {
          // Delayed TF recovery can release old scans after a newer scan.
          // Do not let backlog overwrite fresher data or extend its wall age.
          if (scan_stamp_gate_.accept(
                  rclcpp::Time(msg->header.stamp).nanoseconds(),
                  node_->now().nanoseconds())) {
            scan_ = msg;
            scan_received_ = Clock::now();
          }
        },
        options);
    // Observation only; GT has no localization_status. Never gate on this.
    localization_sub_ = node_->create_subscription<std_msgs::msg::String>(
        value<std::string>("localization_status_topic"),
        rclcpp::QoS(1).reliable().transient_local(),
        [this](std_msgs::msg::String::ConstSharedPtr msg) {
          constexpr size_t max_status_bytes = 8192;
          localization_status_json_ = msg->data.substr(0, max_status_bytes);
          localization_status_truncated_ = msg->data.size() > max_status_bytes;
          localization_received_ = Clock::now();
          localization_received_ros_ns_ = node_->now().nanoseconds();
        },
        options);
    command_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
        value<std::string>("cmd_vel_topic"), 10,
        [this](geometry_msgs::msg::Twist::ConstSharedPtr msg) {
          observed_command_ = *msg;
          command_received_ = Clock::now();
        }, options);
    spin_sub_ = node_->create_subscription<example_interfaces::msg::Float32>(
        value<std::string>("spin_topic"), 10,
        [this](example_interfaces::msg::Float32::ConstSharedPtr msg) {
          spin_speed_ = msg->data;
          spin_observed_ = true;
        },
        options);
    actions_ = std::make_shared<ActionContext>(node_);
    actions_->stop_pub = cmd_pub_;
    actions_->spin_pub = spin_pub_;
    actions_->limit_pub = limit_pub_;
    actions_->group = node_->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive, false);
    actions_->executor.add_callback_group(actions_->group,
                                          node_->get_node_base_interface());
    actions_->planner.client = rclcpp_action::create_client<Compute>(
        node_, "compute_path_to_pose", actions_->group);
    actions_->follower.client = rclcpp_action::create_client<Follow>(
        node_, "follow_path", actions_->group);
    action_thread_ = actions_->start();
  }
  ~TunnelPassage() override {
    try {
      if (!successful_spin_hold_) {
        stopSpin();
      }
      const bool stopped = cancelOwned();
      if (!stopped) {
        actions_->watchCanceled();
        RCLCPP_ERROR(node_->get_logger(),
                     "Tunnel destroyed with unconfirmed cancellation; retained "
                     "action context keeps zeroing until terminal/shutdown");
      }
      actions_->release.store(true);
      if (action_thread_.joinable()) {
        if (stopped) {
          action_thread_.join();
        } else {
          action_thread_.detach();
        }
      }
    } catch (const std::exception &error) {
      RCLCPP_ERROR(node_->get_logger(), "Tunnel cleanup exception: %s",
                   error.what());
      actions_->watchCanceled();
      actions_->release.store(true);
      if (action_thread_.joinable()) {
        action_thread_.detach();
      }
    }
  }
  static BT::PortsList providedPorts() {
    return {
        BT::InputPort<nav_msgs::msg::Path>("path"),
        BT::InputPort<bool>("continuous_path", false, "Continuous fixed three-point route"),
        BT::InputPort<double>("exit_spin_speed", 0.0,
                              "Deferred spin after a safe full-body tunnel exit"),
        BT::InputPort<bool>("keep_exit_spin_at_goal", true,
                            "Keep deferred spin after successful final arrival"),
        BT::OutputPort<bool>("task_completed"),
        BT::InputPort<bool>("use_sim_time", true,
                            "Clock mode inherited from the navigation launch"),
        BT::InputPort<geometry_msgs::msg::PoseStamped>("goal"),
        BT::BidirectionalPort<std::vector<geometry_msgs::msg::PoseStamped>>(
            "goals"),
        BT::InputPort<std::string>("map_frame", "map", ""),
        BT::InputPort<std::string>("body_frame", "sentry_robot1/chassis", ""),
        BT::InputPort<std::string>("control_frame",
                                   "sentry_robot1/gimbal_yaw_fake", ""),
        BT::InputPort<std::string>("odom_topic", "odometry", ""),
        BT::InputPort<std::string>("scan_topic", "obstacle_scan", ""),
        BT::InputPort<std::string>("cmd_vel_topic", "cmd_vel_controller", ""),
        BT::InputPort<std::string>("spin_topic", "cmd_spin", ""),
        BT::InputPort<std::string>("speed_limit_topic", "speed_limit", ""),
        BT::InputPort<std::string>("state_topic", "tunnel_passage/state", ""),
        BT::InputPort<std::string>("deviation_diagnostic_topic",
                                   "tunnel_passage/diagnostic", ""),
        BT::InputPort<std::string>("localization_status_topic",
                                   "localization_status", ""),
        BT::InputPort<std::string>("local_costmap_service",
                                   "local_costmap/get_costmap", ""),
        BT::InputPort<std::string>("global_costmap_service",
                                   "global_costmap/get_costmap", ""),
        BT::InputPort<std::string>("planner_id", "GridBased", ""),
        BT::InputPort<std::string>("controller_id", "FollowPathTunnel", ""),
        BT::InputPort<std::string>("goal_checker_id", "tunnel_goal_checker",
                                   ""),
        BT::InputPort<std::string>("tunnels", "-3.5;-1.5;3.475;1.5;3.5;-3.475",
                                   ""),
        BT::InputPort<double>("tunnel_half_width", 0.4, ""),
        BT::InputPort<double>("entry_distance", 0.5, ""),
        BT::InputPort<double>("exit_distance", 0.6, ""),
        BT::InputPort<double>("approach_speed", 0.3, ""),
        BT::InputPort<double>(
            "approach_slowdown_distance", 1.5,
            "Distance to outside waiting point for latched slowdown"),
        BT::InputPort<double>("passage_speed", 0.8, ""),
        BT::InputPort<double>(
            "rolling_alignment_speed", 0.0,
            "Optional outside straight-corridor alignment; zero disables"),
        BT::InputPort<double>("position_tolerance", 0.015, ""),
        BT::InputPort<double>("waiting_longitudinal_tolerance", 0.30, ""),
        BT::InputPort<double>("centering_speed", 0.15, ""),
        BT::InputPort<double>("rotation_speed", 0.6,
                              "Maximum preparation angular speed"),
        BT::InputPort<double>("yaw_tolerance", 0.034906585, ""),
        BT::InputPort<double>("data_timeout", 0.8, ""),
        BT::InputPort<double>("stage_timeout", 60.0, ""),
        BT::InputPort<int>("max_passages", 2, "")};
  }

private:
  enum class Stage {
    CLASSIFY_BODY,
    WAIT_DATA,
    PLAN_APPROACH,
    FOLLOW_APPROACH,
    HANDOFF_APPROACH,
    MOVE_ALIGN,
    STOP_APPROACH,
    CENTER,
    STOP_CENTER,
    ROTATE,
    STOP_ROTATE,
    PLAN_PASSAGE,
    FOLLOW_PASSAGE,
    FOLLOW_CONTINUATION,
    VERIFY_EXIT,
    PLAN_FINAL,
    CANCELING
  };
  template <typename T> T value(const std::string &key) const {
    T result{};
    if (!getInput<T>(key, result)) {
      throw std::runtime_error("Missing tunnel BT port: " + key);
    }
    return result;
  }
  struct MapInput {
    rclcpp::Client<GetMap>::SharedPtr client;
    rclcpp::Client<GetParameters>::SharedPtr footprint_client;
    rclcpp::Client<GetParameters>::SharedFuture footprint_pending;
    std::vector<geometry_msgs::msg::Point> footprint;
    rclcpp::Client<GetMap>::SharedFuture pending;
    nav2_msgs::msg::Costmap::SharedPtr message;
    std::unique_ptr<nav2_costmap_2d::Costmap2D> grid;
    Clock::time_point received{}, requested{};
  };
  BT::NodeStatus onStart() override {
    try {
      exit_spin_speed_ = value<double>("exit_spin_speed");
      if (!std::isfinite(exit_spin_speed_)) {
        throw std::runtime_error("Nonfinite deferred spin speed");
      }
      const bool managed_spin = value<bool>("continuous_path") &&
          std::abs(exit_spin_speed_) > 1e-5;
      // A repeated route must first stop the preceding run's idle spin.
      start_spin_settling_ = managed_spin || exit_spin_active_ || successful_spin_hold_;
      successful_spin_hold_ = false;
      exit_spin_active_ = false;
      if (start_spin_settling_) {
        stopSpin();
      }
      // Drain before even the no-tunnel early return: an older accepted goal
      // must never be forgotten when a new final target chooses another route.
      if (!cancelOwned()) {
        actions_->watchCanceled();
        stop();
        return BT::NodeStatus::FAILURE;
      }
      actions_->disarmCleanup();
      active_ = false;
      setOutput("task_completed", false);
      continuous_ = continuation_needed_ = continuation_pending_ = false;
      continuation_ready_ = false;
      continuation_replan_pending_ = continuation_replan_ready_ = false;
      continuation_config_started_ = Clock::now();
      continuation_config_checked_ = continuation_config_valid_ = false;
      continuation_config_pending_ = {};
      continuation_path_.poses.clear();
      follower_path_.poses.clear();
      approach_slowed_ = false;
      visited_.clear();
      stable_since_ = {};
      centering_started_ = {};
      center_stop_stamp_ns_ = 0;
      center_observed_vx_ = center_observed_vy_ = 0.0;
      center_previous_stamp_ns_ = 0;
      rotation_previous_stamp_ns_ = 0;
      rotation_observed_rate_ = 0.0;
      alignment_recenters_ = 0;
      passage_trace_ = nav_msgs::msg::Path{};
      remaining_goals_.clear();
      through_poses_ = getInput("goals", remaining_goals_).has_value();
      if (through_poses_ && remaining_goals_.empty()) {
        throw std::runtime_error("NavigateThroughPoses goals are empty");
      }
      goal_ = value<geometry_msgs::msg::PoseStamped>("goal");
      route_ = value<nav_msgs::msg::Path>("path");
      if (through_poses_ && changed(remaining_goals_.back(), goal_)) {
        throw std::runtime_error(
            "Final through-pose goal does not match task goal");
      }
      passage_trace_.header.frame_id = value<std::string>("map_frame");
      if (goal_.header.frame_id != value<std::string>("map_frame") ||
          route_.header.frame_id != value<std::string>("map_frame")) {
        throw std::runtime_error(
            "Tunnel route and final goal must use map_frame");
      }
      const double half_width = value<double>("tunnel_half_width");
      if (!std::isfinite(half_width) || half_width <= 0.0) {
        throw std::runtime_error("tunnel_half_width must be finite and positive");
      }
      tunnels_.clear();
      const auto tunnel_spec = value<std::string>("tunnels");
      const auto fields = BT::splitString(tunnel_spec, ';');
      if (fields.empty() || fields.size() % 3 != 0) {
        throw std::runtime_error("tunnels must contain xmin;xmax;y triples");
      }
      for (size_t i = 0; i < fields.size(); i += 3) {
        Tunnel tunnel{BT::convertFromString<double>(fields[i]),
                      BT::convertFromString<double>(fields[i + 1]),
                      BT::convertFromString<double>(fields[i + 2]), half_width};
        if (!(tunnel.x_min < tunnel.x_max) ||
            !std::isfinite(tunnel.x_min + tunnel.x_max + tunnel.y)) {
          throw std::runtime_error("Invalid tunnel coordinates");
        }
        tunnels_.push_back(tunnel);
      }
      crossing_ = firstCrossing(route_, tunnels_);
      if (!std::isfinite(value<double>("data_timeout")) ||
          value<double>("data_timeout") <= 0.0) {
        throw std::runtime_error("Invalid tunnel classification data timeout");
      }
      // Re-read parameter footprints for every new task. Classification must
      // not reuse a prior task's envelope or publish spin/speed-limit commands.
      for (auto &input : maps_) {
        input.footprint.clear();
        input.footprint_pending = {};
      }
      transition(Stage::CLASSIFY_BODY, "classifying_current_body");
      return BT::NodeStatus::RUNNING;
    } catch (const std::exception &error) {
      return fail(error.what());
    }
  }
  BT::NodeStatus startVerifiedPassage() {
    continuation_config_started_ = Clock::now();
    if (!std::isfinite(value<double>("entry_distance")) ||
        !std::isfinite(value<double>("exit_distance")) ||
        !std::isfinite(value<double>("passage_speed")) ||
        value<double>("entry_distance") < 0.45 ||
        value<double>("exit_distance") < 0.45 ||
        value<double>("passage_speed") <= 0.0 ||
        !std::isfinite(value<double>("rolling_alignment_speed")) ||
        value<double>("rolling_alignment_speed") < 0.0 ||
        value<double>("rolling_alignment_speed") > 1.0 ||
        value<double>("rolling_alignment_speed") >
            value<double>("approach_speed") ||
        !std::isfinite(value<double>("approach_slowdown_distance")) ||
        value<double>("approach_slowdown_distance") <= 0.0 ||
        !validCenteringSettings(
            value<double>("position_tolerance"),
            value<double>("waiting_longitudinal_tolerance"),
            value<double>("approach_speed"),
            value<double>("centering_speed")) ||
        !std::isfinite(value<double>("rotation_speed")) ||
        value<double>("rotation_speed") <= 0.0 ||
        value<double>("rotation_speed") > 0.6 ||
        !std::isfinite(value<double>("yaw_tolerance")) ||
        value<double>("yaw_tolerance") <= 0.0 ||
        value<double>("yaw_tolerance") > 0.061086524 ||
        !std::isfinite(value<double>("data_timeout")) ||
        value<double>("data_timeout") <= 0.0 ||
        !std::isfinite(value<double>("stage_timeout")) ||
        value<double>("stage_timeout") <= 0.5 ||
        value<int>("max_passages") < 1) {
      throw std::runtime_error("Unsafe tunnel distance or speed settings");
    }
    active_ = true;
    clock_last_ = node_->now();
    clock_advanced_ = Clock::now();
    spin_checked_ = false;
    spin_rejected_ = false;
    spin_pending_ = {};
    if (spin_client_->service_is_ready()) {
      auto request = std::make_shared<GetParameters::Request>();
      request->names = {"init_spin_speed"};
      spin_pending_ = spin_client_->async_send_request(request).share();
    }
    setLimit(value<double>("approach_speed"));
    transition(Stage::WAIT_DATA, "waiting_for_fresh_inputs");
    return BT::NodeStatus::RUNNING;
  }
  void updateFootprints() {
    for (auto &input : maps_) {
      if (input.footprint_pending.valid() &&
          input.footprint_pending.wait_for(0s) == std::future_status::ready) {
        const auto values = input.footprint_pending.get()->values;
        input.footprint_pending = {};
        if (values.size() != 2 || values[0].type != 4 || values[1].type != 3 ||
            !std::isfinite(values[1].double_value) || values[1].double_value < 0.0 ||
            !nav2_costmap_2d::makeFootprintFromString(values[0].string_value,
                                                      input.footprint) ||
            input.footprint.size() < 3) {
          throw std::runtime_error(
              "A valid polygon footprint and nonnegative padding are required");
        }
        nav2_costmap_2d::padFootprint(input.footprint, values[1].double_value);
      }
      if (input.footprint.empty() && !input.footprint_pending.valid() &&
          input.footprint_client->service_is_ready()) {
        auto request = std::make_shared<GetParameters::Request>();
        request->names = {"footprint", "footprint_padding"};
        input.footprint_pending =
            input.footprint_client->async_send_request(request).share();
      }
    }
  }
  BT::NodeStatus classifyCurrentBody() {
    executor_.spin_some(1ms);
    updateFootprints();
    std::string unavailable;
    try {
      validateFootprints();
      pose_ = transform(value<std::string>("map_frame"));
    } catch (const std::exception &error) {
      unavailable = error.what();
    }
    if (!unavailable.empty()) {
      if (seconds(stage_started_) >= 3.0) {
        return fail("tunnel_body_classification_unavailable:" + unavailable);
      }
      return BT::NodeStatus::RUNNING;
    }
    for (const auto &tunnel : tunnels_) {
      if (bodyOverlapsTunnel(tunnel, footprint_, pose_.transform.translation.x,
                             pose_.transform.translation.y,
                             tf2::getYaw(pose_.transform.rotation))) {
        const auto reason = passage_failure_latched_
            ? "in_tunnel_failure_requires_manual_exit"
            : "new_task_body_overlaps_tunnel_requires_manual_exit";
        publishState(reason);
        return fail(reason);
      }
    }
    passage_failure_latched_ = false;
    if (crossing_.tunnel < 0) {
      if (pathTouchesTunnel(route_, tunnels_)) {
        publishState("partial_tunnel_route_requires_complete_passage");
        return fail("partial_tunnel_route_requires_complete_passage");
      }
      publishState("outside_tunnels_resuming_ordinary_navigation");
      return BT::NodeStatus::SUCCESS;
    }
    return startVerifiedPassage();
  }
  void updateInputs() {
    executor_.spin_some(1ms);
    updateFootprints();
    if (!spin_checked_ && spin_pending_.valid() &&
        spin_pending_.wait_for(0s) == std::future_status::ready) {
      const auto response = spin_pending_.get();
      if (response->values.size() != 1 || response->values[0].type != 3) {
        throw std::runtime_error("Cannot verify configured spin speed");
      }
      if (!spin_observed_) {
        spin_speed_ = response->values[0].double_value;
      }
      spin_checked_ = true;
    }
    if (!spin_checked_ && !spin_pending_.valid() &&
        spin_client_->service_is_ready()) {
      auto request = std::make_shared<GetParameters::Request>();
      request->names = {"init_spin_speed"};
      spin_pending_ = spin_client_->async_send_request(request).share();
    }
    if (value<bool>("continuous_path") && !continuation_config_checked_ &&
        seconds(continuation_config_started_) > 3.0) {
      continuation_config_checked_ = true;
      continuation_config_valid_ = false;
    }
    if (value<bool>("continuous_path") && !continuation_config_checked_) {
      if (continuation_config_pending_.valid() &&
          continuation_config_pending_.wait_for(0s) == std::future_status::ready) {
        const auto p = continuation_config_pending_.get()->values;
        continuation_config_checked_ = true;
        continuation_config_valid_ =
            p.size() == 5 && p[0].type == 3 && p[1].type == 3 &&
            p[2].type == 1 && p[3].type == 1 && p[4].type == 1 &&
            !p[3].bool_value && !p[4].bool_value;
        if (continuation_config_valid_) {
          continuation_lookahead_ = p[2].bool_value ? p[0].double_value
                                                   : p[1].double_value;
          continuation_config_valid_ = std::isfinite(continuation_lookahead_) &&
              continuation_lookahead_ > 0.0 && continuation_lookahead_ <= 2.0;
        }
      } else if (!continuation_config_pending_.valid() &&
                 continuation_config_client_->service_is_ready()) {
        auto request = std::make_shared<GetParameters::Request>();
        const auto controller = value<std::string>("controller_id");
        request->names = {controller + ".max_lookahead_dist",
                          controller + ".lookahead_dist",
                          controller + ".use_velocity_scaled_lookahead_dist",
                          controller + ".enable_rotation",
                          controller + ".use_rotate_to_heading"};
        continuation_config_pending_ =
            continuation_config_client_->async_send_request(request).share();
      }
    }
    for (auto &input : maps_) {
      if (input.pending.valid() &&
          input.pending.wait_for(0s) == std::future_status::ready) {
        input.message =
            std::make_shared<nav2_msgs::msg::Costmap>(input.pending.get()->map);
        const auto &m = *input.message;
        const auto &md = m.metadata;
        if (md.size_x == 0 || md.size_y == 0 || md.resolution <= 0.0 ||
            m.data.size() != static_cast<size_t>(md.size_x) * md.size_y ||
            std::abs(tf2::getYaw(md.origin.orientation)) > 1e-6) {
          throw std::runtime_error("Malformed or rotated costmap");
        }
        input.grid = std::make_unique<nav2_costmap_2d::Costmap2D>(
            md.size_x, md.size_y, md.resolution, md.origin.position.x,
            md.origin.position.y);
        std::copy(m.data.begin(), m.data.end(), input.grid->getCharMap());
        input.received = Clock::now();
        input.pending = {};
      }
      if (!input.pending.valid() && input.client->service_is_ready() &&
          seconds(input.requested) > 0.2) {
        input.pending =
            input.client
                ->async_send_request(std::make_shared<GetMap::Request>())
                .share();
        input.requested = Clock::now();
      }
    }
    if (seconds(last_limit_) > 0.2) {
      setLimit(limit_);
    }
  }
  bool freshStamp(const builtin_interfaces::msg::Time &stamp) const {
    const auto ns = rclcpp::Time(stamp).nanoseconds();
    const double age = (node_->now().nanoseconds() - ns) * 1e-9;
    return ns > 0 && age >= -kInputFutureToleranceNs * 1e-9 &&
           age <= value<double>("data_timeout");
  }
  geometry_msgs::msg::TransformStamped
  transform(const std::string &frame) const {
    auto result = tf_->lookupTransform(frame, value<std::string>("body_frame"),
                                       tf2::TimePointZero);
    if (!freshStamp(result.header.stamp)) {
      throw std::runtime_error("Stale chassis TF in " + frame);
    }
    return result;
  }
  void guardInputs() {
    const auto now = node_->now();
    if (now < clock_last_) {
      throw std::runtime_error("Simulation time moved backwards");
    }
    if (now > clock_last_) {
      clock_last_ = now;
      clock_advanced_ = Clock::now();
    }
    if (seconds(clock_advanced_) > 0.8) {
      throw std::runtime_error("Clock stopped advancing");
    }
    const bool expected_exit_spin = exit_spin_active_ &&
        stage_ == Stage::FOLLOW_CONTINUATION &&
        std::abs(spin_speed_ - exit_spin_speed_) < 1e-4;
    if (!std::isfinite(spin_speed_) ||
        (std::abs(spin_speed_) > 1e-5 && !expected_exit_spin)) {
      stopSpin();
      spin_rejected_ = true;
      throw std::runtime_error(
          "Nonzero spin was disabled; tunnel request rejected for safe retry");
    }
    if (spin_rejected_) {
      throw std::runtime_error("Nonzero spin rejected; disabled for safety");
    }
    if (!spin_checked_) {
      throw std::runtime_error("Tunnel passage requires verified spin_speed=0");
    }
    if (!odom_ || !scan_ || seconds(odom_received_) > 1.5 ||
        seconds(scan_received_) > 1.5 || !freshStamp(odom_->header.stamp) ||
        !freshStamp(scan_->header.stamp)) {
      std::ostringstream detail;
      detail << "Missing or stale odometry/scan now=" << node_->now().seconds()
             << " clock_type="
             << static_cast<int>(node_->get_clock()->get_clock_type())
             << " odom_topic=" << odom_sub_->get_topic_name()
             << " scan_topic=" << scan_sub_->get_topic_name()
             << " odom_present=" << static_cast<bool>(odom_)
             << " scan_present=" << static_cast<bool>(scan_)
             << " odom_wall_age=" << seconds(odom_received_)
             << " scan_wall_age=" << seconds(scan_received_)
             << " odom_header_age="
             << (odom_ ? (node_->now().nanoseconds() -
                          rclcpp::Time(odom_->header.stamp).nanoseconds()) *
                             1e-9
                       : -999.0)
             << " scan_header_age="
             << (scan_ ? (node_->now().nanoseconds() -
                          rclcpp::Time(scan_->header.stamp).nanoseconds()) *
                             1e-9
                       : -999.0);
      throw std::runtime_error(detail.str());
    }
    for (const auto &input : maps_) {
      if (!input.grid || seconds(input.received) > 1.5 ||
          !freshStamp(input.message->metadata.update_time)) {
        throw std::runtime_error("Missing or stale costmap update");
      }
      transform(input.message->header.frame_id);
    }
    validateFootprints();
    pose_ = transform(value<std::string>("map_frame"));
  }
  void validateFootprints() {
    if (maps_[0].footprint.empty() || maps_[1].footprint.empty() ||
        maps_[0].footprint.size() != maps_[1].footprint.size()) {
      throw std::runtime_error("Missing or inconsistent costmap footprints");
    }
    radius_ = 0.0;
    longitudinal_extent_ = 0.0;
    for (size_t i = 0; i < maps_[0].footprint.size(); ++i) {
      const auto &a = maps_[0].footprint[i];
      const auto &b = maps_[1].footprint[i];
      if (!std::isfinite(a.x + a.y + b.x + b.y) ||
          std::hypot(a.x - b.x, a.y - b.y) > 1e-6) {
        throw std::runtime_error("Local/global padded footprints do not agree");
      }
      radius_ = std::max(radius_, std::hypot(a.x, a.y));
      longitudinal_extent_ = std::max(longitudinal_extent_, std::abs(a.x));
    }
    footprint_ = maps_[0].footprint;
  }
  BT::NodeStatus onRunning() override {
    if (stage_ == Stage::CANCELING) {
      return tickCanceling();
    }
    try {
      if (actions_->executor_failed.load()) {
        return fail("action_executor_failed");
      }
      const auto current_goal = value<geometry_msgs::msg::PoseStamped>("goal");
      if (changed(current_goal, goal_)) {
        return fail("final_goal_updated");
      }
      if (through_poses_) {
        const auto current_goals =
            value<std::vector<geometry_msgs::msg::PoseStamped>>("goals");
        if (current_goals.size() != remaining_goals_.size() ||
            !std::equal(
                current_goals.begin(), current_goals.end(),
                remaining_goals_.begin(),
                [](const auto &a, const auto &b) { return !changed(a, b); })) {
          return fail("through_poses_goal_list_updated");
        }
      }
      if (stage_ == Stage::CLASSIFY_BODY) {
        return classifyCurrentBody();
      }
      updateInputs();
      if (seconds(stage_started_) > value<double>("stage_timeout")) {
        return fail("stage_timeout");
      }
      if (stage_ == Stage::WAIT_DATA) {
        if (start_spin_settling_) {
          stopSpin();
          stop();
          if (!settled(odom_ && freshStamp(odom_->header.stamp) && stopped())) {
            if (seconds(stage_started_) > 5.0) {
              throw std::runtime_error("Previous spin did not settle before route");
            }
            return BT::NodeStatus::RUNNING;
          }
          start_spin_settling_ = false;
          stable_since_ = {};
        }
        try {
          guardInputs();
        } catch (const std::exception &) {
          if (spin_rejected_) {
            throw;
          }
          stop();
          if (seconds(stage_started_) > 3.0) {
            throw;
          }
          return BT::NodeStatus::RUNNING;
        }
        beginPassage();
        return BT::NodeStatus::RUNNING;
      }
      guardInputs();
      recordObservedTrace();
      prepareContinuation();
      switch (stage_) {
      case Stage::PLAN_APPROACH:
      case Stage::PLAN_PASSAGE:
      case Stage::PLAN_FINAL: {
        stop();
        const int result = actions_->planner.poll();
        if (result < 0) {
          return fail("planning_failed");
        }
        if (result == 0) {
          break;
        }
        route_ = actions_->planner.result.get().result->path;
        if (route_.poses.empty()) {
          return fail("planner_returned_empty_path");
        }
        if (stage_ == Stage::PLAN_FINAL) {
          crossing_ = firstCrossing(route_, tunnels_, visited_);
          if (crossing_.tunnel < 0) {
            return complete();
          }
          if (static_cast<int>(visited_.size()) >= value<int>("max_passages")) {
            return fail("maximum_passages_exceeded");
          }
          beginPassage();
          break;
        }
        if (stage_ == Stage::PLAN_PASSAGE) {
          const auto found = firstCrossing(route_, tunnels_);
          if (found.tunnel != crossing_.tunnel ||
              found.direction != crossing_.direction) {
            return fail("replanned_route_does_not_cross_selected_tunnel");
          }
          route_ = straightPassagePath();
        }
        Follow::Goal request;
        request.path = route_;
        request.controller_id = value<std::string>("controller_id");
        request.goal_checker_id = value<std::string>("goal_checker_id");
        actions_->follower.send(request);
        transition(stage_ == Stage::PLAN_APPROACH ? Stage::FOLLOW_APPROACH
                                                  : Stage::FOLLOW_PASSAGE,
                   stage_ == Stage::PLAN_APPROACH ? "following_outside_approach"
                                                  : "following_tunnel_axis");
        prepareContinuation();
        break;
      }
      case Stage::FOLLOW_APPROACH:
      case Stage::FOLLOW_PASSAGE: {
        if (stage_ == Stage::FOLLOW_APPROACH) {
          updateApproachLimit(false);
          const double distance = std::hypot(
              pose_.transform.translation.x - entry_x_,
              pose_.transform.translation.y - tunnels_[crossing_.tunnel].y);
          // The planner/controller owns the curved approach. Take over only a
          // short, independently verified straight corridor completely outside.
          if (value<double>("rolling_alignment_speed") > 0.0 &&
              distance >= 0.6 && distance <= 1.5 && headingError() > 0.1 &&
              rollingCorridorFree()) {
            actions_->follower.cancel();
            transition(Stage::HANDOFF_APPROACH,
                       "canceling_approach_for_rolling_alignment");
            break;
          }
        }
        if (stage_ == Stage::FOLLOW_PASSAGE) {
          const auto deviation = passageDeviation(
              tf2::getYaw(pose_.transform.rotation), target_yaw_,
              pose_.transform.translation.y, tunnels_[crossing_.tunnel].y,
              value<double>("yaw_tolerance") * 1.5);
          if (deviation.exceeded()) {
            const auto decision_time = node_->now();
            // Keep the original stop/cancel path ahead of diagnostic DDS/log I/O.
            const auto result = fail("tunnel_heading_or_centerline_deviation");
            publishDeviationDiagnostic(deviation, decision_time);
            return result;
          }
        }
        if (stage_ == Stage::FOLLOW_PASSAGE && continuous_ && fullBodyCleared()) {
          const size_t passed = passedGoalPrefix(remaining_goals_, passage_trace_, 0.15);
          if (passed != 2) {
            return fail("continuous_waypoint_prefix_not_observed");
          }
          remaining_goals_.erase(remaining_goals_.begin(), remaining_goals_.begin() + passed);
          setOutput("goals", remaining_goals_);
          visited_.insert(crossing_.tunnel);
          setLimit(0.0);
          continuation_replan_pending_ = continuation_replan_ready_ = false;
          continuation_replanned_ = Clock::now() - 1s;
          transition(Stage::FOLLOW_CONTINUATION, "full_body_exit_continuing_without_stop");
          break; // Keep the exact same FollowPath goal alive.
        }
        const int result = actions_->follower.poll();
        if (result < 0) {
          return fail("controller_failed");
        }
        if (result > 0) {
          if (stage_ == Stage::FOLLOW_PASSAGE && continuous_) {
            return fail("continuous_controller_completed_before_exit");
          }
          stop();
          transition(stage_ == Stage::FOLLOW_APPROACH ? Stage::STOP_APPROACH
                                                      : Stage::VERIFY_EXIT,
                     stage_ == Stage::FOLLOW_APPROACH
                         ? "stopping_before_precision_centering"
                         : "verifying_full_body_exit");
        }
        break;
      }
      case Stage::HANDOFF_APPROACH:
        // An acknowledged cancellation is not terminal. Never publish moving
        // commands until the previous FollowPath result has really completed.
        stop();
        actions_->follower.cancel();
        if (!actions_->follower.busy()) {
          if (!actions_->follower.handle ||
              !actions_->follower.result.valid() ||
              actions_->follower.result.wait_for(0s) !=
                  std::future_status::ready ||
              (actions_->follower.result.get().code !=
                   rclcpp_action::ResultCode::CANCELED &&
               actions_->follower.result.get().code !=
                   rclcpp_action::ResultCode::SUCCEEDED)) {
            return fail("rolling_alignment_approach_not_completed_or_canceled");
          }
          rotation_previous_stamp_ns_ = 0;
          rotation_observed_rate_ = 0.0;
          center_previous_stamp_ns_ = 0;
          center_observed_vx_ = center_observed_vy_ = 0.0;
          centering_started_ = Clock::now();
          transition(Stage::MOVE_ALIGN, "moving_and_aligning_outside");
        } else if (seconds(stage_started_) > 2.0) {
          return fail("rolling_alignment_handoff_unconfirmed");
        }
        break;
      case Stage::MOVE_ALIGN: {
        if (seconds(centering_started_) > value<double>("stage_timeout")) {
          return fail("alignment_preparation_timeout");
        }
        if (actions_->follower.busy() || !rollingCorridorFree()) {
          return fail("rolling_alignment_corridor_blocked");
        }
        if (continuous_ && crossing_.direction *
                (pose_.transform.translation.x - entry_x_) >
                value<double>("position_tolerance") + 1e-9) {
          // A small forward overrun cannot be corrected by a command that
          // deliberately forbids pulling back toward the upstream target.
          // Reuse the existing stopped precision recheck instead of waiting
          // motionless until the generic stage timeout.
          stop();
          transition(Stage::STOP_APPROACH, "entry_overrun_stopped_recheck");
          break;
        }
        updateCenteringVelocity(1.0);
        updateRotationVelocity();
        const double tf_age = (node_->now().nanoseconds() -
                               rclcpp::Time(pose_.header.stamp).nanoseconds()) *
                              1e-9;
        const auto translation = continuous_ && continuation_ready_
            ? continuousApproachCommand(
                pose_.transform.translation.x, pose_.transform.translation.y,
                entry_x_, tunnels_[crossing_.tunnel].y,
                value<double>("rolling_alignment_speed"), crossing_.direction,
                value<double>("position_tolerance"),
                value<double>("waiting_longitudinal_tolerance"), headingError())
            : shortApproachCommand(
            pose_.transform.translation.x, pose_.transform.translation.y,
            entry_x_, tunnels_[crossing_.tunnel].y,
            value<double>("rolling_alignment_speed"), crossing_.direction,
            value<double>("position_tolerance"),
            value<double>("waiting_longitudinal_tolerance"),
            center_observed_vx_, center_observed_vy_,
            std::clamp(tf_age + 0.15, 0.15, 0.4));
        const double angular = preparationAngularCommand(
            wrap(target_yaw_ - tf2::getYaw(pose_.transform.rotation)),
            rotation_observed_rate_, std::clamp(tf_age + 0.05, 0.05, 0.3),
            value<double>("rotation_speed"));
        // Acceptance uses the actual pose. Qualified continuous routes carry
        // forward motion into the follower; other routes keep stopped rechecks.
        if (entryPositionReady() &&
            headingError() <= std::min(kRotationStopTolerance,
                                        value<double>("yaw_tolerance")) &&
            (!continuous_ || std::abs(rotation_observed_rate_) <= 0.02)) {
          if (continuous_ && continuation_ready_) {
            startContinuousFollow();
          } else {
            stop();
            transition(Stage::STOP_APPROACH,
                       "stopping_before_precision_centering");
          }
          break;
        }
        guardedTranslation(translation.vx, translation.vy, angular, true);
        break;
      }
      case Stage::STOP_APPROACH:
      case Stage::STOP_CENTER:
      case Stage::STOP_ROTATE:
        stop();
        if (centering_started_ != Clock::time_point{} &&
            seconds(centering_started_) > value<double>("stage_timeout")) {
          return fail("alignment_preparation_timeout");
        }
        if (settled(stopped() &&
                    (stage_ != Stage::STOP_CENTER ||
                     rclcpp::Time(pose_.header.stamp).nanoseconds() >=
                         center_stop_stamp_ns_))) {
          if (stage_ == Stage::STOP_APPROACH) {
            if (centering_started_ == Clock::time_point{}) {
              centering_started_ = Clock::now();
            }
            // Turn once while the full circular envelope is still outside.
            // Lateral precision is checked after rotation, avoiding
            // re-centering caused by the turn itself.
            if (headingError() > std::min(kRotationStopTolerance,
                                          value<double>("yaw_tolerance"))) {
              beginRotation();
            } else if (entryPositionReady()) {
              startPassagePlan();
            } else {
              beginCentering("heading_already_aligned_centering");
            }
          } else if (stage_ == Stage::STOP_CENTER) {
            if (!entryPositionReady()) {
              beginCentering("centering_after_stopped_pose_recheck");
              break;
            }
            if (headingError() > std::min(kRotationStopTolerance,
                                          value<double>("yaw_tolerance"))) {
              if (alignment_recenters_++ >= 3) {
                return fail("maximum_alignment_corrections_exceeded");
              }
              beginRotation();
              break;
            }
            startPassagePlan();
          } else {
            beginCentering("heading_aligned_centering");
          }
        }
        break;
      case Stage::CENTER: {
        if (seconds(centering_started_) > value<double>("stage_timeout")) {
          return fail("centering_timeout");
        }
        updateCenteringVelocity();
        const double tf_age = (node_->now().nanoseconds() -
                               rclcpp::Time(pose_.header.stamp).nanoseconds()) *
                              1e-9;
        const auto command = centeringCommand(
            pose_.transform.translation.x, pose_.transform.translation.y,
            entry_x_, tunnels_[crossing_.tunnel].y, crossing_.direction,
            value<double>("position_tolerance"),
            value<double>("waiting_longitudinal_tolerance"),
            value<double>("centering_speed"), center_observed_vx_,
            center_observed_vy_, std::clamp(tf_age + 0.15, 0.15, 0.4));
        if (command.stop) {
          stop();
          center_stop_stamp_ns_ = node_->now().nanoseconds();
          transition(Stage::STOP_CENTER, "centering_braking_confirming_stop");
        } else {
          guardedTranslation(command.vx, command.vy);
        }
        break;
      }
      case Stage::ROTATE: {
        if (seconds(centering_started_) > value<double>("stage_timeout")) {
          return fail("alignment_preparation_timeout");
        }
        rotationGuard();
        if (std::hypot(pose_.transform.translation.x - rotation_x_,
                       pose_.transform.translation.y - rotation_y_) > 0.035) {
          return fail("position_drift_during_alignment");
        }
        const double error =
            wrap(target_yaw_ - tf2::getYaw(pose_.transform.rotation));
        updateRotationVelocity();
        const double tf_age = (node_->now().nanoseconds() -
                               rclcpp::Time(pose_.header.stamp).nanoseconds()) *
                              1e-9;
        geometry_msgs::msg::Twist command;
        command.angular.z =
            preparationAngularCommand(error, rotation_observed_rate_,
                                      std::clamp(tf_age + 0.05, 0.05, 0.3),
                                      value<double>("rotation_speed"));
        cmd_pub_->publish(command);
        if (settled(command.angular.z == 0.0 &&
                    std::abs(error) <= value<double>("yaw_tolerance") &&
                    stopped())) {
          stop();
          // The same stopped/yaw condition was already held for 0.5 s.
          // Do not add another identical STOP_ROTATE wait before centering.
          if (entryPositionReady()) {
            startPassagePlan();
          } else {
            beginCentering("heading_aligned_centering");
          }
        }
        break;
      }
      case Stage::FOLLOW_CONTINUATION: {
        const int result = actions_->follower.poll();
        if (result < 0) {
          return fail("continuous_final_controller_failed");
        }
        if (result > 0) {
          if (remaining_goals_.size() != 1 || changed(remaining_goals_.back(), goal_)) {
            return fail("continuous_final_goal_not_preserved");
          }
          // A prefetched replan or superseded controller UUID can still be
          // finishing. Final arrival may stop; do not lose their ownership.
          stop();
          if (!cancelOwned()) {
            return fail("continuous_final_cleanup_unconfirmed");
          }
          setOutput("task_completed", true);
          publishState("continuous_final_goal_reached");
          return complete(); // The only normal end-of-route stop.
        }
        updateContinuationPlan();
        tryEnableExitSpin();
        break;
      }
      case Stage::VERIFY_EXIT: {
        stop();
        const auto &tunnel = tunnels_[crossing_.tunnel];
        const double boundary =
            crossing_.direction > 0 ? tunnel.x_max : tunnel.x_min;
        const double clearance =
            crossing_.direction * (pose_.transform.translation.x - boundary);
        if (clearance < std::max(0.30, longitudinal_extent_ + 0.01)) {
          return fail("controller_stopped_before_full_body_exit");
        }
        if (settled(stopped())) {
          visited_.insert(crossing_.tunnel);
          publishState("full_body_exit_confirmed");
          if (through_poses_) {
            recordObservedTrace();
            const size_t passed =
                passedGoalPrefix(remaining_goals_, passage_trace_, 0.15);
            remaining_goals_.erase(remaining_goals_.begin(),
                                   remaining_goals_.begin() + passed);
            setOutput("goals", remaining_goals_);
            publishState("through_poses_consumed_prefix=" +
                         std::to_string(passed));
            return complete();
          }
          setLimit(0.0);
          sendPlan(goal_);
          transition(Stage::PLAN_FINAL, "replanning_to_original_goal");
        }
        break;
      }
      case Stage::WAIT_DATA:
      case Stage::CLASSIFY_BODY:
      case Stage::CANCELING:
        break;
      }
      return BT::NodeStatus::RUNNING;
    } catch (const std::exception &error) {
      return fail(error.what());
    }
  }
  bool entryPositionReady() const {
    return waitingPositionAcceptable(
        pose_.transform.translation.x, pose_.transform.translation.y, entry_x_,
        tunnels_[crossing_.tunnel].y, crossing_.direction,
        value<double>("position_tolerance"),
        value<double>("waiting_longitudinal_tolerance"));
  }
  double headingError() const {
    return std::abs(wrap(target_yaw_ - tf2::getYaw(pose_.transform.rotation)));
  }
  void beginRotation() {
    rotation_previous_stamp_ns_ = 0;
    rotation_observed_rate_ = 0.0;
    rotation_x_ = pose_.transform.translation.x;
    rotation_y_ = pose_.transform.translation.y;
    rotationGuard();
    transition(Stage::ROTATE, "aligning_real_chassis_heading_before_centering");
  }
  void updateRotationVelocity() {
    const auto stamp = rclcpp::Time(pose_.header.stamp).nanoseconds();
    if (stamp <= rotation_previous_stamp_ns_) {
      return;
    }
    const double yaw = tf2::getYaw(pose_.transform.rotation);
    const double dt = (stamp - rotation_previous_stamp_ns_) * 1e-9;
    if (rotation_previous_stamp_ns_ > 0 && dt >= 0.02 && dt <= 0.5) {
      rotation_observed_rate_ =
          std::clamp(wrap(yaw - rotation_previous_yaw_) / dt, -1.0, 1.0);
    }
    rotation_previous_yaw_ = yaw;
    rotation_previous_stamp_ns_ = stamp;
  }
  void beginCentering(const std::string &reason) {
    center_previous_stamp_ns_ = 0;
    center_observed_vx_ = center_observed_vy_ = 0.0;
    transition(Stage::CENTER, reason);
  }
  void updateCenteringVelocity(double speed_bound = 0.4) {
    const auto stamp = rclcpp::Time(pose_.header.stamp).nanoseconds();
    if (stamp <= center_previous_stamp_ns_) {
      return;
    }
    const double x = pose_.transform.translation.x,
                 y = pose_.transform.translation.y;
    const double dt = (stamp - center_previous_stamp_ns_) * 1e-9;
    if (center_previous_stamp_ns_ > 0 && dt >= 0.02 && dt <= 0.5) {
      center_observed_vx_ =
          std::clamp((x - center_previous_x_) / dt, -speed_bound, speed_bound);
      center_observed_vy_ =
          std::clamp((y - center_previous_y_) / dt, -speed_bound, speed_bound);
    }
    center_previous_x_ = x;
    center_previous_y_ = y;
    center_previous_stamp_ns_ = stamp;
  }
  void startPassagePlan() {
    if (headingError() > value<double>("yaw_tolerance") ||
        std::abs(pose_.transform.translation.y - tunnels_[crossing_.tunnel].y) >
            value<double>("position_tolerance")) {
      throw std::runtime_error("Stopped entry alignment is outside tolerance");
    }
    if (continuous_) {
      if (continuation_ready_) {
        startContinuousFollow();
      }
      return;
    }
    setLimit(value<double>("passage_speed"));
    sendPlan(exit_x_, tunnels_[crossing_.tunnel].y);
    transition(Stage::PLAN_PASSAGE, "replanning_after_heading_alignment");
  }
  void recordObservedTrace() {
    if (!through_poses_) {
      return;
    }
    if (!passage_trace_.poses.empty() &&
        rclcpp::Time(passage_trace_.poses.back().header.stamp) >=
            rclcpp::Time(pose_.header.stamp)) {
      return;
    }
    geometry_msgs::msg::PoseStamped sample;
    sample.header = pose_.header;
    sample.pose.position.x = pose_.transform.translation.x;
    sample.pose.position.y = pose_.transform.translation.y;
    sample.pose.orientation = pose_.transform.rotation;
    passage_trace_.poses.push_back(sample);
  }
  void updateApproachLimit(bool initialize) {
    const double distance = std::hypot(pose_.transform.translation.x - entry_x_,
                                       pose_.transform.translation.y -
                                           tunnels_[crossing_.tunnel].y);
    const bool slowed = approachSlowdownRequired(
        distance, value<double>("approach_slowdown_distance"),
        approach_slowed_);
    if (initialize || slowed != approach_slowed_) {
      approach_slowed_ = slowed;
      setLimit(slowed ? value<double>("approach_speed") : 0.0);
    }
  }
  void beginPassage() {
    const auto &tunnel = tunnels_[crossing_.tunnel];
    entry_x_ = (crossing_.direction > 0 ? tunnel.x_min : tunnel.x_max) -
               crossing_.direction * value<double>("entry_distance");
    exit_x_ = (crossing_.direction > 0 ? tunnel.x_max : tunnel.x_min) +
              crossing_.direction * value<double>("exit_distance");
    target_yaw_ = crossing_.direction > 0 ? 0.0 : M_PI;
    centering_started_ = {};
    continuous_ = value<bool>("continuous_path") && through_poses_ &&
        continuousThreePointRoute(remaining_goals_, tunnel, crossing_.direction,
                                  entry_x_, value<double>("position_tolerance"),
                                  longitudinal_extent_) &&
        firstCrossing(route_, tunnels_, {crossing_.tunnel}).tunnel < 0;
    continuation_needed_ = continuous_;
    continuation_pending_ = continuation_ready_ = false;
    continuation_path_.poses.clear();
    approach_slowed_ = false;
    updateApproachLimit(true);
    sendPlan(entry_x_, tunnel.y);
    transition(Stage::PLAN_APPROACH, "planning_outside_waiting_point");
  }
  bool fullBodyCleared() const {
    const auto &tunnel = tunnels_[crossing_.tunnel];
    const double boundary = crossing_.direction > 0 ? tunnel.x_max : tunnel.x_min;
    const double x = pose_.transform.translation.x;
    const double yaw = tf2::getYaw(pose_.transform.rotation);
    if (crossing_.direction * (x - boundary) < 0.30) {
      return false;
    }
    return std::all_of(footprint_.begin(), footprint_.end(), [&](const auto &p) {
      const double world_x = x + std::cos(yaw) * p.x - std::sin(yaw) * p.y;
      return crossing_.direction * (world_x - boundary) >= 0.01;
    });
  }
  void prepareContinuation() {
    if (!continuous_ || !continuation_config_checked_) {
      return;
    }
    if (!continuation_config_valid_) {
      continuous_ = false;
      RCLCPP_WARN(node_->get_logger(), "Continuous route unavailable: controller profile unverified");
      return;
    }
    if (continuation_needed_ && stage_ != Stage::PLAN_APPROACH &&
        !actions_->planner.busy()) {
      const auto &tunnel = tunnels_[crossing_.tunnel];
      const double boundary = crossing_.direction > 0 ? tunnel.x_max : tunnel.x_min;
      exit_x_ = boundary + crossing_.direction * continuousExitDistance(
          value<double>("exit_distance"), longitudinal_extent_, continuation_lookahead_);
      Compute::Goal request;
      request.goal = goal_;
      request.goal.header.stamp = node_->now();
      request.start.header = request.goal.header;
      request.start.pose.position.x = exit_x_;
      request.start.pose.position.y = tunnel.y;
      request.start.pose.orientation.z = std::sin(target_yaw_ / 2.0);
      request.start.pose.orientation.w = std::cos(target_yaw_ / 2.0);
      request.use_start = true;
      request.planner_id = value<std::string>("planner_id");
      actions_->planner.send(request);
      continuation_needed_ = false;
      continuation_pending_ = true;
    }
    if (continuation_pending_) {
      const int result = actions_->planner.poll();
      if (result == 0) {
        return;
      }
      continuation_pending_ = false;
      if (result < 0) {
        continuous_ = false;
        RCLCPP_WARN(node_->get_logger(), "Continuous tail planning failed; using stopped passage");
        return;
      }
      continuation_path_ = actions_->planner.result.get().result->path;
      if (continuation_path_.poses.empty() ||
          firstCrossing(continuation_path_, tunnels_).tunnel >= 0) {
        continuous_ = false;
        RCLCPP_WARN(node_->get_logger(), "Continuous tail empty or selects another tunnel");
        return;
      }
      continuation_ready_ = true;
      RCLCPP_INFO(node_->get_logger(), "Continuous tail prefetched before entry, exit_x=%.3f", exit_x_);
    }
  }
  void startContinuousFollow() {
    auto combined = appendContinuousTail(straightPassagePath(), continuation_path_, goal_);
    combined.header.stamp = node_->now();
    validateContinuousPath(combined);
    setLimit(value<double>("passage_speed"));
    Follow::Goal request;
    follower_path_ = combined;
    request.path = std::move(combined);
    request.controller_id = value<std::string>("controller_id");
    request.goal_checker_id = "general_goal_checker";
    actions_->follower.send(request);
    transition(Stage::FOLLOW_PASSAGE, "following_tunnel_axis_continuous");
  }
  void validateContinuousPath(nav_msgs::msg::Path &path) {
    const auto &global = maps_[1];
    if (path.header.frame_id != value<std::string>("map_frame") ||
        global.message->header.frame_id != path.header.frame_id || path.poses.empty() ||
        changed(path.poses.back(), goal_)) {
      throw std::runtime_error("Continuous path lost map frame or exact final goal");
    }
    path.header.stamp = node_->now();
    if (exit_spin_active_) {
      const double x = pose_.transform.translation.x, y = pose_.transform.translation.y;
      if (!remainingSpinPathSafe(*global.grid, path, x, y, radius_)) {
        throw std::runtime_error("Remaining spinning path blocked in global map");
      }
      // A planner reply may start before spin was enabled. Never revalidate or
      // resend its already-travelled narrow-tunnel prefix with a rotation disk.
      size_t nearest = 0;
      double distance = std::numeric_limits<double>::infinity();
      for (size_t i = 0; i < path.poses.size(); ++i) {
        const auto &p = path.poses[i].pose.position;
        const double d = std::hypot(p.x - x, p.y - y);
        if (d < distance) { nearest = i; distance = d; }
      }
      const size_t first = std::min(nearest + 1, path.poses.size() - 1);
      path.poses.erase(path.poses.begin(), path.poses.begin() + first);
      geometry_msgs::msg::PoseStamped current;
      current.header = path.header;
      current.pose.position.x = x;
      current.pose.position.y = y;
      current.pose.orientation = pose_.transform.rotation;
      path.poses.insert(path.poses.begin(), current);
    }
    const double yaw = tf2::getYaw(pose_.transform.rotation);
    const auto envelope = exit_spin_active_
        ? circumscribedRotationFootprint(radius_) : footprint_;
    for (size_t i = 0; i < path.poses.size(); ++i) {
      if (path.poses[i].header.frame_id != path.header.frame_id) {
        throw std::runtime_error("Continuous path contains mixed frames");
      }
      path.poses[i].header = path.header;
      const auto &a = path.poses[i == 0 ? 0 : i - 1].pose.position;
      const auto &b = path.poses[i].pose.position;
      if (pb_omni_pid_pursuit_controller::sweptFootprintCollision(
              *global.grid, envelope, a.x, a.y, b.x, b.y,
              exit_spin_active_ ? 0.0 : yaw)) {
        throw std::runtime_error("Continuous full-body path blocked in global map");
      }
    }
  }
  void updateContinuationPlan() {
    // The old FollowPath remains in control while the planner runs. Updating
    // an accepted controller goal uses preemption, never cancel/stop handoff.
    if (continuation_replan_pending_) {
      if (!continuation_replan_ready_) {
        const int result = actions_->planner.poll();
        if (result == 0) {
          return;
        }
        if (result < 0) {
          throw std::runtime_error("Continuous ordinary replan failed");
        }
        continuation_replan_ready_ = true;
      }
      if (!actions_->follower.handle) {
        return; // A new controller UUID has not been acknowledged yet.
      }
      auto path = actions_->planner.result.get().result->path;
      if (firstCrossing(path, tunnels_).tunnel >= 0) {
        throw std::runtime_error("Continuous replan requires a new tunnel preparation");
      }
      validateContinuousPath(path);
      Follow::Goal request;
      request.path = path;
      request.controller_id = value<std::string>("controller_id");
      request.goal_checker_id = "general_goal_checker";
      if (!actions_->follower.preempt(request)) {
        return; // Acceptance/final completion raced with this BT tick.
      }
      follower_path_ = std::move(path);
      continuation_replan_pending_ = continuation_replan_ready_ = false;
    }
    if (!continuation_replan_pending_ && !actions_->planner.busy() &&
        seconds(continuation_replanned_) >= 1.0 / 3.0) {
      sendPlan(goal_);
      continuation_replan_pending_ = true;
      continuation_replanned_ = Clock::now();
    }
  }
  void sendPlan(double x, double y) {
    geometry_msgs::msg::PoseStamped target;
    target.header.frame_id = value<std::string>("map_frame");
    target.header.stamp = node_->now();
    target.pose.position.x = x;
    target.pose.position.y = y;
    // YAstar uses actual chassis yaw for footprint validation, not this
    // orientation.
    target.pose.orientation = pose_.transform.rotation;
    sendPlan(target);
  }
  void sendPlan(const geometry_msgs::msg::PoseStamped &target) {
    if (!actions_->planner.client->action_server_is_ready() ||
        !actions_->follower.client->action_server_is_ready()) {
      throw std::runtime_error("Planner/controller action server unavailable");
    }
    Compute::Goal request;
    request.goal = target;
    request.goal.header.stamp = node_->now();
    request.planner_id = value<std::string>("planner_id");
    request.use_start = false;
    actions_->planner.send(request);
  }
  nav_msgs::msg::Path straightPassagePath() {
    // YAstar has confirmed the selected crossing after rotation. The dedicated
    // passage phase holds its axis until the whole chassis has cleared the
    // exit; it must not inherit a planner's early turn toward the ultimate
    // goal.
    nav_msgs::msg::Path result;
    result.header.frame_id = value<std::string>("map_frame");
    result.header.stamp = node_->now();
    const double sx = pose_.transform.translation.x,
                 sy = pose_.transform.translation.y;
    const double distance = std::abs(exit_x_ - sx);
    const int count =
        std::max(1, static_cast<int>(std::ceil(distance / 0.025)));
    for (int i = 0; i <= count; ++i) {
      const double u = static_cast<double>(i) / count;
      geometry_msgs::msg::PoseStamped sample;
      sample.header = result.header;
      sample.pose.position.x = sx + u * (exit_x_ - sx);
      const double center_fraction = std::min(1.0, u * distance / 0.20);
      sample.pose.position.y =
          sy + center_fraction * (tunnels_[crossing_.tunnel].y - sy);
      sample.pose.orientation.z = std::sin(target_yaw_ / 2.0);
      sample.pose.orientation.w = std::cos(target_yaw_ / 2.0);
      result.poses.push_back(sample);
    }
    const auto &global = maps_[1];
    if (global.message->header.frame_id != value<std::string>("map_frame")) {
      throw std::runtime_error(
          "Global costmap must use map_frame for axis-path validation");
    }
    const double yaw = tf2::getYaw(pose_.transform.rotation);
    for (size_t i = 1; i < result.poses.size(); ++i) {
      const auto &a = result.poses[i - 1].pose.position;
      const auto &b = result.poses[i].pose.position;
      if (pb_omni_pid_pursuit_controller::sweptFootprintCollision(
              *global.grid, footprint_, a.x, a.y, b.x, b.y, yaw)) {
        throw std::runtime_error(
            "Straight tunnel axis full-body sweep blocked in global map");
      }
    }
    return result;
  }
  bool localExitSpinCorridorSafe() {
    const auto &input = maps_[0];
    const auto body = transform(input.message->header.frame_id);
    const auto disk = circumscribedRotationFootprint(radius_);
    if (rotationCollision(*input.grid, body.transform.translation.x,
                          body.transform.translation.y, radius_)) {
      return false;
    }
    if (seconds(command_received_) > 0.25 ||
        !std::isfinite(observed_command_.linear.x + observed_command_.linear.y)) {
      return false;
    }
    geometry_msgs::msg::PoseStamped projected;
    projected.header.frame_id = value<std::string>("control_frame");
    projected.header.stamp = pose_.header.stamp;
    projected.pose.orientation.w = 1.0;
    projected.pose.position.x = observed_command_.linear.x * 0.5;
    projected.pose.position.y = observed_command_.linear.y * 0.5;
    const auto command_end = tf_->transform(projected, input.message->header.frame_id);
    if (pb_omni_pid_pursuit_controller::sweptFootprintCollision(
            *input.grid, disk, body.transform.translation.x, body.transform.translation.y,
            command_end.pose.position.x, command_end.pose.position.y, 0.0)) {
      return false;
    }
    const double x = pose_.transform.translation.x, y = pose_.transform.translation.y;
    size_t nearest = 0;
    double nearest_distance = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i < follower_path_.poses.size(); ++i) {
      const auto &p = follower_path_.poses[i].pose.position;
      const double d = std::hypot(p.x - x, p.y - y);
      if (d < nearest_distance) { nearest_distance = d; nearest = i; }
    }
    // First spin command is downstream of Nav2. Check a future corridor here
    // before measured angular velocity lets the controller observe the spin.
    double remaining = std::max(0.5, 0.5 * std::hypot(
        odom_->twist.twist.linear.x, odom_->twist.twist.linear.y));
    double px = x, py = y;
    geometry_msgs::msg::Point previous;
    previous.x = body.transform.translation.x;
    previous.y = body.transform.translation.y;
    for (size_t i = std::min(nearest + 1, follower_path_.poses.size() - 1);
         i < follower_path_.poses.size() && remaining > 1e-6; ++i) {
      const auto &p = follower_path_.poses[i].pose.position;
      const double distance = std::hypot(p.x - px, p.y - py);
      const double fraction = distance > 1e-9 ? std::min(1.0, remaining / distance) : 1.0;
      auto endpoint = follower_path_.poses[i];
      endpoint.header.stamp = pose_.header.stamp;
      endpoint.pose.position.x = px + fraction * (p.x - px);
      endpoint.pose.position.y = py + fraction * (p.y - py);
      const auto local = tf_->transform(endpoint, input.message->header.frame_id);
      if (pb_omni_pid_pursuit_controller::sweptFootprintCollision(
              *input.grid, disk, previous.x, previous.y,
              local.pose.position.x, local.pose.position.y, 0.0)) {
        return false;
      }
      remaining -= distance;
      px = p.x; py = p.y;
      previous = local.pose.position;
    }
    return true;
  }
  void tryEnableExitSpin() {
    if (exit_spin_active_ || std::abs(exit_spin_speed_) <= 1e-5 ||
        follower_path_.poses.empty()) {
      return;
    }
    const auto &global = maps_[1];
    if (!exitSpinSafe(*global.grid, follower_path_, tunnels_[crossing_.tunnel],
                      crossing_.direction, pose_.transform.translation.x,
                      pose_.transform.translation.y, radius_) ||
        !localExitSpinCorridorSafe()) {
      return; // Keep translating out; never rotate an overlapping chassis.
    }
    example_interfaces::msg::Float32 command;
    command.data = static_cast<float>(exit_spin_speed_);
    spin_pub_->publish(command);
    spin_speed_ = command.data;
    exit_spin_active_ = true;
    publishState("exit_rotation_envelope_clear_starting_spin");
  }
  void rotationGuard() {
    for (const auto &input : maps_) {
      const auto body = transform(input.message->header.frame_id);
      if (rotationCollision(*input.grid, body.transform.translation.x,
                            body.transform.translation.y, radius_)) {
        throw std::runtime_error(
            "Full rotation envelope blocked, unknown or out of map");
      }
    }
  }
  bool rollingCorridorFree() {
    const auto &tunnel = tunnels_[crossing_.tunnel];
    const double boundary =
        crossing_.direction > 0 ? tunnel.x_min : tunnel.x_max;
    // Neither the current nor target rotation disk may enter the tunnel.
    if (crossing_.direction * (boundary - pose_.transform.translation.x) <
            radius_ + 0.05 ||
        crossing_.direction * (boundary - entry_x_) < radius_ + 0.05) {
      return false;
    }
    const auto disk = circumscribedRotationFootprint(radius_);
    geometry_msgs::msg::PoseStamped target;
    target.header.frame_id = value<std::string>("map_frame");
    target.header.stamp = pose_.header.stamp;
    target.pose.position.x = entry_x_;
    target.pose.position.y = tunnel.y;
    target.pose.orientation.w = 1.0;
    for (const auto &input : maps_) {
      const auto body = transform(input.message->header.frame_id);
      const auto end = tf_->transform(target, input.message->header.frame_id);
      if (pb_omni_pid_pursuit_controller::sweptFootprintCollision(
              *input.grid, disk, body.transform.translation.x,
              body.transform.translation.y, end.pose.position.x,
              end.pose.position.y, 0.0)) {
        return false;
      }
    }
    return true;
  }
  void guardedTranslation(double vx_map, double vy_map, double angular = 0.0,
                          bool rotating_envelope = false) {
    const auto envelope = rotating_envelope
                              ? circumscribedRotationFootprint(radius_)
                              : footprint_;
    for (const auto &input : maps_) {
      const auto body = transform(input.message->header.frame_id);
      auto map_transform = tf_->lookupTransform(input.message->header.frame_id,
                                                value<std::string>("map_frame"),
                                                tf2::TimePointZero);
      const double theta = tf2::getYaw(map_transform.transform.rotation);
      const double vx = std::cos(theta) * vx_map - std::sin(theta) * vy_map;
      const double vy = std::sin(theta) * vx_map + std::cos(theta) * vy_map;
      if (pb_omni_pid_pursuit_controller::sweptFootprintCollision(
              *input.grid, envelope, body.transform.translation.x,
              body.transform.translation.y,
              body.transform.translation.x + vx * 0.5,
              body.transform.translation.y + vy * 0.5,
              rotating_envelope ? 0.0 : tf2::getYaw(body.transform.rotation))) {
        throw std::runtime_error(
            "Precision-centering current/future full footprint blocked");
      }
    }
    const auto control = tf_->lookupTransform(
        value<std::string>("map_frame"), value<std::string>("control_frame"),
        tf2::TimePointZero);
    const double yaw = tf2::getYaw(control.transform.rotation);
    geometry_msgs::msg::Twist command;
    command.linear.x = std::cos(yaw) * vx_map + std::sin(yaw) * vy_map;
    command.linear.y = -std::sin(yaw) * vx_map + std::cos(yaw) * vy_map;
    command.angular.z = angular;
    cmd_pub_->publish(command);
  }
  bool stopped() const {
    const auto &v = odom_->twist.twist;
    return std::hypot(v.linear.x, v.linear.y) <= 0.02 &&
           std::abs(v.angular.z) <= 0.03;
  }
  bool settled(bool condition) {
    if (!condition) {
      stable_since_ = {};
      return false;
    }
    if (stable_since_ == Clock::time_point{}) {
      stable_since_ = Clock::now();
    }
    return seconds(stable_since_) >= 0.5;
  }
  static double seconds(Clock::time_point since) {
    return std::chrono::duration<double>(Clock::now() - since).count();
  }
  static bool changed(const geometry_msgs::msg::PoseStamped &a,
                      const geometry_msgs::msg::PoseStamped &b) {
    return a.header.frame_id != b.header.frame_id ||
           std::hypot(a.pose.position.x - b.pose.position.x,
                      a.pose.position.y - b.pose.position.y) > 1e-6 ||
           std::abs(wrap(tf2::getYaw(a.pose.orientation) -
                         tf2::getYaw(b.pose.orientation))) > 1e-6;
  }
  void stop() { cmd_pub_->publish(geometry_msgs::msg::Twist()); }
  void stopSpin() {
    example_interfaces::msg::Float32 command;
    command.data = 0.0;
    spin_pub_->publish(command);
    spin_speed_ = 0.0;
  }
  void setLimit(double limit) {
    limit_ = limit;
    last_limit_ = Clock::now();
    nav2_msgs::msg::SpeedLimit message;
    message.header.stamp = node_->now();
    message.percentage = false;
    message.speed_limit = limit;
    limit_pub_->publish(message);
  }
  static const char *jsonBool(bool value) { return value ? "true" : "false"; }
  static std::string receivedAge(Clock::time_point received,
                                 Clock::time_point collected) {
    if (received == Clock::time_point{}) {
      return "null";
    }
    return diagnosticJsonNumber(
        std::chrono::duration<double>(collected - received).count());
  }
  static std::string
  transformDiagnostic(const geometry_msgs::msg::TransformStamped &transform) {
    const auto &p = transform.transform.translation;
    const auto &q = transform.transform.rotation;
    const double norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
    const bool finite = std::isfinite(p.x) && std::isfinite(p.y) &&
                        std::isfinite(p.z) && std::isfinite(norm) &&
                        norm > 1e-12;
    std::ostringstream out;
    out << "{\"available\":true,\"finite_pose\":" << jsonBool(finite)
        << ",\"parent\":" << diagnosticJsonString(transform.header.frame_id)
        << ",\"child\":" << diagnosticJsonString(transform.child_frame_id)
        << ",\"stamp_ns\":"
        << rclcpp::Time(transform.header.stamp).nanoseconds()
        << ",\"x\":" << diagnosticJsonNumber(p.x)
        << ",\"y\":" << diagnosticJsonNumber(p.y)
        << ",\"z\":" << diagnosticJsonNumber(p.z) << ",\"yaw_rad\":"
        << (finite ? diagnosticJsonNumber(tf2::getYaw(q)) : "null") << '}';
    return out.str();
  }
  std::string exactTransformDiagnostic(const std::string &parent,
                                       const std::string &child,
                                       const rclcpp::Time &stamp) const {
    try {
      // Exact-stamp cache lookup: no wait and no latest/identity fallback.
      return transformDiagnostic(tf_->lookupTransform(parent, child, stamp));
    } catch (const std::exception &error) {
      return "{\"available\":false,\"parent\":" + diagnosticJsonString(parent) +
             ",\"child\":" + diagnosticJsonString(child) +
             ",\"requested_stamp_ns\":" + std::to_string(stamp.nanoseconds()) +
             ",\"error\":" + diagnosticJsonString(error.what()) + "}";
    }
  }
  void publishDeviationDiagnostic(const PassageDeviationCheck &check,
                                  const rclcpp::Time &decision_time) noexcept {
    try {
      const auto collected_wall = Clock::now();
      const auto collected_ros = node_->now();
      const auto pose_stamp = rclcpp::Time(pose_.header.stamp);
      const auto map_frame = value<std::string>("map_frame");
      const auto body_frame = value<std::string>("body_frame");
      const auto sourceAge = [&](const builtin_interfaces::msg::Time &stamp) {
        return diagnosticJsonNumber(
            (collected_ros.nanoseconds() - rclcpp::Time(stamp).nanoseconds()) *
            1e-9);
      };
      std::ostringstream out;
      out << "{\"schema\":\"sentry_tunnel_deviation_v1\",\"reason\":"
          << "\"tunnel_heading_or_centerline_deviation\",\"stage\":\"FOLLOW_"
             "PASSAGE\""
          << ",\"tunnel\":" << crossing_.tunnel
          << ",\"direction\":" << crossing_.direction
          << ",\"decision_ros_ns\":" << decision_time.nanoseconds()
          << ",\"collected_ros_ns\":" << collected_ros.nanoseconds()
          << ",\"collected_steady_ns\":"
          << std::chrono::duration_cast<std::chrono::nanoseconds>(
                 collected_wall.time_since_epoch())
                 .count()
          << ",\"input_values_valid\":" << jsonBool(check.valid)
          << ",\"yaw_exceeded\":" << jsonBool(check.yaw_exceeded)
          << ",\"lateral_exceeded\":" << jsonBool(check.lateral_exceeded)
          << ",\"actual_yaw_rad\":" << diagnosticJsonNumber(check.actual_yaw)
          << ",\"target_yaw_rad\":" << diagnosticJsonNumber(check.target_yaw)
          << ",\"yaw_error_rad\":" << diagnosticJsonNumber(check.yaw_error)
          << ",\"yaw_limit_rad\":" << diagnosticJsonNumber(check.yaw_limit)
          << ",\"center_y_m\":" << diagnosticJsonNumber(check.center_y)
          << ",\"signed_lateral_error_m\":"
          << diagnosticJsonNumber(check.signed_lateral_error)
          << ",\"lateral_limit_m\":"
          << diagnosticJsonNumber(check.lateral_limit)
          << ",\"map_body\":" << transformDiagnostic(pose_)
          << ",\"map_body_source_age_sec\":" << sourceAge(pose_.header.stamp);
      if (odom_) {
        const auto &p = odom_->pose.pose.position;
        const auto &q = odom_->pose.pose.orientation;
        const auto &v = odom_->twist.twist;
        const double norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
        const bool finite_pose = std::isfinite(p.x) && std::isfinite(p.y) &&
                                 std::isfinite(p.z) && std::isfinite(norm) &&
                                 norm > 1e-12;
        const std::string odom_frame = odom_->header.frame_id;
        out << ",\"odom_message\":{\"available\":true,\"finite_pose\":"
            << jsonBool(finite_pose)
            << ",\"frame\":" << diagnosticJsonString(odom_frame)
            << ",\"child\":" << diagnosticJsonString(odom_->child_frame_id)
            << ",\"stamp_ns\":"
            << rclcpp::Time(odom_->header.stamp).nanoseconds()
            << ",\"x\":" << diagnosticJsonNumber(p.x)
            << ",\"y\":" << diagnosticJsonNumber(p.y)
            << ",\"z\":" << diagnosticJsonNumber(p.z) << ",\"yaw_rad\":"
            << (finite_pose ? diagnosticJsonNumber(tf2::getYaw(q)) : "null")
            << ",\"vx\":" << diagnosticJsonNumber(v.linear.x)
            << ",\"vy\":" << diagnosticJsonNumber(v.linear.y)
            << ",\"angular_z\":" << diagnosticJsonNumber(v.angular.z)
            << ",\"source_age_sec\":" << sourceAge(odom_->header.stamp)
            << ",\"received_wall_age_sec\":"
            << receivedAge(odom_received_, collected_wall) << '}';
        if (odom_frame.empty()) {
          out << ",\"map_odom_at_pose_stamp\":{\"available\":false,\"error\":"
                 "\"empty_odometry_frame\"}"
              << ",\"odom_body_at_pose_stamp\":{\"available\":false,\"error\":"
                 "\"empty_odometry_frame\"}";
        } else {
          out << ",\"map_odom_at_pose_stamp\":"
              << exactTransformDiagnostic(map_frame, odom_frame, pose_stamp)
              << ",\"odom_body_at_pose_stamp\":"
              << exactTransformDiagnostic(odom_frame, body_frame, pose_stamp);
        }
      } else {
        out << ",\"odom_message\":{\"available\":false}";
      }
      out << ",\"scan\":{\"available\":" << jsonBool(static_cast<bool>(scan_));
      if (scan_) {
        out << ",\"stamp_ns\":"
            << rclcpp::Time(scan_->header.stamp).nanoseconds()
            << ",\"frame\":" << diagnosticJsonString(scan_->header.frame_id)
            << ",\"source_age_sec\":" << sourceAge(scan_->header.stamp)
            << ",\"received_wall_age_sec\":"
            << receivedAge(scan_received_, collected_wall);
      }
      // Escape the original message: external malformed or future payloads
      // cannot corrupt this JSON. No parsing occurs in the high-rate callback.
      out << "},\"localization_status\":{\"available\":"
          << jsonBool(localization_received_ != Clock::time_point{})
          << ",\"sample_relation\":\"last_received_not_time_synchronized\""
          << ",\"topic\":"
          << diagnosticJsonString(localization_sub_->get_topic_name())
          << ",\"received_ros_ns\":" << localization_received_ros_ns_
          << ",\"received_wall_age_sec\":"
          << receivedAge(localization_received_, collected_wall)
          << ",\"truncated\":" << jsonBool(localization_status_truncated_)
          << ",\"last_message_json\":"
          << diagnosticJsonString(localization_status_json_) << "}}";
      std_msgs::msg::String message;
      message.data = out.str();
      deviation_pub_->publish(message);
      RCLCPP_ERROR(node_->get_logger(), "TUNNEL_FAILURE_DIAGNOSTIC %s",
                   message.data.c_str());
    } catch (const std::exception &error) {
      // Never replace the original reason or bypass fail() if observation
      // fails.
      RCLCPP_ERROR(node_->get_logger(),
                   "Tunnel deviation diagnostic unavailable: %s", error.what());
    } catch (...) {
      RCLCPP_ERROR(node_->get_logger(),
                   "Tunnel deviation diagnostic unavailable");
    }
  }
  void publishState(const std::string &text) {
    std_msgs::msg::String state;
    state.data = "tunnel=" + std::to_string(crossing_.tunnel) +
                 " direction=" + std::to_string(crossing_.direction) +
                 " phase=" + text;
    state_pub_->publish(state);
    RCLCPP_INFO(node_->get_logger(),
                "TUNNEL_STAGE %s original_goal=(%.3f,%.3f)", state.data.c_str(),
                goal_.pose.position.x, goal_.pose.position.y);
  }
  void transition(Stage stage, const std::string &text) {
    stage_ = stage;
    stage_started_ = Clock::now();
    stable_since_ = {};
    publishState(text);
  }
  bool cancelOwned() {
    const auto deadline = Clock::now() + 2s;
    actions_->planner.cancel();
    actions_->follower.cancel();
    while (rclcpp::ok() &&
           (actions_->planner.busy() || actions_->follower.busy()) &&
           Clock::now() < deadline) {
      actions_->planner.cancel();
      actions_->follower.cancel();
      stop();
      std::this_thread::sleep_for(10ms);
    }
    const bool done = !actions_->planner.busy() && !actions_->follower.busy();
    if (!done) {
      RCLCPP_ERROR(node_->get_logger(),
                   "Tunnel cancellation unconfirmed; motion ownership retained "
                   "until terminal");
    }
    return done;
  }
  BT::NodeStatus tickCanceling() {
    stopSpin();
    stop();
    actions_->planner.cancel();
    actions_->follower.cancel();
    if (!actions_->planner.busy() && !actions_->follower.busy()) {
      actions_->disarmCleanup();
      setLimit(0.0);
      active_ = false;
      publishState("failed_after_confirmed_stop:" + failure_reason_);
      return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::RUNNING;
  }
  BT::NodeStatus fail(const std::string &reason) {
    successful_spin_hold_ = false;
    stopSpin();
    stop();
    if (!active_) {
      publishState("initialization_failed:" + reason);
      RCLCPP_ERROR(node_->get_logger(), "Tunnel initialization failed: %s",
                   reason.c_str());
      return BT::NodeStatus::FAILURE;
    }
    if (stage_ != Stage::CANCELING) {
      if ((stage_ == Stage::FOLLOW_PASSAGE || stage_ == Stage::VERIFY_EXIT) &&
          crossing_.tunnel >= 0) {
        passage_failure_latched_ = true;
      }
      failure_reason_ = reason;
      transition(Stage::CANCELING, "canceling_after_failure:" + reason);
      actions_->planner.cancel();
      actions_->follower.cancel();
    }
    return tickCanceling();
  }

  BT::NodeStatus complete() {
    successful_spin_hold_ = exit_spin_active_ &&
        value<bool>("keep_exit_spin_at_goal");
    if (exit_spin_active_ && !successful_spin_hold_) {
      stopSpin();
    }
    stop();
    setLimit(0.0);
    publishState("complete_resuming_original_goal");
    active_ = false;
    return BT::NodeStatus::SUCCESS;
  }
  void onHalted() override {
    publishState("halted_canceling_owned_actions");
    successful_spin_hold_ = false;
    stopSpin();
    const bool terminal = cancelOwned();
    stop();
    if (terminal) {
      actions_->disarmCleanup();
      setLimit(0.0);
    } else {
      actions_->watchCanceled();
    }
    active_ = false;
    approach_slowed_ = false;
    center_stop_stamp_ns_ = 0;
    center_previous_stamp_ns_ = 0;
    center_observed_vx_ = center_observed_vy_ = 0.0;
    rotation_previous_stamp_ns_ = 0;
    rotation_observed_rate_ = 0.0;
    alignment_recenters_ = 0;
    centering_started_ = {};
  }

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  rclcpp::CallbackGroup::SharedPtr group_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::shared_ptr<ActionContext> actions_;
  std::thread action_thread_;
  std::string failure_reason_;
  std::array<MapInput, 2> maps_;
  rclcpp::Client<GetParameters>::SharedPtr continuation_config_client_;
  rclcpp::Client<GetParameters>::SharedFuture continuation_config_pending_;
  bool continuous_{false}, continuation_needed_{false}, continuation_pending_{false},
      continuation_ready_{false}, continuation_config_checked_{false},
      continuation_config_valid_{false};
  bool continuation_replan_pending_{false}, continuation_replan_ready_{false};
  Clock::time_point continuation_replanned_{}, continuation_config_started_{};
  double continuation_lookahead_{0.0};
  nav_msgs::msg::Path continuation_path_, follower_path_;
  bool exit_spin_active_{false}, successful_spin_hold_{false},
       start_spin_settling_{false};
  double exit_spin_speed_{0.0};
  rclcpp::Client<GetParameters>::SharedPtr spin_client_;
  rclcpp::Client<GetParameters>::SharedFuture spin_pending_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<example_interfaces::msg::Float32>::SharedPtr spin_pub_;
  rclcpp::Publisher<nav2_msgs::msg::SpeedLimit>::SharedPtr limit_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_,
      deviation_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr command_sub_;
  geometry_msgs::msg::Twist observed_command_;
  Clock::time_point command_received_{};
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr localization_sub_;
  std::string localization_status_json_;
  bool localization_status_truncated_{false};
  Clock::time_point localization_received_{};
  int64_t localization_received_ros_ns_{0};
  rclcpp::Subscription<example_interfaces::msg::Float32>::SharedPtr spin_sub_;
  nav_msgs::msg::Odometry::ConstSharedPtr odom_;
  sensor_msgs::msg::LaserScan::ConstSharedPtr scan_;
  ScanStampGate scan_stamp_gate_;
  geometry_msgs::msg::PoseStamped goal_;
  geometry_msgs::msg::TransformStamped pose_;
  nav_msgs::msg::Path route_;
  std::vector<Tunnel> tunnels_;
  std::vector<geometry_msgs::msg::Point> footprint_;
  std::set<int> visited_;
  Crossing crossing_;
  bool passage_failure_latched_{false};
  Stage stage_{Stage::WAIT_DATA};
  Clock::time_point stage_started_{}, stable_since_{}, odom_received_{},
      scan_received_{}, clock_advanced_{}, last_limit_{}, centering_started_{};
  int64_t center_stop_stamp_ns_{0}, center_previous_stamp_ns_{0},
      rotation_previous_stamp_ns_{0};
  double rotation_previous_yaw_{0.0}, rotation_observed_rate_{0.0};
  double center_observed_vx_{0.0}, center_observed_vy_{0.0},
      center_previous_x_{0.0}, center_previous_y_{0.0};
  bool through_poses_{false}, approach_slowed_{false};
  std::vector<geometry_msgs::msg::PoseStamped> remaining_goals_;
  nav_msgs::msg::Path passage_trace_;
  unsigned int alignment_recenters_{0};
  rclcpp::Time clock_last_{0, 0, RCL_ROS_TIME};
  bool active_{false}, spin_checked_{false}, spin_observed_{false},
      spin_rejected_{false};
  double spin_speed_{0.0}, limit_{0.0}, entry_x_{0.0}, exit_x_{0.0},
      target_yaw_{0.0}, radius_{0.0}, longitudinal_extent_{0.0},
      rotation_x_{0.0}, rotation_y_{0.0};
};
} // namespace sentry_tunnel_bt
BT_REGISTER_NODES(factory) {
  factory.registerNodeType<sentry_tunnel_bt::TunnelTaskCompleted>("TunnelTaskCompleted");
  factory.registerNodeType<sentry_tunnel_bt::TunnelPassage>("TunnelPassage");
  factory.registerNodeType<sentry_tunnel_bt::TunnelFinalGoal>(
      "TunnelFinalGoal");
  factory.registerNodeType<sentry_tunnel_bt::TunnelRouteGuard>(
      "TunnelRouteGuard");
}
