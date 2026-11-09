// Copyright 2018 Open Source Robotics Foundation, Inc.
// Copyright 2026 Diting navigation contributors
// SPDX-License-Identifier: Apache-2.0
// Waitable registration/deletion follows rclcpp_action/create_server.hpp (Humble).
#ifndef SENTRY_PLANNER_SERVER__GOAL_AWARE_ACTION_SERVER_HPP_
#define SENTRY_PLANNER_SERVER__GOAL_AWARE_ACTION_SERVER_HPP_

#include <functional>
#include <memory>
#include <mutex>
#include <utility>
#include "nav2_util/simple_action_server.hpp"

namespace sentry_planner_server
{
// Lock the entire rclcpp cancel decision AND _cancel_goal() transition. Merely
// locking the user's cancel callback leaves a gap before rclcpp changes state;
// the ROS state machine permits CANCELING -> SUCCEEDED, so check-then-succeed
// alone does not close that race. This override does not alter system headers.
template<typename ActionT>
class SerializedCancelServer : public rclcpp_action::Server<ActionT>
{
public:
  using Base = rclcpp_action::Server<ActionT>;
  template<typename ... Args>
  SerializedCancelServer(std::recursive_mutex & transition_mutex, Args && ... args)
  : Base(std::forward<Args>(args)...), transition_mutex_(transition_mutex) {}

protected:
  rclcpp_action::CancelResponse call_handle_cancel_callback(
    const rclcpp_action::GoalUUID & uuid) override
  {
    std::lock_guard<std::recursive_mutex> lock(transition_mutex_);
    return Base::call_handle_cancel_callback(uuid);
  }

private:
  std::recursive_mutex & transition_mutex_;
};

// The executing goal is distinct from pending goals; never use the Humble
// SimpleActionServer::is_cancel_requested() for goal-scoped search cancellation.
template<typename ActionT>
class GoalAwareSimpleActionServer : public nav2_util::SimpleActionServer<ActionT>
{
public:
  using Base = nav2_util::SimpleActionServer<ActionT>;
  using GoalHandle = rclcpp_action::ServerGoalHandle<ActionT>;
  using GoalHandlePtr = std::shared_ptr<GoalHandle>;
  using ResultPtr = std::shared_ptr<typename ActionT::Result>;

  template<typename NodeT>
  GoalAwareSimpleActionServer(
    NodeT node, const std::string & action_name,
    typename Base::ExecuteCallback execute_callback,
    typename Base::CompletionCallback completion_callback = nullptr,
    std::chrono::milliseconds server_timeout = std::chrono::milliseconds(500),
    bool spin_thread = false,
    const rcl_action_server_options_t & options = rcl_action_server_get_default_options(),
    std::function<void()> before_cancel_state_transition = {})
  : Base(node, action_name, execute_callback, completion_callback, server_timeout, false, options)
  {
    // Base binds nonvirtual callbacks during construction. No executor has been
    // started, so replace that inactive waitable before accepting any requests.
    this->action_server_.reset();
    this->spin_thread_ = spin_thread;
    if (spin_thread) {
      this->callback_group_ = this->node_base_interface_->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive, false);
    }
    auto group = this->callback_group_;
    auto weak_group = std::weak_ptr<rclcpp::CallbackGroup>(group);
    auto weak_node = std::weak_ptr<rclcpp::node_interfaces::NodeWaitablesInterface>(
      this->node_waitables_interface_);
    const bool default_group = !group;
    auto deleter = [weak_node, weak_group, default_group](rclcpp_action::Server<ActionT> * ptr) {
        if (auto waitables = weak_node.lock()) {
          std::shared_ptr<rclcpp_action::Server<ActionT>> borrowed(ptr, [](auto *) {});
          if (default_group) {
            waitables->remove_waitable(borrowed, nullptr);
          } else if (auto callback_group = weak_group.lock()) {
            waitables->remove_waitable(borrowed, callback_group);
          }
        }
        delete ptr;
      };
    typename rclcpp_action::Server<ActionT>::GoalCallback goal_callback =
      [this](const auto & uuid, auto goal) {return this->handle_goal(uuid, goal);};
    typename rclcpp_action::Server<ActionT>::CancelCallback cancel_callback =
      [this, before_cancel_state_transition](auto handle) {
        const auto response = this->handle_cancel(handle);
        // Optional deterministic test barrier; production callers leave it empty.
        if (response == rclcpp_action::CancelResponse::ACCEPT && before_cancel_state_transition) {
          before_cancel_state_transition();
        }
        return response;
      };
    typename rclcpp_action::Server<ActionT>::AcceptedCallback accepted_callback =
      [this](auto handle) {this->handle_accepted(handle);};
    this->action_server_ = typename rclcpp_action::Server<ActionT>::SharedPtr(
      new SerializedCancelServer<ActionT>(this->update_mutex_, this->node_base_interface_,
        this->node_clock_interface_, this->node_logging_interface_, action_name, options,
        goal_callback, cancel_callback, accepted_callback), deleter);
    this->node_waitables_interface_->add_waitable(this->action_server_, group);
    if (spin_thread) {
      this->executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
      this->executor_->add_callback_group(group, this->node_base_interface_);
      this->executor_thread_ = std::make_unique<nav2_util::NodeThread>(this->executor_);
    }
  }

  GoalHandlePtr currentGoalForExecution()
  {
    std::lock_guard<std::recursive_mutex> lock(this->update_mutex_);
    if (this->pending_handle_ && this->pending_handle_->is_canceling()) {
      this->terminate(this->pending_handle_);
      this->preempt_requested_ = false;
    } else if (this->preempt_requested_ && this->is_active(this->pending_handle_)) {
      this->terminate(this->current_handle_);
      this->current_handle_ = this->pending_handle_;
      this->pending_handle_.reset();
      this->preempt_requested_ = false;
    }
    return this->is_active(this->current_handle_) ? this->current_handle_ : nullptr;
  }

  bool terminateGoal(const GoalHandlePtr & handle, const ResultPtr & result)
  {
    std::lock_guard<std::recursive_mutex> lock(this->update_mutex_);
    if (!handle || this->current_handle_ != handle) {return false;}
    this->terminate(this->current_handle_, result);
    return true;
  }

  bool succeedGoal(const GoalHandlePtr & handle, const ResultPtr & result)
  {
    std::lock_guard<std::recursive_mutex> lock(this->update_mutex_);
    if (!handle || this->current_handle_ != handle || !handle->is_active()) {return false;}
    if (handle->is_canceling() || !this->server_active_) {
      this->terminate(this->current_handle_, result);
      return false;
    }
    // SerializedCancelServer uses this same mutex through the real state change.
    handle->succeed(result);
    this->current_handle_.reset();
    return true;
  }
};
}  // namespace sentry_planner_server
#endif  // SENTRY_PLANNER_SERVER__GOAL_AWARE_ACTION_SERVER_HPP_
