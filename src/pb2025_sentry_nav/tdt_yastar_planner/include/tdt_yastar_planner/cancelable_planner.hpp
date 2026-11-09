// Copyright 2026 Diting navigation contributors
// SPDX-License-Identifier: Apache-2.0
#ifndef TDT_YASTAR_PLANNER__CANCELABLE_PLANNER_HPP_
#define TDT_YASTAR_PLANNER__CANCELABLE_PLANNER_HPP_

#include <functional>
#include <stdexcept>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

namespace tdt_yastar_planner
{
using CancelChecker = std::function<bool()>;

class PlannerCancelled : public std::runtime_error
{
public:
  PlannerCancelled() : std::runtime_error("Planning canceled") {}
};

// Optional extension: preserve the installed Humble GlobalPlanner ABI.
// The checker belongs to this invocation/goal; an empty checker means no external cancellation.
class CancelablePlanner
{
public:
  virtual ~CancelablePlanner() = default;
  virtual nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    const CancelChecker & cancel_checker) = 0;
};
}  // namespace tdt_yastar_planner
#endif  // TDT_YASTAR_PLANNER__CANCELABLE_PLANNER_HPP_
