#pragma once

#include <Eigen/Core>

#include <cstddef>
#include <functional>
#include <string>
#include <vector>

namespace tdt_yastar_planner
{
// Geometric use of TDT's closed polynomial backend. Segment times condition the
// solve; they are NOT a speed command or a timed-trajectory tracking contract.
struct TrajectoryOptimizationOptions
{
  bool enabled{true};
  double max_time_seconds{0.03};
  std::size_t max_control_points{12};
  std::size_t max_output_points{1200};
  double control_spacing{0.6};
  double corner_tolerance{0.05};
  double sample_spacing{0.025};
  double max_deviation{0.25};
  double max_length_ratio{1.1};
  double time_allocation_speed{1.0};
  double time_allocation_accel{2.0};
};

using GeometricPath = std::vector<Eigen::Vector2d>;
using MotionIsFree = std::function<bool(const Eigen::Vector2d &, const Eigen::Vector2d &)>;

struct TrajectoryOptimizationResult
{
  GeometricPath points;
  bool applied{false};
  std::string reason;
  double elapsed_seconds{0.0};
  std::size_t control_points{0};
};

void validateTrajectoryOptions(const TrajectoryOptimizationOptions & options);

// Input must already be the validated YAstar result, including exact endpoints.
// Any rejected candidate returns that input unchanged. The callback must check
// the complete swept body, including unknown cells and map bounds. Cancellation
// is distinguished from an ordinary optimization rejection; the caller must
// abort its planning request when reason == "cancelled".
// Deadlines are checked around each bounded operation. One Eigen decomposition
// cannot be preempted; max_control_points caps its size (maximum 24).
TrajectoryOptimizationResult optimizeTdtTrajectory(
  const GeometricPath & validated_path, const MotionIsFree & motion_is_free,
  const TrajectoryOptimizationOptions & options = {},
  const std::function<bool()> & cancelled = {});
}  // namespace tdt_yastar_planner
