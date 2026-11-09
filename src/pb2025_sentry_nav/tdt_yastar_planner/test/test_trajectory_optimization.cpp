#include <gtest/gtest.h>
#include <nav2_costmap_2d/cost_values.hpp>
#include <pb_omni_pid_pursuit_controller/footprint_collision.hpp>
#include <tdt_yastar_planner/tdt_trajectory_optimizer.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace
{
using tdt_yastar_planner::GeometricPath;
using tdt_yastar_planner::TrajectoryOptimizationOptions;
using tdt_yastar_planner::optimizeTdtTrajectory;

void expectSamePath(const GeometricPath & a, const GeometricPath & b)
{
  ASSERT_EQ(a.size(), b.size());
  for (std::size_t i = 0; i < a.size(); ++i) {
    EXPECT_DOUBLE_EQ(a[i].x(), b[i].x());
    EXPECT_DOUBLE_EQ(a[i].y(), b[i].y());
  }
}

double maxTurn(const GeometricPath & points)
{
  double turn = 0.0;
  for (std::size_t i = 1; i + 1 < points.size(); ++i) {
    const Eigen::Vector2d a = points[i] - points[i - 1], b = points[i + 1] - points[i];
    if (a.norm() < 1.0e-9 || b.norm() < 1.0e-9) continue;
    turn = std::max(turn, std::acos(std::clamp(a.normalized().dot(b.normalized()), -1.0, 1.0)));
  }
  return turn;
}

TrajectoryOptimizationOptions testOptions()
{
  TrajectoryOptimizationOptions options;
  // Functional tests do not make machine-speed-dependent assertions. Production
  // uses 30 ms; the separate budget test exercises rejection deterministically.
  options.max_time_seconds = 1.0;
  return options;
}

TEST(TrajectoryOptimization, ChangesAnOpenCornerAndPreservesExactDoubleEndpoints)
{
  const GeometricPath path{{0.123456789123, 0.123456789123},
    {2.123456789123, 0.123456789123}, {2.123456789123, 2.123456789123}};
  const auto result = optimizeTdtTrajectory(path, [](const auto &, const auto &) {return true;},
    testOptions());
  ASSERT_TRUE(result.applied) << result.reason;
  EXPECT_EQ(result.reason, "applied");
  EXPECT_GT(result.points.size(), path.size());
  EXPECT_DOUBLE_EQ(result.points.front().x(), path.front().x());
  EXPECT_DOUBLE_EQ(result.points.front().y(), path.front().y());
  EXPECT_DOUBLE_EQ(result.points.back().x(), path.back().x());
  EXPECT_DOUBLE_EQ(result.points.back().y(), path.back().y());
  EXPECT_LT(maxTurn(result.points), maxTurn(path) * 0.5);
  EXPECT_LE(result.control_points, testOptions().max_control_points);
  EXPECT_LE(result.points.size(), testOptions().max_output_points);
}

TEST(TrajectoryOptimization, RejectedBodySweepReturnsTheOriginalPathUnchanged)
{
  const GeometricPath path{{0.0, 0.0}, {2.0, 0.0}, {2.0, 2.0}};
  int validations = 0;
  const auto result = optimizeTdtTrajectory(path, [&](const auto &, const auto &) {
      ++validations;
      return false;
    }, testOptions());
  EXPECT_FALSE(result.applied);
  EXPECT_EQ(result.reason, "body_collision");
  EXPECT_GT(validations, 0);
  expectSamePath(result.points, path);
}

TEST(TrajectoryOptimization, CannotCutAnOccupiedOrUnknownCellUnderTheBody)
{
  const GeometricPath path{{0.5, 0.5}, {2.5, 0.5}, {2.5, 2.5}};
  const auto free_curve = optimizeTdtTrajectory(path,
    [](const auto &, const auto &) {return true;}, testOptions());
  ASSERT_TRUE(free_curve.applied) << free_curve.reason;
  const auto clearance = [](const Eigen::Vector2d & p) {
    return std::min(std::abs(p.y() - 0.5), std::abs(p.x() - 2.5));
  };
  const auto witness = std::max_element(free_curve.points.begin(), free_curve.points.end(),
    [&](const auto & a, const auto & b) {return clearance(a) < clearance(b);});
  ASSERT_GT(clearance(*witness), 0.025);
  nav2_costmap_2d::Costmap2D map(400, 400, 0.01, -0.5, -0.5, 0);
  std::vector<geometry_msgs::msg::Point> footprint;
  for (const auto & xy : std::vector<std::pair<double, double>>{
      {-0.005, -0.005}, {0.005, -0.005}, {0.005, 0.005}, {-0.005, 0.005}})
  {
    geometry_msgs::msg::Point p;
    p.x = xy.first; p.y = xy.second;
    footprint.push_back(p);
  }
  unsigned int mx, my;
  ASSERT_TRUE(map.worldToMap(witness->x(), witness->y(), mx, my));
  for (auto value : {nav2_costmap_2d::LETHAL_OBSTACLE, nav2_costmap_2d::NO_INFORMATION}) {
    map.setCost(mx, my, value);
    const auto safe = [&](const Eigen::Vector2d & a, const Eigen::Vector2d & b) {
      return !pb_omni_pid_pursuit_controller::sweptFootprintCollision(
        map, footprint, a.x(), a.y(), b.x(), b.y(), 0.0);
    };
    for (std::size_t i = 1; i < path.size(); ++i) ASSERT_TRUE(safe(path[i - 1], path[i]));
    const auto result = optimizeTdtTrajectory(path, safe, testOptions());
    EXPECT_FALSE(result.applied);
    EXPECT_EQ(result.reason, "body_collision");
    expectSamePath(result.points, path);
  }
}

TEST(TrajectoryOptimization, DisabledAndPointBudgetsRetainTheValidatedFallback)
{
  const GeometricPath path{{0.0, 0.0}, {2.0, 0.0}, {2.0, 2.0}};
  auto options = testOptions();
  options.enabled = false;
  const auto disabled = optimizeTdtTrajectory(path,
    [](const auto &, const auto &) {ADD_FAILURE(); return false;}, options);
  EXPECT_EQ(disabled.reason, "disabled");
  expectSamePath(disabled.points, path);
  options.enabled = true;
  options.max_control_points = 3;
  const auto bounded = optimizeTdtTrajectory(path,
    [](const auto &, const auto &) {return true;}, options);
  EXPECT_EQ(bounded.reason, "control_point_budget");
  expectSamePath(bounded.points, path);
  options = testOptions();
  options.max_output_points = 12;
  const auto output = optimizeTdtTrajectory(path,
    [](const auto &, const auto &) {return true;}, options);
  EXPECT_EQ(output.reason, "output_point_budget");
  expectSamePath(output.points, path);
}

TEST(TrajectoryOptimization, ExcessCornersAreSkippedInsteadOfDeleted)
{
  const GeometricPath path{{0.0, 0.0}, {1.0, 0.0}, {1.0, 1.0},
    {2.0, 1.0}, {2.0, 2.0}, {3.0, 2.0}};
  auto options = testOptions();
  options.max_control_points = 3;
  options.control_spacing = 10.0;
  const auto result = optimizeTdtTrajectory(path,
    [](const auto &, const auto &) {return true;}, options);
  EXPECT_EQ(result.reason, "corner_point_budget");
  expectSamePath(result.points, path);
}

TEST(TrajectoryOptimization, CancellationAndDeadlineCannotPublishALateCurve)
{
  const GeometricPath path{{0.0, 0.0}, {2.0, 0.0}, {2.0, 2.0}};
  auto options = testOptions();
  const auto cancelled = optimizeTdtTrajectory(path,
    [](const auto &, const auto &) {return true;}, options, [] {return true;});
  EXPECT_EQ(cancelled.reason, "cancelled");
  expectSamePath(cancelled.points, path);
  options.max_time_seconds = 0.001;
  const auto expired = optimizeTdtTrajectory(path,
    [](const auto &, const auto &) {return true;}, options, [] {
      const auto end = std::chrono::steady_clock::now() + std::chrono::milliseconds(2);
      while (std::chrono::steady_clock::now() < end) {}
      return false;
    });
  EXPECT_EQ(expired.reason, "time_budget");
  expectSamePath(expired.points, path);
}

TEST(TrajectoryOptimization, DeviationLimitCannotSilentlyChangeTheRoute)
{
  const GeometricPath path{{0.0, 0.0}, {2.0, 0.0}, {2.0, 2.0}};
  auto options = testOptions();
  options.max_deviation = 0.000001;
  const auto result = optimizeTdtTrajectory(path,
    [](const auto &, const auto &) {return true;}, options);
  EXPECT_EQ(result.reason, "deviation_limit");
  expectSamePath(result.points, path);
}

TEST(TrajectoryOptimization, RejectsInvalidOptionsAndNonfiniteInput)
{
  const GeometricPath path{{0.0, 0.0}, {2.0, 0.0}, {2.0, 2.0}};
  auto options = testOptions();
  options.max_control_points = 100;
  EXPECT_THROW(optimizeTdtTrajectory(path,
    [](const auto &, const auto &) {return true;}, options), std::invalid_argument);
  options = testOptions();
  const GeometricPath nonfinite{{0.0, 0.0}, {std::numeric_limits<double>::quiet_NaN(), 1.0}};
  EXPECT_THROW(optimizeTdtTrajectory(nonfinite,
    [](const auto &, const auto &) {return true;}, options), std::invalid_argument);
}
}  // namespace
