// Copyright 2026 diting
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <cmath>
#include <limits>
#include <vector>

#include "gtest/gtest.h"
#include "nav2_core/exceptions.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/costmap_filters/filter_values.hpp"
#include "pb_omni_pid_pursuit_controller/footprint_collision.hpp"
#include "pb_omni_pid_pursuit_controller/omni_pid_pursuit_controller.hpp"

namespace pb_omni_pid_pursuit_controller
{
namespace
{
class SpeedLimitController : public OmniPidPursuitController
{
public:
  using OmniPidPursuitController::applySpeedLimit;
  using OmniPidPursuitController::applyVelocityConstraints;
  using OmniPidPursuitController::dynamicParametersCallback;
};

geometry_msgs::msg::Twist diagonalCommand()
{
  geometry_msgs::msg::Twist command;
  command.linear.x = -1.8;
  command.linear.y = 2.4;
  command.angular.z = 0.25;
  return command;
}

TEST(ControllerSpeedLimit, AbsoluteLimitCapsVectorAndPreservesHeadingAndAngularSpeed)
{
  SpeedLimitController controller;
  controller.setSpeedLimit(0.4, false);
  auto command = diagonalCommand();
  controller.applySpeedLimit(command);
  EXPECT_NEAR(std::hypot(command.linear.x, command.linear.y), 0.4, 1e-12);
  EXPECT_NEAR(command.linear.x, -0.24, 1e-12);
  EXPECT_NEAR(command.linear.y, 0.32, 1e-12);
  EXPECT_DOUBLE_EQ(command.angular.z, 0.25);
}

TEST(ControllerSpeedLimit, PercentageIsBasedOnBaseLimitAndDoesNotCompound)
{
  SpeedLimitController controller;
  controller.setSpeedLimit(50.0, true);
  controller.setSpeedLimit(50.0, true);
  auto command = diagonalCommand();
  controller.applySpeedLimit(command);
  EXPECT_NEAR(std::hypot(command.linear.x, command.linear.y), 1.5, 1e-12);
  controller.setSpeedLimit(25.0, true);
  command = diagonalCommand();
  controller.applySpeedLimit(command);
  EXPECT_NEAR(std::hypot(command.linear.x, command.linear.y), 0.75, 1e-12);
}

TEST(ControllerSpeedLimit, ZeroRestoresBaseLimitForBothMessageTypes)
{
  SpeedLimitController controller;
  for (const bool percentage : {false, true}) {
    controller.setSpeedLimit(0.4, false);
    controller.setSpeedLimit(nav2_costmap_2d::NO_SPEED_LIMIT, percentage);
    auto command = diagonalCommand();
    controller.applySpeedLimit(command);
    EXPECT_NEAR(std::hypot(command.linear.x, command.linear.y), 3.0, 1e-12);
  }
}

TEST(ControllerSpeedLimit, InvalidMessagesKeepLastValidLimit)
{
  SpeedLimitController controller;
  controller.setSpeedLimit(0.4, false);
  for (double invalid :
       {-1.0, std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity()}) {
    controller.setSpeedLimit(invalid, false);
    auto command = diagonalCommand();
    controller.applySpeedLimit(command);
    EXPECT_NEAR(std::hypot(command.linear.x, command.linear.y), 0.4, 1e-12);
  }
  controller.setSpeedLimit(101.0, true);
  auto command = diagonalCommand();
  controller.applySpeedLimit(command);
  EXPECT_NEAR(std::hypot(command.linear.x, command.linear.y), 0.4, 1e-12);
}

TEST(ControllerSpeedLimit, LimitsNeverAccelerateOrExceedConfiguredMaximum)
{
  SpeedLimitController controller;
  controller.setSpeedLimit(100.0, false);
  auto command = diagonalCommand();
  controller.applySpeedLimit(command);
  EXPECT_NEAR(std::hypot(command.linear.x, command.linear.y), 3.0, 1e-12);
  command.linear.x = 0.05;
  command.linear.y = -0.1;
  controller.setSpeedLimit(0.4, false);
  controller.applySpeedLimit(command);
  EXPECT_DOUBLE_EQ(command.linear.x, 0.05);
  EXPECT_DOUBLE_EQ(command.linear.y, -0.1);
  command.linear.x = 0.0;
  command.linear.y = 0.0;
  controller.applySpeedLimit(command);
  EXPECT_DOUBLE_EQ(command.linear.x, 0.0);
  EXPECT_DOUBLE_EQ(command.linear.y, 0.0);
}

TEST(ControllerSpeedLimit, DynamicLowerBoundsTakeEffectWithoutReconfiguringPid)
{
  SpeedLimitController controller;
  // The test instance has an empty plugin name; only this protected callback is exercised.
  ASSERT_TRUE(
    controller
      .dynamicParametersCallback(
        {rclcpp::Parameter(".v_linear_min", -0.4), rclcpp::Parameter(".v_linear_max", 0.4)})
      .successful);
  auto command = diagonalCommand();
  controller.applySpeedLimit(command);
  EXPECT_NEAR(std::hypot(command.linear.x, command.linear.y), 0.4, 1e-12);
  controller.setSpeedLimit(50.0, true);
  command = diagonalCommand();
  controller.applySpeedLimit(command);
  EXPECT_NEAR(std::hypot(command.linear.x, command.linear.y), 0.2, 1e-12);
  controller.setSpeedLimit(nav2_costmap_2d::NO_SPEED_LIMIT, false);
  command = diagonalCommand();
  controller.applySpeedLimit(command);
  EXPECT_NEAR(std::hypot(command.linear.x, command.linear.y), 0.4, 1e-12);
}

TEST(ControllerSpeedLimit, InvalidBoundsAreRejectedWithoutPartialUpdate)
{
  SpeedLimitController controller;
  EXPECT_FALSE(
    controller
      .dynamicParametersCallback(
        {rclcpp::Parameter(".v_linear_min", 0.5), rclcpp::Parameter(".v_linear_max", 0.4)})
      .successful);
  auto command = diagonalCommand();
  controller.applySpeedLimit(command);
  EXPECT_NEAR(std::hypot(command.linear.x, command.linear.y), 3.0, 1e-12);
  EXPECT_FALSE(controller
                 .dynamicParametersCallback(
                   {rclcpp::Parameter(".v_linear_max", std::numeric_limits<double>::infinity())})
                 .successful);
  EXPECT_FALSE(
    controller.dynamicParametersCallback({rclcpp::Parameter(".v_linear_max", 1)}).successful);
}

TEST(ControllerSpeedLimit, NonFiniteCommandIsRejectedBeforeCollisionPrediction)
{
  SpeedLimitController controller;
  auto command = diagonalCommand();
  command.linear.x = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(controller.applySpeedLimit(command), nav2_core::PlannerException);
}

TEST(ControllerSpeedLimit, NonFiniteScalarIsRejectedBeforeAnyConstraint)
{
  SpeedLimitController controller;
  const nav_msgs::msg::Path empty_path;
  const geometry_msgs::msg::PoseStamped carrot;
  for (double invalid :
       {std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity(),
        -std::numeric_limits<double>::infinity()}) {
    double command = invalid;
    EXPECT_THROW(
      controller.applyVelocityConstraints(empty_path, carrot, command),
      nav2_core::PlannerException);
  }
}

nav_msgs::msg::Path constraintPath(bool curved, double length)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = "base_link";
  for (int i = 0; i <= 60; ++i) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = length * i / 60.0;
    pose.pose.position.y = curved ? 0.4 * pose.pose.position.x * pose.pose.position.x : 0.0;
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
  }
  return path;
}

geometry_msgs::msg::PoseStamped constraintCarrot(bool curved)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "base_link";
  pose.pose.position.x = 0.3;
  pose.pose.position.y = curved ? 0.4 * 0.3 * 0.3 : 0.0;
  pose.pose.orientation.w = 1.0;
  return pose;
}

class ControllerConstraintTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }

  void SetUp() override
  {
    costmap_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>("constraint_costmap");
    costmap_->set_parameters(
      {rclcpp::Parameter("plugins", std::vector<std::string>{}),
       rclcpp::Parameter("filters", std::vector<std::string>{}), rclcpp::Parameter("width", 10),
       rclcpp::Parameter("height", 10)});
    costmap_->configure();
    ASSERT_EQ(costmap_->get_current_state().label(), "inactive");
  }

  void TearDown() override { costmap_->cleanup(); }

  struct ConfiguredController
  {
    rclcpp_lifecycle::LifecycleNode::SharedPtr node;
    std::shared_ptr<tf2_ros::Buffer> tf;
    SpeedLimitController controller;

    ConfiguredController(
      const std::string & name, double maximum,
      const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> & costmap)
    {
      node = std::make_shared<rclcpp_lifecycle::LifecycleNode>(name);
      node->declare_parameter("FollowPath.v_linear_min", -maximum);
      node->declare_parameter("FollowPath.v_linear_max", maximum);
      node->declare_parameter("FollowPath.min_approach_linear_velocity", 0.15);
      node->declare_parameter("FollowPath.approach_velocity_scaling_dist", 2.0);
      node->declare_parameter("FollowPath.max_velocity_scaling_factor_rate", 0.9);
      tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
      controller.configure(node, "FollowPath", tf, costmap);
      controller.activate();
    }

    ~ConfiguredController()
    {
      controller.deactivate();
      controller.cleanup();
    }
  };

  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_;
};

TEST_F(ControllerConstraintTest, RegionalProfileMatchesHardLimitForStraightCurvedNearAndFarPaths)
{
  const auto warmup_path = constraintPath(false, 3.0);
  const auto warmup_carrot = constraintCarrot(false);
  for (const bool curved : {false, true}) {
    for (const double length : {1.0, 3.0}) {
      SCOPED_TRACE(::testing::Message() << "curved=" << curved << " length=" << length);
      ConfiguredController old("hard_limit_controller", 0.4, costmap_);
      ConfiguredController regional("regional_limit_controller", 1.0, costmap_);
      PID old_pid(0.05, 0.4, -0.4, 3.0, 0.3, 0.1);
      PID regional_pid(0.05, 1.0, -1.0, 3.0, 0.3, 0.1);
      // Seed the real curvature state with faster normal-road motion before
      // entering the limited area, rather than testing only a cap in isolation.
      double a = 0.0, b = 0.0;
      for (int i = 0; i < 40; ++i) {
        a = old_pid.calculate(0.3, 0.0);
        b = regional_pid.calculate(0.3, 0.0);
        old.controller.applyVelocityConstraints(warmup_path, warmup_carrot, a);
        regional.controller.applyVelocityConstraints(warmup_path, warmup_carrot, b);
        ASSERT_LE(a, 0.4);
        ASSERT_LE(b, 1.0);
      }
      ASSERT_GT(b, a);
      regional.controller.setSpeedLimit(0.4, false);
      const auto path = constraintPath(curved, length);
      const auto carrot = constraintCarrot(curved);
      for (const double error : {0.3, 0.25, 0.35, 0.2, 0.3}) {
        double a = old_pid.calculate(error, 0.0);
        double b = regional_pid.calculate(error, 0.0);
        old.controller.applyVelocityConstraints(path, carrot, a);
        regional.controller.applyVelocityConstraints(path, carrot, b);
        EXPECT_NEAR(a, b, 1e-12);
        geometry_msgs::msg::Twist final_command;
        final_command.linear.x = b * 0.6;
        final_command.linear.y = b * 0.8;
        regional.controller.applySpeedLimit(final_command);
        EXPECT_NEAR(std::hypot(final_command.linear.x, final_command.linear.y), std::abs(a), 1e-12);
      }
    }
  }
}

TEST_F(ControllerConstraintTest, ClearingRegionRampsToNormalSpeedWithoutACommandJump)
{
  ConfiguredController configured("restored_limit_controller", 1.0, costmap_);
  const auto path = constraintPath(false, 3.0);
  const auto carrot = constraintCarrot(false);
  configured.controller.setSpeedLimit(0.4, false);
  double limited = 0.0;
  for (int tick = 0; tick < 20; ++tick) {
    limited = 0.9;
    configured.controller.applyVelocityConstraints(path, carrot, limited);
  }
  ASSERT_NEAR(limited, 0.4, 1e-12);
  configured.controller.setSpeedLimit(nav2_costmap_2d::NO_SPEED_LIMIT, false);
  double restored = 0.9;
  configured.controller.applyVelocityConstraints(path, carrot, restored);
  EXPECT_NEAR(restored, 0.445, 1e-12);
  EXPECT_GT(restored, limited);
  for (int tick = 0; tick < 20; ++tick) {
    const double previous = restored;
    restored = 0.9;
    configured.controller.applyVelocityConstraints(path, carrot, restored);
    EXPECT_LE(restored - previous, 0.045 + 1e-12);
    EXPECT_LE(restored, 0.9);
  }
  EXPECT_NEAR(restored, 0.9, 1e-12);
}

TEST_F(ControllerConstraintTest, BendToStraightReplanningKeepsBothSpeedChangesBounded)
{
  ConfiguredController configured("continuous_speed_controller", 1.0, costmap_);
  double previous = 0.0;
  for (int tick = 0; tick < 100; ++tick) {
    const bool curved = (tick / 15) % 2 == 1;
    double speed = 1.0;
    configured.controller.applyVelocityConstraints(
      constraintPath(curved, 3.0), constraintCarrot(curved), speed);
    EXPECT_LE(std::abs(speed - previous), 0.045 + 1e-12);
    previous = speed;
  }
  configured.controller.setSpeedLimit(0.2, false);
  double speed = 1.0;
  configured.controller.applyVelocityConstraints(
    constraintPath(false, 3.0), constraintCarrot(false), speed);
  EXPECT_LE(speed, 0.2);
}

std::vector<geometry_msgs::msg::Point> body()
{
  std::vector<geometry_msgs::msg::Point> result;
  // The same outward-rounded collision AABB and 1 cm padding as simulation.
  for (auto xy :
       {std::pair{-0.298, -0.277}, std::pair{0.298, -0.277}, std::pair{0.298, 0.277},
        std::pair{-0.298, 0.277}}) {
    geometry_msgs::msg::Point p;
    p.x = xy.first;
    p.y = xy.second;
    result.push_back(p);
  }
  return result;
}

TEST(FootprintCollision, AlignedBodyPassesEightHundredMillimeterCorridor)
{
  nav2_costmap_2d::Costmap2D map(120, 120, 0.01, -0.6, -0.6, 0);
  for (unsigned int x = 0; x < 120; ++x) {
    map.setCost(x, 19, 254);   // Upper edge y=-0.4.
    map.setCost(x, 100, 254);  // Lower edge y=+0.4.
  }
  EXPECT_FALSE(sweptFootprintCollision(map, body(), -0.2, 0, 0.2, 0, 0));
  EXPECT_TRUE(sweptFootprintCollision(map, body(), 0, 0.14, 0, 0.14, 0));
}

TEST(FootprintCollision, OrientationIsPhysicalBodyYaw)
{
  nav2_costmap_2d::Costmap2D map(200, 200, 0.01, -1, -1, 0);
  map.setCost(128, 100, 254);  // x=0.28 intersects the long side only.
  EXPECT_TRUE(sweptFootprintCollision(map, body(), 0, 0, 0, 0, 0));
  EXPECT_FALSE(sweptFootprintCollision(map, body(), 0, 0, 0, 0, M_PI / 2));
}

TEST(FootprintCollision, ObstacleInsideBodyIsNotMissedByAnEdgeOnlyChecker)
{
  nav2_costmap_2d::Costmap2D map(200, 200, 0.01, -1, -1, 0);
  map.setCost(100, 100, 254);
  EXPECT_TRUE(sweptFootprintCollision(map, body(), 0, 0, 0, 0, 0));
}

TEST(FootprintCollision, CornerOverlapAndCellAreaAreChecked)
{
  nav2_costmap_2d::Costmap2D map(200, 200, 0.01, -0.996, -0.996, 0);
  map.setCost(129, 127, 254);  // Corner cell: center is outside the body.
  EXPECT_TRUE(sweptFootprintCollision(map, body(), 0, 0, 0, 0, 0));
  map.setCost(129, 127, 0);
  map.setCost(130, 128, 254);
  EXPECT_FALSE(sweptFootprintCollision(map, body(), 0, 0, 0, 0, 0));
}

TEST(FootprintCollision, SweptSegmentCannotSkipAnObstacleBetweenPathSamples)
{
  nav2_costmap_2d::Costmap2D map(400, 200, 0.01, -2, -1, 0);
  map.setCost(200, 100, 254);
  EXPECT_FALSE(sweptFootprintCollision(map, body(), -1, 0, -1, 0, 0));
  EXPECT_FALSE(sweptFootprintCollision(map, body(), 1, 0, 1, 0, 0));
  EXPECT_TRUE(sweptFootprintCollision(map, body(), -1, 0, 1, 0, 0));
}

TEST(FootprintCollision, InflationIsNotAppliedTwiceButUnknownAndBoundsBlock)
{
  nav2_costmap_2d::Costmap2D map(200, 200, 0.01, -1, -1, 253);
  EXPECT_FALSE(sweptFootprintCollision(map, body(), 0, 0, 0, 0, 0));
  map.setCost(100, 100, 255);
  EXPECT_TRUE(sweptFootprintCollision(map, body(), 0, 0, 0, 0, 0));
  EXPECT_TRUE(sweptFootprintCollision(map, body(), 0.9, 0, 0.9, 0, 0));
}

TEST(FootprintCollision, InvalidGeometryFailsClosed)
{
  nav2_costmap_2d::Costmap2D map(200, 200, 0.01, -1, -1, 0);
  EXPECT_TRUE(sweptFootprintCollision(map, {}, 0, 0, 0, 0, 0));
  EXPECT_TRUE(
    sweptFootprintCollision(map, body(), 0, 0, 0, 0, std::numeric_limits<double>::quiet_NaN()));
}
}  // namespace
}  // namespace pb_omni_pid_pursuit_controller
