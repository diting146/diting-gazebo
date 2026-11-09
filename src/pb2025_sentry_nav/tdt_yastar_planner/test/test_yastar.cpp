#include <gtest/gtest.h>
#include <nav2_costmap_2d/cost_values.hpp>
#include <tdt_yastar_planner/yastar_planner.hpp>
#include <pb_omni_pid_pursuit_controller/footprint_collision.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <future>
#include <thread>
#include <cmath>
#include <limits>
#include <rcl/time.h>

namespace
{
class CoreTest : public ::testing::Test
{
protected:
  YAstar planner;
  void map(int width, int height, const std::vector<int8_t> & cells)
  {
    auto occupancy = cells;
    planner.setMap(width, height, 1.0F, 0.0F, 0.0F, occupancy);
    std::vector<float> costs;
    for (auto cell : cells) {
      costs.push_back(cell ? std::numeric_limits<float>::infinity() : 1.0F);
    }
    planner.setCostMapSnapshot(costs);
    planner.setCostWeight(1.0F);
  }
};

TEST_F(CoreTest, BoundsAndNonFiniteCoordinates)
{
  map(3, 3, std::vector<int8_t>(9, 0));
  for (const auto & point : std::vector<Eigen::Vector2f>{
      {-0.01F, 0.5F}, {3.0F, 0.5F}, {0.5F, 3.0F},
      {std::numeric_limits<float>::max(), 0.5F},
      {std::numeric_limits<float>::quiet_NaN(), 0.5F}})
  {
    EXPECT_TRUE(planner.search(point, {1.5F, 1.5F}).empty());
    EXPECT_TRUE(planner.search({1.5F, 1.5F}, point).empty());
    EXPECT_TRUE(planner.pointInObsticle(point.x(), point.y()));
  }
}

TEST_F(CoreTest, NoPathAndBlockedEndpoints)
{
  map(3, 3, {0, 100, 0, 0, 100, 0, 0, 100, 0});
  EXPECT_TRUE(planner.search({0.5F, 0.5F}, {2.5F, 0.5F}).empty());
  EXPECT_TRUE(planner.search({1.5F, 0.5F}, {2.5F, 0.5F}).empty());
  EXPECT_TRUE(planner.search({0.5F, 0.5F}, {1.5F, 0.5F}).empty());
}

TEST_F(CoreTest, CannotCutBlockedDiagonal)
{
  map(2, 2, {0, 100, 100, 0});
  EXPECT_TRUE(planner.search({0.5F, 0.5F}, {1.5F, 1.5F}).empty());
  EXPECT_TRUE(planner.lineInObsticle({0.5F, 0.5F}, {1.5F, 1.5F}));
}

TEST_F(CoreTest, PathRunsFromStartToGoalAtCellCenters)
{
  map(4, 1, {0, 0, 0, 0});
  const auto path = planner.search({0.2F, 0.2F}, {3.8F, 0.8F});
  ASSERT_EQ(path.size(), 4U);
  for (size_t i = 0; i < path.size(); ++i) {
    EXPECT_FLOAT_EQ(path[i].x(), static_cast<float>(i) + 0.5F);
    EXPECT_FLOAT_EQ(path[i].y(), 0.5F);
  }
}

TEST_F(CoreTest, ExpansionBudgetAndCancellation)
{
  map(4, 1, {0, 0, 0, 0});
  planner.setSearchLimits(1, 0.0);
  EXPECT_THROW(planner.search({0.5F, 0.5F}, {3.5F, 0.5F}), std::runtime_error);
  planner.setSearchLimits(0, 0.0, [] {return true;});
  EXPECT_THROW(planner.search({0.5F, 0.5F}, {3.5F, 0.5F}), std::runtime_error);
  EXPECT_THROW(planner.setSearchLimits(-1, 1.0), std::invalid_argument);
  EXPECT_THROW(planner.setSearchLimits(1, -1.0), std::invalid_argument);
}

TEST_F(CoreTest, SteadyClockBudget)
{
  map(4, 1, {0, 0, 0, 0});
  // Time is the behavior under test. Consume the budget in the synchronous
  // callback, so expiration does not depend on machine speed or a sleep.
  planner.setSearchLimits(0, 0.001, [] {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(2);
    while (std::chrono::steady_clock::now() < deadline) {}
    return false;
  });
  EXPECT_THROW(planner.search({0.5F, 0.5F}, {3.5F, 0.5F}), std::runtime_error);
}

class TestCostmap : public nav2_costmap_2d::Costmap2DROS
{
public:
  TestCostmap() : Costmap2DROS("yastar_test_costmap")
  {
    global_frame_ = "map";
    layered_costmap_ = std::make_unique<nav2_costmap_2d::LayeredCostmap>("map", false, false);
    getCostmap()->resizeMap(4, 3, 1.0, 0.0, 0.0);
    std::fill_n(getCostmap()->getCharMap(), 12, nav2_costmap_2d::FREE_SPACE);
    rectangle(0.1, 0.1);
  }
  void rectangle(double hx, double hy)
  {
    padded_footprint_.clear();
    for (const auto & xy : std::vector<std::pair<double, double>>{
        {-hx, -hy}, {hx, -hy}, {hx, hy}, {-hx, hy}})
    {
      geometry_msgs::msg::Point p;
      p.x = xy.first; p.y = xy.second;
      padded_footprint_.push_back(p);
    }
  }
  void tunnelAndDetour()
  {
    auto * map = getCostmap();
    map->resizeMap(120, 100, 0.05, 0.0, 0.0);
    std::fill_n(map->getCharMap(), 120 * 100, nav2_costmap_2d::FREE_SPACE);
    for (unsigned int y = 16; y < 84; ++y) {
      for (unsigned int x = 40; x < 80; ++x) {
        map->setCost(x, y, y >= 43 && y < 57 ? 188 : 254);
      }
    }
    rectangle(0.298, 0.277);
  }

  void corridor()
  {
    auto * map = getCostmap();
    map->resizeMap(80, 40, 0.05, 0.0, 0.0);
    // Exactly 65 cm between occupied cell edges, as in the tunnel map.
    for (unsigned int y = 0; y < 40; ++y) {
      for (unsigned int x = 0; x < 80; ++x) {
        map->setCost(x, y, y < 9 || y >= 22 ? 254 : 0);
      }
    }
    rectangle(0.298, 0.277);
  }
};

class AdapterTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node;
  std::shared_ptr<TestCostmap> costmap;
  tdt_yastar_planner::YastarPlanner planner;
  void SetUp() override
  {
    node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("yastar_test_parent");
    node->set_parameter(rclcpp::Parameter("use_sim_time", true));
    ASSERT_EQ(rcl_enable_ros_time_override(node->get_clock()->get_clock_handle()), RCL_RET_OK);
    ASSERT_EQ(rcl_set_ros_time_override(node->get_clock()->get_clock_handle(), 123000000456LL),
      RCL_RET_OK);
    costmap = std::make_shared<TestCostmap>();
  }
  void configure(bool unknown = false)
  {
    node->declare_parameter("YAstar.allow_unknown", unknown);
    planner.configure(node, "YAstar", nullptr, costmap);
    planner.activate();
  }
  geometry_msgs::msg::PoseStamped pose(double x, double y)
  {
    geometry_msgs::msg::PoseStamped result;
    result.header.frame_id = "map";
    result.pose.position.x = x;
    result.pose.position.y = y;
    result.pose.orientation.w = 1.0;
    return result;
  }
};

TEST_F(AdapterTest, FiniteDefaultsHeadersAndHeadings)
{
  configure();
  EXPECT_EQ(node->get_parameter("YAstar.max_expansions").as_int(), 100000);
  EXPECT_DOUBLE_EQ(node->get_parameter("YAstar.max_planning_time").as_double(), 1.0);
  auto start = pose(0.5, 0.5);
  auto goal = pose(3.5, 2.5);
  goal.pose.orientation.z = std::sin(0.3);
  goal.pose.orientation.w = std::cos(0.3);
  const auto result = planner.createPlan(start, goal);
  ASSERT_GE(result.poses.size(), 2U);
  EXPECT_EQ(result.header.frame_id, "map");
  const builtin_interfaces::msg::Time expected_stamp = node->now();
  EXPECT_EQ(result.header.stamp, expected_stamp);
  EXPECT_DOUBLE_EQ(result.poses.front().pose.position.x, start.pose.position.x);
  EXPECT_DOUBLE_EQ(result.poses.back().pose.position.y, goal.pose.position.y);
  EXPECT_EQ(result.poses.back().pose.orientation, goal.pose.orientation);
  for (size_t i = 0; i < result.poses.size(); ++i) {
    EXPECT_EQ(result.poses[i].header, result.header);
    if (i + 1 == result.poses.size()) continue;
    const auto & current = result.poses[i].pose;
    const auto & next = result.poses[i + 1].pose;
    const double yaw = std::atan2(next.position.y - current.position.y,
      next.position.x - current.position.x);
    EXPECT_NEAR(current.orientation.z, std::sin(yaw / 2.0), 1e-6);
    EXPECT_NEAR(current.orientation.w, std::cos(yaw / 2.0), 1e-6);
  }
}

TEST_F(AdapterTest, UnknownAndLethalObstaclesAreBlockedByDefault)
{
  configure();
  for (auto cost : {nav2_costmap_2d::NO_INFORMATION,
      nav2_costmap_2d::LETHAL_OBSTACLE})
  {
    costmap->getCostmap()->setCost(3, 2, cost);
    EXPECT_THROW(planner.createPlan(pose(0.5, 0.5), pose(3.5, 2.5)), std::runtime_error);
  }
  EXPECT_THROW(planner.createPlan(pose(-0.01, 0.5), pose(2.5, 0.5)), std::invalid_argument);
  EXPECT_THROW(planner.createPlan(pose(0.5, 0.5), pose(4.0, 0.5)), std::invalid_argument);
}

TEST_F(AdapterTest, UnknownTraversalCannotBypassControllerPolicy)
{
  EXPECT_THROW(configure(true), std::invalid_argument);
}

TEST_F(AdapterTest, InflationAloneIsNotASecondBodyCollision)
{
  configure();
  costmap->getCostmap()->setCost(3, 2, nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
  EXPECT_FALSE(planner.createPlan(pose(0.5, 0.5), pose(3.5, 2.5)).poses.empty());
}

TEST_F(AdapterTest, UnknownUnderTheBodyIsBlockedEvenWithAFreeCenter)
{
  costmap->rectangle(0.6, 0.1);
  costmap->getCostmap()->setCost(1, 1, nav2_costmap_2d::NO_INFORMATION);
  configure();
  EXPECT_THROW(planner.createPlan(pose(0.9, 1.5), pose(2.5, 1.5)), std::runtime_error);
}

TEST_F(AdapterTest, CorridorPreservesValidExactGoalWithoutItsCollidingCellCenter)
{
  costmap->corridor();
  configure();
  auto start = pose(1.0, 0.775);
  start.pose.orientation.z = std::sin(M_PI / 4.0);
  start.pose.orientation.w = std::cos(M_PI / 4.0);
  // This exact goal fits, but its containing cell center y=0.725 does not.
  const auto goal = pose(3.0, 0.75);
  ASSERT_TRUE(pb_omni_pid_pursuit_controller::sweptFootprintCollision(
    *costmap->getCostmap(), costmap->getRobotFootprint(), 3.025, 0.725, 3.025, 0.725,
    M_PI / 2.0));
  const auto result = planner.createPlan(start, goal);
  ASSERT_GE(result.poses.size(), 2U);
  EXPECT_DOUBLE_EQ(result.poses.back().pose.position.y, 0.75);
  for (size_t i = 1; i < result.poses.size(); ++i) {
    const auto & a = result.poses[i - 1].pose.position;
    const auto & b = result.poses[i].pose.position;
    EXPECT_FALSE(pb_omni_pid_pursuit_controller::sweptFootprintCollision(
      *costmap->getCostmap(), costmap->getRobotFootprint(), a.x, a.y, b.x, b.y,
      M_PI / 2.0));
  }
}

TEST_F(AdapterTest, BodyOrientationCannotBeReplacedByPathTangent)
{
  costmap->corridor();
  costmap->rectangle(0.4, 0.1);
  configure();
  auto start = pose(1.0, 0.775);
  const auto goal = pose(3.0, 0.775);
  EXPECT_FALSE(planner.createPlan(start, goal).poses.empty());
  start.pose.orientation.z = std::sin(M_PI / 4.0);
  start.pose.orientation.w = std::cos(M_PI / 4.0);
  EXPECT_THROW(planner.createPlan(start, goal), std::runtime_error);
}

TEST_F(AdapterTest, ChecksSweptEdgesNotOnlyCollisionFreeNodes)
{
  auto * map = costmap->getCostmap();
  map->resizeMap(8, 8, 0.05, 0.0, 0.0);
  std::fill_n(map->getCharMap(), 64, nav2_costmap_2d::FREE_SPACE);
  map->setCost(3, 3, nav2_costmap_2d::LETHAL_OBSTACLE);
  costmap->rectangle(0.055, 0.01);
  configure();
  const auto start = pose(0.225, 0.225), goal = pose(0.275, 0.175);
  ASSERT_FALSE(pb_omni_pid_pursuit_controller::sweptFootprintCollision(
    *map, costmap->getRobotFootprint(), 0.225, 0.225, 0.225, 0.225, 0.0));
  ASSERT_FALSE(pb_omni_pid_pursuit_controller::sweptFootprintCollision(
    *map, costmap->getRobotFootprint(), 0.275, 0.175, 0.275, 0.175, 0.0));
  ASSERT_TRUE(pb_omni_pid_pursuit_controller::sweptFootprintCollision(
    *map, costmap->getRobotFootprint(), 0.225, 0.225, 0.275, 0.175, 0.0));
  const auto result = planner.createPlan(start, goal);
  ASSERT_GE(result.poses.size(), 3U);
  for (size_t i = 1; i < result.poses.size(); ++i) {
    const auto & a = result.poses[i - 1].pose.position;
    const auto & b = result.poses[i].pose.position;
    EXPECT_FALSE(pb_omni_pid_pursuit_controller::sweptFootprintCollision(
      *map, costmap->getRobotFootprint(), a.x, a.y, b.x, b.y, 0.0));
  }
}

TEST_F(AdapterTest, SameCellUsesTheExactBodyMotion)
{
  configure();
  const auto result = planner.createPlan(pose(0.2, 0.2), pose(0.8, 0.8));
  EXPECT_DOUBLE_EQ(result.poses.front().pose.position.x, 0.2);
  EXPECT_DOUBLE_EQ(result.poses.back().pose.position.x, 0.8);
}

TEST_F(AdapterTest, TunnelDiscountChangesRouteAndCanBeDisabled)
{
  costmap->tunnelAndDetour();
  const auto original = std::vector<unsigned char>(costmap->getCostmap()->getCharMap(),
    costmap->getCostmap()->getCharMap() + 120 * 100);
  const auto start = pose(1.0, 2.525), goal = pose(5.0, 2.525);
  const auto uses_tunnel = [](const nav_msgs::msg::Path & path) {
    return std::any_of(path.poses.begin(), path.poses.end(), [](const auto & p) {
      return p.pose.position.x > 2.9 && p.pose.position.x < 3.1 &&
             p.pose.position.y > 2.15 && p.pose.position.y < 2.85;
    });
  };
  node->declare_parameter("YAstar.tunnel_cost_regions", std::vector<double>{1.5, 2.15, 4.5, 2.85});
  node->declare_parameter("YAstar.tunnel_cost_enabled", false);
  node->declare_parameter("YAstar.tunnel_cost_scale", 0.05);
  configure();
  const auto baseline = planner.createPlan(start, goal);
  EXPECT_FALSE(uses_tunnel(baseline));
  planner.deactivate();
  node->set_parameter(rclcpp::Parameter("YAstar.tunnel_cost_enabled", true));
  planner.configure(node, "YAstar", nullptr, costmap);
  planner.activate();
  const auto discounted = planner.createPlan(start, goal);
  EXPECT_TRUE(uses_tunnel(discounted));
  for (size_t i = 1; i < discounted.poses.size(); ++i) {
    const auto & a = discounted.poses[i - 1].pose.position;
    const auto & b = discounted.poses[i].pose.position;
    EXPECT_FALSE(pb_omni_pid_pursuit_controller::sweptFootprintCollision(
      *costmap->getCostmap(), costmap->getRobotFootprint(), a.x, a.y, b.x, b.y, 0.0));
  }
  EXPECT_EQ(original, std::vector<unsigned char>(costmap->getCostmap()->getCharMap(),
    costmap->getCostmap()->getCharMap() + 120 * 100));
  // Unit scale restores the original route despite the enabled switch.
  planner.deactivate();
  node->set_parameter(rclcpp::Parameter("YAstar.tunnel_cost_scale", 1.0));
  planner.configure(node, "YAstar", nullptr, costmap);
  planner.activate();
  const auto restored = planner.createPlan(start, goal);
  ASSERT_EQ(restored.poses.size(), baseline.poses.size());
  for (size_t i = 0; i < restored.poses.size(); ++i) {
    EXPECT_EQ(restored.poses[i].pose, baseline.poses[i].pose);
  }
  // An unrelated region cannot change this route.
  planner.deactivate();
  node->set_parameter(rclcpp::Parameter("YAstar.tunnel_cost_scale", 0.1));
  node->set_parameter(rclcpp::Parameter("YAstar.tunnel_cost_regions",
    std::vector<double>{10.0, 10.0, 11.0, 11.0}));
  planner.configure(node, "YAstar", nullptr, costmap);
  planner.activate();
  const auto outside = planner.createPlan(start, goal);
  ASSERT_EQ(outside.poses.size(), baseline.poses.size());
  for (size_t i = 0; i < outside.poses.size(); ++i) {
    EXPECT_EQ(outside.poses[i].pose, baseline.poses[i].pose);
  }
}

TEST_F(AdapterTest, TunnelDiscountStillBlocksLethalUnknownAndBodyCollisions)
{
  costmap->tunnelAndDetour();
  node->declare_parameter("YAstar.tunnel_cost_enabled", true);
  node->declare_parameter("YAstar.tunnel_cost_regions", std::vector<double>{1.5, 2.15, 4.5, 2.85});
  configure();
  auto * map = costmap->getCostmap();
  for (const auto obstacle : {nav2_costmap_2d::LETHAL_OBSTACLE, nav2_costmap_2d::NO_INFORMATION}) {
    // A blocking strip closes the tunnel. The long detour must still be used.
    for (unsigned int y = 43; y < 57; ++y) map->setCost(60, y, obstacle);
    const auto result = planner.createPlan(pose(1.0, 2.525), pose(5.0, 2.525));
    EXPECT_TRUE(std::none_of(result.poses.begin(), result.poses.end(), [](const auto & p) {
      return p.pose.position.x > 2.9 && p.pose.position.x < 3.1 &&
             p.pose.position.y > 2.15 && p.pose.position.y < 2.85;
    }));
    EXPECT_THROW(planner.createPlan(pose(1.0, 2.525), pose(3.025, 2.525)), std::runtime_error);
  }
  // Even an unoccupied goal center fails if its body overlaps a side wall.
  EXPECT_THROW(planner.createPlan(pose(1.0, 2.525), pose(2.5, 2.2)), std::runtime_error);
}

TEST_F(AdapterTest, RejectsInvalidTunnelCompensationParameters)
{
  node->declare_parameter("YAstar.tunnel_cost_enabled", true);
  node->declare_parameter("YAstar.tunnel_cost_scale", 0.1);
  node->declare_parameter("YAstar.tunnel_cost_regions", std::vector<double>{});
  EXPECT_THROW(configure(), std::invalid_argument);
  for (const auto & regions : std::vector<std::vector<double>>{
      {0.0, 1.0, 2.0}, {2.0, 0.0, 1.0, 2.0},
      {0.0, 0.0, std::numeric_limits<double>::quiet_NaN(), 2.0}}) {
    node->set_parameter(rclcpp::Parameter("YAstar.tunnel_cost_regions", regions));
    EXPECT_THROW(planner.configure(node, "YAstar", nullptr, costmap), std::invalid_argument);
  }
  node->set_parameter(rclcpp::Parameter("YAstar.tunnel_cost_regions",
    std::vector<double>{0.0, 0.0, 2.0, 2.0}));
  for (double scale : {0.0, -1.0, 1.1, std::numeric_limits<double>::infinity()}) {
    node->set_parameter(rclcpp::Parameter("YAstar.tunnel_cost_scale", scale));
    EXPECT_THROW(planner.configure(node, "YAstar", nullptr, costmap), std::invalid_argument);
  }
}

TEST_F(AdapterTest, MissingFootprintFailsClosed)
{
  costmap->rectangle(0.0, 0.0);
  // An empty footprint must not silently restore center-only planning.
  costmap->setRobotFootprint({});
  configure();
  EXPECT_THROW(planner.createPlan(pose(0.5, 0.5), pose(2.5, 0.5)), std::runtime_error);
}

TEST_F(AdapterTest, GoalOwnedCancellationReachesTheAdapterAndSearch)
{
  configure();
  EXPECT_THROW(planner.createPlan(pose(0.5, 0.5), pose(3.5, 2.5), [] {return true;}),
    tdt_yastar_planner::PlannerCancelled);
  int checks = 0;
  EXPECT_THROW(planner.createPlan(pose(0.5, 0.5), pose(3.5, 2.5), [&] {return ++checks >= 6;}),
    tdt_yastar_planner::PlannerCancelled);
  EXPECT_GE(checks, 6);
  EXPECT_FALSE(planner.createPlan(pose(0.5, 0.5), pose(3.5, 2.5), [] {return false;}).poses.empty());
}

TEST_F(AdapterTest, CancelsWhileAnotherThreadHoldsCostmapLock)
{
  configure();
  std::promise<void> acquired;
  std::promise<void> release;
  auto released = release.get_future();
  std::thread holder([&] {
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> map_lock(*costmap->getCostmap()->getMutex());
    acquired.set_value();
    released.wait();
  });
  acquired.get_future().wait();
  std::atomic<bool> cancel{false};
  auto worker = std::async(std::launch::async, [&] {
    try {
      planner.createPlan(pose(0.5, 0.5), pose(3.5, 2.5), [&] {return cancel.load();});
      return false;
    } catch (const tdt_yastar_planner::PlannerCancelled &) {
      return true;
    }
  });
  EXPECT_EQ(worker.wait_for(std::chrono::milliseconds(30)), std::future_status::timeout);
  cancel.store(true);
  const auto result = worker.wait_for(std::chrono::seconds(1));
  // Release before assertions so a regression cannot hang test cleanup.
  release.set_value();
  holder.join();
  EXPECT_EQ(result, std::future_status::ready);
  EXPECT_TRUE(worker.get());
}
}  // namespace
