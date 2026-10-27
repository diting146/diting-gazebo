#include <gtest/gtest.h>
#include <nav2_costmap_2d/cost_values.hpp>
#include <tdt_yastar_planner/yastar_planner.hpp>

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

TEST_F(AdapterTest, UnknownAndInflatedObstaclesAreBlockedByDefault)
{
  configure();
  for (auto cost : {nav2_costmap_2d::NO_INFORMATION,
      nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE, nav2_costmap_2d::LETHAL_OBSTACLE})
  {
    costmap->getCostmap()->setCost(3, 2, cost);
    EXPECT_THROW(planner.createPlan(pose(0.5, 0.5), pose(3.5, 2.5)), std::runtime_error);
  }
  EXPECT_THROW(planner.createPlan(pose(-0.01, 0.5), pose(2.5, 0.5)), std::invalid_argument);
  EXPECT_THROW(planner.createPlan(pose(0.5, 0.5), pose(4.0, 0.5)), std::invalid_argument);
}

TEST_F(AdapterTest, UnknownCanBeExplicitlyAllowed)
{
  configure(true);
  costmap->getCostmap()->setCost(3, 2, nav2_costmap_2d::NO_INFORMATION);
  EXPECT_FALSE(planner.createPlan(pose(0.5, 0.5), pose(3.5, 2.5)).poses.empty());
}
}  // namespace
