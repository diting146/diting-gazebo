#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "fake_vel_transform/fake_vel_transform.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

namespace
{
using namespace std::chrono_literals;

class VelocityControl : public ::testing::TestWithParam<bool>
{
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }

  void SetUp() override
  {
    static unsigned sequence = 0;
    prefix = "/velocity_control_test_" + std::to_string(++sequence);
    observer = std::make_shared<rclcpp::Node>("observer", prefix);
    output = observer->create_subscription<geometry_msgs::msg::Twist>(
      prefix + "/output", 100,
      [this](geometry_msgs::msg::Twist::SharedPtr message) { commands.push_back(*message); });
    transforms = observer->create_subscription<tf2_msgs::msg::TFMessage>(
      "/tf", 100, [this](tf2_msgs::msg::TFMessage::SharedPtr message) {
        for (const auto & transform : message->transforms) {
          if (transform.child_frame_id == prefix.substr(1) + "/fake") {
            rotation = transform.transform.rotation;
            saw_transform = true;
          }
        }
      });
    input = observer->create_publisher<geometry_msgs::msg::Twist>(prefix + "/input", 10);
    odometry = observer->create_publisher<nav_msgs::msg::Odometry>(prefix + "/odom", 10);
    plans = observer->create_publisher<nav_msgs::msg::Path>(prefix + "/plan", 10);
    spins = observer->create_publisher<example_interfaces::msg::Float32>(prefix + "/spin", 10);
    rclcpp::NodeOptions options;
    options.parameter_overrides(
      {rclcpp::Parameter("use_sim_time", GetParam()),
       rclcpp::Parameter("robot_base_frame", prefix.substr(1) + "/chassis"),
       rclcpp::Parameter("fake_robot_base_frame", prefix.substr(1) + "/fake"),
       rclcpp::Parameter("input_cmd_vel_topic", prefix + "/input"),
       rclcpp::Parameter("output_cmd_vel_topic", prefix + "/output"),
       rclcpp::Parameter("odom_topic", prefix + "/odom"),
       rclcpp::Parameter("local_plan_topic", prefix + "/plan"),
       rclcpp::Parameter("cmd_spin_topic", prefix + "/spin"),
       rclcpp::Parameter("init_spin_speed", 0.0)});
    transform = std::make_shared<fake_vel_transform::FakeVelTransform>(options);
    executor.add_node(observer);
    executor.add_node(transform);
    ASSERT_TRUE(wait([this]() {
      return input->get_subscription_count() && odometry->get_subscription_count() &&
             plans->get_subscription_count() && spins->get_subscription_count();
    }));
  }

  void TearDown() override
  {
    executor.remove_node(transform);
    executor.remove_node(observer);
    transform.reset();
    observer.reset();
  }

  bool wait(const std::function<bool()> & ready, double seconds = 2.0)
  {
    const auto end = std::chrono::steady_clock::now() + std::chrono::duration<double>(seconds);
    do {
      executor.spin_some();
      if (ready()) {
        return true;
      }
      std::this_thread::sleep_for(2ms);
    } while (std::chrono::steady_clock::now() < end);
    return false;
  }

  void spin(double seconds)
  {
    (void)wait([]() { return false; }, seconds);
  }

  void command(double x)
  {
    geometry_msgs::msg::Twist message;
    message.linear.x = x;
    input->publish(message);
  }

  void pair(int seconds, double yaw = 0.0)
  {
    nav_msgs::msg::Odometry odom;
    odom.header.stamp.sec = seconds;
    odom.pose.pose.orientation.z = std::sin(yaw / 2.0);
    odom.pose.pose.orientation.w = std::cos(yaw / 2.0);
    nav_msgs::msg::Path plan;
    plan.header.stamp = odom.header.stamp;
    odometry->publish(odom);
    plans->publish(plan);
  }

  bool lastX(double x) const
  {
    return !commands.empty() && std::abs(commands.back().linear.x - x) < 1e-6;
  }

  std::string prefix;
  rclcpp::executors::SingleThreadedExecutor executor;
  std::shared_ptr<rclcpp::Node> observer;
  std::shared_ptr<fake_vel_transform::FakeVelTransform> transform;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr input;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr plans;
  rclcpp::Publisher<example_interfaces::msg::Float32>::SharedPtr spins;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr output;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr transforms;
  std::vector<geometry_msgs::msg::Twist> commands;
  geometry_msgs::msg::Quaternion rotation;
  bool saw_transform{false};
};

TEST_P(VelocityControl, StartupAndFirstOdometryAreValid)
{
  ASSERT_TRUE(wait([this]() { return saw_transform; }));
  EXPECT_DOUBLE_EQ(rotation.x, 0.0);
  EXPECT_DOUBLE_EQ(rotation.y, 0.0);
  EXPECT_DOUBLE_EQ(rotation.z, 0.0);
  EXPECT_DOUBLE_EQ(rotation.w, 1.0);
  nav_msgs::msg::Odometry message;
  message.pose.pose.orientation.z = std::sin(0.25);
  message.pose.pose.orientation.w = std::cos(0.25);
  odometry->publish(message);
  ASSERT_TRUE(wait([this]() { return std::abs(rotation.z + std::sin(0.25)) < 1e-6; }));
}

TEST_P(VelocityControl, StopCannotReplayCachedVelocity)
{
  pair(1);
  spin(0.05);
  command(0.6);
  spin(0.02);
  for (int stamp = 2; stamp < 5; ++stamp) {
    pair(stamp);
    spin(0.02);
  }
  ASSERT_TRUE(wait([this]() { return lastX(0.6); }));
  command(0.0);
  ASSERT_TRUE(wait([this]() { return lastX(0.0); }));
  const auto stopped = commands.size();
  for (int stamp = 5; stamp < 9; ++stamp) {
    pair(stamp);
    spin(0.02);
  }
  ASSERT_GT(commands.size(), stopped);
  for (size_t i = stopped; i < commands.size(); ++i) {
    EXPECT_DOUBLE_EQ(commands[i].linear.x, 0.0);
    EXPECT_DOUBLE_EQ(commands[i].linear.y, 0.0);
    EXPECT_DOUBLE_EQ(commands[i].angular.z, 0.0);
  }
}

TEST_P(VelocityControl, CommandGapStopsEvenWithPausedClock)
{
  command(0.4);
  ASSERT_TRUE(wait([this]() { return lastX(0.4); }));
  ASSERT_TRUE(wait([this]() { return lastX(0.0); }, 1.2));
  command(0.2);
  ASSERT_TRUE(wait([this]() { return lastX(0.2); }));
  if (GetParam()) {
    EXPECT_EQ(transform->now().nanoseconds(), 0);
  }
}

TEST_P(VelocityControl, ExplicitIdleSpinSurvivesTimeout)
{
  example_interfaces::msg::Float32 spin;
  spin.data = 0.7F;
  spins->publish(spin);
  this->spin(0.03);
  command(0.3);
  ASSERT_TRUE(wait([this]() { return lastX(0.3); }));
  ASSERT_TRUE(wait([this]() { return lastX(0.0); }, 1.2));
  EXPECT_NEAR(commands.back().angular.z, 0.7, 1e-6);
}

INSTANTIATE_TEST_SUITE_P(SystemAndPausedSimulationTime, VelocityControl, ::testing::Bool());
}  // namespace
