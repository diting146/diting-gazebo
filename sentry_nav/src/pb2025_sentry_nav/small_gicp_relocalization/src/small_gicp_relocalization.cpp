// Copyright 2025 Lihan Chen
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

#include "small_gicp_relocalization/small_gicp_relocalization.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <stdexcept>

#include "pcl/common/transforms.h"
#include "pcl_conversions/pcl_conversions.h"
#include "small_gicp/pcl/pcl_registration.hpp"
#include "small_gicp/util/downsampling_omp.hpp"
#include "tf2_eigen/tf2_eigen.hpp"

namespace small_gicp_relocalization
{

SmallGicpRelocalizationNode::SmallGicpRelocalizationNode(const rclcpp::NodeOptions & options)
: Node("small_gicp_relocalization", options),
  result_t_(Eigen::Isometry3d::Identity()),
  previous_result_t_(Eigen::Isometry3d::Identity())
{
  this->declare_parameter("num_threads", 4);
  this->declare_parameter("num_neighbors", 20);
  this->declare_parameter("global_leaf_size", 0.25);
  this->declare_parameter("registered_leaf_size", 0.25);
  this->declare_parameter("max_dist_sq", 1.0);
  this->declare_parameter("map_frame", "map");
  this->declare_parameter("odom_frame", "odom");
  this->declare_parameter("base_frame", "");
  this->declare_parameter("robot_base_frame", "");
  this->declare_parameter("lidar_frame", "");
  this->declare_parameter("prior_pcd_file", "");
  this->declare_parameter("init_pose", std::vector<double>{0., 0., 0., 0., 0., 0.});
  this->declare_parameter("input_cloud_topic", "registered_scan");
  // initial_robot_pose is a map-frame base_footprint pose. init_pose retains its
  // original map->odom transform meaning for callers which do not opt in.
  this->declare_parameter("initial_robot_pose", std::vector<double>{});
  this->declare_parameter("required_consecutive_matches", 3);
  this->declare_parameter("registration_timeout_sec", 1.5);
  this->declare_parameter("min_registration_inliers", 100);
  this->declare_parameter("min_registration_inlier_ratio", 0.2);
  this->declare_parameter("max_registration_mean_error", 5.0);
  this->declare_parameter("max_registration_translation_step", 0.75);
  this->declare_parameter("max_registration_rotation_step", 0.6);

  this->get_parameter("num_threads", num_threads_);
  this->get_parameter("num_neighbors", num_neighbors_);
  this->get_parameter("global_leaf_size", global_leaf_size_);
  this->get_parameter("registered_leaf_size", registered_leaf_size_);
  this->get_parameter("max_dist_sq", max_dist_sq_);
  this->get_parameter("map_frame", map_frame_);
  this->get_parameter("odom_frame", odom_frame_);
  this->get_parameter("base_frame", base_frame_);
  this->get_parameter("robot_base_frame", robot_base_frame_);
  this->get_parameter("lidar_frame", lidar_frame_);
  this->get_parameter("prior_pcd_file", prior_pcd_file_);
  this->get_parameter("init_pose", init_pose_);
  this->get_parameter("input_cloud_topic", input_cloud_topic_);
  this->get_parameter("initial_robot_pose", initial_robot_pose_);
  this->get_parameter("required_consecutive_matches", required_consecutive_matches_);
  this->get_parameter("registration_timeout_sec", registration_timeout_sec_);
  int min_inliers = this->get_parameter("min_registration_inliers").as_int();
  quality_limits_.min_inlier_ratio =
    this->get_parameter("min_registration_inlier_ratio").as_double();
  quality_limits_.max_mean_error = this->get_parameter("max_registration_mean_error").as_double();
  quality_limits_.max_translation_step =
    this->get_parameter("max_registration_translation_step").as_double();
  quality_limits_.max_rotation_step =
    this->get_parameter("max_registration_rotation_step").as_double();
  const auto finite_pose = [](const std::vector<double> & pose) {
    return pose.size() == 6 &&
           std::all_of(pose.begin(), pose.end(), [](double value) { return std::isfinite(value); });
  };
  if (
    !finite_pose(init_pose_) ||
    (!initial_robot_pose_.empty() && !finite_pose(initial_robot_pose_)) ||
    required_consecutive_matches_ < 1 || min_inliers < 1 ||
    !std::isfinite(registration_timeout_sec_) || registration_timeout_sec_ <= 0.0 ||
    !std::isfinite(quality_limits_.min_inlier_ratio) || quality_limits_.min_inlier_ratio <= 0.0 ||
    quality_limits_.min_inlier_ratio > 1.0 || !std::isfinite(quality_limits_.max_mean_error) ||
    quality_limits_.max_mean_error <= 0.0 || !std::isfinite(quality_limits_.max_translation_step) ||
    quality_limits_.max_translation_step <= 0.0 ||
    !std::isfinite(quality_limits_.max_rotation_step) || quality_limits_.max_rotation_step <= 0.0) {
    throw std::invalid_argument("Invalid localization initialization or quality thresholds");
  }
  quality_limits_.min_inliers = static_cast<std::size_t>(min_inliers);
  initial_robot_pose_pending_ = !initial_robot_pose_.empty();
  localization_status_pub_ = this->create_publisher<std_msgs::msg::String>(
    "localization_status", rclcpp::QoS(1).reliable().transient_local());

  // [x, y, z, roll, pitch, yaw] - init_pose parameters
  if (!init_pose_.empty() && init_pose_.size() >= 6) {
    result_t_.translation() << init_pose_[0], init_pose_[1], init_pose_[2];
    result_t_.linear() =
      Eigen::AngleAxisd(init_pose_[5], Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(init_pose_[4], Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(init_pose_[3], Eigen::Vector3d::UnitX()).toRotationMatrix();
  }
  initial_result_t_ = previous_result_t_ = result_t_;

  accumulated_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  global_map_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  register_ = std::make_shared<
    small_gicp::Registration<small_gicp::GICPFactor, small_gicp::ParallelReductionOMP>>();

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

  loadGlobalMap(prior_pcd_file_);
  if (global_map_->empty()) {
    throw std::runtime_error("Localization prior point cloud is empty");
  }

  // Downsample points and convert them into pcl::PointCloud<pcl::PointCovariance>
  target_ = small_gicp::voxelgrid_sampling_omp<
    pcl::PointCloud<pcl::PointXYZ>, pcl::PointCloud<pcl::PointCovariance>>(
    *global_map_, global_leaf_size_);

  // Estimate covariances of points
  small_gicp::estimate_covariances_omp(*target_, num_neighbors_, num_threads_);

  // Create KdTree for target
  target_tree_ = std::make_shared<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>>(
    target_, small_gicp::KdTreeBuilderOMP(num_threads_));

  pcd_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    input_cloud_topic_, 10,
    std::bind(&SmallGicpRelocalizationNode::registeredPcdCallback, this, std::placeholders::_1));

  initial_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "initialpose", 10,
    std::bind(&SmallGicpRelocalizationNode::initialPoseCallback, this, std::placeholders::_1));

  register_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(500),  // 2 Hz
    std::bind(&SmallGicpRelocalizationNode::performRegistration, this));

  publishLocalizationStatus();
  transform_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(50),  // 20 Hz
    std::bind(&SmallGicpRelocalizationNode::publishTransform, this));
}

void SmallGicpRelocalizationNode::loadGlobalMap(const std::string & file_name)
{
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(file_name, *global_map_) == -1) {
    RCLCPP_ERROR(this->get_logger(), "Couldn't read PCD file: %s", file_name.c_str());
    return;
  }
  RCLCPP_INFO(this->get_logger(), "Loaded global map with %zu points", global_map_->points.size());

  // NOTE: Transform global pcd_map (based on `lidar_odom` frame) to the `odom` frame
  Eigen::Affine3d odom_to_lidar_odom;
  while (true) {
    try {
      auto tf_stamped = tf_buffer_->lookupTransform(
        base_frame_, lidar_frame_, this->now(), rclcpp::Duration::from_seconds(1.0));
      odom_to_lidar_odom = tf2::transformToEigen(tf_stamped.transform);
      RCLCPP_INFO_STREAM(
        this->get_logger(), "odom_to_lidar_odom: translation = "
                              << odom_to_lidar_odom.translation().transpose() << ", rpy = "
                              << odom_to_lidar_odom.rotation().eulerAngles(0, 1, 2).transpose());
      break;
    } catch (tf2::TransformException & ex) {
      RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s Retrying...", ex.what());
      rclcpp::sleep_for(std::chrono::seconds(1));
    }
  }
  pcl::transformPointCloud(*global_map_, *global_map_, odom_to_lidar_odom);
}

void SmallGicpRelocalizationNode::invalidateLocalization(const std::string & reason)
{
  latest_registration_ok_ = false;
  consecutive_matches_ = 0;
  localization_reason_ = reason;
}

bool SmallGicpRelocalizationNode::setInitialRobotPose(const rclcpp::Time & stamp)
{
  if (!initial_robot_pose_pending_) {
    return true;
  }
  try {
    // T_map_odom = T_map_base * T_base_odom, at the same stamp as the cloud.
    const auto base_to_odom = tf_buffer_->lookupTransform(robot_base_frame_, odom_frame_, stamp);
    Eigen::Isometry3d map_to_base = Eigen::Isometry3d::Identity();
    map_to_base.translation() << initial_robot_pose_[0], initial_robot_pose_[1],
      initial_robot_pose_[2];
    map_to_base.linear() =
      Eigen::AngleAxisd(initial_robot_pose_[5], Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(initial_robot_pose_[4], Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(initial_robot_pose_[3], Eigen::Vector3d::UnitX()).toRotationMatrix();
    result_t_ = previous_result_t_ = map_to_base * tf2::transformToEigen(base_to_odom.transform);
    initial_robot_pose_pending_ = false;
    invalidateLocalization("initial_robot_pose_applied");
    RCLCPP_INFO(
      this->get_logger(), "Applied map-frame initial robot pose at scan stamp %.6f",
      stamp.seconds());
    return true;
  } catch (const tf2::TransformException & ex) {
    invalidateLocalization("awaiting_initial_pose_tf");
    return false;
  }
}

void SmallGicpRelocalizationNode::registeredPcdCallback(
  const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  const rclcpp::Time stamp(msg->header.stamp, this->get_clock()->get_clock_type());
  if (stamp.nanoseconds() <= 0 || msg->header.frame_id != odom_frame_) {
    invalidateLocalization("invalid_cloud_stamp_or_frame");
    return;
  }
  if (have_scan_ && stamp < last_scan_time_) {
    accumulated_cloud_->clear();
    initial_robot_pose_pending_ = !initial_robot_pose_.empty();
    result_t_ = previous_result_t_ = initial_result_t_;
    invalidateLocalization("source_time_reversed");
  } else if (have_scan_ && stamp == last_scan_time_) {
    return;
  }
  last_scan_time_ = stamp;
  have_scan_ = true;
  current_scan_frame_id_ = msg->header.frame_id;
  pcl::PointCloud<pcl::PointXYZ>::Ptr scan(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(*msg, *scan);
  for (const auto & point : *scan) {
    if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z)) {
      accumulated_cloud_->push_back(point);
    }
  }
}

void SmallGicpRelocalizationNode::performRegistration()
{
  const auto now = this->now();
  if (
    !have_scan_ || last_scan_time_ > now ||
    (now - last_scan_time_).seconds() > registration_timeout_sec_) {
    accumulated_cloud_->clear();
    invalidateLocalization("cloud_expired_or_not_received");
    publishLocalizationStatus();
    return;
  }
  if (accumulated_cloud_->empty()) {
    RCLCPP_WARN(this->get_logger(), "No accumulated points to process.");
    return;
  }
  // Allow same-stamp operational odometry TF to arrive after the cloud callback.
  // A missing transform must not grow the pending cloud without bound.
  if (!setInitialRobotPose(last_scan_time_)) {
    accumulated_cloud_->clear();
    publishLocalizationStatus();
    return;
  }

  source_ = small_gicp::voxelgrid_sampling_omp<
    pcl::PointCloud<pcl::PointXYZ>, pcl::PointCloud<pcl::PointCovariance>>(
    *accumulated_cloud_, registered_leaf_size_);

  if (!source_ || source_->size() < static_cast<std::size_t>(num_neighbors_)) {
    accumulated_cloud_->clear();
    invalidateLocalization("insufficient_source_points");
    publishLocalizationStatus();
    return;
  }
  small_gicp::estimate_covariances_omp(*source_, num_neighbors_, num_threads_);

  source_tree_ = std::make_shared<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>>(
    source_, small_gicp::KdTreeBuilderOMP(num_threads_));

  if (!source_ || !source_tree_) {
    return;
  }

  register_->reduction.num_threads = num_threads_;
  register_->rejector.max_dist_sq = max_dist_sq_;
  register_->optimizer.max_iterations = 10;

  auto result = register_->align(*target_, *source_, *target_tree_, previous_result_t_);

  last_inliers_ = result.num_inliers;
  last_source_points_ = source_->size();
  last_mean_error_ = result.num_inliers > 0 ? result.error / result.num_inliers : 0.0;
  const auto correction = previous_result_t_.inverse() * result.T_target_source;
  last_translation_step_ = correction.translation().norm();
  last_rotation_step_ = Eigen::AngleAxisd(correction.rotation()).angle();
  const auto rejected = registrationRejection(
    result.converged,
    result.T_target_source.matrix().allFinite() && result.H.allFinite() && result.b.allFinite(),
    result.num_inliers, source_->size(), result.error, last_translation_step_, last_rotation_step_,
    quality_limits_);
  if (!rejected) {
    result_t_ = previous_result_t_ = result.T_target_source;
    last_accepted_scan_time_ = last_scan_time_;
    last_accepted_wall_ = std::chrono::steady_clock::now();
    latest_registration_ok_ = true;
    ++consecutive_matches_;
    localization_reason_ =
      consecutive_matches_ >= static_cast<std::size_t>(required_consecutive_matches_)
        ? "trusted"
        : "confirming_registration";
  } else {
    invalidateLocalization(rejected);
    RCLCPP_WARN(this->get_logger(), "GICP rejected: %s", rejected);
  }
  accumulated_cloud_->clear();
  publishLocalizationStatus();
}

void SmallGicpRelocalizationNode::publishTransform()
{
  publishLocalizationStatus();
  if (!have_scan_ || initial_robot_pose_pending_ || result_t_.matrix().isZero()) {
    return;
  }

  geometry_msgs::msg::TransformStamped transform_stamped;
  // `+ 0.1` means transform into future. according to https://robotics.stackexchange.com/a/96615
  transform_stamped.header.stamp = last_scan_time_ + rclcpp::Duration::from_seconds(0.1);
  transform_stamped.header.frame_id = map_frame_;
  transform_stamped.child_frame_id = odom_frame_;

  const Eigen::Vector3d translation = result_t_.translation();
  const Eigen::Quaterniond rotation(result_t_.rotation());

  transform_stamped.transform.translation.x = translation.x();
  transform_stamped.transform.translation.y = translation.y();
  transform_stamped.transform.translation.z = translation.z();
  transform_stamped.transform.rotation.x = rotation.x();
  transform_stamped.transform.rotation.y = rotation.y();
  transform_stamped.transform.rotation.z = rotation.z();
  transform_stamped.transform.rotation.w = rotation.w();

  tf_broadcaster_->sendTransform(transform_stamped);
}

void SmallGicpRelocalizationNode::publishLocalizationStatus()
{
  const auto now = this->now();
  if (last_clock_time_.nanoseconds() > 0 && now < last_clock_time_) {
    accumulated_cloud_->clear();
    have_scan_ = false;
    initial_robot_pose_pending_ = !initial_robot_pose_.empty();
    result_t_ = previous_result_t_ = initial_result_t_;
    invalidateLocalization("clock_time_reversed");
  }
  last_clock_time_ = now;
  const double wall_age =
    latest_registration_ok_
      ? std::chrono::duration<double>(std::chrono::steady_clock::now() - last_accepted_wall_)
          .count()
      : 0.0;
  if (
    latest_registration_ok_ &&
    (last_accepted_scan_time_ > now ||
     (now - last_accepted_scan_time_).seconds() > registration_timeout_sec_ ||
     wall_age > std::max(5.0, 3.0 * registration_timeout_sec_))) {
    invalidateLocalization("registration_expired");
  }
  const bool trusted =
    latest_registration_ok_ &&
    consecutive_matches_ >= static_cast<std::size_t>(required_consecutive_matches_);
  const auto finite_or_null = [](double value) {
    if (!std::isfinite(value)) {
      return std::string("null");
    }
    std::ostringstream number;
    number << std::setprecision(17) << value;
    return number.str();
  };
  std::ostringstream status;
  status << "{\"trusted\":" << (trusted ? "true" : "false")
         << ",\"stamp_ns\":" << last_accepted_scan_time_.nanoseconds()
         << ",\"latest_scan_stamp_ns\":" << (have_scan_ ? last_scan_time_.nanoseconds() : 0)
         << ",\"consecutive_matches\":" << consecutive_matches_ << ",\"inliers\":" << last_inliers_
         << ",\"source_points\":" << last_source_points_
         << ",\"mean_gicp_error\":" << finite_or_null(last_mean_error_)
         << ",\"translation_step\":" << finite_or_null(last_translation_step_)
         << ",\"rotation_step\":" << finite_or_null(last_rotation_step_) << ",\"reason\":\""
         << localization_reason_ << "\"}";
  std_msgs::msg::String message;
  message.data = status.str();
  localization_status_pub_->publish(message);
}

void SmallGicpRelocalizationNode::initialPoseCallback(
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  invalidateLocalization("manual_initial_pose");
  accumulated_cloud_->clear();
  publishLocalizationStatus();
  if (
    (!msg->header.frame_id.empty() && msg->header.frame_id != map_frame_) ||
    !std::isfinite(msg->pose.pose.position.x) || !std::isfinite(msg->pose.pose.position.y) ||
    !std::isfinite(msg->pose.pose.position.z)) {
    invalidateLocalization("invalid_initial_pose");
    return;
  }
  RCLCPP_INFO(
    this->get_logger(), "Received initial pose: [x: %f, y: %f, z: %f]", msg->pose.pose.position.x,
    msg->pose.pose.position.y, msg->pose.pose.position.z);

  Eigen::Isometry3d map_to_robot_base = Eigen::Isometry3d::Identity();
  map_to_robot_base.translation() << msg->pose.pose.position.x, msg->pose.pose.position.y,
    msg->pose.pose.position.z;
  Eigen::Quaterniond quaternion(
    msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
    msg->pose.pose.orientation.z);
  if (!quaternion.coeffs().allFinite() || quaternion.norm() < 1e-6) {
    invalidateLocalization("invalid_initial_orientation");
    return;
  }
  map_to_robot_base.linear() = quaternion.normalized().toRotationMatrix();

  try {
    auto transform =
      tf_buffer_->lookupTransform(robot_base_frame_, odom_frame_, tf2::TimePointZero);
    Eigen::Isometry3d robot_base_to_odom = tf2::transformToEigen(transform.transform);
    Eigen::Isometry3d map_to_odom = map_to_robot_base * robot_base_to_odom;

    previous_result_t_ = result_t_ = map_to_odom;
    initial_robot_pose_pending_ = false;
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(
      this->get_logger(), "Could not transform initial pose from %s to %s: %s",
      robot_base_frame_.c_str(), current_scan_frame_id_.c_str(), ex.what());
  }
}

}  // namespace small_gicp_relocalization

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(small_gicp_relocalization::SmallGicpRelocalizationNode)
