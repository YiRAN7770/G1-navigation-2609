#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <nav_msgs/msg/odometry.hpp>
#include <pcl/common/transforms.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/gicp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include "navigation_frames.hpp"

namespace
{
using PointT = pcl::PointXYZI;
using CloudT = pcl::PointCloud<PointT>;

Eigen::Matrix4f poseToMatrix(const geometry_msgs::msg::Pose & pose)
{
  Eigen::Quaternionf q(
    static_cast<float>(pose.orientation.w), static_cast<float>(pose.orientation.x),
    static_cast<float>(pose.orientation.y), static_cast<float>(pose.orientation.z));
  q = q.norm() < 1.0e-6F ? Eigen::Quaternionf::Identity() : q.normalized();
  Eigen::Matrix4f result = Eigen::Matrix4f::Identity();
  result.block<3, 3>(0, 0) = q.toRotationMatrix();
  result(0, 3) = static_cast<float>(pose.position.x);
  result(1, 3) = static_cast<float>(pose.position.y);
  result(2, 3) = static_cast<float>(pose.position.z);
  return result;
}

geometry_msgs::msg::Pose matrixToPose(const Eigen::Matrix4f & matrix)
{
  geometry_msgs::msg::Pose pose;
  const Eigen::Quaternionf q(matrix.block<3, 3>(0, 0));
  pose.position.x = matrix(0, 3);
  pose.position.y = matrix(1, 3);
  pose.position.z = matrix(2, 3);
  pose.orientation.x = q.x();
  pose.orientation.y = q.y();
  pose.orientation.z = q.z();
  pose.orientation.w = q.w();
  return pose;
}

double rotationAngle(const Eigen::Matrix3f & rotation)
{
  const double cosine =
    std::clamp((static_cast<double>(rotation.trace()) - 1.0) * 0.5, -1.0, 1.0);
  return std::acos(cosine);
}

diagnostic_msgs::msg::KeyValue keyValue(const std::string & key, const std::string & value)
{
  diagnostic_msgs::msg::KeyValue item;
  item.key = key;
  item.value = value;
  return item;
}
}  // namespace

class PriorMapLocalizer : public rclcpp::Node
{
public:
  using CloudMsg = sensor_msgs::msg::PointCloud2;
  using OdomMsg = nav_msgs::msg::Odometry;
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<CloudMsg, OdomMsg>;

  PriorMapLocalizer()
  : Node("prior_map_localizer"),
    tf_broadcaster_(std::make_unique<tf2_ros::TransformBroadcaster>(*this))
  {
    map_path_ = declare_parameter<std::string>("map_path", "");
    cloud_topic_ = declare_parameter<std::string>("cloud_topic", "/cloud_registered");
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/Odometry");
    initial_pose_topic_ = declare_parameter<std::string>("initial_pose_topic", "/initialpose");
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    local_frame_ = declare_parameter<std::string>("local_frame", "camera_init");
    body_frame_ = declare_parameter<std::string>("body_frame", "body");
    navigation_frame_ = declare_parameter<std::string>("navigation_frame", body_frame_);
    const bool inverted = declare_parameter<bool>("lidar_inverted", false);
    if (inverted) {
      if (navigation_frame_ == body_frame_) {
        throw std::invalid_argument("An inverted lidar needs a distinct upright navigation_frame");
      }
      // Physical 180 degree roll: forward X stays forward; Y and Z reverse.
      // FAST-LIO's IMU/body frame and its estimated extrinsics stay untouched.
      body_to_navigation_(1, 1) = -1.0f;
      body_to_navigation_(2, 2) = -1.0f;
    }
    correction_rate_hz_ = declare_parameter<double>("correction_rate_hz", 0.5);
    sync_slop_sec_ = declare_parameter<double>("sync_slop_sec", 0.03);
    input_timeout_sec_ = declare_parameter<double>("input_timeout_sec", 0.5);
    coarse_leaf_size_ = declare_parameter<double>("coarse_leaf_size", 1.0);
    fine_scan_leaf_size_ = declare_parameter<double>("fine_scan_leaf_size", 0.15);
    fine_map_leaf_size_ = declare_parameter<double>("fine_map_leaf_size", 0.25);
    submap_radius_ = declare_parameter<double>("submap_radius", 30.0);
    submap_z_radius_ = declare_parameter<double>("submap_z_radius", 8.0);
    coarse_max_correspondence_ = declare_parameter<double>("coarse_max_correspondence", 3.0);
    fine_max_correspondence_ = declare_parameter<double>("fine_max_correspondence", 0.8);
    max_iterations_ = declare_parameter<int>("max_iterations", 40);
    min_scan_points_ = declare_parameter<int>("min_scan_points", 500);
    min_target_points_ = declare_parameter<int>("min_target_points", 1000);
    min_overlap_ = declare_parameter<double>("min_overlap", 0.35);
    max_inlier_rmse_ = declare_parameter<double>("max_inlier_rmse", 0.35);
    max_translation_jump_ = declare_parameter<double>("max_translation_jump", 2.0);
    max_rotation_jump_deg_ = declare_parameter<double>("max_rotation_jump_deg", 25.0);
    initial_heading_search_ = declare_parameter<bool>("initial_heading_search", false);
    required_consecutive_matches_ = declare_parameter<int>("required_consecutive_matches", 2);
    publish_prior_map_ = declare_parameter<bool>("publish_prior_map", true);

    ready_pub_ = create_publisher<std_msgs::msg::Bool>("/localization_ready", 10);
    diagnostics_pub_ =
      create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/localization_diagnostics", 10);
    state_pub_ = create_publisher<OdomMsg>("/state_estimation", 10);
    registered_scan_pub_ = create_publisher<CloudMsg>("/registered_scan", 10);
    prior_map_pub_ = create_publisher<CloudMsg>(
      "/prior_map_cloud", rclcpp::QoS(1).transient_local().reliable());

    input_callback_group_ =
      create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    localization_callback_group_ =
      create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    health_callback_group_ =
      create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions input_options;
    input_options.callback_group = input_callback_group_;
    initial_pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      initial_pose_topic_, rclcpp::QoS(10),
      std::bind(&PriorMapLocalizer::initialPoseCallback, this, std::placeholders::_1),
      input_options);

    cloud_sub_.subscribe(this, cloud_topic_, rmw_qos_profile_sensor_data, input_options);
    odom_sub_.subscribe(this, odom_topic_, rmw_qos_profile_sensor_data, input_options);
    sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
      SyncPolicy(50), cloud_sub_, odom_sub_);
    sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(sync_slop_sec_));
    sync_->registerCallback(
      std::bind(
        &PriorMapLocalizer::synchronizedCallback, this, std::placeholders::_1,
        std::placeholders::_2));

    loadMap();
    const auto localization_period = std::chrono::duration<double>(
      1.0 / std::max(0.05, correction_rate_hz_));
    localization_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(localization_period),
      std::bind(&PriorMapLocalizer::localizationTimer, this), localization_callback_group_);
    health_timer_ = create_wall_timer(
      std::chrono::milliseconds(100), std::bind(&PriorMapLocalizer::healthTimer, this),
      health_callback_group_);

    publishReady(false);
  }

private:
  struct Metrics
  {
    bool converged{false};
    std::size_t scan_points{0};
    std::size_t target_points{0};
    std::size_t inliers{0};
    double overlap{0.0};
    double rmse{std::numeric_limits<double>::infinity()};
    double translation_jump{0.0};
    double rotation_jump_deg{0.0};
    double initial_yaw_offset_deg{0.0};
    bool heading_search_evaluated{false};
    double heading_score{std::numeric_limits<double>::infinity()};
    double alternate_heading_score{std::numeric_limits<double>::infinity()};
  };

  CloudT::Ptr voxelize(const CloudT::ConstPtr & input, double leaf_size) const
  {
    auto output = std::make_shared<CloudT>();
    pcl::VoxelGrid<PointT> filter;
    filter.setInputCloud(input);
    const float leaf = static_cast<float>(leaf_size);
    filter.setLeafSize(leaf, leaf, leaf);
    filter.filter(*output);
    return output;
  }

  CloudT::Ptr cropMap(
    const CloudT::ConstPtr & input, const Eigen::Vector3f & center) const
  {
    auto output = std::make_shared<CloudT>();
    output->reserve(input->size() / 4);
    const float radius_sq = static_cast<float>(submap_radius_ * submap_radius_);
    const float z_radius = static_cast<float>(submap_z_radius_);
    for (const auto & point : input->points) {
      const float dx = point.x - center.x();
      const float dy = point.y - center.y();
      if (dx * dx + dy * dy <= radius_sq && std::abs(point.z - center.z()) <= z_radius) {
        output->push_back(point);
      }
    }
    output->width = static_cast<std::uint32_t>(output->size());
    output->height = 1;
    output->is_dense = true;
    return output;
  }

  void loadMap()
  {
    if (map_path_.empty()) {
      throw std::runtime_error("parameter 'map_path' is empty");
    }
    auto raw_map = std::make_shared<CloudT>();
    RCLCPP_INFO(get_logger(), "Loading prior map: %s", map_path_.c_str());
    if (pcl::io::loadPCDFile<PointT>(map_path_, *raw_map) < 0) {
      throw std::runtime_error("failed to load prior PCD: " + map_path_);
    }
    auto finite_map = std::make_shared<CloudT>();
    std::vector<int> kept_indices;
    pcl::removeNaNFromPointCloud(*raw_map, *finite_map, kept_indices);
    if (finite_map->empty()) {
      throw std::runtime_error("prior PCD contains no finite points: " + map_path_);
    }
    prior_map_ = voxelize(finite_map, fine_map_leaf_size_);
    map_loaded_ = true;
    RCLCPP_INFO(
      get_logger(), "Prior map loaded: %zu points, downsampled to %zu points",
      finite_map->size(), prior_map_->size());
    if (publish_prior_map_) {
      CloudMsg message;
      pcl::toROSMsg(*prior_map_, message);
      message.header.stamp = now();
      message.header.frame_id = map_frame_;
      prior_map_pub_->publish(message);
    }
  }

  void initialPoseCallback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr message)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (message->header.frame_id != map_frame_) {
      RCLCPP_ERROR(
        get_logger(), "Rejected /initialpose frame '%s'; expected '%s'",
        message->header.frame_id.c_str(), map_frame_.c_str());
      publishReadyLocked(false);
      status_reason_ = "initialpose_frame_mismatch";
      return;
    }
    if (!g1_navigation::finitePose(message->pose.pose)) {
      publishReadyLocked(false);
      initialized_ = false;
      has_valid_transform_ = false;
      pending_initial_pose_.reset();
      ++initialization_epoch_;
      status_reason_ = "invalid_initialpose";
      RCLCPP_ERROR(get_logger(), "Rejected non-finite or zero-quaternion initialpose");
      return;
    }
    pending_initial_pose_ = message;
    initializeFromPendingPoseLocked();
  }

  void initializeFromPendingPoseLocked()
  {
    if (!pending_initial_pose_) {
      return;
    }
    if (!latest_odom_) {
      status_reason_ = "waiting_for_initialpose_and_odometry";
      return;
    }
    // RViz initialpose describes the upright navigation origin, not the
    // upside-down IMU. Both origins coincide; only their orientation differs.
    const Eigen::Matrix4f map_to_body =
      poseToMatrix(pending_initial_pose_->pose.pose) * body_to_navigation_.inverse();
    const Eigen::Matrix4f local_to_body = poseToMatrix(latest_odom_->pose.pose);
    map_to_local_ = map_to_body * local_to_body.inverse();
    initial_heading_reference_ = map_to_local_;
    // Rotate the seed orientation around its own position, never around the
    // map origin. This changes registration, not the map or mounting rotation.
    Eigen::Matrix4f half_turn = Eigen::Matrix4f::Identity();
    half_turn(0, 0) = -1.0f;
    half_turn(1, 1) = -1.0f;
    const Eigen::Vector3f pivot = map_to_body.block<3, 1>(0, 3);
    half_turn.block<3, 1>(0, 3) = pivot - half_turn.block<3, 3>(0, 0) * pivot;
    initial_heading_opposite_ = half_turn * map_to_local_;
    heading_search_pending_ = initial_heading_search_;
    selected_initial_heading_ = -1;
    candidate_transform_ = map_to_local_;
    initialized_ = true;
    has_valid_transform_ = false;
    ++initialization_epoch_;
    consecutive_matches_ = 0;
    last_processed_sequence_ = 0;
    pending_initial_pose_.reset();
    publishReadyLocked(false);
    status_reason_ = "initialized_waiting_for_valid_icp";
    RCLCPP_INFO(
      get_logger(), "Initial pose accepted; waiting for %d consecutive valid ICP matches",
      required_consecutive_matches_);
  }

  void synchronizedCallback(
    const CloudMsg::ConstSharedPtr & cloud, const OdomMsg::ConstSharedPtr & odom)
  {
    if (cloud->header.frame_id != local_frame_ || odom->header.frame_id != local_frame_ ||
        odom->child_frame_id != body_frame_ || !g1_navigation::finitePose(odom->pose.pose)) {
      std::lock_guard<std::mutex> lock(data_mutex_);
      publishReadyLocked(false);
      consecutive_matches_ = 0;
      status_reason_ = "invalid_input_frame_or_pose";
      return;
    }
    const double cloud_stamp = rclcpp::Time(cloud->header.stamp).seconds();
    const double odom_stamp = rclcpp::Time(odom->header.stamp).seconds();
    const double delta = std::abs(cloud_stamp - odom_stamp);
    bool publish_tf = false;
    bool publish_outputs = false;
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      latest_cloud_ = cloud;
      latest_odom_ = odom;
      latest_sync_delta_ = delta;
      last_input_steady_ = std::chrono::steady_clock::now();
      have_received_input_ = true;
      ++input_sequence_;
      initializeFromPendingPoseLocked();
      publish_tf = has_valid_transform_;
      publish_outputs = localization_ready_;
      transform = map_to_local_;
    }
    if (publish_tf) {
      publishTransform(odom->header.stamp, transform);
    }
    if (publish_outputs) {
      publishGlobalOutputs(*cloud, *odom, transform);
    }
  }

  bool runGicp(
    const CloudT::ConstPtr & source, const CloudT::ConstPtr & target,
    double max_correspondence, const Eigen::Matrix4f & guess,
    Eigen::Matrix4f & result) const
  {
    pcl::GeneralizedIterativeClosestPoint<PointT, PointT> gicp;
    gicp.setInputSource(source);
    gicp.setInputTarget(target);
    gicp.setMaximumIterations(max_iterations_);
    gicp.setMaxCorrespondenceDistance(max_correspondence);
    gicp.setTransformationEpsilon(1.0e-4);
    gicp.setEuclideanFitnessEpsilon(1.0e-5);
    CloudT aligned;
    gicp.align(aligned, guess);
    if (!gicp.hasConverged()) {
      return false;
    }
    result = gicp.getFinalTransformation();
    return result.allFinite();
  }

  Metrics calculateMetrics(
    const CloudT::ConstPtr & aligned_scan, const CloudT::ConstPtr & target,
    const Eigen::Matrix4f & previous, const Eigen::Matrix4f & result) const
  {
    Metrics metrics;
    metrics.converged = true;
    metrics.scan_points = aligned_scan->size();
    metrics.target_points = target->size();
    pcl::KdTreeFLANN<PointT> tree;
    tree.setInputCloud(target);
    const double max_distance_sq = fine_max_correspondence_ * fine_max_correspondence_;
    double squared_error_sum = 0.0;
    std::vector<int> indices(1);
    std::vector<float> distances(1);
    for (const auto & point : aligned_scan->points) {
      if (tree.nearestKSearch(point, 1, indices, distances) == 1 &&
        distances[0] <= max_distance_sq)
      {
        ++metrics.inliers;
        squared_error_sum += distances[0];
      }
    }
    if (metrics.scan_points > 0) {
      metrics.overlap =
        static_cast<double>(metrics.inliers) / static_cast<double>(metrics.scan_points);
    }
    if (metrics.inliers > 0) {
      metrics.rmse = std::sqrt(squared_error_sum / static_cast<double>(metrics.inliers));
    }
    const Eigen::Matrix4f delta = previous.inverse() * result;
    metrics.translation_jump = delta.block<3, 1>(0, 3).norm();
    metrics.rotation_jump_deg =
      rotationAngle(delta.block<3, 3>(0, 0)) * 180.0 / 3.14159265358979323846;
    return metrics;
  }

  bool qualityValid(const Metrics & metrics) const
  {
    return metrics.converged &&
      metrics.scan_points >= static_cast<std::size_t>(min_scan_points_) &&
      metrics.target_points >= static_cast<std::size_t>(min_target_points_) &&
      metrics.overlap >= min_overlap_ && metrics.rmse <= max_inlier_rmse_ &&
      metrics.translation_jump <= max_translation_jump_ &&
      metrics.rotation_jump_deg <= max_rotation_jump_deg_;
  }

  // Penalize unmatched points as well as inlier error, so matching a small
  // subset of a symmetric room cannot win simply by reporting low inlier RMSE.
  double headingScore(const Metrics & metrics) const
  {
    if (!metrics.converged || metrics.inliers == 0) {
      return std::numeric_limits<double>::infinity();
    }
    return std::sqrt(metrics.overlap * metrics.rmse * metrics.rmse +
      (1.0 - metrics.overlap) * fine_max_correspondence_ * fine_max_correspondence_);
  }

  void localizationTimer()
  {
    std::unique_lock<std::mutex> run_lock(localization_mutex_, std::try_to_lock);
    if (!run_lock.owns_lock()) {
      return;
    }
    CloudMsg::ConstSharedPtr cloud_message;
    OdomMsg::ConstSharedPtr odom_message;
    Eigen::Matrix4f guess;
    Eigen::Matrix4f opposite_guess;
    bool search_heading = false;
    std::uint64_t sequence = 0;
    std::uint64_t epoch = 0;
    double sync_delta = 0.0;
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      if (!map_loaded_ || !initialized_ || !latest_cloud_ || !latest_odom_) {
        return;
      }
      if (input_sequence_ == last_processed_sequence_) {
        return;
      }
      cloud_message = latest_cloud_;
      odom_message = latest_odom_;
      guess = map_to_local_;
      search_heading = heading_search_pending_;
      if (search_heading) {
        guess = initial_heading_reference_;
        opposite_guess = initial_heading_opposite_;
      }
      sequence = input_sequence_;
      epoch = initialization_epoch_;
      sync_delta = latest_sync_delta_;
      last_processed_sequence_ = sequence;
    }

    Metrics metrics;
    if (sync_delta > sync_slop_sec_) {
      rejectMatch("sync_delta_exceeded", metrics, epoch);
      return;
    }
    if (cloud_message->header.frame_id != local_frame_ ||
      odom_message->header.frame_id != local_frame_ ||
      odom_message->child_frame_id != body_frame_)
    {
      std::ostringstream reason;
      reason << "frame_mismatch cloud=" << cloud_message->header.frame_id
             << " odom=" << odom_message->header.frame_id
             << " child=" << odom_message->child_frame_id;
      rejectMatch(reason.str(), metrics, epoch);
      return;
    }

    auto source_raw_unfiltered = std::make_shared<CloudT>();
    auto source_raw = std::make_shared<CloudT>();
    pcl::fromROSMsg(*cloud_message, *source_raw_unfiltered);
    std::vector<int> kept_indices;
    pcl::removeNaNFromPointCloud(*source_raw_unfiltered, *source_raw, kept_indices);
    auto source_fine = voxelize(source_raw, fine_scan_leaf_size_);
    metrics.scan_points = source_fine->size();
    if (source_fine->size() < static_cast<std::size_t>(min_scan_points_)) {
      rejectMatch("too_few_scan_points", metrics, epoch);
      return;
    }

    const Eigen::Matrix4f local_to_body = poseToMatrix(odom_message->pose.pose);
    const Eigen::Vector3f predicted_body = (guess * local_to_body).block<3, 1>(0, 3);
    auto target_crop = cropMap(prior_map_, predicted_body);
    auto target_fine = voxelize(target_crop, fine_map_leaf_size_);
    metrics.target_points = target_fine->size();
    if (target_fine->size() < static_cast<std::size_t>(min_target_points_)) {
      rejectMatch("too_few_prior_submap_points", metrics, epoch);
      return;
    }

    auto source_coarse = voxelize(source_raw, coarse_leaf_size_);
    auto target_coarse = voxelize(target_crop, coarse_leaf_size_);
    auto fit = [&](const Eigen::Matrix4f & seed, Eigen::Matrix4f & result,
        Metrics & measured) -> std::string {
        measured.scan_points = source_fine->size();
        measured.target_points = target_fine->size();
        Eigen::Matrix4f coarse_result = seed;
        if (!runGicp(source_coarse, target_coarse, coarse_max_correspondence_,
            seed, coarse_result)) return "coarse_gicp_not_converged";
        if (!runGicp(source_fine, target_fine, fine_max_correspondence_,
            coarse_result, result)) return "fine_gicp_not_converged";
        auto aligned = std::make_shared<CloudT>();
        pcl::transformPointCloud(*source_fine, *aligned, result);
        measured = calculateMetrics(aligned, target_fine, seed, result);
        return "";
      };

    Eigen::Matrix4f fine_result = guess;
    const std::string fit_error = fit(guess, fine_result, metrics);
    int heading_choice = 0;
    if (search_heading) {
      Eigen::Matrix4f opposite_result = opposite_guess;
      Metrics opposite_metrics;
      fit(opposite_guess, opposite_result, opposite_metrics);
      const double first_score = headingScore(metrics);
      const double second_score = headingScore(opposite_metrics);
      if (second_score < first_score) {
        heading_choice = 1;
        fine_result = opposite_result;
        metrics = opposite_metrics;
      }
      metrics.initial_yaw_offset_deg = heading_choice * 180.0;
      metrics.heading_search_evaluated = true;
      metrics.heading_score = std::min(first_score, second_score);
      metrics.alternate_heading_score = std::max(first_score, second_score);
      // This optional recovery is stricter than normal tracking. Never relax
      // its jump limits or accept a half-turn just because ICP converged.
      if (!qualityValid(metrics) || metrics.overlap < 0.90 || metrics.rmse > 0.20) {
        rejectMatch("initial_heading_quality_rejected", metrics, epoch);
        return;
      }
      if (metrics.alternate_heading_score - metrics.heading_score < 0.05) {
        rejectMatch("initial_heading_ambiguous", metrics, epoch);
        return;
      }
    } else {
      if (!fit_error.empty()) {
        rejectMatch(fit_error, metrics, epoch);
        return;
      }
      if (!qualityValid(metrics)) {
        rejectMatch("icp_quality_gate_rejected", metrics, epoch);
        return;
      }
    }

    bool ready_now = false;
    bool input_fresh = false;
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      if (epoch != initialization_epoch_) {
        return;
      }
      input_fresh = have_received_input_ &&
        std::chrono::duration<double>(
        std::chrono::steady_clock::now() - last_input_steady_).count() <= input_timeout_sec_;
      if (!input_fresh) {
        consecutive_matches_ = 0;
        publishReadyLocked(false);
        status_reason_ = "synchronized_input_stale_after_icp";
        last_metrics_ = metrics;
      } else {
      if (search_heading && selected_initial_heading_ != heading_choice) {
        consecutive_matches_ = 0;
        selected_initial_heading_ = heading_choice;
      }
      candidate_transform_ = fine_result;
      map_to_local_ = fine_result;
      ++consecutive_matches_;
      ready_now = consecutive_matches_ >= required_consecutive_matches_;
      has_valid_transform_ = !search_heading || ready_now;
      if (search_heading && ready_now) {
        heading_search_pending_ = false;
        RCLCPP_INFO(get_logger(),
          "Initial heading accepted: seed yaw offset %.0f deg, scores %.3f / %.3f",
          metrics.initial_yaw_offset_deg, metrics.heading_score, metrics.alternate_heading_score);
      }
      publishReadyLocked(ready_now);
      status_reason_ = ready_now ? "localized" : "waiting_for_consecutive_matches";
      last_metrics_ = metrics;
      }
    }
    if (!input_fresh) {
      publishDiagnostics(
        diagnostic_msgs::msg::DiagnosticStatus::ERROR,
        "synchronized_input_stale_after_icp", metrics);
      return;
    }
    const std::string accepted_reason =
      ready_now ? "localized" : "waiting_for_consecutive_matches";
    publishDiagnostics(ready_now ? diagnostic_msgs::msg::DiagnosticStatus::OK :
      diagnostic_msgs::msg::DiagnosticStatus::WARN, accepted_reason, metrics);
    // Publish accepted corrections on the next synchronized input only.
    // Publishing the ICP snapshot here races with fresh input and sends
    // older/duplicate timestamps to the downstream planner.
  }

  void rejectMatch(
    const std::string & reason, const Metrics & metrics, std::uint64_t expected_epoch)
  {
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      if (expected_epoch != initialization_epoch_) {
        return;
      }
      consecutive_matches_ = 0;
      publishReadyLocked(false);
      status_reason_ = reason;
      last_metrics_ = metrics;
    }
    publishDiagnostics(diagnostic_msgs::msg::DiagnosticStatus::ERROR, reason, metrics);
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 3000,
      "Localization rejected: %s (overlap %.3f, RMSE %.3f m, jump %.3f m / %.2f deg)",
      reason.c_str(), metrics.overlap, metrics.rmse, metrics.translation_jump,
      metrics.rotation_jump_deg);
  }

  void publishGlobalOutputs(
    const CloudMsg & cloud_message, const OdomMsg & odom_message,
    const Eigen::Matrix4f & transform)
  {
    auto local_cloud = std::make_shared<CloudT>();
    auto global_cloud = std::make_shared<CloudT>();
    pcl::fromROSMsg(cloud_message, *local_cloud);
    pcl::transformPointCloud(*local_cloud, *global_cloud, transform);
    CloudMsg output_cloud;
    pcl::toROSMsg(*global_cloud, output_cloud);
    output_cloud.header.stamp = cloud_message.header.stamp;
    output_cloud.header.frame_id = map_frame_;
    registered_scan_pub_->publish(output_cloud);

    const OdomMsg output_odom = g1_navigation::navigationOdometry(
      odom_message, transform, body_to_navigation_, map_frame_, navigation_frame_);
    state_pub_->publish(output_odom);
  }

  void publishTransform(
    const builtin_interfaces::msg::Time & stamp, const Eigen::Matrix4f & transform)
  {
    geometry_msgs::msg::TransformStamped message;
    message.header.stamp = stamp;
    message.header.frame_id = map_frame_;
    message.child_frame_id = local_frame_;
    const auto pose = matrixToPose(transform);
    message.transform.translation.x = pose.position.x;
    message.transform.translation.y = pose.position.y;
    message.transform.translation.z = pose.position.z;
    message.transform.rotation = pose.orientation;
    tf_broadcaster_->sendTransform(message);
  }

  void healthTimer()
  {
    bool ready = false;
    bool stale = false;
    bool report_stale = false;
    std::string reason;
    Metrics metrics;
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      stale = !have_received_input_ ||
        std::chrono::duration<double>(
        std::chrono::steady_clock::now() - last_input_steady_).count() > input_timeout_sec_;
      if (stale) {
        consecutive_matches_ = 0;
        publishReadyLocked(false);
        status_reason_ = "synchronized_input_stale";
        report_stale = has_valid_transform_ && !stale_reported_;
        stale_reported_ = true;
      } else {
        stale_reported_ = false;
      }
      ready = localization_ready_;
      reason = status_reason_;
      metrics = last_metrics_;
    }
    publishReady(ready);
    if (report_stale) {
      publishDiagnostics(diagnostic_msgs::msg::DiagnosticStatus::ERROR, reason, metrics);
    }
  }

  void publishReady(bool ready)
  {
    std_msgs::msg::Bool message;
    message.data = ready;
    ready_pub_->publish(message);
  }

  void publishReadyLocked(bool ready)
  {
    localization_ready_ = ready;
  }

  void publishDiagnostics(
    std::uint8_t level, const std::string & reason, const Metrics & metrics)
  {
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.level = level;
    status.name = "g1_fast_lio_bringup/prior_map_localizer";
    status.hardware_id = "prior_map";
    status.message = reason;
    status.values.push_back(keyValue("map_loaded", map_loaded_ ? "true" : "false"));
    status.values.push_back(keyValue("reason", reason));
    status.values.push_back(keyValue("scan_points", std::to_string(metrics.scan_points)));
    status.values.push_back(keyValue("target_points", std::to_string(metrics.target_points)));
    status.values.push_back(keyValue("inliers", std::to_string(metrics.inliers)));
    status.values.push_back(keyValue("overlap", std::to_string(metrics.overlap)));
    status.values.push_back(keyValue("inlier_rmse_m", std::to_string(metrics.rmse)));
    status.values.push_back(
      keyValue("translation_jump_m", std::to_string(metrics.translation_jump)));
    status.values.push_back(
      keyValue("rotation_jump_deg", std::to_string(metrics.rotation_jump_deg)));
    status.values.push_back(
      keyValue("initial_yaw_offset_deg", std::to_string(metrics.initial_yaw_offset_deg)));
    status.values.push_back(
      keyValue("heading_search_evaluated", metrics.heading_search_evaluated ? "true" : "false"));
    status.values.push_back(keyValue("heading_score_m", std::to_string(metrics.heading_score)));
    status.values.push_back(
      keyValue("alternate_heading_score_m", std::to_string(metrics.alternate_heading_score)));
    array.status.push_back(status);
    diagnostics_pub_->publish(array);
  }

  std::string map_path_;
  std::string cloud_topic_;
  std::string odom_topic_;
  std::string initial_pose_topic_;
  std::string map_frame_;
  std::string local_frame_;
  std::string body_frame_;
  std::string navigation_frame_;
  Eigen::Matrix4f body_to_navigation_ = Eigen::Matrix4f::Identity();
  double correction_rate_hz_;
  double sync_slop_sec_;
  double input_timeout_sec_;
  double coarse_leaf_size_;
  double fine_scan_leaf_size_;
  double fine_map_leaf_size_;
  double submap_radius_;
  double submap_z_radius_;
  double coarse_max_correspondence_;
  double fine_max_correspondence_;
  int max_iterations_;
  int min_scan_points_;
  int min_target_points_;
  double min_overlap_;
  double max_inlier_rmse_;
  double max_translation_jump_;
  double max_rotation_jump_deg_;
  bool initial_heading_search_;
  int required_consecutive_matches_;
  bool publish_prior_map_;

  std::mutex data_mutex_;
  std::mutex localization_mutex_;
  CloudMsg::ConstSharedPtr latest_cloud_;
  OdomMsg::ConstSharedPtr latest_odom_;
  geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr pending_initial_pose_;
  CloudT::Ptr prior_map_;
  Eigen::Matrix4f map_to_local_{Eigen::Matrix4f::Identity()};
  Eigen::Matrix4f candidate_transform_{Eigen::Matrix4f::Identity()};
  Eigen::Matrix4f initial_heading_reference_{Eigen::Matrix4f::Identity()};
  Eigen::Matrix4f initial_heading_opposite_{Eigen::Matrix4f::Identity()};
  bool heading_search_pending_{false};
  int selected_initial_heading_{-1};
  Metrics last_metrics_;
  std::chrono::steady_clock::time_point last_input_steady_{};
  std::uint64_t input_sequence_{0};
  std::uint64_t last_processed_sequence_{0};
  std::uint64_t initialization_epoch_{0};
  int consecutive_matches_{0};
  double latest_sync_delta_{0.0};
  bool map_loaded_{false};
  bool initialized_{false};
  bool has_valid_transform_{false};
  bool localization_ready_{false};
  bool have_received_input_{false};
  bool stale_reported_{false};
  std::string status_reason_{"waiting_for_initialpose_and_synchronized_input"};

  message_filters::Subscriber<CloudMsg> cloud_sub_;
  message_filters::Subscriber<OdomMsg> odom_sub_;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    initial_pose_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ready_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::Publisher<OdomMsg>::SharedPtr state_pub_;
  rclcpp::Publisher<CloudMsg>::SharedPtr registered_scan_pub_;
  rclcpp::Publisher<CloudMsg>::SharedPtr prior_map_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr localization_timer_;
  rclcpp::TimerBase::SharedPtr health_timer_;
  rclcpp::CallbackGroup::SharedPtr input_callback_group_;
  rclcpp::CallbackGroup::SharedPtr localization_callback_group_;
  rclcpp::CallbackGroup::SharedPtr health_callback_group_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<PriorMapLocalizer>();
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 3);
    executor.add_node(node);
    executor.spin();
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(rclcpp::get_logger("prior_map_localizer"), "%s", exception.what());
  }
  rclcpp::shutdown();
  return 0;
}
