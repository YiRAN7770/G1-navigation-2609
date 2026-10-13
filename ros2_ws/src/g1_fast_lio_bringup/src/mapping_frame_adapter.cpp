#include <memory>
#include <string>
#include <stdexcept>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include "navigation_frames.hpp"

// Mapping only: FAST-LIO retains raw camera_init/body coordinates. All planners
// and the PCD/VGH save consumers receive the same upright map coordinates.
class MappingFrameAdapter : public rclcpp::Node
{
  using Cloud = sensor_msgs::msg::PointCloud2;
  using Odom = nav_msgs::msg::Odometry;
  using Policy = message_filters::sync_policies::ApproximateTime<Cloud, Odom>;
public:
  MappingFrameAdapter() : Node("mapping_frame_adapter")
  {
    const bool inverted = declare_parameter<bool>("lidar_inverted", false);
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    local_frame_ = declare_parameter<std::string>("local_frame", "camera_init");
    body_frame_ = declare_parameter<std::string>("body_frame", "body");
    navigation_frame_ = declare_parameter<std::string>("navigation_frame", inverted ? "sensor" : "body");
    if (map_frame_ == local_frame_ || (inverted && navigation_frame_ == body_frame_)) {
      throw std::invalid_argument("Mapping requires distinct world frames and an upright child for inverted mounting");
    }
    map_from_local_ = g1_navigation::mountingRotation(inverted);
    body_from_navigation_ = g1_navigation::mountingRotation(inverted);
    slop_ = declare_parameter<double>("sync_slop_sec", 0.03);
    cloud_pub_ = create_publisher<Cloud>("/registered_scan", 10);
    odom_pub_ = create_publisher<Odom>("/state_estimation", 10);
    tf_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);
    geometry_msgs::msg::TransformStamped tr;
    tr.header.stamp = now(); tr.header.frame_id = map_frame_; tr.child_frame_id = local_frame_;
    tr.transform.rotation = g1_navigation::matrixToPose(map_from_local_).orientation;
    tf_->sendTransform(tr);
    cloud_sub_.subscribe(this, "/cloud_registered", rmw_qos_profile_sensor_data);
    odom_sub_.subscribe(this, "/Odometry", rmw_qos_profile_sensor_data);
    sync_ = std::make_shared<message_filters::Synchronizer<Policy>>(Policy(20), cloud_sub_, odom_sub_);
    sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(slop_));
    sync_->registerCallback(std::bind(&MappingFrameAdapter::receive, this,
      std::placeholders::_1, std::placeholders::_2));
    RCLCPP_INFO(get_logger(), "Mapping coordinates: %s -> %s, inverted=%s, navigation=%s; raw FAST-LIO unchanged",
      local_frame_.c_str(), map_frame_.c_str(), inverted ? "true" : "false", navigation_frame_.c_str());
  }
private:
  void receive(const Cloud::ConstSharedPtr & cloud, const Odom::ConstSharedPtr & odom)
  {
    const auto stamp = rclcpp::Time(odom->header.stamp).nanoseconds();
    if (cloud->header.frame_id != local_frame_ || odom->header.frame_id != local_frame_ ||
        odom->child_frame_id != body_frame_ || !g1_navigation::finitePose(odom->pose.pose) ||
        stamp <= last_stamp_ || std::abs((rclcpp::Time(cloud->header.stamp) -
          rclcpp::Time(odom->header.stamp)).seconds()) > slop_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Rejected mapping input: frame, pose or timestamp mismatch");
      return;
    }
    pcl::PointCloud<pcl::PointXYZI> input, output;
    pcl::fromROSMsg(*cloud, input);
    if (input.empty()) return;
    pcl::transformPointCloud(input, output, map_from_local_);
    Cloud message;
    pcl::toROSMsg(output, message);
    message.header = cloud->header; message.header.frame_id = map_frame_;
    const auto state = g1_navigation::navigationOdometry(
      *odom, map_from_local_, body_from_navigation_, map_frame_, navigation_frame_);
    last_stamp_ = stamp;
    odom_pub_->publish(state);
    cloud_pub_->publish(message);
  }
  std::string map_frame_, local_frame_, body_frame_, navigation_frame_;
  double slop_;
  int64_t last_stamp_{0};
  Eigen::Matrix4f map_from_local_, body_from_navigation_;
  rclcpp::Publisher<Cloud>::SharedPtr cloud_pub_;
  rclcpp::Publisher<Odom>::SharedPtr odom_pub_;
  message_filters::Subscriber<Cloud> cloud_sub_;
  message_filters::Subscriber<Odom> odom_sub_;
  std::shared_ptr<message_filters::Synchronizer<Policy>> sync_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> tf_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MappingFrameAdapter>());
  rclcpp::shutdown();
  return 0;
}
