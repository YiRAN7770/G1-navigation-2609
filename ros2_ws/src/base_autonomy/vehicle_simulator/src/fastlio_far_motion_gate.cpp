#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

// Safety gate used only by system_real_robot_with_fastlio_far.launch.py.
// It deliberately publishes to a private topic; the existing /cmd_vel chain
// used by other navigation launches is not changed.
class FastlioFarMotionGate : public rclcpp::Node
{
public:
  using SteadyTime = std::chrono::steady_clock::time_point;

  FastlioFarMotionGate()
  : Node("fastlio_far_motion_gate")
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/cmd_vel");
    output_topic_ = declare_parameter<std::string>("output_topic", "/cmd_vel_fastlio_far_safe");
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/state_estimation");
    cloud_topic_ = declare_parameter<std::string>("cloud_topic", "/registered_scan");
    path_topic_ = declare_parameter<std::string>("path_topic", "/path");
    ready_topic_ = declare_parameter<std::string>("ready_topic", "/localization_ready");
    enable_motion_ = declare_parameter<bool>("enable_motion", false);
    require_localization_ready_ = declare_parameter<bool>("require_localization_ready", false);
    sensor_timeout_sec_ = declare_parameter<double>("sensor_timeout_sec", 0.5);
    ready_timeout_sec_ = declare_parameter<double>("ready_timeout_sec", 0.5);
    command_timeout_sec_ = declare_parameter<double>("command_timeout_sec", 0.25);
    max_x_speed_ = declare_parameter<double>("max_x_speed", 0.20);
    max_y_speed_ = declare_parameter<double>("max_y_speed", 0.20);
    max_angular_speed_ = declare_parameter<double>("max_angular_speed", 0.35);

    command_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(output_topic_, 10);
    command_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
      input_topic_, rclcpp::QoS(1),
      std::bind(&FastlioFarMotionGate::commandCallback, this, std::placeholders::_1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::QoS(10),
      [this](const nav_msgs::msg::Odometry::SharedPtr message) {
        if (!stampFresh(message->header.stamp, sensor_timeout_sec_)) return;
        const auto & p = message->pose.pose.position;
        const auto & q = message->pose.pose.orientation;
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
          !std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z) ||
          !std::isfinite(q.w)) {
          return;
        }
        std::lock_guard<std::mutex> lock(mutex_);
        last_odom_ = nowSteady();
      });
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_topic_, rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::PointCloud2::SharedPtr message) {
        if (!stampFresh(message->header.stamp, sensor_timeout_sec_)) return;
        if (message->data.empty() || message->width == 0 || message->height == 0) {
          return;
        }
        std::lock_guard<std::mutex> lock(mutex_);
        last_cloud_ = nowSteady();
      });
    ready_sub_ = create_subscription<std_msgs::msg::Bool>(
      ready_topic_, rclcpp::QoS(10),
      [this](const std_msgs::msg::Bool::SharedPtr message) {
        std::lock_guard<std::mutex> lock(mutex_);
        localization_ready_ = message->data;
        last_ready_ = nowSteady();
      });
    stop_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/fastlio_far/stop", rclcpp::QoS(1).reliable(),
      [this](const std_msgs::msg::Bool::SharedPtr message) {
        {
          std::lock_guard<std::mutex> lock(mutex_);
          manual_stop_ = message->data;
        }
        if (message->data) {
          publishStop("manual_stop");
          RCLCPP_ERROR(get_logger(), "Motion latched STOP by /fastlio_far/stop");
        } else {
          RCLCPP_WARN(get_logger(), "Manual stop latch cleared; health checks still apply");
        }
      });
    ready_pub_ = create_publisher<std_msgs::msg::Bool>("/fastlio_far/motion_ready", 10);
    status_pub_ = create_publisher<std_msgs::msg::String>("/fastlio_far/motion_status", 10);
    watchdog_timer_ = create_wall_timer(
      std::chrono::milliseconds(50), std::bind(&FastlioFarMotionGate::watchdog, this));

    rclcpp::on_shutdown([this]() { publishStop("node_shutdown"); });
    RCLCPP_WARN(
      get_logger(),
      "FAST-LIO/FAR motion gate starts %s; output=%s, limits=(%.2f, %.2f, %.2f)",
      enable_motion_ ? "OPEN only after health checks" : "CLOSED (enable_motion=false)",
      output_topic_.c_str(), max_x_speed_, max_y_speed_, max_angular_speed_);
  }

private:
  SteadyTime nowSteady() const { return std::chrono::steady_clock::now(); }

  bool fresh(const SteadyTime & stamp, double timeout) const
  {
    if (stamp == SteadyTime{}) {
      return false;
    }
    return std::chrono::duration<double>(nowSteady() - stamp).count() <= timeout;
  }

  bool stampFresh(const builtin_interfaces::msg::Time & stamp, double timeout) const
  {
    const double age = (now() - rclcpp::Time(stamp, get_clock()->get_clock_type())).seconds();
    return (stamp.sec != 0 || stamp.nanosec != 0) && age >= -0.1 && age <= timeout;
  }

  bool healthOk() const
  {
    if (!enable_motion_ || manual_stop_ || !sources_unique_ || !fresh(last_odom_, sensor_timeout_sec_) ||
      !fresh(last_cloud_, sensor_timeout_sec_)) {
      return false;
    }
    if (require_localization_ready_ &&
      (!localization_ready_ || !fresh(last_ready_, ready_timeout_sec_))) {
      return false;
    }
    return true;
  }

  std::string statusReason() const
  {
    if (!enable_motion_) {
      return "disabled";
    }
    if (manual_stop_) {
      return "manual_stop";
    }
    if (!sources_unique_) {
      return "missing_or_duplicate_publishers";
    }
    if (!fresh(last_odom_, sensor_timeout_sec_)) {
      return "odometry_stale";
    }
    if (!fresh(last_cloud_, sensor_timeout_sec_)) {
      return "registered_scan_stale";
    }
    if (require_localization_ready_ && !localization_ready_) {
      return "localization_not_ready";
    }
    if (require_localization_ready_ && !fresh(last_ready_, ready_timeout_sec_)) {
      return "localization_ready_stale";
    }
    return "ready";
  }

  void commandCallback(const geometry_msgs::msg::TwistStamped::SharedPtr message)
  {
    if (!stampFresh(message->header.stamp, command_timeout_sec_)) {
      rejectCommand("command_stamp_stale");
      return;
    }
    const auto & input = message->twist;
    if (!std::isfinite(input.linear.x) || !std::isfinite(input.linear.y) ||
      !std::isfinite(input.angular.z)) {
      rejectCommand("non_finite_command");
      return;
    }
    geometry_msgs::msg::TwistStamped output;
    output.header.stamp = now();
    output.header.frame_id = message->header.frame_id.empty() ? "vehicle" : message->header.frame_id;
    bool open;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      command_issue_.clear();
      last_command_ = nowSteady();
      open = healthOk();
    }
    if (!open) {
      command_pub_->publish(output);
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Blocked velocity: FAST-LIO/FAR safety gate is closed");
      return;
    }

    output.twist.linear.x = std::clamp(message->twist.linear.x, -max_x_speed_, max_x_speed_);
    output.twist.linear.y = std::clamp(message->twist.linear.y, -max_y_speed_, max_y_speed_);
    output.twist.linear.z = 0.0;
    output.twist.angular.x = 0.0;
    output.twist.angular.y = 0.0;
    output.twist.angular.z = std::clamp(message->twist.angular.z, -max_angular_speed_, max_angular_speed_);
    command_pub_->publish(output);
  }

  void watchdog()
  {
    bool stop = false;
    bool ready = false;
    std::string reason;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      sources_unique_ = count_publishers(odom_topic_) == 1 &&
        count_publishers(cloud_topic_) == 1 && count_publishers(path_topic_) == 1 &&
        count_publishers(input_topic_) == 1 && count_publishers(output_topic_) == 1;
      ready = healthOk();
      reason = statusReason();
      if (!ready) {
        stop = true;
      } else if (!command_issue_.empty()) {
        ready = false;
        stop = true;
        reason = command_issue_;
      } else if (!fresh(last_command_, command_timeout_sec_)) {
        ready = false;
        stop = true;
        reason = "command_timeout";
      }
    }
    std_msgs::msg::Bool ready_message;
    ready_message.data = ready;
    ready_pub_->publish(ready_message);
    std_msgs::msg::String status_message;
    status_message.data = reason;
    status_pub_->publish(status_message);
    if (stop) {
      publishStop(reason);
    }
  }

  void rejectCommand(const std::string & reason)
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      command_issue_ = reason;
    }
    // Preserve the command rejection until a valid command arrives. Previously
    // the watchdog could advertise "ready" while this callback emitted zeros.
    std_msgs::msg::Bool ready;
    ready.data = false;
    ready_pub_->publish(ready);
    std_msgs::msg::String status;
    status.data = reason;
    status_pub_->publish(status);
    publishStop(reason);
  }

  void publishStop(const std::string & reason)
  {
    geometry_msgs::msg::TwistStamped stop;
    stop.header.stamp = now();
    stop.header.frame_id = "vehicle";
    command_pub_->publish(stop);
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Published zero velocity: %s", reason.c_str());
  }

  bool sources_unique_{false};
  std::string command_issue_;
  std::string path_topic_;
  std::string input_topic_, output_topic_, odom_topic_, cloud_topic_, ready_topic_;
  bool enable_motion_{false};
  bool require_localization_ready_{false};
  double sensor_timeout_sec_{0.5};
  double ready_timeout_sec_{0.5};
  double command_timeout_sec_{0.25};
  double max_x_speed_{0.20};
  double max_y_speed_{0.20};
  double max_angular_speed_{0.35};

  mutable std::mutex mutex_;
  bool localization_ready_{false};
  bool manual_stop_{false};
  SteadyTime last_odom_{};
  SteadyTime last_cloud_{};
  SteadyTime last_ready_{};
  SteadyTime last_command_{};

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr command_pub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr command_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr ready_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr stop_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ready_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FastlioFarMotionGate>());
  rclcpp::shutdown();
  return 0;
}
