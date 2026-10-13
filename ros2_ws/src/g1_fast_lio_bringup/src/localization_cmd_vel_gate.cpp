#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

class LocalizationCmdVelGate : public rclcpp::Node
{
public:
  LocalizationCmdVelGate()
  : Node("localization_cmd_vel_gate")
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/cmd_vel_nav");
    output_topic_ = declare_parameter<std::string>("output_topic", "/cmd_vel_safe");
    ready_topic_ = declare_parameter<std::string>("ready_topic", "/localization_ready");
    lio_health_topic_ = declare_parameter<std::string>("lio_health_topic", "/lio_health");
    require_lio_health_ = declare_parameter<bool>("require_lio_health", false);
    ready_timeout_sec_ = declare_parameter<double>("ready_timeout_sec", 0.5);
    lio_health_timeout_sec_ = declare_parameter<double>("lio_health_timeout_sec", 0.5);
    command_timeout_sec_ = declare_parameter<double>("command_timeout_sec", 0.25);
    max_linear_speed_ = declare_parameter<double>("max_linear_speed", 0.20);
    max_angular_speed_ = declare_parameter<double>("max_angular_speed", 0.35);

    command_pub_ = create_publisher<geometry_msgs::msg::Twist>(output_topic_, 10);
    command_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      input_topic_, rclcpp::QoS(10),
      std::bind(&LocalizationCmdVelGate::commandCallback, this, std::placeholders::_1));
    ready_sub_ = create_subscription<std_msgs::msg::Bool>(
      ready_topic_, rclcpp::QoS(10),
      std::bind(&LocalizationCmdVelGate::readyCallback, this, std::placeholders::_1));
    if (require_lio_health_) {
      lio_health_sub_ = create_subscription<std_msgs::msg::Bool>(
        lio_health_topic_, rclcpp::QoS(10),
        std::bind(&LocalizationCmdVelGate::lioHealthCallback, this, std::placeholders::_1));
    }
    watchdog_timer_ = create_wall_timer(
      std::chrono::milliseconds(50), std::bind(&LocalizationCmdVelGate::watchdog, this));
    RCLCPP_WARN(
      get_logger(),
      "Velocity gate starts CLOSED. Output is limited to %.2f m/s and %.2f rad/s.",
      max_linear_speed_, max_angular_speed_);
  }

private:
  using SteadyTime = std::chrono::steady_clock::time_point;

  bool isFresh(const SteadyTime & stamp, double timeout_sec) const
  {
    if (stamp == SteadyTime{}) {
      return false;
    }
    return std::chrono::duration<double>(
      std::chrono::steady_clock::now() - stamp).count() <= timeout_sec;
  }

  bool gateOpenLocked() const
  {
    const bool ready_ok =
      localization_ready_ && isFresh(last_ready_steady_, ready_timeout_sec_);
    const bool lio_ok = !require_lio_health_ ||
      (lio_healthy_ && isFresh(last_lio_health_steady_, lio_health_timeout_sec_));
    return ready_ok && lio_ok;
  }

  void readyCallback(const std_msgs::msg::Bool::SharedPtr message)
  {
    bool must_stop = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      localization_ready_ = message->data;
      last_ready_steady_ = std::chrono::steady_clock::now();
      must_stop = !gateOpenLocked() && command_active_;
      if (must_stop) {
        command_active_ = false;
      }
    }
    if (must_stop) {
      publishStop("localization_not_ready");
    }
  }

  void lioHealthCallback(const std_msgs::msg::Bool::SharedPtr message)
  {
    bool must_stop = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      lio_healthy_ = message->data;
      last_lio_health_steady_ = std::chrono::steady_clock::now();
      must_stop = !gateOpenLocked() && command_active_;
      if (must_stop) {
        command_active_ = false;
      }
    }
    if (must_stop) {
      publishStop("lio_not_healthy");
    }
  }

  void commandCallback(const geometry_msgs::msg::Twist::SharedPtr message)
  {
    geometry_msgs::msg::Twist output;
    bool open = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      open = gateOpenLocked();
      last_command_steady_ = std::chrono::steady_clock::now();
      command_active_ = open;
    }
    if (!open) {
      command_pub_->publish(output);
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Blocked navigation velocity: safety gate is closed");
      return;
    }
    output = *message;
    output.linear.x = std::clamp(output.linear.x, -max_linear_speed_, max_linear_speed_);
    output.linear.y = std::clamp(output.linear.y, -max_linear_speed_, max_linear_speed_);
    output.linear.z = 0.0;
    output.angular.x = 0.0;
    output.angular.y = 0.0;
    output.angular.z = std::clamp(output.angular.z, -max_angular_speed_, max_angular_speed_);
    command_pub_->publish(output);
  }

  void watchdog()
  {
    bool must_stop = false;
    std::string reason;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!command_active_) {
        return;
      }
      if (!gateOpenLocked()) {
        must_stop = true;
        reason = "health_timeout";
      } else if (!isFresh(last_command_steady_, command_timeout_sec_)) {
        must_stop = true;
        reason = "command_timeout";
      }
      if (must_stop) {
        command_active_ = false;
      }
    }
    if (must_stop) {
      publishStop(reason);
    }
  }

  void publishStop(const std::string & reason)
  {
    command_pub_->publish(geometry_msgs::msg::Twist());
    RCLCPP_WARN(get_logger(), "Published zero velocity: %s", reason.c_str());
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string ready_topic_;
  std::string lio_health_topic_;
  bool require_lio_health_{false};
  double ready_timeout_sec_;
  double lio_health_timeout_sec_;
  double command_timeout_sec_;
  double max_linear_speed_;
  double max_angular_speed_;

  mutable std::mutex mutex_;
  bool localization_ready_{false};
  bool lio_healthy_{false};
  bool command_active_{false};
  SteadyTime last_ready_steady_{};
  SteadyTime last_lio_health_steady_{};
  SteadyTime last_command_steady_{};

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr command_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr command_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr ready_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr lio_health_sub_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LocalizationCmdVelGate>());
  rclcpp::shutdown();
  return 0;
}
