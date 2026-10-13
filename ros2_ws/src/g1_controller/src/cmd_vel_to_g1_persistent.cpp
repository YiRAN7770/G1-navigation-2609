#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

#include <unitree/robot/g1/loco/g1_loco_client.hpp>
#include <unitree/robot/channel/channel_factory.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>

// Persistent ROS -> Unitree SDK2 bridge.  SDK calls are confined to one worker
// thread; the ROS callback only stores the newest command.  This avoids the
// multi-second std::system(g1_loco_client ...) stall that used to replay stale
// angular commands from PathFollower.
class CmdVelToG1 : public rclcpp::Node
{
public:
  CmdVelToG1()
  : Node("cmd_vel_to_g1")
  {
    network_interface_ = declare_parameter<std::string>("network_interface", "eno1");
    const int sdk_domain_id = declare_parameter<int>("sdk_domain_id", 0);
    if (sdk_domain_id < 0 || sdk_domain_id > 232) {
      throw std::invalid_argument("sdk_domain_id must be between 0 and 232");
    }
    // Retained for launch-file compatibility; no external process is started.
    g1_loco_client_path_ = declare_parameter<std::string>(
      "g1_loco_client_path", "");
    unitree_sdk_lib_path_ = declare_parameter<std::string>(
      "unitree_sdk_lib_path", "");
    command_timeout_sec_ = declare_parameter<double>("command_timeout", 0.20);
    command_period_sec_ = declare_parameter<double>("command_period", 0.10);
    command_duration_sec_ = declare_parameter<double>("command_duration", 0.25);
    // Disabled by default because this is a locomotion-mode API call. Enable
    // only after the operator has accepted the live FSM/step-transition test.
    enable_continuous_gait_ = declare_parameter<bool>("enable_continuous_gait", false);
    max_x_linear_vel_ = declare_parameter<double>("max_x_linear_velocity", 0.20);
    max_y_linear_vel_ = declare_parameter<double>("max_y_linear_velocity", 0.15);
    max_angular_vel_ = declare_parameter<double>("max_angular_velocity", 0.35);

    for (double value : {command_timeout_sec_, command_period_sec_, command_duration_sec_,
      max_x_linear_vel_, max_y_linear_vel_, max_angular_vel_}) {
      if (!std::isfinite(value) || value <= 0.0) {
        throw std::invalid_argument("Timing and velocity limits must be finite and positive");
      }
    }
    if (command_timeout_sec_ <= 0.0 || command_period_sec_ <= 0.0 ||
      command_duration_sec_ < command_period_sec_) {
      throw std::invalid_argument(
        "command_timeout/period must be positive and duration must be >= period");
    }

    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
      "cmd_vel", rclcpp::QoS(1),
      std::bind(&CmdVelToG1::cmdVelCallback, this, std::placeholders::_1));

    // The ROS Cyclone participant already owns the domain configuration.
    // Init(domain, interface) would explicitly create it a second time and
    // fail when ROS and SDK share domain 0. Empty Config reuses the domain;
    // the launch sets CYCLONEDDS_URI to the requested network interface.
    unitree::common::JsonMap sdk_config;
    sdk_config["DomainId"] = sdk_domain_id;
    unitree::robot::ChannelFactory::Instance()->Init(sdk_config);
    // LocoClient's constructor creates DDS channels. Construct it only after
    // ChannelFactory has a participant; an eager value member runs too early.
    loco_client_ = std::make_unique<unitree::robot::g1::LocoClient>();
    loco_client_->Init();
    loco_client_->SetTimeout(1.0f);

    worker_ = std::thread(&CmdVelToG1::workerLoop, this);
    RCLCPP_WARN(get_logger(),
      "Persistent SDK2 bridge ready on %s; period=%.3fs timeout=%.3fs duration=%.3fs "
      "limits=(%.2f, %.2f, %.2f); continuous_gait=%s; g1_loco_client_path is ignored",
      network_interface_.c_str(), command_period_sec_, command_timeout_sec_, command_duration_sec_,
      max_x_linear_vel_, max_y_linear_vel_, max_angular_vel_,
      enable_continuous_gait_ ? "true" : "false");
  }

  ~CmdVelToG1() override
  {
    stopping_.store(true);
    command_cv_.notify_one();
    if (worker_.joinable()) {
      worker_.join();
    }
    const int ret = loco_client_->StopMove();
    if (ret != 0) {
      RCLCPP_ERROR(get_logger(), "SDK2 StopMove() at shutdown failed: %d", ret);
    }
  }

private:
  using Clock = std::chrono::steady_clock;
  struct Command { double vx{0.0}; double vy{0.0}; double wz{0.0}; };

  static double clamp(double value, double max_abs)
  {
    return std::clamp(value, -max_abs, max_abs);
  }

  void cmdVelCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
  {
    const auto & t = msg->twist;
    if (!std::isfinite(t.linear.x) || !std::isfinite(t.linear.y) ||
      !std::isfinite(t.angular.z)) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
        "Ignoring non-finite cmd_vel");
      return;
    }
    Command command;
    command.vx = clamp(t.linear.x, max_x_linear_vel_);
    command.vy = clamp(t.linear.y, max_y_linear_vel_);
    command.wz = clamp(t.angular.z, max_angular_vel_);
    {
      std::lock_guard<std::mutex> lock(command_mutex_);
      latest_command_ = command;
      last_command_time_ = Clock::now();
      have_command_ = true;
    }
    command_cv_.notify_one();
  }

  void workerLoop()
  {
    bool stopped = true;
    while (!stopping_.load()) {
      Command command;
      bool valid = false;
      {
        std::unique_lock<std::mutex> lock(command_mutex_);
        command_cv_.wait_for(lock, std::chrono::duration<double>(command_period_sec_),
          [this]() { return stopping_.load(); });
        if (stopping_.load()) {
          break;
        }
        command = latest_command_;
        valid = have_command_ &&
          std::chrono::duration<double>(Clock::now() - last_command_time_).count() <=
          command_timeout_sec_;
      }

      if (!valid) {
        if (!stopped) {
          const int ret = loco_client_->StopMove();
          if (ret != 0) {
            RCLCPP_ERROR(get_logger(), "SDK2 StopMove() failed: %d", ret);
          }
          stopped = (ret == 0);
          gait_enabled_ = false;
        }
        continue;
      }

      const bool active = std::abs(command.vx) > 1e-6 || std::abs(command.vy) > 1e-6 ||
        std::abs(command.wz) > 1e-6;
      if (!active) {
        // Keep the existing zero-velocity SetVelocity behavior and re-arm the
        // gait transition on the next nonzero command.
        gait_enabled_ = false;
      } else if (enable_continuous_gait_ && !gait_enabled_) {
        // SetVelocity returning 0 only acknowledges the RPC. On this G1
        // firmware a low-speed straight command can remain in balance gait
        // after a stop, even though turning commands briefly produce steps.
        // Re-arm continuous gait at the zero -> nonzero edge without changing
        // velocity limits or bypassing the motion gate.
        const int gait_ret = loco_client_->ContinuousGait(true);
        if (gait_ret != 0) {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
            "SDK2 ContinuousGait(true) failed: %d; sending velocity unchanged", gait_ret);
        } else {
          RCLCPP_INFO(get_logger(), "SDK2 ContinuousGait(true) armed for nonzero command");
        }
        // Do not retry on every 100 ms tick if the firmware rejects the mode
        // request. A subsequent zero -> nonzero edge retries it.
        gait_enabled_ = true;
      }

      // A bounded-duration command makes the robot stop even if this process
      // is killed before its destructor can call StopMove().
      stopped = false;  // A failed RPC may still have reached the robot.
      const int ret = loco_client_->SetVelocity(
        static_cast<float>(command.vx), static_cast<float>(command.vy),
        static_cast<float>(command.wz), static_cast<float>(command_duration_sec_));
      if (ret != 0) {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
          "SDK2 Move(%.3f, %.3f, %.3f) failed: %d",
          command.vx, command.vy, command.wz, ret);
      } else {
        stopped = false;
      }
    }

    const int ret = loco_client_->StopMove();
    if (ret != 0) {
      RCLCPP_ERROR(get_logger(), "SDK2 StopMove() in worker failed: %d", ret);
    }
  }

  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_sub_;
  std::unique_ptr<unitree::robot::g1::LocoClient> loco_client_;
  std::string network_interface_;
  std::string g1_loco_client_path_;
  std::string unitree_sdk_lib_path_;
  double command_timeout_sec_{0.20};
  double command_period_sec_{0.10};
  double command_duration_sec_{0.25};
  bool enable_continuous_gait_{false};
  double max_x_linear_vel_{0.20};
  double max_y_linear_vel_{0.15};
  double max_angular_vel_{0.35};
  std::mutex command_mutex_;
  std::condition_variable command_cv_;
  Command latest_command_;
  Clock::time_point last_command_time_{};
  bool have_command_{false};
  bool gait_enabled_{false};
  std::atomic<bool> stopping_{false};
  std::thread worker_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<CmdVelToG1>());
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("cmd_vel_to_g1"), "Exception: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
