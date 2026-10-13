#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <chrono>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <csignal>
#include <atomic>
#include <sstream>
#include <iomanip>
#include <deque>
#include <nlohmann/json.hpp>
#include <stdexcept>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/clock.hpp"
#include "builtin_interfaces/msg/time.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

#include "tf2/transform_datatypes.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "rmw/types.h"
#include "rmw/qos_profiles.h"

#include "serial/serial.h"

using namespace std;

const double PI = 3.1415926;
using SteadyClock = std::chrono::steady_clock;

double normalizeAngle(double angle) {
  return atan2(sin(angle), cos(angle));
}

bool realRobot = false;
string serialPort = "/dev/ttyACM0";
int baudrate = 115200;
double sensorOffsetX = 0;
double sensorOffsetY = 0;
int pubSkipNum = 1;
int pubSkipCount = 0;
bool twoWayDrive = true;
double reverseMaxGoalDistance = 0.2;
double lookAheadDis = 0.5;
double yawRateGain = 7.5;
double stopYawRateGain = 7.5;
double maxYawRate = 45.0;
double maxSpeed = 1.0;
double maxAccel = 1.0;
double switchTimeThre = 1.0;
double dirDiffThre = 0.1;
double omniDirGoalThre = 1.0;
double omniDirDiffThre = 1.5;
double stopDisThre = 0.2;
double slowDwnDisThre = 1.0;
bool useInclRateToSlow = false;
double inclRateThre = 120.0;
double slowRate1 = 0.25;
double slowRate2 = 0.5;
double slowRate3 = 0.75;
double slowTime1 = 2.0;
double slowTime2 = 2.0;
bool useInclToStop = false;
double inclThre = 45.0;
double stopTime = 5.0;
bool noRotAtStop = false;
bool noRotAtGoal = true;
bool manualMode = false;
bool autonomyMode = false;
double autonomySpeed = 1.0;
double joyToSpeedDelay = 2.0;

double goalYaw = 0;
bool hasGoalYaw = false;
bool useGoalYaw = true;
double goalYawGain = 2.0;

// A PoseStamped goal carries the final yaw that FAR deliberately drops when
// it converts /goal_pose to its position-only /way_point interface.  Keep
// this small state machine in pathFollower so there is still one velocity
// publisher and one final-arrival decision.
bool useFinalGoalPose = false;
bool goalRequireLocalizationReady = false;
bool goalUseRawOdometry = false;
struct MotionSample { double time, x, y, yaw; };
std::deque<MotionSample> rawMotionSamples;
bool rawMotionValid = false;
double rawMotionLinearSpeed = 0, rawMotionAngularSpeed = 0;
SteadyClock::time_point lastRawMotionReceipt;

double finalGoalX = 0.0;
double finalGoalY = 0.0;
double finalGoalYaw = 0.0;
double goalPositionTolerance = 0.10;
double goalAlignmentEnterDistance = 0.10;  // dedicated G1 launch reserves measured noise margin
double goalAlignmentExitDistance = 0.15;
int goalAlignmentExitFrames = 3;
double goalYawControlTolerance = 0.3490658503988659;
double goalYawTolerance = 0.3490658503988659;  // exactly 20 degrees
int goalConfirmFrames = 3;
double goalStoppedLinearSpeed = 0.05;
double goalStoppedAngularSpeed = 0.05;
double finalApproachRange = 0.6;
double finalApproachMaxSpeed = 0.15;
geometry_msgs::msg::PoseStamped originalGoal;
bool plannerRotationClear = false, plannerMatchesGoal = false;
SteadyClock::time_point lastPlannerStatus, lastReplanRequest;
rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr replanPub;
SteadyClock::time_point outsideToleranceSince;
bool finalGoalActive = false;
bool positionOnlyGoalActive = false;
bool finalGoalValid = false;
bool finalGoalReached = false;
int goalConfirmCount = 0;
enum class FinalGoalPhase { Approach, Align, Reached };
FinalGoalPhase finalGoalPhase = FinalGoalPhase::Approach;
int goalExitCount = 0;
bool goalWaitingForPath = false;
uint64_t pathSequence = 0, goalExitPathSequence = 0;
int64_t lastPathStamp = 0, goalExitOdomStamp = 0;
bool lastPathValid = false;
SteadyClock::time_point lastPathReceipt;
constexpr double goalRecoveryPathTimeout = 0.5;
std::string trackingReason = "waiting_for_initial_path";
double trackingDistance = 0.0;
double trackingEndDistance = 0.0;
double trackingStopDistance = 0.0;
geometry_msgs::msg::Twist lastGoalCommand;

bool localizationReady = false;
bool localizationReadyReceived = false;
bool fastlioFarStop = false;
bool odomValid = false;
double odomLinearSpeed = 0.0;
double odomAngularSpeed = 0.0;
uint64_t odomSequence = 0;
int64_t odomStampNanosec = 0;
uint64_t lastGoalEvalOdomSequence = 0;
int64_t lastGoalEvalOdomStampNanosec = 0;
SteadyClock::time_point lastOdomReceipt;
SteadyClock::time_point lastLocalizationReadyReceipt;
constexpr double goalLocalizationTimeout = 0.5;

float joySpeed = 0;
float joySpeedRaw = 0;
float joyYaw = 0;
float joyManualFwd = 0;
float joyManualLeft = 0;
float joyManualYaw = 0;
int safetyStop = 0;
int slowDown = 0;

float vehicleX = 0;
float vehicleY = 0;
float vehicleZ = 0;
float vehicleRoll = 0;
float vehiclePitch = 0;
float vehicleYaw = 0;

float vehicleXRec = 0;
float vehicleYRec = 0;
float vehicleZRec = 0;
float vehicleRollRec = 0;
float vehiclePitchRec = 0;
float vehicleYawRec = 0;

float vehicleYawRate = 0;
float vehicleSpeed = 0;

double odomTime = 0;
double joyTime = 0;
double slowInitTime = 0;
double stopInitTime = false;
int pathPointID = 0;
bool pathInit = false;
bool navFwd = true;
double switchTime = 0;
bool serialOpen = false;

nav_msgs::msg::Path path;
rclcpp::Node::SharedPtr nh;
rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr goalReachedPub;
rclcpp::Publisher<std_msgs::msg::String>::SharedPtr goalStatusPub;
std::atomic_bool stopRequested{false};

bool reverseAllowed()
{
  // Use the original PoseStamped inspection goal, never a FAR waypoint or
  // the cropped local path endpoint. Legacy/manual modes retain their policy.
  return twoWayDrive && (!autonomyMode || !useFinalGoalPose ||
    (finalGoalActive && finalGoalValid && odomValid &&
     hypot(finalGoalX - vehicleX, finalGoalY - vehicleY) <= reverseMaxGoalDistance));
}

void enforceReverseLimit(geometry_msgs::msg::TwistStamped & command)
{
  if (autonomyMode && useFinalGoalPose && !reverseAllowed() && command.twist.linear.x < 0) {
    // Also covers residual negative acceleration and holonomic projection.
    command.twist.linear.x = command.twist.linear.y = 0;
    vehicleSpeed = 0;
  }
}

void signalHandler(int)
{
  stopRequested.store(true);
}

double clampValue(double value, double lower, double upper)
{
  return value < lower ? lower : (value > upper ? upper : value);
}

bool isFresh(const SteadyClock::time_point & receipt, double timeout)
{
  if (receipt == SteadyClock::time_point{}) return false;
  const double age = std::chrono::duration<double>(SteadyClock::now() - receipt).count();
  return age >= 0.0 && age <= timeout;
}

void publishGoalReached(bool reached)
{
  if (goalReachedPub) {
    std_msgs::msg::Bool message;
    message.data = reached;
    goalReachedPub->publish(message);
  }
}

void resetFinalGoalState(bool active)
{
  finalGoalActive = active;
  finalGoalValid = false;
  finalGoalReached = false;
  goalConfirmCount = 0;
  goalExitCount = 0;
  finalGoalPhase = FinalGoalPhase::Approach;
  goalWaitingForPath = false;
  // Confirmation must use frames received after this goal, not the last frame
  // retained from the previous target.
  lastGoalEvalOdomSequence = odomSequence;
  lastGoalEvalOdomStampNanosec = odomStampNanosec;
  publishGoalReached(false);
}

bool finalGoalLocalizationValid()
{
  if (!odomValid || !isFresh(lastOdomReceipt, goalLocalizationTimeout)) return false;
  const double odomAge = (nh->now().nanoseconds()-odomStampNanosec)*1e-9;
  if (odomAge < -0.1 || odomAge > goalLocalizationTimeout) return false;
  if (fastlioFarStop) return false;
  if (goalRequireLocalizationReady &&
      (!localizationReadyReceived || !localizationReady ||
       !isFresh(lastLocalizationReadyReceipt, goalLocalizationTimeout))) {
    return false;
  }
  return true;
}

bool isMapFrame(const std::string & frame)
{
  return frame == "map" || frame == "/map";
}

void odomHandler(const nav_msgs::msg::Odometry::ConstSharedPtr odomIn)
{
  const auto & p = odomIn->pose.pose.position;
  const auto & q = odomIn->pose.pose.orientation;
  const double qNorm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
  if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
      !std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z) ||
      !std::isfinite(q.w) || qNorm < 1e-12) {
    odomValid = false;
    return;
  }
  odomTime = rclcpp::Time(odomIn->header.stamp).seconds();
  double roll, pitch, yaw;
  geometry_msgs::msg::Quaternion geoQuat = odomIn->pose.pose.orientation;
  tf2::Matrix3x3(tf2::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w)).getRPY(roll, pitch, yaw);

  vehicleRoll = roll;
  vehiclePitch = pitch;
  vehicleYaw = yaw;
  vehicleX = odomIn->pose.pose.position.x - cos(yaw) * sensorOffsetX + sin(yaw) * sensorOffsetY;
  vehicleY = odomIn->pose.pose.position.y - sin(yaw) * sensorOffsetX - cos(yaw) * sensorOffsetY;
  vehicleZ = odomIn->pose.pose.position.z;
  odomLinearSpeed = hypot(odomIn->twist.twist.linear.x, odomIn->twist.twist.linear.y);
  odomAngularSpeed = fabs(odomIn->twist.twist.angular.z);
  odomValid = std::isfinite(odomLinearSpeed) && std::isfinite(odomAngularSpeed);
  if (!odomValid) return;
  ++odomSequence;
  odomStampNanosec = rclcpp::Time(odomIn->header.stamp).nanoseconds();
  lastOdomReceipt = SteadyClock::now();

  if ((fabs(roll) > inclThre * PI / 180.0 || fabs(pitch) > inclThre * PI / 180.0) && useInclToStop) {
    stopInitTime = rclcpp::Time(odomIn->header.stamp).seconds();
  }

  if ((fabs(odomIn->twist.twist.angular.x) > inclRateThre * PI / 180.0 || fabs(odomIn->twist.twist.angular.y) > inclRateThre * PI / 180.0) && useInclRateToSlow) {
    slowInitTime = rclcpp::Time(odomIn->header.stamp).seconds();
  }
}

void rawMotionHandler(const nav_msgs::msg::Odometry::ConstSharedPtr odom)
{
  const auto& p=odom->pose.pose.position; const auto& q=odom->pose.pose.orientation;
  const double t=rclcpp::Time(odom->header.stamp).seconds();
  const double age=nh->now().seconds()-t;
  const double norm=q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w;
  if (!std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(norm)||norm<1e-12 || age < -0.1 || age > 0.5) {
    rawMotionValid=false; return;
  }
  const double yaw=atan2(2*(q.w*q.z+q.x*q.y)/norm,1-2*(q.y*q.y+q.z*q.z)/norm);
  if (!rawMotionSamples.empty() && t<=rawMotionSamples.back().time) return;
  rawMotionSamples.push_back({t,p.x,p.y,yaw});
  while (rawMotionSamples.size()>2 && t-rawMotionSamples[1].time>=0.5) rawMotionSamples.pop_front();
  const auto& first=rawMotionSamples.front(); const double dt=t-first.time;
  rawMotionValid=dt>=0.4 && dt<=0.8;
  if (rawMotionValid) {
    rawMotionLinearSpeed=hypot(p.x-first.x,p.y-first.y)/dt;
    rawMotionAngularSpeed=fabs(normalizeAngle(yaw-first.yaw))/dt;
  }
  lastRawMotionReceipt=SteadyClock::now();
}

void goalPoseHandler(const geometry_msgs::msg::PoseStamped::ConstSharedPtr goal)
{
  if (!useFinalGoalPose) return;

  const auto & p = goal->pose.position;
  const auto & q = goal->pose.orientation;
  const double qNorm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
  const bool valid = isMapFrame(goal->header.frame_id) &&
    std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z) &&
    std::isfinite(q.x) && std::isfinite(q.y) && std::isfinite(q.z) &&
    std::isfinite(q.w) && qNorm >= 1e-12;

  positionOnlyGoalActive = false;
  resetFinalGoalState(true);
  if (!valid) {
    RCLCPP_ERROR(nh->get_logger(),
      "Rejected /goal_pose: expected finite pose in frame 'map' with a non-zero quaternion");
    return;
  }

  tf2::Quaternion quaternion(q.x, q.y, q.z, q.w);
  quaternion.normalize();
  double roll, pitch, yaw;
  tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);
  originalGoal = *goal;
  plannerMatchesGoal = false;
  outsideToleranceSince = {};
  finalGoalX = p.x;
  finalGoalY = p.y;
  finalGoalYaw = normalizeAngle(yaw);
  finalGoalValid = std::isfinite(finalGoalYaw);
  if (finalGoalValid) {
    // Keep the legacy fields coherent for non-FAR diagnostics; final-goal
    // control below uses the dedicated state and is independent of /path.
    goalYaw = finalGoalYaw;
    hasGoalYaw = true;
    RCLCPP_INFO(nh->get_logger(),
      "Accepted /goal_pose at (%.3f, %.3f), yaw %.3f rad; final alignment enabled",
      finalGoalX, finalGoalY, finalGoalYaw);
  }
}

void goalPointHandler(const geometry_msgs::msg::PointStamped::ConstSharedPtr goal)
{
  (void)goal;
  // A position-only goal is the legacy FAR contract.  It must cancel any
  // previously stored PoseStamped yaw so an old target cannot be confirmed.
  if (useFinalGoalPose) { resetFinalGoalState(false); positionOnlyGoalActive = true; }
  hasGoalYaw = false;
}

void localizationReadyHandler(const std_msgs::msg::Bool::ConstSharedPtr ready)
{
  localizationReady = ready->data;
  localizationReadyReceived = true;
  lastLocalizationReadyReceipt = SteadyClock::now();
  if (!localizationReady) goalConfirmCount = goalExitCount = 0;
}

void fastlioFarStopHandler(const std_msgs::msg::Bool::ConstSharedPtr stop)
{
  fastlioFarStop = stop->data;
  if (fastlioFarStop) goalConfirmCount = goalExitCount = 0;
}

void pathHandler(const nav_msgs::msg::Path::ConstSharedPtr pathIn)
{
  ++pathSequence;
  lastPathValid = false;
  if (pathIn->header.frame_id != "vehicle" && pathIn->header.frame_id != "/vehicle") {
    RCLCPP_ERROR(rclcpp::get_logger("pathFollower"),
      "Rejected path frame '%s': expected vehicle-relative localPlanner path",
      pathIn->header.frame_id.c_str());
    return;
  }
  int pathSize = pathIn->poses.size();
  for (const auto & pose : pathIn->poses) {
    const auto & p = pose.pose.position;
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) return;
  }
  lastPathStamp = rclcpp::Time(pathIn->header.stamp).nanoseconds();
  lastPathReceipt = SteadyClock::now();
  lastPathValid = pathSize > 1;
  // An empty path cancels the previous command and must never be indexed.
  if (pathSize == 0) {
    vehicleSpeed = 0;
    vehicleYawRate = 0;
    hasGoalYaw = false;
    path.poses.resize(1);
    path.poses[0] = geometry_msgs::msg::PoseStamped();
    vehicleXRec = vehicleX;
    vehicleYRec = vehicleY;
    vehicleYawRec = vehicleYaw;
    pathPointID = 0;
    pathInit = true;
    return;
  }
  path.poses.resize(pathSize);
  for (int i = 0; i < pathSize; i++) {
    path.poses[i].pose.position.x = pathIn->poses[i].pose.position.x;
    path.poses[i].pose.position.y = pathIn->poses[i].pose.position.y;
    path.poses[i].pose.position.z = pathIn->poses[i].pose.position.z;
    path.poses[i].pose.orientation = pathIn->poses[i].pose.orientation;
  }

  if (pathSize > 0 && useGoalYaw) {
    auto& lastOrientation = path.poses[pathSize - 1].pose.orientation;
    if (lastOrientation.w != 0 || lastOrientation.x != 0 ||
        lastOrientation.y != 0 || lastOrientation.z != 0) {
      tf2::Quaternion q(lastOrientation.x, lastOrientation.y,
                        lastOrientation.z, lastOrientation.w);
      double roll, pitch, yaw;
      tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
      goalYaw = yaw;
      hasGoalYaw = true;
    } else {
      hasGoalYaw = false;
    }
  } else {
    hasGoalYaw = false;
  }

  vehicleXRec = vehicleX;
  vehicleYRec = vehicleY;
  vehicleZRec = vehicleZ;
  vehicleRollRec = vehicleRoll;
  vehiclePitchRec = vehiclePitch;
  vehicleYawRec = vehicleYaw;

  pathPointID = 0;
  pathInit = true;
}

void plannerStatusHandler(const std_msgs::msg::String::ConstSharedPtr msg)
{
  plannerMatchesGoal = false;
  try {
    const auto j = nlohmann::json::parse(msg->data);
    plannerMatchesGoal = finalGoalValid && j.at("final_goal_valid").get<bool>() &&
      j.at("goal_stamp_ns").get<std::string>() == std::to_string(rclcpp::Time(originalGoal.header.stamp).nanoseconds()) &&
      fabs(j.at("target_x").get<double>()-finalGoalX) < 1e-6 &&
      fabs(j.at("target_y").get<double>()-finalGoalY) < 1e-6 &&
      fabs(normalizeAngle(j.at("target_yaw").get<double>()-finalGoalYaw)) < 1e-6;
    plannerRotationClear = j.at("rotation_clear").get<bool>();
    lastPlannerStatus = SteadyClock::now();
  } catch (const std::exception &) { plannerRotationClear = false; }
}

void requestReplan()
{
  if (finalGoalValid && !isFresh(lastReplanRequest, 1.0)) {
    replanPub->publish(originalGoal);
    lastReplanRequest = SteadyClock::now();
  }
}

void publishFinalGoalStatus(const std::string & reason, double distance, double yawError)
{
  static std::string previousState;
  static SteadyClock::time_point lastPublish;
  const char * phase = !finalGoalActive ? "idle" :
    finalGoalPhase == FinalGoalPhase::Align ? "align" :
    finalGoalPhase == FinalGoalPhase::Reached ? "reached" : "approach";
  const std::string state = std::string(phase) + ":" + reason;
  const bool changed = state != previousState;
  if (!changed && isFresh(lastPublish, 0.1)) return;
  if (changed) {
    RCLCPP_INFO(nh->get_logger(), "Final goal %s -> %s", previousState.c_str(), state.c_str());
  }
  previousState = state;
  lastPublish = SteadyClock::now();
  std::ostringstream out;
  out << "{\"phase\":\"" << phase << "\",\"reason\":\"" << reason << "\",\"distance\":";
  if (finalGoalActive && finalGoalValid && odomValid && std::isfinite(distance)) out << distance;
  else out << "null";
  out << ",\"yaw_error\":";
  if (finalGoalActive && finalGoalValid && odomValid && std::isfinite(yawError)) out << yawError;
  else out << "null";
  out << ",\"goal_stamp_ns\":\"" << rclcpp::Time(originalGoal.header.stamp).nanoseconds() << "\""
      << ",\"target_x\":" << std::setprecision(17) << finalGoalX
      << ",\"target_y\":" << finalGoalY
      << ",\"target_yaw\":" << finalGoalYaw
      << ",\"drive_mode\":\"" << (navFwd ? "forward" : "reverse") << "\""
      << ",\"reverse_allowed\":" << (reverseAllowed() ? "true" : "false")
      << ",\"reverse_max_goal_distance\":" << reverseMaxGoalDistance
      << ",\"planner_status_fresh\":" << (plannerMatchesGoal && isFresh(lastPlannerStatus,0.5) ? "true":"false")
      << ",\"observed_linear_speed\":" << rawMotionLinearSpeed
      << ",\"observed_angular_speed\":" << rawMotionAngularSpeed
      << ",\"motion_observation_valid\":" << (rawMotionValid && isFresh(lastRawMotionReceipt,0.5) ? "true":"false")
      << ",\"confirm_frames\":" << goalConfirmCount
      << ",\"exit_frames\":" << goalExitCount
      << ",\"waiting_for_path\":" << (goalWaitingForPath ? "true" : "false")
      << ",\"tracking_reason\":\"" << trackingReason << "\""
      << ",\"path_points\":" << path.poses.size()
      << ",\"lookahead_distance\":" << trackingDistance
      << ",\"path_end_distance\":" << trackingEndDistance
      << ",\"effective_stop_distance\":" << trackingStopDistance
      << ",\"path_valid\":" << (lastPathValid ? "true" : "false")
      << ",\"path_age\":";
  if (lastPathReceipt == SteadyClock::time_point{}) out << "null";
  else out << std::chrono::duration<double>(SteadyClock::now() - lastPathReceipt).count();
  out << ",\"raw_vx\":" << lastGoalCommand.linear.x
      << ",\"raw_vy\":" << lastGoalCommand.linear.y
      << ",\"raw_wz\":" << lastGoalCommand.angular.z
      << ",\"reached\":" << (finalGoalReached ? "true" : "false") << "}";
  std_msgs::msg::String message;
  message.data = out.str();
  goalStatusPub->publish(message);
}

void applyFinalGoalControl(geometry_msgs::msg::TwistStamped & command)
{
  if (!useFinalGoalPose) return;
  const double distance = hypot(finalGoalX - vehicleX, finalGoalY - vehicleY);
  const double yawError = normalizeAngle(finalGoalYaw - vehicleYaw);
  if (!finalGoalActive) {
    // A restarted follower must not execute a path retained from an old goal.
    // Explicit PointStamped goals preserve the position-only legacy interface.
    if (!positionOnlyGoalActive) {
      command.twist = geometry_msgs::msg::Twist();
      vehicleSpeed = vehicleYawRate = 0;
      lastGoalCommand = command.twist;
    }
    enforceReverseLimit(command);
    lastGoalCommand = command.twist;
    publishFinalGoalStatus("no_pose_goal", distance, yawError);
    return;
  }

  bool distinctFrame = false;
  if (odomSequence != lastGoalEvalOdomSequence) {
    distinctFrame = odomStampNanosec > 0 &&
      odomStampNanosec > lastGoalEvalOdomStampNanosec;
    lastGoalEvalOdomSequence = odomSequence;
    lastGoalEvalOdomStampNanosec = std::max(lastGoalEvalOdomStampNanosec, odomStampNanosec);
  }
  bool stop = true;
  double alignmentRate = 0.0;
  std::string reason;
  if (!finalGoalValid || !finalGoalLocalizationValid() || safetyStop != 0 ||
      (stopInitTime > 0 && odomTime < stopInitTime + stopTime)) {
    reason = !finalGoalValid ? "invalid_goal" : fastlioFarStop ? "manual_stop" :
      safetyStop != 0 ? "safety_stop" :
      !finalGoalLocalizationValid() ? "localization_unavailable" : "inclination_stop";
    goalConfirmCount = goalExitCount = 0;
  } else if (finalGoalReached) {
    reason = "arrival_latched";
  } else {
    if (finalGoalPhase == FinalGoalPhase::Approach && distance <= goalAlignmentEnterDistance) {
      finalGoalPhase = FinalGoalPhase::Align;
      goalWaitingForPath = false;
      goalExitCount = 0;
    }
    if (finalGoalPhase == FinalGoalPhase::Align) {
      if (distance > goalPositionTolerance && outsideToleranceSince == SteadyClock::time_point{})
        outsideToleranceSince = SteadyClock::now();
      if (distance <= goalPositionTolerance) outsideToleranceSince = {};
      const bool persistentPositionError = distance > goalPositionTolerance &&
        std::chrono::duration<double>(SteadyClock::now()-outsideToleranceSince).count() >= 0.5;
      if (distance > goalAlignmentExitDistance || persistentPositionError) {
        // Stop immediately, then require distinct localization frames before
        // abandoning alignment. Never rotate outside the exit radius.
        reason = "outside_exit_radius";
        goalConfirmCount = 0;
        if (distinctFrame) ++goalExitCount;
        if (goalExitCount >= goalAlignmentExitFrames) {
          finalGoalPhase = FinalGoalPhase::Approach;
          goalWaitingForPath = true;
          goalExitPathSequence = pathSequence;
          goalExitOdomStamp = odomStampNanosec;
          goalExitCount = 0;
          reason = "waiting_for_new_path";
        }
      } else {
        goalExitCount = 0;
        if (distance > goalPositionTolerance) {
          goalConfirmCount = 0;
          reason = "position_correction_pending";
        } else if (fabs(yawError) > goalYawControlTolerance) {
          alignmentRate = clampValue(goalYawGain * yawError,
            -maxYawRate * PI / 180.0, maxYawRate * PI / 180.0);
          goalConfirmCount = 0;
          reason = "aligning_yaw";
        } else if (distance > goalPositionTolerance) {
          goalConfirmCount = 0;
          reason = "position_hysteresis_band";
        } else {
          reason = "confirming_arrival";
          if (odomLinearSpeed > goalStoppedLinearSpeed || odomAngularSpeed > goalStoppedAngularSpeed ||
              (goalUseRawOdometry && (!rawMotionValid || !isFresh(lastRawMotionReceipt,0.5) ||
               rawMotionLinearSpeed > goalStoppedLinearSpeed || rawMotionAngularSpeed > goalStoppedAngularSpeed))) {
            goalConfirmCount = 0;
            reason = "waiting_for_standstill";
          } else if (distinctFrame) {
            ++goalConfirmCount;
          }
          if (goalConfirmCount >= goalConfirmFrames) {
            finalGoalReached = true;
            finalGoalPhase = FinalGoalPhase::Reached;
            publishGoalReached(true);
            reason = "arrival_latched";
          }
        }
      }
    } else {
      goalConfirmCount = 0;
      if (goalWaitingForPath) {
        const double age = (nh->now().nanoseconds() - lastPathStamp) * 1e-9;
        if (lastPathValid && pathSequence > goalExitPathSequence &&
            lastPathStamp > goalExitOdomStamp && age >= 0.0 &&
            age <= goalRecoveryPathTimeout && isFresh(lastPathReceipt, goalRecoveryPathTimeout)) {
          goalWaitingForPath = false;
        }
      }
      // A healthy odometry stream must not keep an obsolete or rejected local
      // path alive. A fresh one-point path explicitly requests a stop.
      const double pathAge = (nh->now().nanoseconds() - lastPathStamp) * 1e-9;
      stop = true;
      if (goalWaitingForPath) reason = "waiting_for_new_path";
      else if (!lastPathValid) reason = "recovering_local_path";
      else if (!plannerMatchesGoal || !isFresh(lastPlannerStatus,0.5)) reason = "waiting_for_planner_observation";
      else if (!isFresh(lastPathReceipt, goalRecoveryPathTimeout) ||
               pathAge < 0.0 || pathAge > goalRecoveryPathTimeout) reason = "local_path_stale";
      else {
        stop = false;
        reason = trackingReason;
      }
    }
  }
  if (alignmentRate != 0 && (!plannerMatchesGoal || !isFresh(lastPlannerStatus,0.5) || !plannerRotationClear)) {
    alignmentRate = 0;
    reason = "rotation_blocked_or_unobserved";
    requestReplan();
  }
  if (!finalGoalReached && finalGoalValid &&
      (goalWaitingForPath || !lastPathValid || trackingReason == "local_path_endpoint" ||
       reason == "local_path_stale")) requestReplan();
  if (!stop && distance <= finalApproachRange) {
    const double limit = std::min(finalApproachMaxSpeed, 0.6 * distance);
    const double speed = hypot(command.twist.linear.x, command.twist.linear.y);
    if (speed > limit) {
      command.twist.linear.x *= limit / speed;
      command.twist.linear.y *= limit / speed;
    }
  }
  // Any autonomous rotation must have a fresh obstacle clearance observation.
  if (!stop && fabs(command.twist.angular.z) > 1e-6 &&
      (!plannerMatchesGoal || !isFresh(lastPlannerStatus,0.5) || !plannerRotationClear)) {
    stop = true;
    reason = "rotation_blocked_or_unobserved";
    requestReplan();
  }
  if (stop) {
    vehicleSpeed = 0;
    vehicleYawRate = alignmentRate;
    command.twist.linear.x = command.twist.linear.y = 0;
    command.twist.angular.z = alignmentRate;
  }
  enforceReverseLimit(command);
  lastGoalCommand = command.twist;
  publishFinalGoalStatus(reason, distance, yawError);
}

void joystickHandler(const sensor_msgs::msg::Joy::ConstSharedPtr joy)
{
  joyTime = nh->now().seconds(); 
  joySpeedRaw = sqrt(joy->axes[3] * joy->axes[3] + joy->axes[4] * joy->axes[4]);
  joySpeed = joySpeedRaw;
  if (joySpeed > 1.0) joySpeed = 1.0;
  if (joy->axes[4] == 0) joySpeed = 0;
  joyYaw = joy->axes[3];
  if (joySpeed == 0 && noRotAtStop) joyYaw = 0;

  if (joy->axes[4] < 0 && !twoWayDrive) {
    joySpeed = 0;
    joyYaw = 0;
  }

  joyManualFwd = joy->axes[4];
  joyManualLeft = joy->axes[3];
  joyManualYaw = joy->axes[0];

  if (joy->axes[2] > -0.1) {
    autonomyMode = false;
  } else {
    autonomyMode = true;
  }

  if (joy->axes[5] > -0.1) {
    manualMode = false;
  } else {
    manualMode = true;
  }
}

void speedHandler(const std_msgs::msg::Float32::ConstSharedPtr speed)
{
  double speedTime = nh->now().seconds();
  if (autonomyMode && speedTime - joyTime > joyToSpeedDelay && joySpeedRaw == 0) {
    joySpeed = speed->data / maxSpeed;

    if (joySpeed < 0) joySpeed = 0;
    else if (joySpeed > 1.0) joySpeed = 1.0;
  }
}

void stopHandler(const std_msgs::msg::Int8::ConstSharedPtr stop)
{
  safetyStop = stop->data;
}

void slowDownHandler(const std_msgs::msg::Int8::ConstSharedPtr slow)
{
  slowDown = slow->data;
}

int main(int argc, char** argv)
{
  rclcpp::InitOptions initOptions;
  // Let this node stop its executor and destroy ROS entities before the DDS
  // context is shut down.  The default rclcpp handler can tear down the
  // context first, which makes Fast-DDS service destruction race at SIGINT.
  initOptions.shutdown_on_signal = false;
  rclcpp::init(argc, argv, initOptions);
  std::signal(SIGINT, signalHandler);
  std::signal(SIGTERM, signalHandler);
  nh = rclcpp::Node::make_shared("pathFollower");

  nh->declare_parameter<bool>("realRobot", realRobot);
  nh->declare_parameter<string>("serialPort", serialPort);
  nh->declare_parameter<int>("baudrate", baudrate);
  nh->declare_parameter<double>("sensorOffsetX", sensorOffsetX);
  nh->declare_parameter<double>("sensorOffsetY", sensorOffsetY);
  nh->declare_parameter<int>("pubSkipNum", pubSkipNum);
  nh->declare_parameter<bool>("twoWayDrive", twoWayDrive);
  nh->declare_parameter<double>("reverseMaxGoalDistance", reverseMaxGoalDistance);
  nh->declare_parameter<double>("lookAheadDis", lookAheadDis);
  nh->declare_parameter<double>("yawRateGain", yawRateGain);
  nh->declare_parameter<double>("stopYawRateGain", stopYawRateGain);
  nh->declare_parameter<double>("maxYawRate", maxYawRate);
  nh->declare_parameter<double>("maxSpeed", maxSpeed);
  nh->declare_parameter<double>("maxAccel", maxAccel);
  nh->declare_parameter<double>("switchTimeThre", switchTimeThre);
  nh->declare_parameter<double>("dirDiffThre", dirDiffThre);
  nh->declare_parameter<double>("omniDirGoalThre", omniDirGoalThre);
  nh->declare_parameter<double>("omniDirDiffThre", omniDirDiffThre);
  nh->declare_parameter<double>("stopDisThre", stopDisThre);
  nh->declare_parameter<double>("slowDwnDisThre", slowDwnDisThre);
  nh->declare_parameter<bool>("useInclRateToSlow", useInclRateToSlow);
  nh->declare_parameter<double>("inclRateThre", inclRateThre);
  nh->declare_parameter<double>("slowRate1", slowRate1);
  nh->declare_parameter<double>("slowRate2", slowRate2);
  nh->declare_parameter<double>("slowRate3", slowRate3);
  nh->declare_parameter<double>("slowTime1", slowTime1);
  nh->declare_parameter<double>("slowTime2", slowTime2);
  nh->declare_parameter<bool>("useInclToStop", useInclToStop);
  nh->declare_parameter<double>("inclThre", inclThre);
  nh->declare_parameter<double>("stopTime", stopTime);
  nh->declare_parameter<bool>("noRotAtStop", noRotAtStop);
  nh->declare_parameter<bool>("noRotAtGoal", noRotAtGoal);
  nh->declare_parameter<bool>("autonomyMode", autonomyMode);
  nh->declare_parameter<double>("autonomySpeed", autonomySpeed);
  nh->declare_parameter<double>("joyToSpeedDelay", joyToSpeedDelay);
  nh->declare_parameter<double>("goalYawGain", goalYawGain);
  nh->declare_parameter<bool>("useGoalYaw", useGoalYaw);
  nh->declare_parameter<bool>("useFinalGoalPose", useFinalGoalPose);
  nh->declare_parameter<bool>("goalRequireLocalizationReady", goalRequireLocalizationReady);
  nh->declare_parameter<bool>("goalUseRawOdometry", goalUseRawOdometry);
  nh->declare_parameter<double>("goalPositionTolerance", goalPositionTolerance);
  nh->declare_parameter<double>("goalAlignmentEnterDistance", goalAlignmentEnterDistance);
  nh->declare_parameter<int>("goalAlignmentExitFrames", goalAlignmentExitFrames);
  nh->declare_parameter<double>("goalAlignmentExitDistance", goalAlignmentExitDistance);
  nh->declare_parameter<double>("goalYawTolerance", goalYawTolerance);
  nh->declare_parameter<double>("goalYawControlTolerance", goalYawControlTolerance);
  nh->declare_parameter<int>("goalConfirmFrames", goalConfirmFrames);
  nh->declare_parameter<double>("goalStoppedLinearSpeed", goalStoppedLinearSpeed);
  nh->declare_parameter<double>("goalStoppedAngularSpeed", goalStoppedAngularSpeed);

  nh->get_parameter("realRobot", realRobot);
  nh->get_parameter("serialPort", serialPort);
  nh->get_parameter("baudrate", baudrate);
  nh->get_parameter("sensorOffsetX", sensorOffsetX);
  nh->get_parameter("sensorOffsetY", sensorOffsetY);
  nh->get_parameter("pubSkipNum", pubSkipNum);
  nh->get_parameter("twoWayDrive", twoWayDrive);
  nh->get_parameter("reverseMaxGoalDistance", reverseMaxGoalDistance);
  if (!std::isfinite(reverseMaxGoalDistance) || reverseMaxGoalDistance < 0)
    throw std::invalid_argument("reverseMaxGoalDistance must be finite and nonnegative");
  nh->get_parameter("lookAheadDis", lookAheadDis);
  nh->get_parameter("yawRateGain", yawRateGain);
  nh->get_parameter("stopYawRateGain", stopYawRateGain);
  nh->get_parameter("maxYawRate", maxYawRate);
  nh->get_parameter("maxSpeed", maxSpeed);
  nh->get_parameter("maxAccel", maxAccel);
  nh->get_parameter("switchTimeThre", switchTimeThre);
  nh->get_parameter("dirDiffThre", dirDiffThre);
  nh->get_parameter("omniDirGoalThre", omniDirGoalThre);
  nh->get_parameter("omniDirDiffThre", omniDirDiffThre);
  nh->get_parameter("stopDisThre", stopDisThre);
  nh->get_parameter("slowDwnDisThre", slowDwnDisThre);
  nh->get_parameter("useInclRateToSlow", useInclRateToSlow);
  nh->get_parameter("inclRateThre", inclRateThre);
  nh->get_parameter("slowRate1", slowRate1);
  nh->get_parameter("slowRate2", slowRate2);
  nh->get_parameter("slowRate3", slowRate3);
  nh->get_parameter("slowTime1", slowTime1);
  nh->get_parameter("slowTime2", slowTime2);
  nh->get_parameter("useInclToStop", useInclToStop);
  nh->get_parameter("inclThre", inclThre);
  nh->get_parameter("stopTime", stopTime);
  nh->get_parameter("noRotAtStop", noRotAtStop);
  nh->get_parameter("noRotAtGoal", noRotAtGoal);
  nh->get_parameter("autonomyMode", autonomyMode);
  nh->get_parameter("autonomySpeed", autonomySpeed);
  nh->get_parameter("joyToSpeedDelay", joyToSpeedDelay);
  nh->get_parameter("goalYawGain", goalYawGain);
  nh->get_parameter("useGoalYaw", useGoalYaw);
  nh->get_parameter("useFinalGoalPose", useFinalGoalPose);
  nh->get_parameter("goalRequireLocalizationReady", goalRequireLocalizationReady);
  nh->get_parameter("goalUseRawOdometry", goalUseRawOdometry);
  nh->get_parameter("goalPositionTolerance", goalPositionTolerance);
  nh->get_parameter("goalAlignmentEnterDistance", goalAlignmentEnterDistance);
  if (!std::isfinite(goalAlignmentEnterDistance) || goalAlignmentEnterDistance <= 0 || goalAlignmentEnterDistance > goalPositionTolerance)
    throw std::invalid_argument("goalAlignmentEnterDistance must be positive and no greater than final position tolerance");
  nh->get_parameter("goalAlignmentExitFrames", goalAlignmentExitFrames);
  nh->get_parameter("goalAlignmentExitDistance", goalAlignmentExitDistance);
  nh->get_parameter("goalYawTolerance", goalYawTolerance);
  nh->get_parameter("goalYawControlTolerance", goalYawControlTolerance);
  if (!std::isfinite(goalYawControlTolerance) || goalYawControlTolerance <= 0 || goalYawControlTolerance > goalYawTolerance)
    throw std::invalid_argument("goalYawControlTolerance must be positive and no greater than final yaw tolerance");
  nh->get_parameter("goalConfirmFrames", goalConfirmFrames);
  nh->get_parameter("goalStoppedLinearSpeed", goalStoppedLinearSpeed);
  nh->get_parameter("goalStoppedAngularSpeed", goalStoppedAngularSpeed);
  goalPositionTolerance = std::max(0.0, goalPositionTolerance);
  if (!std::isfinite(goalAlignmentExitDistance) || goalAlignmentExitDistance <= goalPositionTolerance ||
      goalAlignmentExitFrames < 1) {
    throw std::invalid_argument("goalAlignmentExitDistance must exceed goalPositionTolerance; exit frames must be positive");
  }
  goalYawTolerance = std::max(0.0, goalYawTolerance);
  goalConfirmFrames = std::max(1, goalConfirmFrames);
  goalStoppedLinearSpeed = std::max(0.0, goalStoppedLinearSpeed);
  goalStoppedAngularSpeed = std::max(0.0, goalStoppedAngularSpeed);

  auto subRawMotion = nh->create_subscription<nav_msgs::msg::Odometry>("/Odometry", 5, rawMotionHandler);
  auto subOdom = nh->create_subscription<nav_msgs::msg::Odometry>("/state_estimation", 5, odomHandler);

  replanPub = nh->create_publisher<geometry_msgs::msg::PoseStamped>("/far_planner/replan", 5);
  auto subPlannerStatus = nh->create_subscription<std_msgs::msg::String>("/local_planner/status", 5, plannerStatusHandler);
  auto subPath = nh->create_subscription<nav_msgs::msg::Path>("/path", 5, pathHandler);

  auto subGoalPose = nh->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/goal_pose", rclcpp::QoS(5), goalPoseHandler);

  auto subGoalPoint = nh->create_subscription<geometry_msgs::msg::PointStamped>(
    "/goal_point", rclcpp::QoS(5), goalPointHandler);

  auto subLocalizationReady = nh->create_subscription<std_msgs::msg::Bool>(
    "/localization_ready", rclcpp::QoS(10), localizationReadyHandler);

  auto subFastlioFarStop = nh->create_subscription<std_msgs::msg::Bool>(
    "/fastlio_far/stop", rclcpp::QoS(1).reliable(), fastlioFarStopHandler);

  auto subJoystick = nh->create_subscription<sensor_msgs::msg::Joy>("/joy", 5, joystickHandler);

  auto subSpeed = nh->create_subscription<std_msgs::msg::Float32>("/speed", 5, speedHandler);

  auto subStop = nh->create_subscription<std_msgs::msg::Int8>("/stop", 5, stopHandler);

  auto subSlowDown = nh->create_subscription<std_msgs::msg::Int8>("/slow_down", 5, slowDownHandler);

  auto pubSpeed = nh->create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel", 5);
  goalReachedPub = nh->create_publisher<std_msgs::msg::Bool>("/goal_reached_confirmed", 10);
  goalStatusPub = nh->create_publisher<std_msgs::msg::String>("/fastlio_far/goal_status", 10);
  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header.frame_id = "vehicle";

  if (autonomyMode) {
    joySpeed = autonomySpeed / maxSpeed;

    if (joySpeed < 0) joySpeed = 0;
    else if (joySpeed > 1.0) joySpeed = 1.0;
  }

  serial::Serial *motorCtrSerial = new serial::Serial();
  serial::Timeout timeout(serial::Timeout::simpleTimeout(500));
  motorCtrSerial->setTimeout(timeout);
  motorCtrSerial->setPort(serialPort.c_str());
  motorCtrSerial->setBaudrate(baudrate);

  float value;
  size_t size = sizeof(value);
  uint8_t *serialBuffer = static_cast<uint8_t*>(alloca(3 * size + 1));

  int initFrameCount = 0;
  rclcpp::Rate rate(100);
  // Reuse the executor: spin_some(node) constructs a new guard condition on
  // every call, which can throw when a shutdown arrives between loop ticks.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(nh);
  while (rclcpp::ok() && !stopRequested.load()) {
    executor.spin_some();
    if (!rclcpp::ok()) break;

    if (pathInit) {
      float vehicleXRel = cos(vehicleYawRec) * (vehicleX - vehicleXRec) 
                        + sin(vehicleYawRec) * (vehicleY - vehicleYRec);
      float vehicleYRel = -sin(vehicleYawRec) * (vehicleX - vehicleXRec) 
                        + cos(vehicleYawRec) * (vehicleY - vehicleYRec);

      int pathSize = path.poses.size();
      float endDisX = path.poses[pathSize - 1].pose.position.x - vehicleXRel;
      float endDisY = path.poses[pathSize - 1].pose.position.y - vehicleYRel;
      float endDis = sqrt(endDisX * endDisX + endDisY * endDisY);

      float disX, disY, dis;
      while (pathPointID < pathSize - 1) {
        disX = path.poses[pathPointID].pose.position.x - vehicleXRel;
        disY = path.poses[pathPointID].pose.position.y - vehicleYRel;
        dis = sqrt(disX * disX + disY * disY);
        if (dis < lookAheadDis) {
          pathPointID++;
        } else {
          break;
        }
      }

      disX = path.poses[pathPointID].pose.position.x - vehicleXRel;
      disY = path.poses[pathPointID].pose.position.y - vehicleYRel;
      dis = sqrt(disX * disX + disY * disY);
      // Local paths are cropped/sampled before the waypoint. The legacy 0.4 m
      // G1 stop radius must not prevent reaching a 0.1 m final pose. Retain a
      // smaller path-end guard; never drive through an empty/blocked path.
      const double effectiveStopDistance = useFinalGoalPose && finalGoalActive && finalGoalValid ?
        std::min(stopDisThre, goalPositionTolerance * 0.5) : stopDisThre;
      trackingDistance = dis;
      trackingEndDistance = endDis;
      trackingStopDistance = effectiveStopDistance;
      float pathDir = atan2(disY, disX);

      float dirDiff = vehicleYaw - vehicleYawRec - pathDir;
      if (dirDiff > PI) dirDiff -= 2 * PI;
      else if (dirDiff < -PI) dirDiff += 2 * PI;
      if (dirDiff > PI) dirDiff -= 2 * PI;
      else if (dirDiff < -PI) dirDiff += 2 * PI;

      const bool allowReverse = reverseAllowed();
      if (!allowReverse) {
        // Leaving the 0.2 m region (or receiving a farther goal) immediately
        // cancels reverse; the direction-switch dwell must not delay this.
        navFwd = true;
        if (vehicleSpeed < 0) vehicleSpeed = 0;
      } else {
        double time = nh->now().seconds();
        if (fabs(dirDiff) > 100.0 * PI / 180.0 && navFwd && time - switchTime > switchTimeThre) {
          navFwd = false;
          vehicleSpeed = 0;
          switchTime = time;
        } else if (fabs(dirDiff) < 80.0 * PI / 180.0 && !navFwd && time - switchTime > switchTimeThre) {
          navFwd = true;
          vehicleSpeed = 0;
          switchTime = time;
        }
      }

      float joySpeed2 = maxSpeed * joySpeed;
      if (!navFwd) {
        dirDiff += PI;
        if (dirDiff > PI) dirDiff -= 2 * PI;
        joySpeed2 *= -1;
      }

      if (fabs(vehicleSpeed) < 2.0 * maxAccel / 100.0) vehicleYawRate = -stopYawRateGain * dirDiff;
      else vehicleYawRate = -yawRateGain * dirDiff;

      if (vehicleYawRate > maxYawRate * PI / 180.0) vehicleYawRate = maxYawRate * PI / 180.0;
      else if (vehicleYawRate < -maxYawRate * PI / 180.0) vehicleYawRate = -maxYawRate * PI / 180.0;

      if (!useFinalGoalPose && hasGoalYaw && pathPointID >= pathSize - 1 && endDis < stopDisThre && !noRotAtGoal) {
        double yawError = normalizeAngle(goalYaw - vehicleYaw);
        vehicleYawRate = goalYawGain * yawError;
        if (vehicleYawRate > maxYawRate * PI / 180.0) vehicleYawRate = maxYawRate * PI / 180.0;
        else if (vehicleYawRate < -maxYawRate * PI / 180.0) vehicleYawRate = -maxYawRate * PI / 180.0;
        joySpeed2 = 0;
      }

      if (joySpeed2 == 0 && !autonomyMode) {
        vehicleYawRate = maxYawRate * joyYaw * PI / 180.0;
      } else if ((pathSize <= 1 && !hasGoalYaw) || (dis < effectiveStopDistance && noRotAtGoal && !hasGoalYaw)) {
        vehicleYawRate = 0;
      }

      if (pathSize <= 1) {
        joySpeed2 = 0;
      } else if (endDis / slowDwnDisThre < joySpeed) {
        joySpeed2 *= endDis / slowDwnDisThre;
      }

      if (useFinalGoalPose && finalGoalActive && finalGoalValid && pathSize > 1 &&
          hypot(finalGoalX-vehicleX,finalGoalY-vehicleY) <= finalApproachRange) {
        // Final speed depends on the original goal, not an intermediate endpoint.
        const double requested = std::min(maxSpeed*joySpeed,
          std::min(finalApproachMaxSpeed,0.6*hypot(finalGoalX-vehicleX,finalGoalY-vehicleY)));
        joySpeed2 = navFwd ? requested : -requested;
      }
      float joySpeed3 = joySpeed2;
      if (odomTime < slowInitTime + slowTime1 && slowInitTime > 0 || slowDown == 1) joySpeed3 *= slowRate1;
      else if (odomTime < slowInitTime + slowTime1 + slowTime2 && slowInitTime > 0 || slowDown == 2) joySpeed3 *= slowRate2;
      else if (slowDown == 3) joySpeed3 *= slowRate3;

      const bool directionAllowed = fabs(dirDiff) < dirDiffThre ||
        (dis < omniDirGoalThre && fabs(dirDiff) < omniDirDiffThre);
      trackingReason = pathSize <= 1 ? "local_path_stop" :
        dis <= effectiveStopDistance ? "local_path_endpoint" :
        !directionAllowed ? "turning_to_path" :
        fabs(joySpeed3) < 1e-6 ? "zero_requested_speed" : "following_path";
      if (directionAllowed && dis > effectiveStopDistance) {
        if (vehicleSpeed < joySpeed3) vehicleSpeed += maxAccel / 100.0;
        else if (vehicleSpeed > joySpeed3) vehicleSpeed -= maxAccel / 100.0;
      } else {
        if (vehicleSpeed > 0) vehicleSpeed -= maxAccel / 100.0;
        else if (vehicleSpeed < 0) vehicleSpeed += maxAccel / 100.0;
      }

      if (odomTime < stopInitTime + stopTime && stopInitTime > 0) {
        vehicleSpeed = 0;
        vehicleYawRate = 0;
      }

      if (safetyStop >= 1) vehicleSpeed = 0;
      if (safetyStop >= 2) vehicleYawRate = 0;

      pubSkipCount--;
      if (pubSkipCount < 0) {
        // This stamp identifies command generation; sensor age is checked separately.
        cmd_vel.header.stamp = nh->now();
        cmd_vel.twist.linear.x = 0;
        cmd_vel.twist.linear.y = 0;
        cmd_vel.twist.angular.z = vehicleYawRate;

        if (fabs(vehicleSpeed) > maxAccel / 100.0) {
          if (omniDirGoalThre > 0) {
            cmd_vel.twist.linear.x = cos(dirDiff) * vehicleSpeed;
            cmd_vel.twist.linear.y = -sin(dirDiff) * vehicleSpeed;
          } else {
            cmd_vel.twist.linear.x = vehicleSpeed;
          }
        }

        if (manualMode) {
          cmd_vel.twist.linear.x = maxSpeed * joyManualFwd;
          if (omniDirGoalThre > 0) cmd_vel.twist.linear.y = maxSpeed / 2.0 * joyManualLeft;
          cmd_vel.twist.angular.z = maxYawRate * PI / 180.0 * joyManualYaw;
        }

        applyFinalGoalControl(cmd_vel);

        pubSpeed->publish(cmd_vel);
        pubSkipCount = pubSkipNum;

        if (realRobot) {
          if (serialOpen) {
            value = cmd_vel.twist.linear.x;
            memcpy(serialBuffer, &value, size);
            value = cmd_vel.twist.linear.y;
            memcpy(serialBuffer + size, &value, size);
            value = cmd_vel.twist.angular.z;
            memcpy(serialBuffer + 2 * size, &value, size);
            serialBuffer[3 * size] = '\n';

            motorCtrSerial->write(serialBuffer, 3 * size + 1);
          } else {
            if (initFrameCount >= 100 && initFrameCount % 50 == 0) {
              try {
                motorCtrSerial->open();
              } catch (serial::IOException) {
              }

              if (motorCtrSerial->isOpen()) {
                serialOpen = true;
                RCLCPP_INFO(nh->get_logger(), "Serial port open.");
              } else {
                RCLCPP_INFO(nh->get_logger(), "Opening serial port %s...", serialPort.c_str());
              }
            }
            initFrameCount++;
          }
        }
      }
    }

    // Report the startup wait even before localPlanner has sent its first path.
    if (!pathInit && useFinalGoalPose) {
      publishFinalGoalStatus(finalGoalActive ? "waiting_for_initial_path" : "no_pose_goal",
        hypot(finalGoalX - vehicleX, finalGoalY - vehicleY),
        normalizeAngle(finalGoalYaw - vehicleYaw));
    }
    rate.sleep();
  }

  if (serialOpen) motorCtrSerial->close();
  delete motorCtrSerial;

  // Destroy all entities while the rcl context is still valid, then shut DDS
  // down.  This also makes SIGINT/SIGTERM cleanup deterministic in launch.
  executor.remove_node(nh);
  subSlowDown.reset();
  subStop.reset();
  subSpeed.reset();
  subJoystick.reset();
  subFastlioFarStop.reset();
  subLocalizationReady.reset();
  subGoalPoint.reset();
  subGoalPose.reset();
  subPath.reset();
  subOdom.reset();
  goalStatusPub.reset();
  goalReachedPub.reset();
  pubSpeed.reset();
  nh.reset();
  rclcpp::shutdown();

  return 0;
}
