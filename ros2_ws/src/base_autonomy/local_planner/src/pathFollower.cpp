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
#include "finalGoalPolicy.h"

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
constexpr double reverseGoalDistanceLimit = final_goal::reverseDistance;
bool reverseGoalValid = false;
double reverseGoalX = 0.0, reverseGoalY = 0.0;
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
double finalGoalX = 0.0;
double finalGoalY = 0.0;
double finalGoalYaw = 0.0;
double goalPositionTolerance = 0.10;
double goalAlignmentExitDistance = final_goal::correctionExitDistance;
int goalAlignmentExitFrames = 3;
double goalYawTolerance = 0.3490658503988659;  // exactly 20 degrees
int goalConfirmFrames = 3;
double goalStoppedLinearSpeed = 0.03;
double goalStoppedAngularSpeed = 0.05;
bool finalGoalActive = false;
bool finalGoalValid = false;
bool finalGoalReached = false;
int goalConfirmCount = 0;
enum class FinalGoalPhase { Approach, Align, Translate, Reached };
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
nav_msgs::msg::Path finalCorridor;
SteadyClock::time_point finalCorridorReceipt;
int64_t finalGoalReceiptNs = 0;

void finalCorridorHandler(const nav_msgs::msg::Path::ConstSharedPtr message)
{
  finalCorridor = *message;
  finalCorridorReceipt = SteadyClock::now();
}

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

bool reverseAllowedAtGoal()
{
  // Use the navigation goal in map coordinates, never a cropped local path
  // or FAR waypoint. An unknown goal/pose cannot grant reverse permission.
  return twoWayDrive && reverseGoalValid && odomValid &&
    isFresh(lastOdomReceipt, goalLocalizationTimeout) &&
    hypot(reverseGoalX - vehicleX, reverseGoalY - vehicleY) <= reverseGoalDistanceLimit;
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
  finalCorridor.poses.clear();
  finalGoalReceiptNs = nh->now().nanoseconds();
  // Confirmation must use frames received after this goal, not the last frame
  // retained from the previous target.
  lastGoalEvalOdomSequence = odomSequence;
  lastGoalEvalOdomStampNanosec = odomStampNanosec;
  publishGoalReached(false);
}

bool finalGoalLocalizationValid()
{
  if (!odomValid || !isFresh(lastOdomReceipt, goalLocalizationTimeout)) return false;
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

void goalPoseHandler(const geometry_msgs::msg::PoseStamped::ConstSharedPtr goal)
{
  const auto & p = goal->pose.position;
  const auto & q = goal->pose.orientation;
  const double qNorm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
  const bool valid = isMapFrame(goal->header.frame_id) &&
    std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z) &&
    std::isfinite(q.x) && std::isfinite(q.y) && std::isfinite(q.z) &&
    std::isfinite(q.w) && qNorm >= 1e-12;

  reverseGoalValid = valid;
  if (valid) {
    reverseGoalX = p.x;
    reverseGoalY = p.y;
  }
  if (!useFinalGoalPose) return;

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
  reverseGoalValid = isMapFrame(goal->header.frame_id) &&
    std::isfinite(goal->point.x) && std::isfinite(goal->point.y) && std::isfinite(goal->point.z);
  if (reverseGoalValid) {
    reverseGoalX = goal->point.x;
    reverseGoalY = goal->point.y;
  }
  // A position-only goal is the legacy FAR contract.  It must cancel any
  // previously stored PoseStamped yaw so an old target cannot be confirmed.
  if (useFinalGoalPose) resetFinalGoalState(false);
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

void publishFinalGoalStatus(const std::string & reason, double distance, double yawError)
{
  static std::string previousState;
  static SteadyClock::time_point lastPublish;
  const char * phase = !finalGoalActive ? "idle" :
    finalGoalPhase == FinalGoalPhase::Align ? "align" :
    finalGoalPhase == FinalGoalPhase::Translate ? "translate" :
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
  out << ",\"confirm_frames\":" << goalConfirmCount
      << ",\"exit_frames\":" << goalExitCount
      << ",\"waiting_for_path\":" << (goalWaitingForPath ? "true" : "false")
      << ",\"reverse_allowed\":" << (reverseAllowedAtGoal() ? "true" : "false")
      << ",\"reverse_limit\":" << reverseGoalDistanceLimit
      << ",\"align_enter_distance\":" << final_goal::alignDistance
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

bool finalCorridorValid()
{
  if (finalCorridor.header.frame_id != "map" || finalCorridor.poses.size() != 2 ||
      !isFresh(finalCorridorReceipt, final_goal::pathTimeout) ||
      nh->count_publishers("/local_planner/final_path") != 1) return false;
  const int64_t stamp = rclcpp::Time(finalCorridor.header.stamp).nanoseconds();
  const double age = (nh->now().nanoseconds()-stamp)*1e-9;
  if (stamp < finalGoalReceiptNs || age < 0 || age > final_goal::pathTimeout) return false;
  const auto & start = finalCorridor.poses.front().pose.position;
  const auto & end = finalCorridor.poses.back().pose;
  const auto & q = end.orientation;
  if (!std::isfinite(start.x) || !std::isfinite(start.y) ||
      !std::isfinite(end.position.x) || !std::isfinite(end.position.y) ||
      !std::isfinite(q.z) || !std::isfinite(q.w)) return false;
  // Certificate belongs to this goal, with little motion since its footprint
  // check. A certificate for FAR's shifted goal cannot authorize correction.
  return hypot(end.position.x-finalGoalX, end.position.y-finalGoalY) <= 1e-4 &&
    fabs(normalizeAngle(2*atan2(q.z,q.w)-finalGoalYaw)) <= 1e-4 &&
    hypot(start.x-vehicleX, start.y-vehicleY) <= final_goal::footprintMargin;
}

void applyFinalGoalControl(geometry_msgs::msg::TwistStamped & command)
{
  if (!useFinalGoalPose) return;
  const double distance = hypot(finalGoalX - vehicleX, finalGoalY - vehicleY);
  const double yawError = normalizeAngle(finalGoalYaw - vehicleYaw);
  if (!finalGoalActive) {
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
  } else {
    if (finalGoalReached &&
        (distance > goalPositionTolerance || fabs(yawError) > goalYawTolerance)) {
      // Do not keep advertising success if the pose drifts during patrol dwell.
      finalGoalReached = false;
      goalConfirmCount = 0;
      finalGoalPhase = FinalGoalPhase::Align;
      publishGoalReached(false);
    }
    if (finalGoalPhase == FinalGoalPhase::Approach && distance <= final_goal::alignDistance) {
      finalGoalPhase = FinalGoalPhase::Align;
      goalWaitingForPath = false;
      goalExitCount = 0;
    }
    if (finalGoalReached) {
      reason = "arrival_latched";
    } else if (distance <= goalPositionTolerance && fabs(yawError) <= goalYawTolerance) {
      reason = "confirming_arrival";
      if (odomLinearSpeed > goalStoppedLinearSpeed || odomAngularSpeed > goalStoppedAngularSpeed) {
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
    } else if (finalGoalPhase == FinalGoalPhase::Align || finalGoalPhase == FinalGoalPhase::Translate) {
      goalConfirmCount = 0;
      if (distance > goalAlignmentExitDistance) {
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
        if (!finalCorridorValid()) {
          reason = "final_corridor_blocked_or_stale";
        } else {
          if (finalGoalPhase == FinalGoalPhase::Translate && fabs(yawError) > goalYawTolerance)
            finalGoalPhase = FinalGoalPhase::Align;
          if (finalGoalPhase == FinalGoalPhase::Align &&
              fabs(yawError) <= std::min(goalYawTolerance, final_goal::alignReleaseYaw))
            finalGoalPhase = FinalGoalPhase::Translate;
          alignmentRate = clampValue(goalYawGain * yawError,
            -std::min(maxYawRate*PI/180.0, final_goal::maxAlignmentRate),
             std::min(maxYawRate*PI/180.0, final_goal::maxAlignmentRate));
          if (finalGoalPhase == FinalGoalPhase::Align) {
            reason = "aligning_yaw";
          } else {
            const double dx = finalGoalX-vehicleX, dy = finalGoalY-vehicleY;
            const double bodyX = cos(vehicleYaw)*dx + sin(vehicleYaw)*dy;
            const double bodyY = -sin(vehicleYaw)*dx + cos(vehicleYaw)*dy;
            if (bodyX < -1e-6 && !reverseAllowedAtGoal()) {
              reason = "final_translation_reverse_not_allowed";
              alignmentRate = 0;
            } else {
              // Follow the checked straight corridor without pointing the
              // body at the waypoint. Scale the vector, preserving direction.
              double scale = std::min(0.6, final_goal::maxTranslationSpeed/std::max(distance, 1e-6));
              if (fabs(bodyY)*scale > final_goal::maxLateralSpeed)
                scale = final_goal::maxLateralSpeed/fabs(bodyY);
              if (slowDown == 1) scale *= slowRate1;
              else if (slowDown == 2) scale *= slowRate2;
              else if (slowDown == 3) scale *= slowRate3;
              stop = false;
              vehicleSpeed = 0;
              command.twist.linear.x = scale*bodyX;
              command.twist.linear.y = scale*bodyY;
              command.twist.angular.z = alignmentRate;
              reason = "translating_with_goal_yaw";
            }
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
      else if (!lastPathValid) reason = "local_path_invalid_or_stop";
      else if (!isFresh(lastPathReceipt, goalRecoveryPathTimeout) ||
               pathAge < 0.0 || pathAge > goalRecoveryPathTimeout) reason = "local_path_stale";
      else {
        stop = false;
        reason = trackingReason;
      }
    }
  }
  if (stop) {
    vehicleSpeed = 0;
    vehicleYawRate = alignmentRate;
    command.twist.linear.x = command.twist.linear.y = 0;
    command.twist.angular.z = alignmentRate;
  }
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
  nh->declare_parameter<double>("goalPositionTolerance", goalPositionTolerance);
  nh->declare_parameter<int>("goalAlignmentExitFrames", goalAlignmentExitFrames);
  nh->declare_parameter<double>("goalAlignmentExitDistance", goalAlignmentExitDistance);
  nh->declare_parameter<double>("goalYawTolerance", goalYawTolerance);
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
  nh->get_parameter("goalPositionTolerance", goalPositionTolerance);
  nh->get_parameter("goalAlignmentExitFrames", goalAlignmentExitFrames);
  nh->get_parameter("goalAlignmentExitDistance", goalAlignmentExitDistance);
  nh->get_parameter("goalYawTolerance", goalYawTolerance);
  nh->get_parameter("goalConfirmFrames", goalConfirmFrames);
  nh->get_parameter("goalStoppedLinearSpeed", goalStoppedLinearSpeed);
  nh->get_parameter("goalStoppedAngularSpeed", goalStoppedAngularSpeed);
  goalPositionTolerance = std::max(0.0, goalPositionTolerance);
  if (!std::isfinite(goalAlignmentExitDistance) || goalAlignmentExitDistance <= final_goal::alignDistance ||
      goalAlignmentExitDistance > final_goal::reverseDistance ||
      goalAlignmentExitFrames < 1) {
    throw std::invalid_argument("goalAlignmentExitDistance must be >0.20 and <=0.50; exit frames must be positive");
  }
  goalYawTolerance = std::max(0.0, goalYawTolerance);
  goalConfirmFrames = std::max(1, goalConfirmFrames);
  goalStoppedLinearSpeed = std::max(0.0, goalStoppedLinearSpeed);
  goalStoppedAngularSpeed = std::max(0.0, goalStoppedAngularSpeed);

  auto subOdom = nh->create_subscription<nav_msgs::msg::Odometry>("/state_estimation", 5, odomHandler);

  auto subPath = nh->create_subscription<nav_msgs::msg::Path>("/path", 5, pathHandler);
  auto subFinalCorridor = nh->create_subscription<nav_msgs::msg::Path>(
    "/local_planner/final_path", 5, finalCorridorHandler);

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

      const bool reverseAllowed = reverseAllowedAtGoal();
      if (!reverseAllowed) {
        // Leaving the 50 cm zone revokes reverse immediately, without waiting
        // for the normal direction-switch delay or deceleration ramp.
        navFwd = true;
        if (vehicleSpeed < 0) vehicleSpeed = 0;
      } else {
        double time = nh->now().seconds();
        if (fabs(dirDiff) > PI / 2 && navFwd && time - switchTime > switchTimeThre) {
          navFwd = false;
          switchTime = time;
        } else if (fabs(dirDiff) < PI / 2 && !navFwd && time - switchTime > switchTimeThre) {
          navFwd = true;
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
        cmd_vel.header.stamp = rclcpp::Time(static_cast<uint64_t>(odomTime * 1e9));
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

        // Cover both acceleration residue and negative x from omnidirectional
        // projection/manual override before the final controller/status sees it.
        if (!reverseAllowed && cmd_vel.twist.linear.x < 0) {
          cmd_vel.twist.linear.x = 0;
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
