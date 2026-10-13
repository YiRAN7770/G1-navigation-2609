#pragma once

#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <nav_msgs/msg/odometry.hpp>

namespace g1_navigation
{
inline Eigen::Matrix4f mountingRotation(bool inverted)
{
  Eigen::Matrix4f result = Eigen::Matrix4f::Identity();
  if (inverted) {result(1, 1) = -1.0f; result(2, 2) = -1.0f;}
  return result;
}

inline bool finitePose(const geometry_msgs::msg::Pose & p)
{
  const auto & q = p.orientation;
  const double norm = q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z;
  return std::isfinite(p.position.x) && std::isfinite(p.position.y) &&
    std::isfinite(p.position.z) && std::isfinite(norm) && norm > 1e-12;
}

inline Eigen::Matrix4f poseToMatrix(const geometry_msgs::msg::Pose & p)
{
  Eigen::Quaternionf q(p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z);
  Eigen::Matrix4f result = Eigen::Matrix4f::Identity();
  result.block<3, 3>(0, 0) = q.normalized().toRotationMatrix();
  result.block<3, 1>(0, 3) = Eigen::Vector3f(p.position.x, p.position.y, p.position.z);
  return result;
}

inline geometry_msgs::msg::Pose matrixToPose(const Eigen::Matrix4f & matrix)
{
  geometry_msgs::msg::Pose p;
  const Eigen::Quaternionf q(matrix.block<3, 3>(0, 0));
  p.position.x = matrix(0, 3); p.position.y = matrix(1, 3); p.position.z = matrix(2, 3);
  p.orientation.x = q.x(); p.orientation.y = q.y();
  p.orientation.z = q.z(); p.orientation.w = q.w();
  return p;
}

inline nav_msgs::msg::Odometry navigationOdometry(
  const nav_msgs::msg::Odometry & input, const Eigen::Matrix4f & map_from_local,
  const Eigen::Matrix4f & body_from_navigation, const std::string & map_frame,
  const std::string & navigation_frame)
{
  auto output = input;
  output.header.frame_id = map_frame;
  output.child_frame_id = navigation_frame;
  // Left rotation changes world coordinates; right rotation changes the
  // coincident child frame. For an inverted sensor BOTH rotations are needed.
  output.pose.pose = matrixToPose(
    map_from_local * poseToMatrix(input.pose.pose) * body_from_navigation);
  const Eigen::Matrix3d child_rotation =
    body_from_navigation.block<3, 3>(0, 0).cast<double>().transpose();
  auto rotate = [&child_rotation](const geometry_msgs::msg::Vector3 & v) {
    const Eigen::Vector3d rotated = child_rotation * Eigen::Vector3d(v.x, v.y, v.z);
    geometry_msgs::msg::Vector3 out;
    out.x = rotated.x(); out.y = rotated.y(); out.z = rotated.z(); return out;
  };
  output.twist.twist.linear = rotate(input.twist.twist.linear);
  output.twist.twist.angular = rotate(input.twist.twist.angular);
  using Covariance = Eigen::Matrix<double, 6, 6, Eigen::RowMajor>;
  Eigen::Matrix<double, 6, 6> rotation = Eigen::Matrix<double, 6, 6>::Zero();
  rotation.block<3, 3>(0, 0) = child_rotation;
  rotation.block<3, 3>(3, 3) = child_rotation;
  Eigen::Map<Covariance>(output.twist.covariance.data()) = rotation *
    Eigen::Map<const Covariance>(input.twist.covariance.data()) * rotation.transpose();
  // Pose covariance is expressed on fixed axes in the header frame.
  rotation.block<3, 3>(0, 0) = map_from_local.block<3, 3>(0, 0).cast<double>();
  rotation.block<3, 3>(3, 3) = rotation.block<3, 3>(0, 0);
  Eigen::Map<Covariance>(output.pose.covariance.data()) = rotation *
    Eigen::Map<const Covariance>(input.pose.covariance.data()) * rotation.transpose();
  return output;
}
}  // namespace g1_navigation
