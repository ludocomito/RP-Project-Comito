// Shared 2D pose and angle helpers used by both the viewer and demo simulator.
#pragma once

#include <cmath>

#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

namespace rp_simple_rviz {

struct Pose2D {
  // Planar robot pose in x, y, yaw (heading in radians)
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
};

// Keep yaw in the conventional [-pi, pi] interval. This avoids slow drift when
// commands keep rotating the robot.
inline double normalizeAngle(double angle) {
  while (angle > M_PI) angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
}

// ROS poses use quaternions; this project is planar, so only yaw matters.
// The yaw is computed from the quaternion using the atan2 function.
inline double yawFromQuaternion(const geometry_msgs::msg::Quaternion& q) {  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

inline geometry_msgs::msg::Quaternion quaternionFromYaw(double yaw) {
  // Roll and pitch stay zero for the 2D mobile-base setting.
  geometry_msgs::msg::Quaternion q;
  q.w = std::cos(yaw * 0.5);
  q.z = std::sin(yaw * 0.5);
  return q;
}

// Compose a child pose expressed in a parent frame. This is the 2D equivalent
// of multiplying homogeneous transforms, and is used for robot->laser->scan.
inline Pose2D composePoses(const Pose2D& parent, const Pose2D& child) {
  const double c = std::cos(parent.yaw);
  const double s = std::sin(parent.yaw);

  Pose2D composed;
  composed.x = parent.x + c * child.x - s * child.y;
  composed.y = parent.y + s * child.x + c * child.y;
  composed.yaw = normalizeAngle(parent.yaw + child.yaw);
  return composed;
}

inline Pose2D transformToPose2D(
    const geometry_msgs::msg::TransformStamped& transform) {
  Pose2D pose;
  pose.x = transform.transform.translation.x;
  pose.y = transform.transform.translation.y;
  pose.yaw = yawFromQuaternion(transform.transform.rotation);
  return pose;
}

}  // namespace rp_simple_rviz
