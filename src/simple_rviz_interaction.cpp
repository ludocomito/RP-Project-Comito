// Handles keyboard/mouse input and publishes RViz-compatible command messages.
#include "rp_simple_rviz/simple_rviz_node.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>

#include <opencv2/highgui.hpp>

namespace rp_simple_rviz {

// The function that handles the keyboard events.
void SimpleRvizNode::handleKey(int key) {
  // If the escape key is pressed, shutdown the node.
  if (key == 27) {
    rclcpp::shutdown();
    return;
  }

  // If the key is not a valid ASCII character, return.
  if (key < 0 || key > 255) return;
  const int lower = std::tolower(key);

  switch (lower) {
    case 'i':
      // Next mouse drag publishes /initialpose.
      mode_ = InteractionMode::kSetInitialPose;
      dragging_ = false;
      break;
    case 'g':
      // Next mouse drag publishes /move_base/goal and any configured aliases.
      mode_ = InteractionMode::kSetGoal;
      dragging_ = false;
      break;
    case 'w':
      publishCmdVel(linear_speed_, 0.0);
      break;
    case 's':
      publishCmdVel(-linear_speed_, 0.0);
      break;
    case 'a':
      publishCmdVel(0.0, angular_speed_);
      break;
    case 'd':
      publishCmdVel(0.0, -angular_speed_);
      break;
    case 'x':
      publishCmdVel(0.0, 0.0);
      break;
    default:
      break;
  }
}

// Handle the publishing of the command velocity.
void SimpleRvizNode::publishCmdVel(double linear, double angular) {
  // The stop timer is used to stop the command velocity after a certain time.
  if (cmd_vel_stop_timer_) {
    cmd_vel_stop_timer_->cancel();
  }

  // Create the command velocity message.
  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = linear;
  cmd.angular.z = angular;
  cmd_vel_pub_->publish(cmd);

  // If the linear and angular velocities are close to zero, return.
  if (std::abs(linear) < 1e-9 && std::abs(angular) < 1e-9) return;
  if (cmd_vel_pulse_seconds_ <= 0.0) return;

  // Schedule an automatic stop so the command is discrete.
  const auto delay = std::chrono::milliseconds(
      std::max(1, static_cast<int>(std::round(1000.0 * cmd_vel_pulse_seconds_))));
  cmd_vel_stop_timer_ = create_wall_timer(delay, [this]() {
    publishStopCmdVel();
    if (cmd_vel_stop_timer_) {
      cmd_vel_stop_timer_->cancel();
    }
  });
}

// Publish the stop command velocity.
void SimpleRvizNode::publishStopCmdVel() {
  geometry_msgs::msg::Twist cmd;
  cmd_vel_pub_->publish(cmd);
}

void SimpleRvizNode::mouseCallback(int event, int x, int y, int flags,
                                   void* userdata) {
  (void)flags;
  auto* node = static_cast<SimpleRvizNode*>(userdata);
  if (node) node->handleMouse(event, x, y);
}

// Handle the mouse events.
void SimpleRvizNode::handleMouse(int event, int x, int y) {
  if (!map_msg_ || mode_ == InteractionMode::kIdle) return;

  // When le
  const cv::Point map_pixel = displayToMapPixel(cv::Point(x, y));
  if (event == cv::EVENT_LBUTTONDOWN) {
    // Start point is the pose position.
    dragging_ = true;
    drag_start_pixel_ = map_pixel;
    drag_current_pixel_ = map_pixel;
    return;
  }

  if (event == cv::EVENT_MOUSEMOVE && dragging_) {
    drag_current_pixel_ = map_pixel;
    return;
  }

  if (event == cv::EVENT_LBUTTONUP && dragging_) {
    drag_current_pixel_ = map_pixel;
    publishInteractivePose();
    dragging_ = false;
    mode_ = InteractionMode::kIdle;
  }
}

// Convert display pixel to map pixel.
cv::Point SimpleRvizNode::displayToMapPixel(const cv::Point& display_pixel) const {
  if (std::abs(display_scale_ - 1.0) < 1e-6) return display_pixel;
  return cv::Point(
      static_cast<int>(std::round(display_pixel.x / display_scale_)),
      static_cast<int>(std::round(display_pixel.y / display_scale_)));
}

// Publish the interactive pose.
void SimpleRvizNode::publishInteractivePose() {
  double x0 = 0.0;
  double y0 = 0.0;
  double x1 = 0.0;
  double y1 = 0.0;
  if (!pixelToWorld(drag_start_pixel_, x0, y0) ||
      !pixelToWorld(drag_current_pixel_, x1, y1)) {
    return;
  }

  // RViz uses click-and-drag: click gives translation, drag direction gives yaw.
  const double dx = x1 - x0;
  const double dy = y1 - y0;
  const double yaw = std::hypot(dx, dy) > 1e-6 ? std::atan2(dy, dx) : 0.0;

  if (mode_ == InteractionMode::kSetInitialPose) {
    publishInitialPose(x0, y0, yaw);
  } else if (mode_ == InteractionMode::kSetGoal) {
    publishGoal(x0, y0, yaw);
  }
}

// Publish the initial pose to the initialpose topic.
void SimpleRvizNode::publishInitialPose(double x, double y, double yaw) {
  geometry_msgs::msg::PoseWithCovarianceStamped msg;
  msg.header.stamp = now();
  msg.header.frame_id = fixed_frame_;
  msg.pose.pose.position.x = x;
  msg.pose.pose.position.y = y;
  msg.pose.pose.orientation = quaternionFromYaw(yaw);
  msg.pose.covariance[0] = initialpose_xy_covariance_;
  msg.pose.covariance[7] = initialpose_xy_covariance_;
  msg.pose.covariance[35] = initialpose_yaw_covariance_;

  initialpose_pub_->publish(msg);
  RCLCPP_INFO(get_logger(), "Published initial pose to %s: %.2f %.2f %.2f",
              initialpose_topic_.c_str(), x, y, yaw);
}

// Publish the goal to the goal topics.
void SimpleRvizNode::publishGoal(double x, double y, double yaw) {
  geometry_msgs::msg::PoseStamped msg;
  msg.header.stamp = now();
  msg.header.frame_id = fixed_frame_;
  msg.pose.position.x = x;
  msg.pose.position.y = y;
  msg.pose.orientation = quaternionFromYaw(yaw);

  for (const auto& publisher : goal_pubs_) publisher->publish(msg);

  RCLCPP_INFO(get_logger(), "Published goal: %.2f %.2f %.2f", x, y, yaw);
}

}  // namespace rp_simple_rviz
