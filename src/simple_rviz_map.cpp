// Converts ROS maps and TF transforms into drawable OpenCV coordinates.
#include "rp_simple_rviz/simple_rviz_node.hpp"

#include <cmath>
#include <cstdint>

#include <opencv2/imgproc.hpp>
#include <tf2/exceptions.h>
#include <tf2/time.h>

namespace rp_simple_rviz {

void SimpleRvizNode::mapCallback(
    const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  if (msg->info.width == 0 || msg->info.height == 0 ||
      msg->info.resolution <= 0.0) {
    RCLCPP_WARN(get_logger(), "Ignoring invalid OccupancyGrid on %s",
                map_topic_.c_str());
    return;
  }

  if (map_msg_ && msg->info == map_msg_->info && msg->data == map_msg_->data) {
    map_msg_->header = msg->header;
    return;
  }

  map_msg_ = msg;
  map_origin_.x = msg->info.origin.position.x;
  map_origin_.y = msg->info.origin.position.y;
  map_origin_.yaw = yawFromQuaternion(msg->info.origin.orientation);
  map_background_ = makeMapCanvas(*msg);
}

cv::Mat SimpleRvizNode::makeMapCanvas(
    const nav_msgs::msg::OccupancyGrid& map) const {
  // Create a grayscale image from the occupancy grid.
  cv::Mat gray(static_cast<int>(map.info.height),
               static_cast<int>(map.info.width), CV_8UC1, cv::Scalar(180));

  for (int display_row = 0; display_row < static_cast<int>(map.info.height);
       ++display_row) {
    // display_row is top-to-bottom for OpenCV; map_row is bottom-to-top for ROS.
    const int map_row = static_cast<int>(map.info.height) - display_row - 1;
    for (int col = 0; col < static_cast<int>(map.info.width); ++col) {
      const int index = map_row * static_cast<int>(map.info.width) + col;
      const int occupancy = static_cast<int>(map.data[index]);

      uint8_t value = 180;
      if (occupancy >= 50) {
        // Occupied cells are dark.
        value = 20;
      } else if (occupancy >= 0) {
        // Free cells are light; unknown cells keep the default gray.
        value = 245;
      }
      gray.at<uint8_t>(display_row, col) = value;
    }
  }

  cv::Mat bgr;
  cv::cvtColor(gray, bgr, cv::COLOR_GRAY2BGR);
  return bgr;
}

bool SimpleRvizNode::worldToPixel(double wx, double wy, cv::Point& pixel) const {
  if (!map_msg_) return false;

  const auto& info = map_msg_->info;
  const double dx = wx - map_origin_.x;
  const double dy = wy - map_origin_.y;

  // OccupancyGrid origin can have a yaw. Rotate the world point into the map
  // grid frame before converting meters to cell indices.
  const double c = std::cos(-map_origin_.yaw);
  const double s = std::sin(-map_origin_.yaw);
  const double mx = c * dx - s * dy;
  const double my = s * dx + c * dy;

  const int col = static_cast<int>(std::floor(mx / info.resolution));
  const int row_from_bottom = static_cast<int>(std::floor(my / info.resolution));

  if (col < 0 || row_from_bottom < 0 || col >= static_cast<int>(info.width) ||
      row_from_bottom >= static_cast<int>(info.height)) {
    return false;
  }

  // ROS maps count rows from the bottom; OpenCV images count rows from the top.
  pixel.x = col;
  pixel.y = static_cast<int>(info.height) - row_from_bottom - 1;
  return true;
}

bool SimpleRvizNode::pixelToWorld(const cv::Point& pixel, double& wx,
                                  double& wy) const {
  // Convert map pixel to world coordinates.
  if (!map_msg_) return false;

  const auto& info = map_msg_->info;
  if (pixel.x < 0 || pixel.y < 0 || pixel.x >= static_cast<int>(info.width) ||
      pixel.y >= static_cast<int>(info.height)) {
    return false;
  }

  const double mx = (static_cast<double>(pixel.x) + 0.5) * info.resolution;
  const double my =
      (static_cast<double>(info.height - pixel.y - 1) + 0.5) * info.resolution;

  // Rotate back from map-grid coordinates into the fixed/world frame.
  const double c = std::cos(map_origin_.yaw);
  const double s = std::sin(map_origin_.yaw);
  wx = map_origin_.x + c * mx - s * my;
  wy = map_origin_.y + s * mx + c * my;
  return true;
}

std::optional<Pose2D> SimpleRvizNode::lookupTransform2D(
    const std::string& source_frame,
    const builtin_interfaces::msg::Time& stamp) const {
  if (source_frame.empty()) return std::nullopt;

  try {
    geometry_msgs::msg::TransformStamped transform;
    if (stamp.sec == 0 && stamp.nanosec == 0) {
      transform = tf_buffer_->lookupTransform(fixed_frame_, source_frame,
                                              tf2::TimePointZero);
    } else {
      // Do not block rendering while waiting for an exact-time transform. If it
      // is not already available, fall back immediately to the latest TF below.
      transform = tf_buffer_->lookupTransform(
          fixed_frame_, source_frame, rclcpp::Time(stamp),
          rclcpp::Duration::from_seconds(0.0));
    }

    return transformToPose2D(transform);
  } catch (const tf2::TransformException&) {
    // When exact-time TF is not available, falling back to the latest transform
    // is enough for a teaching viewer and keeps the drawing responsive.
    try {
      const auto transform = tf_buffer_->lookupTransform(
          fixed_frame_, source_frame, tf2::TimePointZero);
      return transformToPose2D(transform);
    } catch (const tf2::TransformException&) {
      return std::nullopt;
    }
  }
}

}  // namespace rp_simple_rviz
