// Loads or generates the demo occupancy map and answers collision queries.
#include "rp_simple_rviz/demo_world_node.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <stdexcept>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

namespace rp_simple_rviz {

void DemoWorldNode::makeGeneratedMap() {
  // Fallback map for running without assets. The default demo uses diag.png.
  map_msg_.header.frame_id = fixed_frame_;
  map_msg_.info.resolution = resolution_;
  map_msg_.info.width = static_cast<uint32_t>(width_);
  map_msg_.info.height = static_cast<uint32_t>(height_);
  map_msg_.info.origin.position.x = origin_x_;
  map_msg_.info.origin.position.y = origin_y_;
  map_msg_.info.origin.orientation.w = 1.0;
  map_msg_.data.assign(static_cast<size_t>(width_ * height_), 0);

  addBorder();
  addRect(-7.8, -1.4, -5.6, 1.0);
  addRect(-2.0, -6.0, -1.6, -0.4);
  addRect(-2.0, 1.2, -1.6, 5.4);
  addRect(1.1, 0.8, 7.2, 1.2);
  addRect(4.8, -5.2, 6.8, -3.2);
  addRect(2.6, 3.1, 3.0, 5.8);
}

void DemoWorldNode::loadMapFromImage(const std::string& image_path) {
  // The course diag image is RGBA. Alpha encodes unknown space; grayscale
  // encodes free versus occupied cells.
  cv::Mat image = cv::imread(image_path, cv::IMREAD_UNCHANGED);
  if (image.empty()) {
    throw std::runtime_error("Unable to load map image: " + image_path);
  }

  cv::Mat gray;
  if (image.channels() == 4) {
    cv::cvtColor(image, gray, cv::COLOR_BGRA2GRAY);
  } else if (image.channels() == 3) {
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
  } else {
    gray = image;
  }

  width_ = gray.cols;
  height_ = gray.rows;

  map_msg_.header.frame_id = fixed_frame_;
  map_msg_.info.resolution = resolution_;
  map_msg_.info.width = static_cast<uint32_t>(width_);
  map_msg_.info.height = static_cast<uint32_t>(height_);
  map_msg_.info.origin.position.x = origin_x_;
  map_msg_.info.origin.position.y = origin_y_;
  map_msg_.info.origin.orientation.w = 1.0;
  map_msg_.data.assign(static_cast<size_t>(width_ * height_), 0);

  for (int row = 0; row < height_; ++row) {
    for (int col = 0; col < width_; ++col) {
      int8_t occupancy = 0;
      const uint8_t alpha =
          image.channels() == 4 ? image.at<cv::Vec4b>(row, col)[3] : 255;
      const uint8_t value = gray.at<uint8_t>(row, col);

      if (alpha < 127) {
        occupancy = -1;
      } else if (value < 147) {
        occupancy = 100;
      }

      // The PNG rows are top-to-bottom; OccupancyGrid data is bottom-to-top.
      const int grid_row = height_ - row - 1;
      map_msg_.data[static_cast<size_t>(grid_row * width_ + col)] = occupancy;
    }
  }

  RCLCPP_INFO(get_logger(), "Loaded embedded map image %s (%dx%d, %.3f m/px)",
              image_path.c_str(), width_, height_, resolution_);
}

void DemoWorldNode::addBorder() {
  for (int x = 0; x < width_; ++x) {
    setCell(x, 0, 100);
    setCell(x, height_ - 1, 100);
  }
  for (int y = 0; y < height_; ++y) {
    setCell(0, y, 100);
    setCell(width_ - 1, y, 100);
  }
}

void DemoWorldNode::addRect(double min_x, double min_y, double max_x,
                            double max_y) {
  int x0 = 0;
  int y0 = 0;
  int x1 = 0;
  int y1 = 0;
  worldToGrid(min_x, min_y, x0, y0);
  worldToGrid(max_x, max_y, x1, y1);
  x0 = std::clamp(x0, 0, width_ - 1);
  x1 = std::clamp(x1, 0, width_ - 1);
  y0 = std::clamp(y0, 0, height_ - 1);
  y1 = std::clamp(y1, 0, height_ - 1);

  for (int y = std::min(y0, y1); y <= std::max(y0, y1); ++y) {
    for (int x = std::min(x0, x1); x <= std::max(x0, x1); ++x) {
      setCell(x, y, 100);
    }
  }
}

bool DemoWorldNode::worldToGrid(double wx, double wy, int& gx, int& gy) const {
  gx = static_cast<int>(std::floor((wx - origin_x_) / resolution_));
  gy = static_cast<int>(std::floor((wy - origin_y_) / resolution_));
  return gx >= 0 && gy >= 0 && gx < width_ && gy < height_;
}

void DemoWorldNode::setCell(int gx, int gy, int8_t value) {
  if (gx < 0 || gy < 0 || gx >= width_ || gy >= height_) return;
  map_msg_.data[static_cast<size_t>(gy * width_ + gx)] = value;
}

bool DemoWorldNode::isOccupied(double wx, double wy) const {
  int gx = 0;
  int gy = 0;
  // Outside the map is treated as occupied so the robot cannot leave the world.
  if (!worldToGrid(wx, wy, gx, gy)) return true;
  return map_msg_.data[static_cast<size_t>(gy * width_ + gx)] >= 50;
}

bool DemoWorldNode::footprintFree(const Pose2D& pose) const {
  if (isOccupied(pose.x, pose.y)) return false;

  // Check a few points around the circular robot body, not only its center.
  constexpr int kSamples = 16;
  for (int i = 0; i < kSamples; ++i) {
    const double angle = 2.0 * M_PI * static_cast<double>(i) / kSamples;
    const double x = pose.x + robot_radius_ * std::cos(angle);
    const double y = pose.y + robot_radius_ * std::sin(angle);
    if (isOccupied(x, y)) return false;
  }
  return true;
}

}  // namespace rp_simple_rviz
