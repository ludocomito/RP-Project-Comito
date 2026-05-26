#include <algorithm>
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>

namespace {

class MapServerNode : public rclcpp::Node {
 public:
  MapServerNode() : Node("simple_map_server") {
    fixed_frame_ = declare_parameter<std::string>("fixed_frame", "map");
    map_topic_ = declare_parameter<std::string>("map_topic", "/map");
    map_path_ = declare_parameter<std::string>("map_path", "");
    resolution_ = declare_parameter<double>("resolution", 0.1);
    origin_x_ = declare_parameter<double>("origin_x", 0.0);
    origin_y_ = declare_parameter<double>("origin_y", -98.7);
    occupied_threshold_ =
        declare_parameter<int>("occupied_threshold", 147);

    if (map_path_.empty()) {
      throw std::runtime_error("map_path parameter is required");
    }
    resolution_ = std::max(1e-6, resolution_);
    occupied_threshold_ = std::clamp(occupied_threshold_, 0, 255);

    loadMap();

    map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
        map_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());
    timer_ = create_wall_timer(std::chrono::seconds(1),
                               std::bind(&MapServerNode::publishMap, this));
    publishMap();

    RCLCPP_INFO(get_logger(), "Publishing map %s on %s", map_path_.c_str(),
                map_topic_.c_str());
  }

 private:
  void loadMap() {
    const cv::Mat image = cv::imread(map_path_, cv::IMREAD_UNCHANGED);
    if (image.empty()) {
      throw std::runtime_error("Unable to load map image: " + map_path_);
    }

    cv::Mat gray;
    if (image.channels() == 4) {
      cv::cvtColor(image, gray, cv::COLOR_BGRA2GRAY);
    } else if (image.channels() == 3) {
      cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    } else {
      gray = image;
    }

    map_msg_.header.frame_id = fixed_frame_;
    map_msg_.info.resolution = resolution_;
    map_msg_.info.width = static_cast<uint32_t>(gray.cols);
    map_msg_.info.height = static_cast<uint32_t>(gray.rows);
    map_msg_.info.origin.position.x = origin_x_;
    map_msg_.info.origin.position.y = origin_y_;
    map_msg_.info.origin.orientation.w = 1.0;
    map_msg_.data.assign(static_cast<size_t>(gray.cols * gray.rows), 0);

    for (int row = 0; row < gray.rows; ++row) {
      for (int col = 0; col < gray.cols; ++col) {
        int8_t occupancy = 0;
        const uint8_t alpha =
            image.channels() == 4 ? image.at<cv::Vec4b>(row, col)[3] : 255;
        const uint8_t value = gray.at<uint8_t>(row, col);

        if (alpha < 127) {
          occupancy = -1;
        } else if (value < occupied_threshold_) {
          occupancy = 100;
        }

        const int grid_row = gray.rows - row - 1;
        map_msg_.data[static_cast<size_t>(grid_row * gray.cols + col)] =
            occupancy;
      }
    }
  }

  void publishMap() {
    map_msg_.header.stamp = now();
    map_pub_->publish(map_msg_);
  }

  std::string fixed_frame_;
  std::string map_topic_;
  std::string map_path_;
  double resolution_ = 0.1;
  double origin_x_ = 0.0;
  double origin_y_ = -98.7;
  int occupied_threshold_ = 147;

  nav_msgs::msg::OccupancyGrid map_msg_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapServerNode>());
  rclcpp::shutdown();
  return 0;
}
