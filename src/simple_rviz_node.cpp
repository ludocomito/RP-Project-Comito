// Implements viewer startup: parameters, ROS subscriptions/publishers, and main.
#include "rp_simple_rviz/simple_rviz_node.hpp"

#include <algorithm>
#include <cmath>
#include <functional>

#include <opencv2/highgui.hpp>

namespace rp_simple_rviz {

// The class that implements the simple RViz node.
SimpleRvizNode::SimpleRvizNode() : Node("simple_rviz_node") {
  fixed_frame_ = declare_parameter<std::string>("fixed_frame", "map");
  map_topic_ = declare_parameter<std::string>("map_topic", "/map");
  scan_topics_ = declare_parameter<std::vector<std::string>>(
      "scan_topics", std::vector<std::string>{"/laser_1/scan"});
  particle_topics_ = declare_parameter<std::vector<std::string>>(
      "particle_topics",
      std::vector<std::string>{"/particle_cloud", "/particlecloud",
                               "/particles"});
  particle_coordinate_mode_ =
      declare_parameter<std::string>("particle_coordinate_mode", "auto");
  particle_radius_px_ = std::max(
      1, static_cast<int>(declare_parameter<int>("particle_radius_px", 3)));
  particle_heading_px_ = std::max(
      0, static_cast<int>(declare_parameter<int>("particle_heading_px", 10)));
  max_drawn_particles_ = std::max(
      1, static_cast<int>(declare_parameter<int>("max_drawn_particles", 3000)));
  robot_frames_ = declare_parameter<std::vector<std::string>>(
      "robot_frames", std::vector<std::string>{"robot_1"});

  robot_radius_ = declare_parameter<double>("robot_radius", 0.5);
  draw_rate_hz_ = declare_parameter<double>("draw_rate_hz", 20.0);
  display_scale_ = declare_parameter<double>("display_scale", 1.0);
  window_name_ = declare_parameter<std::string>("window_name", "simple_rviz");

  initialpose_topic_ =
      declare_parameter<std::string>("initialpose_topic", "/initialpose");
  goal_topics_ = declare_parameter<std::vector<std::string>>(
      "goal_topics", std::vector<std::string>{"/move_base/goal"});
  cmd_vel_topic_ =
      declare_parameter<std::string>("cmd_vel_topic", "/robot_1/cmd_vel");

  linear_speed_ = declare_parameter<double>("linear_speed", 0.25);
  angular_speed_ = declare_parameter<double>("angular_speed", 0.8);
  // The time in seconds that the command velocity is published.
  cmd_vel_pulse_seconds_ =
      declare_parameter<double>("cmd_vel_pulse_seconds", 0.2);

  initialpose_xy_covariance_ =
      declare_parameter<double>("initialpose_xy_covariance", 0.25);
  initialpose_yaw_covariance_ =
      declare_parameter<double>("initialpose_yaw_covariance", 0.07);

  draw_rate_hz_ = std::max(1.0, draw_rate_hz_);
  display_scale_ = std::max(0.1, display_scale_);

  // The TransformListener subscribes to /tf and fills tf_buffer_ in the
  // background; drawing code later asks the buffer for frame poses.
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  createMapSubscriptions();
  createScanSubscriptions();
  createParticleSubscriptions();
  createPublishers();

  // OpenCV provides the window and mouse callback. This keeps the project
  // simple and avoids a full GUI toolkit.
  cv::namedWindow(window_name_, cv::WINDOW_AUTOSIZE);
  cv::setMouseCallback(window_name_, &SimpleRvizNode::mouseCallback, this);

  const auto period = std::chrono::milliseconds(
      static_cast<int>(std::round(1000.0 / draw_rate_hz_)));
  draw_timer_ =
      create_wall_timer(period, std::bind(&SimpleRvizNode::drawLoop, this));

  RCLCPP_INFO(get_logger(),
              "simple_rviz_node running. Press i/g and drag on the map to "
              "publish /initialpose or a goal; w/a/s/d/x publish cmd_vel pulses.");
}

SimpleRvizNode::~SimpleRvizNode() { cv::destroyWindow(window_name_); }

void SimpleRvizNode::createMapSubscriptions() {
  const auto map_callback =
      [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        mapCallback(msg);
      };

  // Some map sources behave like a simulator and publish repeatedly with
  // volatile durability. Map servers often publish once as transient-local.
  // Keeping both subscriptions makes the viewer tolerant to either source.
  map_subs_.push_back(create_subscription<nav_msgs::msg::OccupancyGrid>(
      map_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile(),
      map_callback));
  map_subs_.push_back(create_subscription<nav_msgs::msg::OccupancyGrid>(
      map_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
      map_callback));
}

void SimpleRvizNode::createScanSubscriptions() {
  for (const auto& topic : scan_topics_) {
    if (topic.empty()) continue;
    // SensorDataQoS matches LaserScan publishers that prioritize latest data
    // over guaranteed delivery.
    scan_subs_.push_back(create_subscription<sensor_msgs::msg::LaserScan>(
        topic, rclcpp::SensorDataQoS(),
        [this, topic](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
          latest_scans_[topic] = msg;
        }));
  }
}

void SimpleRvizNode::createParticleSubscriptions() {
  for (const auto& topic : particle_topics_) {
    if (topic.empty()) continue;
    particle_subs_.push_back(create_subscription<geometry_msgs::msg::PoseArray>(
        topic, 10,
        [this, topic](const geometry_msgs::msg::PoseArray::SharedPtr msg) {
          latest_particle_clouds_[topic] = msg;
        }));
  }
}

// Create the publishers for the initial pose, goal, and command velocity.
void SimpleRvizNode::createPublishers() {
  initialpose_pub_ =
      create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
          initialpose_topic_, 10);

  for (const auto& topic : goal_topics_) {
    if (topic.empty()) continue;
    goal_pubs_.push_back(
        create_publisher<geometry_msgs::msg::PoseStamped>(topic, 10));
  }

  cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);
}

}  // namespace rp_simple_rviz

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<rp_simple_rviz::SimpleRvizNode>());
  rclcpp::shutdown();
  return 0;
}
