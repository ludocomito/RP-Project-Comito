// Declares the viewer node: subscriptions, drawing state, and user interaction.
#pragma once

#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "rp_simple_rviz/pose2d.hpp"

namespace rp_simple_rviz {

enum class InteractionMode {
  // Idle means mouse drags do nothing except move the cursor.
  kIdle,
  // Initial pose and goal modes mimic RViz's 2D Pose Estimate / 2D Nav Goal.
  kSetInitialPose,
  kSetGoal,
};

class SimpleRvizNode : public rclcpp::Node {
 public:
  SimpleRvizNode();
  ~SimpleRvizNode() override;

 private:
  // ROS wiring: input topics for visualization and output topics for commands.
  void createMapSubscriptions();
  void createScanSubscriptions();
  void createParticleSubscriptions();
  void createPublishers();

  // Map handling: OccupancyGrid storage plus conversions between meters and
  // image pixels. The viewer draws in pixels but ROS messages use world meters.
  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  cv::Mat makeMapCanvas(const nav_msgs::msg::OccupancyGrid& map) const;
  bool worldToPixel(double wx, double wy, cv::Point& pixel) const;
  bool pixelToWorld(const cv::Point& pixel, double& wx, double& wy) const;
  std::optional<Pose2D> lookupTransform2D(
      const std::string& source_frame,
      const builtin_interfaces::msg::Time& stamp) const;

  // Rendering. Each function draws one logical layer on top of the static map.
  void drawLoop();
  void show(const cv::Mat& canvas);
  void drawParticles(cv::Mat& canvas);
  void drawRobotFrames(cv::Mat& canvas);
  void drawLaserScans(cv::Mat& canvas);
  void drawInteraction(cv::Mat& canvas) const;
  void drawMode(cv::Mat& canvas) const;

  // Interaction. Keyboard commands publish short Twist pulses; mouse drags
  // publish RViz-compatible initial pose and goal messages.
  void handleKey(int key);
  void publishCmdVel(double linear, double angular);
  void publishStopCmdVel();
  static void mouseCallback(int event, int x, int y, int flags, void* userdata);
  void handleMouse(int event, int x, int y);
  cv::Point displayToMapPixel(const cv::Point& display_pixel) const;
  void publishInteractivePose();
  void publishInitialPose(double x, double y, double yaw);
  void publishGoal(double x, double y, double yaw);

  // Topic/frame parameters are kept as members so launch files can remap the
  // viewer to a real robot without changing source code.
  std::string fixed_frame_;
  std::string map_topic_;
  std::vector<std::string> scan_topics_;
  std::vector<std::string> particle_topics_;
  std::vector<std::string> robot_frames_;
  double robot_radius_ = 0.5;
  double draw_rate_hz_ = 20.0;
  double display_scale_ = 1.0;
  std::string window_name_;

  std::string initialpose_topic_;
  std::vector<std::string> goal_topics_;
  std::string cmd_vel_topic_;
  double linear_speed_ = 0.25;
  double angular_speed_ = 0.8;
  double cmd_vel_pulse_seconds_ = 0.2;
  double initialpose_xy_covariance_ = 0.25;
  double initialpose_yaw_covariance_ = 0.07;

  // tf2 stores the live transform tree. The renderer queries it whenever it
  // needs to draw a robot, scan, or particle cloud in the fixed map frame.
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::vector<rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr>
      map_subs_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr>
      scan_subs_;
  std::vector<rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr>
      particle_subs_;

  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      initialpose_pub_;
  std::vector<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr>
      goal_pubs_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::TimerBase::SharedPtr draw_timer_;
  rclcpp::TimerBase::SharedPtr cmd_vel_stop_timer_;

  // The map is cached twice: as the ROS message for coordinates and as an
  // OpenCV image for fast redraws.
  nav_msgs::msg::OccupancyGrid::SharedPtr map_msg_;
  cv::Mat map_background_;
  Pose2D map_origin_;
  std::unordered_map<std::string, sensor_msgs::msg::LaserScan::SharedPtr>
      latest_scans_;
  std::unordered_map<std::string, geometry_msgs::msg::PoseArray::SharedPtr>
      latest_particle_clouds_;

  InteractionMode mode_ = InteractionMode::kIdle;
  bool dragging_ = false;
  cv::Point drag_start_pixel_{0, 0};
  cv::Point drag_current_pixel_{0, 0};
};

}  // namespace rp_simple_rviz
