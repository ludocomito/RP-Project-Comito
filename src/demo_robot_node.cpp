#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "rp_simple_rviz/pose2d.hpp"

namespace rp_simple_rviz {
namespace {

class DemoRobotNode : public rclcpp::Node {
 public:
  DemoRobotNode() : Node("demo_robot_node"), rng_(7) {
    fixed_frame_ = declare_parameter<std::string>("fixed_frame", "map");
    map_topic_ = declare_parameter<std::string>("map_topic", "/map");
    robot_frame_ = declare_parameter<std::string>("robot_frame", "robot_1");
    laser_frame_ = declare_parameter<std::string>("laser_frame", "laser_1");
    scan_topic_ = declare_parameter<std::string>("scan_topic", "/laser_1/scan");
    particle_topic_ =
        declare_parameter<std::string>("particle_topic", "/particle_cloud");
    cmd_vel_topic_ =
        declare_parameter<std::string>("cmd_vel_topic", "/robot_1/cmd_vel");
    initialpose_topic_ =
        declare_parameter<std::string>("initialpose_topic", "/initialpose");
    goal_topic_ = declare_parameter<std::string>("goal_topic", "/move_base/goal");

    robot_pose_.x = declare_parameter<double>("robot_x", 18.0);
    robot_pose_.y = declare_parameter<double>("robot_y", -25.0);
    robot_pose_.yaw = declare_parameter<double>("robot_yaw", 0.0);
    robot_radius_ = declare_parameter<double>("robot_radius", 0.5);

    laser_in_robot_.x = declare_parameter<double>("laser_x", 1.0);
    laser_in_robot_.y = declare_parameter<double>("laser_y", 0.0);
    laser_in_robot_.yaw = declare_parameter<double>("laser_yaw", 0.0);
    laser_range_min_ = declare_parameter<double>("laser_range_min", 0.05);
    laser_range_max_ = declare_parameter<double>("laser_range_max", 20.0);
    laser_angle_min_ =
        declare_parameter<double>("laser_angle_min", -0.5 * M_PI);
    laser_angle_max_ =
        declare_parameter<double>("laser_angle_max", 0.5 * M_PI);
    laser_angle_increment_ =
        declare_parameter<double>("laser_angle_increment", M_PI / 180.0);

    cmd_timeout_seconds_ = declare_parameter<double>("cmd_timeout_seconds", 0.4);
    update_rate_hz_ =
        std::max(1.0, declare_parameter<double>("update_rate_hz", 30.0));
    scan_rate_hz_ =
        std::max(1.0, declare_parameter<double>("scan_rate_hz", 15.0));
    particle_rate_hz_ =
        std::max(0.2, declare_parameter<double>("particle_rate_hz", 2.0));
    particle_count_ = std::max(
        0, static_cast<int>(declare_parameter<int>("particle_count", 120)));

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    scan_pub_ =
        create_publisher<sensor_msgs::msg::LaserScan>(scan_topic_, 10);
    particle_pub_ =
        create_publisher<geometry_msgs::msg::PoseArray>(particle_topic_, 10);

    createMapSubscriptions();
    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        cmd_vel_topic_, 10,
        [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
          linear_velocity_ = msg->linear.x;
          angular_velocity_ = msg->angular.z;
          last_cmd_time_seconds_ = now().seconds();
        });
    initialpose_sub_ =
        create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            initialpose_topic_, 10,
            [this](
                const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr
                    msg) {
              Pose2D pose;
              pose.x = msg->pose.pose.position.x;
              pose.y = msg->pose.pose.position.y;
              pose.yaw = yawFromQuaternion(msg->pose.pose.orientation);
              if (!map_msg_ || footprintFree(pose)) {
                robot_pose_ = pose;
              }
            });
    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        goal_topic_, 10,
        [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
          RCLCPP_INFO(get_logger(), "Received goal on %s: %.2f %.2f",
                      goal_topic_.c_str(), msg->pose.position.x,
                      msg->pose.position.y);
        });

    update_timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::duration<double>(1.0 / update_rate_hz_)),
        [this]() { updateRobot(); });
    scan_timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::duration<double>(1.0 / scan_rate_hz_)),
        [this]() { publishScan(); });
    particle_timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::duration<double>(1.0 / particle_rate_hz_)),
        [this]() { publishParticles(); });

    RCLCPP_INFO(get_logger(),
                "Headless demo robot publishing TF, %s, and %s.",
                scan_topic_.c_str(), particle_topic_.c_str());
  }

 private:
  void createMapSubscriptions() {
    const auto callback =
        [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
          mapCallback(msg);
        };

    map_subs_.push_back(create_subscription<nav_msgs::msg::OccupancyGrid>(
        map_topic_,
        rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile(),
        callback));
    map_subs_.push_back(create_subscription<nav_msgs::msg::OccupancyGrid>(
        map_topic_,
        rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
        callback));
  }

  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    if (msg->info.width == 0 || msg->info.height == 0 ||
        msg->info.resolution <= 0.0) {
      return;
    }
    map_msg_ = msg;
    map_origin_.x = msg->info.origin.position.x;
    map_origin_.y = msg->info.origin.position.y;
    map_origin_.yaw = yawFromQuaternion(msg->info.origin.orientation);
  }

  void updateRobot() {
    const double now_seconds = now().seconds();
    double linear = 0.0;
    double angular = 0.0;
    if (now_seconds - last_cmd_time_seconds_ <= cmd_timeout_seconds_) {
      linear = linear_velocity_;
      angular = angular_velocity_;
    }

    const double dt = std::clamp(now_seconds - last_update_seconds_, 0.0, 0.1);
    last_update_seconds_ = now_seconds;

    Pose2D next = robot_pose_;
    next.x += linear * std::cos(robot_pose_.yaw) * dt;
    next.y += linear * std::sin(robot_pose_.yaw) * dt;
    next.yaw = normalizeAngle(robot_pose_.yaw + angular * dt);

    if (!map_msg_ || footprintFree(next)) {
      robot_pose_ = next;
    } else {
      robot_pose_.yaw = next.yaw;
    }

    publishTransforms();
  }

  void publishTransforms() {
    const auto stamp = now();
    geometry_msgs::msg::TransformStamped robot_tf;
    robot_tf.header.stamp = stamp;
    robot_tf.header.frame_id = fixed_frame_;
    robot_tf.child_frame_id = robot_frame_;
    robot_tf.transform.translation.x = robot_pose_.x;
    robot_tf.transform.translation.y = robot_pose_.y;
    robot_tf.transform.rotation = quaternionFromYaw(robot_pose_.yaw);

    geometry_msgs::msg::TransformStamped laser_tf;
    laser_tf.header.stamp = stamp;
    laser_tf.header.frame_id = robot_frame_;
    laser_tf.child_frame_id = laser_frame_;
    laser_tf.transform.translation.x = laser_in_robot_.x;
    laser_tf.transform.translation.y = laser_in_robot_.y;
    laser_tf.transform.rotation = quaternionFromYaw(laser_in_robot_.yaw);

    tf_broadcaster_->sendTransform(robot_tf);
    tf_broadcaster_->sendTransform(laser_tf);
  }

  void publishScan() {
    if (!map_msg_) return;

    const Pose2D sensor_pose = composePoses(robot_pose_, laser_in_robot_);
    const int beam_count = std::max(
        1, static_cast<int>(
               std::floor((laser_angle_max_ - laser_angle_min_) /
                          laser_angle_increment_)) +
               1);

    sensor_msgs::msg::LaserScan scan;
    scan.header.stamp = now();
    scan.header.frame_id = laser_frame_;
    scan.angle_min = static_cast<float>(laser_angle_min_);
    scan.angle_max = static_cast<float>(laser_angle_max_);
    scan.angle_increment = static_cast<float>(laser_angle_increment_);
    scan.time_increment = 0.0f;
    scan.scan_time = static_cast<float>(1.0 / scan_rate_hz_);
    scan.range_min = static_cast<float>(laser_range_min_);
    scan.range_max = static_cast<float>(laser_range_max_);
    scan.ranges.assign(static_cast<size_t>(beam_count),
                       std::numeric_limits<float>::infinity());

    const double step =
        std::max(0.02, static_cast<double>(map_msg_->info.resolution) * 0.5);
    for (int i = 0; i < beam_count; ++i) {
      const double local_angle =
          laser_angle_min_ + static_cast<double>(i) * laser_angle_increment_;
      const double world_angle = sensor_pose.yaw + local_angle;

      for (double r = laser_range_min_; r <= laser_range_max_; r += step) {
        const double x = sensor_pose.x + r * std::cos(world_angle);
        const double y = sensor_pose.y + r * std::sin(world_angle);
        if (isOccupied(x, y)) {
          scan.ranges[static_cast<size_t>(i)] = static_cast<float>(r);
          break;
        }
      }
    }

    scan_pub_->publish(scan);
  }

  void publishParticles() {
    if (particle_count_ == 0) return;

    std::normal_distribution<double> xy_noise(0.0, 0.25);
    std::normal_distribution<double> yaw_noise(0.0, 0.18);

    geometry_msgs::msg::PoseArray cloud;
    cloud.header.stamp = now();
    cloud.header.frame_id = fixed_frame_;
    cloud.poses.reserve(static_cast<size_t>(particle_count_));

    for (int i = 0; i < particle_count_; ++i) {
      geometry_msgs::msg::Pose pose;
      pose.position.x = robot_pose_.x + xy_noise(rng_);
      pose.position.y = robot_pose_.y + xy_noise(rng_);
      pose.orientation =
          quaternionFromYaw(robot_pose_.yaw + yaw_noise(rng_));
      cloud.poses.push_back(pose);
    }

    particle_pub_->publish(cloud);
  }

  bool worldToGrid(double wx, double wy, int& gx, int& gy) const {
    if (!map_msg_) return false;

    const double dx = wx - map_origin_.x;
    const double dy = wy - map_origin_.y;
    const double c = std::cos(-map_origin_.yaw);
    const double s = std::sin(-map_origin_.yaw);
    const double mx = c * dx - s * dy;
    const double my = s * dx + c * dy;

    gx = static_cast<int>(std::floor(mx / map_msg_->info.resolution));
    gy = static_cast<int>(std::floor(my / map_msg_->info.resolution));
    return gx >= 0 && gy >= 0 &&
           gx < static_cast<int>(map_msg_->info.width) &&
           gy < static_cast<int>(map_msg_->info.height);
  }

  bool isOccupied(double wx, double wy) const {
    int gx = 0;
    int gy = 0;
    if (!worldToGrid(wx, wy, gx, gy)) return true;

    const auto width = static_cast<int>(map_msg_->info.width);
    const int occupancy =
        static_cast<int>(map_msg_->data[static_cast<size_t>(gy * width + gx)]);
    return occupancy >= 50;
  }

  bool footprintFree(const Pose2D& pose) const {
    if (isOccupied(pose.x, pose.y)) return false;

    constexpr int kSamples = 16;
    for (int i = 0; i < kSamples; ++i) {
      const double angle = 2.0 * M_PI * static_cast<double>(i) / kSamples;
      const double x = pose.x + robot_radius_ * std::cos(angle);
      const double y = pose.y + robot_radius_ * std::sin(angle);
      if (isOccupied(x, y)) return false;
    }
    return true;
  }

  std::string fixed_frame_;
  std::string map_topic_;
  std::string robot_frame_;
  std::string laser_frame_;
  std::string scan_topic_;
  std::string particle_topic_;
  std::string cmd_vel_topic_;
  std::string initialpose_topic_;
  std::string goal_topic_;

  Pose2D robot_pose_;
  Pose2D laser_in_robot_;
  Pose2D map_origin_;
  double robot_radius_ = 0.5;
  double laser_range_min_ = 0.05;
  double laser_range_max_ = 20.0;
  double laser_angle_min_ = -0.5 * M_PI;
  double laser_angle_max_ = 0.5 * M_PI;
  double laser_angle_increment_ = M_PI / 180.0;
  double cmd_timeout_seconds_ = 0.4;
  double update_rate_hz_ = 30.0;
  double scan_rate_hz_ = 15.0;
  double particle_rate_hz_ = 2.0;
  int particle_count_ = 120;

  double linear_velocity_ = 0.0;
  double angular_velocity_ = 0.0;
  double last_cmd_time_seconds_ = -1000.0;
  double last_update_seconds_ = 0.0;

  std::mt19937 rng_;
  nav_msgs::msg::OccupancyGrid::SharedPtr map_msg_;
  std::vector<rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr>
      map_subs_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      initialpose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr particle_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr update_timer_;
  rclcpp::TimerBase::SharedPtr scan_timer_;
  rclcpp::TimerBase::SharedPtr particle_timer_;
};

}  // namespace
}  // namespace rp_simple_rviz

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<rp_simple_rviz::DemoRobotNode>());
  rclcpp::shutdown();
  return 0;
}
