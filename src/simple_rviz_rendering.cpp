// Draws the viewer layers: map, particles, scans, robot frames, and overlays.
#include "rp_simple_rviz/simple_rviz_node.hpp"

#include <algorithm>
#include <cmath>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

namespace rp_simple_rviz {

void SimpleRvizNode::drawLoop() {
  // This is the main loop that draws the viewer layers.
  
  // Draw the waiting screen until the map is available.
  if (!map_msg_ || map_background_.empty()) {
    cv::Mat waiting(120, 520, CV_8UC3, cv::Scalar(245, 245, 245));
    cv::putText(waiting, "waiting for map on " + map_topic_, cv::Point(16, 64),
                cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(40, 40, 40), 1,
                cv::LINE_AA);
    show(waiting);
    return;
  }

  // Draw uncertain localization first, then robot/laser on top.
  cv::Mat canvas = map_background_.clone();
  drawParticles(canvas);
  drawLaserScans(canvas);
  drawRobotFrames(canvas);
  drawInteraction(canvas);
  drawMode(canvas);
  show(canvas);
}

void SimpleRvizNode::show(const cv::Mat& canvas) {
  // Show the canvas in the window.
  if (std::abs(display_scale_ - 1.0) < 1e-6) {
    cv::imshow(window_name_, canvas);
  } else {
    cv::Mat scaled;
    cv::resize(canvas, scaled, cv::Size(), display_scale_, display_scale_,
               cv::INTER_NEAREST);
    cv::imshow(window_name_, scaled);
  }

  const int key = cv::waitKey(1);
  if (key >= 0) handleKey(key);
}

void SimpleRvizNode::drawParticles(cv::Mat& canvas) {
  // Draw the particle clouds.
  constexpr size_t kMaxDrawnParticles = 2000;

  // Iterate over the particle clouds.
  for (const auto& item : latest_particle_clouds_) {
    const auto& cloud = item.second;
    if (!cloud || cloud->poses.empty()) continue;

    Pose2D cloud_in_fixed;
    bool needs_transform = false;
    if (!cloud->header.frame_id.empty() && cloud->header.frame_id != fixed_frame_) {
      const auto cloud_pose =
          lookupTransform2D(cloud->header.frame_id, cloud->header.stamp);
      if (!cloud_pose.has_value()) continue;
      cloud_in_fixed = cloud_pose.value();
      needs_transform = true;
    }

      //Subsample the particle cloud for display while keeping the cloud shape visible.
      const size_t stride =
        std::max<size_t>(1, cloud->poses.size() / kMaxDrawnParticles);
    for (size_t i = 0; i < cloud->poses.size(); i += stride) {
      Pose2D particle;
      particle.x = cloud->poses[i].position.x;
      particle.y = cloud->poses[i].position.y;
      particle.yaw = yawFromQuaternion(cloud->poses[i].orientation);

      const Pose2D particle_in_fixed =
          needs_transform ? composePoses(cloud_in_fixed, particle) : particle;

      cv::Point pixel;
      if (worldToPixel(particle_in_fixed.x, particle_in_fixed.y, pixel)) {
        cv::circle(canvas, pixel, 1, cv::Scalar(255, 70, 70), -1);
      }
    }
  }
}

void SimpleRvizNode::drawRobotFrames(cv::Mat& canvas) {
  // Draw the robot frames.
  if (!map_msg_) return;

  builtin_interfaces::msg::Time latest;
  const int radius_px = std::max(
      2, static_cast<int>(std::round(robot_radius_ / map_msg_->info.resolution)));

  for (const auto& frame : robot_frames_) {
    const auto pose = lookupTransform2D(frame, latest);
    if (!pose.has_value()) continue;

    cv::Point center;
    if (!worldToPixel(pose->x, pose->y, center)) continue;

    cv::circle(canvas, center, radius_px, cv::Scalar(0, 0, 230), 2);

    cv::Point nose;
    const double nx = pose->x + robot_radius_ * std::cos(pose->yaw);
    const double ny = pose->y + robot_radius_ * std::sin(pose->yaw);
    if (worldToPixel(nx, ny, nose)) {
      cv::line(canvas, center, nose, cv::Scalar(0, 0, 230), 2, cv::LINE_AA);
    }

    cv::putText(canvas, frame, center + cv::Point(4, -4),
                cv::FONT_HERSHEY_SIMPLEX, 0.35, cv::Scalar(0, 0, 180), 1,
                cv::LINE_AA);
  }
}

void SimpleRvizNode::drawLaserScans(cv::Mat& canvas) {
  // Draw the laser scans.
  for (const auto& item : latest_scans_) {
    const auto& scan = item.second;
    if (!scan) continue;

    // Laser ranges arrive in the laser frame. The transform tells us where that frame is
    // in the fixed map frame.
    const auto sensor_pose =
        lookupTransform2D(scan->header.frame_id, scan->header.stamp);
    if (!sensor_pose.has_value()) continue;

    cv::Point sensor_pixel;
    const bool sensor_visible =
        worldToPixel(sensor_pose->x, sensor_pose->y, sensor_pixel);

    double angle = static_cast<double>(scan->angle_min);
    for (const float range : scan->ranges) {
      const bool has_hit = std::isfinite(range) && range >= scan->range_min &&
                           range <= scan->range_max;
      const bool no_return = !std::isfinite(range) || range > scan->range_max;

      if (has_hit || no_return) {
        const double draw_range =
            has_hit ? static_cast<double>(range)
                    : static_cast<double>(scan->range_max);

        Pose2D beam_end_in_sensor;
        beam_end_in_sensor.x = draw_range * std::cos(angle);
        beam_end_in_sensor.y = draw_range * std::sin(angle);

        const Pose2D beam_end_in_fixed =
            composePoses(sensor_pose.value(), beam_end_in_sensor);
        cv::Point beam_end_pixel;
        bool beam_end_visible =
            worldToPixel(beam_end_in_fixed.x, beam_end_in_fixed.y,
                         beam_end_pixel);

        if (!beam_end_visible && sensor_visible && no_return) {
          cv::Point clipped_pixel = sensor_pixel;
          double low = 0.0;
          double high = 1.0;
          for (int i = 0; i < 16; ++i) {
            const double t = 0.5 * (low + high);
            const double x =
                sensor_pose->x + t * (beam_end_in_fixed.x - sensor_pose->x);
            const double y =
                sensor_pose->y + t * (beam_end_in_fixed.y - sensor_pose->y);

            cv::Point candidate;
            if (worldToPixel(x, y, candidate)) {
              low = t;
              clipped_pixel = candidate;
              beam_end_visible = true;
            } else {
              high = t;
            }
          }
          beam_end_pixel = clipped_pixel;
        }

        if (beam_end_visible) {
          if (sensor_visible) {
            const cv::Scalar ray_color =
                has_hit ? cv::Scalar(160, 210, 160)
                        : cv::Scalar(205, 230, 205);
            cv::line(canvas, sensor_pixel, beam_end_pixel, ray_color, 1,
                     cv::LINE_AA);
          }
          if (has_hit) {
            cv::circle(canvas, beam_end_pixel, 1, cv::Scalar(0, 160, 0), -1);
          }
        }
      }
      angle += static_cast<double>(scan->angle_increment);
    }
  }
}

void SimpleRvizNode::drawInteraction(cv::Mat& canvas) const {
  if (!dragging_ || mode_ == InteractionMode::kIdle) return;
  cv::arrowedLine(canvas, drag_start_pixel_, drag_current_pixel_,
                  cv::Scalar(0, 210, 230), 2, cv::LINE_AA);
}

void SimpleRvizNode::drawMode(cv::Mat& canvas) const {
  std::string text = "mode: view";
  if (mode_ == InteractionMode::kSetInitialPose) {
    text = "mode: initial pose";
  } else if (mode_ == InteractionMode::kSetGoal) {
    text = "mode: goal";
  }

  const std::string controls = "wasd: move   i: initial pose   g: goal";
  constexpr int kFont = cv::FONT_HERSHEY_SIMPLEX;
  constexpr double kModeScale = 0.62;
  constexpr double kControlsScale = 0.52;
  constexpr int kThickness = 1;

  int mode_baseline = 0;
  int controls_baseline = 0;
  const cv::Size mode_size =
      cv::getTextSize(text, kFont, kModeScale, kThickness, &mode_baseline);
  const cv::Size controls_size = cv::getTextSize(
      controls, kFont, kControlsScale, kThickness, &controls_baseline);

  const int panel_x = 6;
  const int panel_y = 6;
  const int padding = 8;
  const int line_gap = 8;
  const int panel_width =
      std::max(mode_size.width, controls_size.width) + 2 * padding;
  const int panel_height = padding + mode_size.height + line_gap +
                           controls_size.height + padding;

  cv::rectangle(canvas, cv::Rect(panel_x, panel_y, panel_width, panel_height),
                cv::Scalar(245, 245, 245), -1);
  cv::rectangle(canvas, cv::Rect(panel_x, panel_y, panel_width, panel_height),
                cv::Scalar(40, 40, 40), 1);

  const cv::Scalar text_color(0, 0, 0);
  const cv::Point mode_pos(panel_x + padding, panel_y + padding + mode_size.height);
  const cv::Point controls_pos(panel_x + padding,
                               mode_pos.y + line_gap + controls_size.height);
  cv::putText(canvas, text, mode_pos, kFont, kModeScale, text_color,
              kThickness, cv::LINE_AA);
  cv::putText(canvas, controls, controls_pos, kFont, kControlsScale,
              text_color, kThickness, cv::LINE_AA);
}

}  // namespace rp_simple_rviz
