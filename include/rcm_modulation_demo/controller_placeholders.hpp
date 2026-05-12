#pragma once

#include <Eigen/Geometry>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <optional>

namespace rcm_modulation_demo
{

struct CameraIntrinsics
{
  double cx_px{424.0};
  double cy_px{240.0};
  double fx_px{427.4};
  double fy_px{427.4};
};

struct NominalControllerParams
{
  double deadband_px{15.0};
  double depth_tracking_distance_m{0.20};
  bool depth_tracking_enabled{false};
};

/**
 * Placeholder for the non-public nominal visual-servoing controller.
 *
 * This function intentionally keeps only a simple pinhole/deadband sketch.
 * Replace with your own public controller if you want a runnable system.
 */
Eigen::Vector3d computeNominalCameraVelocityPseudocode(
    const std::optional<geometry_msgs::msg::PointStamped>& object_center,
    const CameraIntrinsics& intrinsics,
    const NominalControllerParams& params);

/**
 * Placeholder for the non-public RCM maintenance correction.
 *
 * This is deliberately minimal. The real controller may include PI terms,
 * depth-dependent behavior, robot-specific frame conventions, and additional
 * safety logic.
 */
Eigen::Vector3d computeRcmCorrectionPseudocode(
    const Eigen::Isometry3d& T_world_ee,
    const Eigen::Vector3d& rcm_point_world,
    double proportional_gain);

/**
 * Compute a simple angular velocity that approximately pivots the shaft around
 * the RCM point. This is retained as a geometric sketch, not a full controller.
 */
Eigen::Vector3d computeApproximateRcmAngularVelocity(
    const Eigen::Isometry3d& T_world_ee,
    const Eigen::Vector3d& rcm_point_world,
    const Eigen::Vector3d& ee_linear_velocity_world);

}  // namespace rcm_modulation_demo
