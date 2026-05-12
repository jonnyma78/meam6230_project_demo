#include "rcm_modulation_demo/controller_placeholders.hpp"

#include <cmath>

namespace rcm_modulation_demo
{

Eigen::Vector3d computeNominalCameraVelocityPseudocode(
    const std::optional<geometry_msgs::msg::PointStamped>& object_center,
    const CameraIntrinsics& intrinsics,
    const NominalControllerParams& params)
{
  if (!object_center.has_value()) {
    return Eigen::Vector3d::Zero();
  }

  const double u = object_center->point.x;
  const double v = object_center->point.y;
  const double z = object_center->point.z;

  if (z <= 0.0 || !std::isfinite(z)) {
    return Eigen::Vector3d::Zero();
  }

  const double du = u - intrinsics.cx_px;
  const double dv = v - intrinsics.cy_px;
  const double pixel_error = std::hypot(du, dv);

  if (pixel_error <= params.deadband_px) {
    return Eigen::Vector3d::Zero();
  }

  // Pseudocode-level visual servoing sketch:
  //   1. Convert pixel error to an approximate camera-frame metric error.
  //   2. Optionally use depth error for forward/backward motion.
  //   3. Apply controller gains and robot-specific sign conventions elsewhere.
  const double scale = z / intrinsics.fx_px;
  const double dz = params.depth_tracking_enabled
      ? z - params.depth_tracking_distance_m
      : 0.0;

  return Eigen::Vector3d(scale * du, scale * dv, dz);
}

Eigen::Vector3d computeRcmCorrectionPseudocode(
    const Eigen::Isometry3d& T_world_ee,
    const Eigen::Vector3d& rcm_point_world,
    double proportional_gain)
{
  // Pseudocode-level RCM correction sketch:
  //   1. Treat the end-effector z-axis as the laparoscope shaft axis.
  //   2. Project the RCM point onto that axis.
  //   3. Drive the perpendicular shaft error toward zero.
  const Eigen::Vector3d shaft_axis_world = T_world_ee.rotation().col(2).normalized();
  const Eigen::Vector3d ee_to_rcm = rcm_point_world - T_world_ee.translation();
  const Eigen::Vector3d projection = shaft_axis_world * shaft_axis_world.dot(ee_to_rcm);
  const Eigen::Vector3d perpendicular_error = ee_to_rcm - projection;

  return proportional_gain * perpendicular_error;
}

Eigen::Vector3d computeApproximateRcmAngularVelocity(
    const Eigen::Isometry3d& T_world_ee,
    const Eigen::Vector3d& rcm_point_world,
    const Eigen::Vector3d& ee_linear_velocity_world)
{
  const Eigen::Vector3d shaft_axis_world = T_world_ee.rotation().col(2).normalized();
  const Eigen::Vector3d ee_to_rcm = rcm_point_world - T_world_ee.translation();
  const Eigen::Vector3d projected_radius = shaft_axis_world * shaft_axis_world.dot(ee_to_rcm);

  const double denom = projected_radius.squaredNorm();
  if (denom < 1e-9) {
    return Eigen::Vector3d::Zero();
  }

  // Geometric sketch: choose an angular velocity that compensates linear motion
  // at the RCM point. Sign conventions may need adjustment for a real robot.
  return -projected_radius.cross(ee_linear_velocity_world) / denom;
}

}  // namespace rcm_modulation_demo
