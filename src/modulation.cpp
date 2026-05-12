#include "rcm_modulation_demo/modulation.hpp"

#include <algorithm>
#include <cmath>

namespace rcm_modulation_demo
{

Eigen::Vector3d modulateCameraVelocityAroundCylinder(
    const Eigen::Vector3d& velocity_camera,
    const Eigen::Isometry3d& T_world_camera,
    const CylinderObstacle& obstacle)
{
  const Eigen::Matrix3d R_world_camera = T_world_camera.rotation();
  Eigen::Vector3d velocity_world = R_world_camera * velocity_camera;
  const Eigen::Vector3d camera_position_world = T_world_camera.translation();

  const double r_safe = obstacle.radius_m + obstacle.radial_safety_margin_m;
  const double half_height_safe = 0.5 * obstacle.height_m + obstacle.axial_safety_margin_m;

  const Eigen::Vector3d d = camera_position_world - obstacle.center_world;
  const double z_rel = d.z();

  // Do not change the velocity outside the finite cylinder's vertical influence band.
  if (std::abs(z_rel) > half_height_safe) {
    return velocity_camera;
  }

  const Eigen::Vector2d d_xy(d.x(), d.y());
  const double rho = d_xy.norm();

  // Do not change the velocity outside the radial influence region.
  if (rho >= r_safe) {
    return velocity_camera;
  }

  // Degenerate case: camera is exactly on the cylinder axis.
  if (rho < 1e-8) {
    return Eigen::Vector3d::Zero();
  }

  // Cylinder basis in the world frame:
  // e_radial points outward from the cylinder axis.
  // e_tangent is the direction around the cylinder.
  // e_axis is the cylinder axis. This version assumes a vertical world-z cylinder.
  const Eigen::Vector3d e_radial(d.x() / rho, d.y() / rho, 0.0);
  const Eigen::Vector3d e_axis(0.0, 0.0, 1.0);
  const Eigen::Vector3d e_tangent = e_axis.cross(e_radial).normalized();

  Eigen::Matrix3d E;
  E.col(0) = e_radial;
  E.col(1) = e_tangent;
  E.col(2) = e_axis;

  // Gamma = 1 on the obstacle boundary and increases with radial clearance.
  const double gamma = rho * rho - obstacle.radius_m * obstacle.radius_m + 1.0;
  if (gamma <= 1e-6) {
    return Eigen::Vector3d::Zero();
  }

  double lambda_radial = 1.0 - 1.0 / gamma;
  const double lambda_tangent = 1.0 + 1.0 / gamma;
  const double lambda_axis = 1.0 + 1.0 / gamma;

  // Small positive floor prevents complete sticking near the boundary.
  lambda_radial = std::max(0.05, lambda_radial);

  Eigen::Matrix3d D = Eigen::Matrix3d::Zero();
  D(0, 0) = lambda_radial;
  D(1, 1) = lambda_tangent;
  D(2, 2) = lambda_axis;

  const Eigen::Matrix3d M = E * D * E.transpose();
  velocity_world = M * velocity_world;

  return R_world_camera.transpose() * velocity_world;
}

}  // namespace rcm_modulation_demo
