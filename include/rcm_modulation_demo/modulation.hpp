#pragma once

#include <Eigen/Geometry>

namespace rcm_modulation_demo
{

struct CylinderObstacle
{
  Eigen::Vector3d center_world{0.33, 0.15, -0.01};
  double radius_m{0.03};
  double height_m{0.10};
  double radial_safety_margin_m{0.10};
  double axial_safety_margin_m{0.10};
};

/**
 * Modulate a nominal camera-frame velocity around a finite vertical cylinder.
 *
 * The implementation converts the velocity into world coordinates, builds a
 * cylinder-aligned basis using the radial normal, tangential direction, and
 * cylinder axis, applies M = E D E^T, then converts the result back into the
 * camera frame.
 */
Eigen::Vector3d modulateCameraVelocityAroundCylinder(
    const Eigen::Vector3d& velocity_camera,
    const Eigen::Isometry3d& T_world_camera,
    const CylinderObstacle& obstacle);

}  // namespace rcm_modulation_demo
