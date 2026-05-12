#include "rcm_modulation_demo/rcm_tracking_node.hpp"

#include <tf2_eigen/tf2_eigen.hpp>

#include <chrono>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace rcm_modulation_demo
{
namespace
{
Eigen::Vector3d declareVector3(
    rclcpp::Node& node,
    const std::string& name,
    const std::vector<double>& default_value)
{
  const auto values = node.declare_parameter<std::vector<double>>(name, default_value);
  if (values.size() != 3) {
    throw std::runtime_error("Parameter '" + name + "' must contain exactly 3 values.");
  }
  return Eigen::Vector3d(values[0], values[1], values[2]);
}

Eigen::Matrix3d declareMatrix3(
    rclcpp::Node& node,
    const std::string& name,
    const std::vector<double>& default_value)
{
  const auto values = node.declare_parameter<std::vector<double>>(name, default_value);
  if (values.size() != 9) {
    throw std::runtime_error("Parameter '" + name + "' must contain exactly 9 values.");
  }

  Eigen::Matrix3d matrix;
  matrix << values[0], values[1], values[2],
            values[3], values[4], values[5],
            values[6], values[7], values[8];
  return matrix;
}
}  // namespace

RcmTrackingNode::RcmTrackingNode()
: rclcpp::Node("rcm_tracking_node")
{
  loadParameters();

  twist_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(
      "/servo_node/delta_twist_cmds", rclcpp::QoS(1));
  object_cam_pub_ = create_publisher<geometry_msgs::msg::PointStamped>(
      "/object_center_cam", rclcpp::QoS(1));
  rcm_dev_pub_ = create_publisher<std_msgs::msg::Float64>(
      "/rcm_deviation", rclcpp::QoS(10));

  lin_x_pub_ = create_publisher<std_msgs::msg::Float64>("/ee/linear/x", 10);
  lin_y_pub_ = create_publisher<std_msgs::msg::Float64>("/ee/linear/y", 10);
  lin_z_pub_ = create_publisher<std_msgs::msg::Float64>("/ee/linear/z", 10);
  ang_x_pub_ = create_publisher<std_msgs::msg::Float64>("/ee/angular/x", 10);
  ang_y_pub_ = create_publisher<std_msgs::msg::Float64>("/ee/angular/y", 10);
  ang_z_pub_ = create_publisher<std_msgs::msg::Float64>("/ee/angular/z", 10);

  rclcpp::QoS detection_qos(1);
  detection_qos.best_effort();
  object_center_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
      "/object_center",
      detection_qos,
      [this](const geometry_msgs::msg::PointStamped::SharedPtr msg) {
        object_center_ = *msg;
      });

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  start_servo_client_ = create_client<std_srvs::srv::Trigger>("/servo_node/start_servo");
  startServoIfAvailable();

  using namespace std::chrono_literals;
  timer_ = create_wall_timer(5ms, std::bind(&RcmTrackingNode::controlStep, this));
}

void RcmTrackingNode::loadParameters()
{
  rcm_point_world_ = declareVector3(*this, "rcm_xyz", {0.0, 0.0, 0.0});

  intrinsics_.cx_px = declare_parameter<double>("center_x", 424.0);
  intrinsics_.cy_px = declare_parameter<double>("center_y", 240.0);
  intrinsics_.fx_px = declare_parameter<double>("fx", 427.4);
  intrinsics_.fy_px = declare_parameter<double>("fy", 427.4);

  nominal_params_.deadband_px = declare_parameter<double>("deadband_range", 15.0);
  nominal_params_.depth_tracking_distance_m = declare_parameter<double>("depth_tracking_distance", 0.20);
  nominal_params_.depth_tracking_enabled = declare_parameter<bool>("depth_tracking", false);

  speed_scaling_factor_ = declare_parameter<double>("speed_scaling_factor", 1.0);
  rcm_kp_ = declare_parameter<double>("rcm_kp", 0.0);
  enable_modulation_ = declare_parameter<bool>("enable_modulation", true);

  obstacle_.center_world = declareVector3(*this, "obstacle_center_xyz", {0.33, 0.15, -0.01});
  obstacle_.radius_m = declare_parameter<double>("obstacle_radius", 0.03);
  obstacle_.height_m = declare_parameter<double>("obstacle_height", 0.10);
  obstacle_.radial_safety_margin_m = declare_parameter<double>("obstacle_radial_margin", 0.10);
  obstacle_.axial_safety_margin_m = declare_parameter<double>("obstacle_axial_margin", 0.10);

  const Eigen::Matrix3d R_ee_camera = declareMatrix3(
      *this,
      "tec_rot_matrix",
      {1.0, 0.0, 0.0,
       0.0, 1.0, 0.0,
       0.0, 0.0, 1.0});
  const Eigen::Vector3d t_ee_camera = declareVector3(*this, "tec_translation", {0.0, 0.0, 0.0});

  T_ee_camera_.setIdentity();
  T_ee_camera_.linear() = R_ee_camera;
  T_ee_camera_.translation() = t_ee_camera;
}

void RcmTrackingNode::startServoIfAvailable()
{
  if (!start_servo_client_->wait_for_service(std::chrono::seconds(2))) {
    RCLCPP_WARN(get_logger(), "MoveIt Servo start service is not available; continuing anyway.");
    return;
  }

  const auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  (void)start_servo_client_->async_send_request(request);
}

void RcmTrackingNode::controlStep()
{
  geometry_msgs::msg::TransformStamped tf_msg;
  try {
    tf_msg = tf_buffer_->lookupTransform("base_link", "tool0", tf2::TimePointZero);
  } catch (const tf2::TransformException& ex) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "TF lookup failed: %s", ex.what());
    return;
  }

  const Eigen::Isometry3d T_world_ee = tf2::transformToEigen(tf_msg);
  const Eigen::Isometry3d T_world_camera = T_world_ee * T_ee_camera_;

  publishObjectCenterCamera();

  // Non-public/Optosurgical-specific logic is intentionally reduced to a sketch here.
  Eigen::Vector3d velocity_camera = computeNominalCameraVelocityPseudocode(
      object_center_, intrinsics_, nominal_params_);

  if (enable_modulation_) {
    velocity_camera = modulateCameraVelocityAroundCylinder(
        velocity_camera, T_world_camera, obstacle_);
  }

  Eigen::Vector3d velocity_ee = convertCameraVelocityToEeVelocityPseudocode(velocity_camera);
  Eigen::Vector3d velocity_world = T_world_ee.rotation() * velocity_ee;

  velocity_world += computeRcmCorrectionPseudocode(T_world_ee, rcm_point_world_, rcm_kp_);

  const Eigen::Vector3d omega_world = computeApproximateRcmAngularVelocity(
      T_world_ee, rcm_point_world_, velocity_world);

  geometry_msgs::msg::TwistStamped twist_msg;
  twist_msg.header.stamp = now();
  twist_msg.header.frame_id = "base_link";
  twist_msg.twist.linear.x = velocity_world.x();
  twist_msg.twist.linear.y = velocity_world.y();
  twist_msg.twist.linear.z = velocity_world.z();
  twist_msg.twist.angular.x = omega_world.x();
  twist_msg.twist.angular.y = omega_world.y();
  twist_msg.twist.angular.z = omega_world.z();

  twist_pub_->publish(twist_msg);
  publishRcmDeviation(T_world_ee);
  publishTwistMetrics(twist_msg);
}

Eigen::Vector3d RcmTrackingNode::convertCameraVelocityToEeVelocityPseudocode(
    const Eigen::Vector3d& velocity_camera) const
{
  // Public sketch only. The original system used robot-specific signs, RCM scaling,
  // and calibrated camera-to-EE transforms. Keep this simple for teaching review.
  return speed_scaling_factor_ * T_ee_camera_.rotation() * velocity_camera;
}

void RcmTrackingNode::publishObjectCenterCamera()
{
  if (!object_center_.has_value()) {
    return;
  }

  const double u = object_center_->point.x;
  const double v = object_center_->point.y;
  const double z = object_center_->point.z;

  if (z <= 0.0 || !std::isfinite(z)) {
    return;
  }

  geometry_msgs::msg::PointStamped msg;
  msg.header.stamp = now();
  msg.header.frame_id = "cam_link";
  msg.point.x = (u - intrinsics_.cx_px) / intrinsics_.fx_px * z;
  msg.point.y = (v - intrinsics_.cy_px) / intrinsics_.fy_px * z;
  msg.point.z = z;
  object_cam_pub_->publish(msg);
}

void RcmTrackingNode::publishRcmDeviation(const Eigen::Isometry3d& T_world_ee)
{
  const Eigen::Vector3d shaft_axis = T_world_ee.rotation().col(2).normalized();
  const Eigen::Vector3d ee_to_rcm = rcm_point_world_ - T_world_ee.translation();
  const Eigen::Vector3d perpendicular_error = ee_to_rcm - shaft_axis * shaft_axis.dot(ee_to_rcm);

  std_msgs::msg::Float64 msg;
  msg.data = perpendicular_error.norm();
  rcm_dev_pub_->publish(msg);
}

void RcmTrackingNode::publishTwistMetrics(const geometry_msgs::msg::TwistStamped& twist_msg)
{
  std_msgs::msg::Float64 value;

  value.data = twist_msg.twist.linear.x;
  lin_x_pub_->publish(value);
  value.data = twist_msg.twist.linear.y;
  lin_y_pub_->publish(value);
  value.data = twist_msg.twist.linear.z;
  lin_z_pub_->publish(value);

  value.data = twist_msg.twist.angular.x;
  ang_x_pub_->publish(value);
  value.data = twist_msg.twist.angular.y;
  ang_y_pub_->publish(value);
  value.data = twist_msg.twist.angular.z;
  ang_z_pub_->publish(value);
}

}  // namespace rcm_modulation_demo
