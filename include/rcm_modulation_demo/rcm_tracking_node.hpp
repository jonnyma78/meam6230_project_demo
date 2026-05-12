#pragma once

#include "rcm_modulation_demo/controller_placeholders.hpp"
#include "rcm_modulation_demo/modulation.hpp"

#include <Eigen/Geometry>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <optional>

namespace rcm_modulation_demo
{

class RcmTrackingNode : public rclcpp::Node
{
public:
  RcmTrackingNode();

private:
  void controlStep();
  void publishObjectCenterCamera();
  void publishRcmDeviation(const Eigen::Isometry3d& T_world_ee);
  void publishTwistMetrics(const geometry_msgs::msg::TwistStamped& twist_msg);
  void startServoIfAvailable();
  void loadParameters();

  Eigen::Vector3d convertCameraVelocityToEeVelocityPseudocode(
      const Eigen::Vector3d& velocity_camera) const;

  // Parameters
  CameraIntrinsics intrinsics_;
  NominalControllerParams nominal_params_;
  CylinderObstacle obstacle_;

  Eigen::Vector3d rcm_point_world_{0.0, 0.0, 0.0};
  Eigen::Isometry3d T_ee_camera_{Eigen::Isometry3d::Identity()};
  double speed_scaling_factor_{1.0};
  double rcm_kp_{0.0};
  bool enable_modulation_{true};

  // State
  std::optional<geometry_msgs::msg::PointStamped> object_center_;

  // ROS interfaces
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr object_cam_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr rcm_dev_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr lin_x_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr lin_y_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr lin_z_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr ang_x_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr ang_y_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr ang_z_pub_;

  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr object_center_sub_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr start_servo_client_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace rcm_modulation_demo
