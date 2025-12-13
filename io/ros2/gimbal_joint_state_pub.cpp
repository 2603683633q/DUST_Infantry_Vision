#include "gimbal_joint_state_pub.hpp"

namespace io
{
GimbalJointStatePublisher::GimbalJointStatePublisher()
  : Node("gimbal_joint_state_publisher")
{
  publisher_ = this->create_publisher<sensor_msgs::msg::JointState>(
    "serial/gimbal_joint_state", 10);

  RCLCPP_INFO(this->get_logger(), "Gimbal JointState Publisher initialized");
}

GimbalJointStatePublisher::~GimbalJointStatePublisher() {}

void GimbalJointStatePublisher::start()
{
  rclcpp::spin(std::make_shared<GimbalJointStatePublisher>());
}

void GimbalJointStatePublisher::publish_joint_state(
  const std::vector<double> & position,
  const std::vector<double> & velocity,
  const std::vector<double> & effort)
{
  auto msg = std::make_unique<sensor_msgs::msg::JointState>();

  // 设置时间戳
  msg->header.stamp = this->now();
  msg->header.frame_id = "gimbal";

  // 设置关节名称 (pitch 和 yaw)
  msg->name = {"gimbal_pitch", "gimbal_yaw"};

  // 设置位置、速度和力矩
  msg->position = position;
  msg->velocity = velocity;
  msg->effort = effort;

  publisher_->publish(*msg);
}

}  // namespace io
