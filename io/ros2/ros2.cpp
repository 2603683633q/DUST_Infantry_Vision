#include "ros2.hpp"
namespace io
{
ROS2::ROS2()
{
  rclcpp::init(0, nullptr);

  publish2nav_ = std::make_shared<Publish2Nav>();

  subscribe2nav_ = std::make_shared<Subscribe2Nav>();

  gimbal_joint_state_pub_ = std::make_shared<GimbalJointStatePublisher>();

  publish_spin_thread_ = std::make_unique<std::thread>([this]() { publish2nav_->start(); });

  subscribe_spin_thread_ = std::make_unique<std::thread>([this]() { subscribe2nav_->start(); });

  gimbal_spin_thread_ = std::make_unique<std::thread>([this]() { gimbal_joint_state_pub_->start(); });
}

ROS2::~ROS2()
{
  rclcpp::shutdown();
  publish_spin_thread_->join();
  subscribe_spin_thread_->join();
  gimbal_spin_thread_->join();
}

void ROS2::publish(const Eigen::Vector4d & target_pos) { publish2nav_->send_data(target_pos); }

std::vector<int8_t> ROS2::subscribe_enemy_status()
{
  return subscribe2nav_->subscribe_enemy_status();
}

std::vector<int8_t> ROS2::subscribe_autoaim_target()
{
  return subscribe2nav_->subscribe_autoaim_target();
}

void ROS2::publish_gimbal_joint_state(
  const Gimbal & gimbal,
  const std::chrono::steady_clock::time_point & timestamp)
{
  GimbalState state = gimbal.state();
  std::vector<double> position = {state.yaw, state.pitch};
  std::vector<double> velocity = {state.yaw_vel, state.pitch_vel};
  std::vector<double> effort = {0.0, 0.0};  // 如果没有力矩数据，使用0
  
  gimbal_joint_state_pub_->publish_joint_state(position, velocity, effort);
}

}  // namespace io
