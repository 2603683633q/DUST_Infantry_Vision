#ifndef IO__ROS2__GIMBAL_JOINT_STATE_PUB_HPP
#define IO__ROS2__GIMBAL_JOINT_STATE_PUB_HPP

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <memory>
#include <vector>

namespace io
{
class GimbalJointStatePublisher : public rclcpp::Node
{
public:
  GimbalJointStatePublisher();

  ~GimbalJointStatePublisher();

  void start();

  void publish_joint_state(
    const std::vector<double> & position,
    const std::vector<double> & velocity,
    const std::vector<double> & effort);

private:
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
};

}  // namespace io

#endif
