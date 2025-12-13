#ifndef IO__ROS2_HPP
#define IO__ROS2_HPP

#include "publish2nav.hpp"
#include "subscribe2nav.hpp"
#include "gimbal_joint_state_pub.hpp"
#include "../gimbal/gimbal.hpp"

namespace io
{
class ROS2
{
public:
  ROS2();

  ~ROS2();

  void publish(const Eigen::Vector4d & target_pos);

  std::vector<int8_t> subscribe_enemy_status();

  std::vector<int8_t> subscribe_autoaim_target();

  void publish_gimbal_joint_state(
    const Gimbal & gimbal,
    const std::chrono::steady_clock::time_point & timestamp);

  template <typename T>
  std::shared_ptr<rclcpp::Publisher<T>> create_publisher(
    const std::string & node_name, const std::string & topic_name, size_t queue_size)
  {
    auto node = std::make_shared<rclcpp::Node>(node_name);

    auto publisher = node->create_publisher<T>(topic_name, queue_size);

    // 运行一个单独的线程来 spin 这个节点，确保消息可以被正确发布
    std::thread([node]() { rclcpp::spin(node); }).detach();

    return publisher;
  }

private:
  std::shared_ptr<Publish2Nav> publish2nav_;
  std::shared_ptr<Subscribe2Nav> subscribe2nav_;
  std::shared_ptr<GimbalJointStatePublisher> gimbal_joint_state_pub_;

  std::unique_ptr<std::thread> publish_spin_thread_;
  std::unique_ptr<std::thread> subscribe_spin_thread_;
  std::unique_ptr<std::thread> gimbal_spin_thread_;
};

}  // namespace io
#endif
