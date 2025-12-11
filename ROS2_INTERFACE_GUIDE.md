# ROS2 接口使用指南 - io::ROS2 类

## 概览
`io::ROS2` 类是预留的 ROS2 封装接口，提供了发布和订阅功能。

## 可用接口

### 1. 发布目标信息（已实现）
```cpp
void publish(const Eigen::Vector4d & target_pos);
```
发布目标位置信息到导航系统。

### 2. 订阅敌人状态（已实现）
```cpp
std::vector<int8_t> subscribe_enemy_status();
```
订阅敌人状态信息。

### 3. 订阅自动瞄准目标（已实现）
```cpp
std::vector<int8_t> subscribe_autoaim_target();
```
订阅自动瞄准目标信息。

### 4. 通用发布器模板（推荐用于新功能）
```cpp
template <typename T>
std::shared_ptr<rclcpp::Publisher<T>> create_publisher(
  const std::string & node_name, 
  const std::string & topic_name, 
  size_t queue_size)
```

## 在 sentry.cpp 中使用 ROS2 接口发布云台关节状态

### 方案 1：使用 `create_publisher<T>()` 模板（推荐）

在 `sentry.cpp` 中，使用预留的通用发布器模板：

```cpp
#include "sensor_msgs/msg/joint_state.hpp"
#include "rclcpp/rclcpp.hpp"

// 在 main() 函数中初始化
io::ROS2 ros2;

// 创建云台关节状态发布器（建议在初始化时执行一次）
auto gimbal_joint_publisher = ros2.create_publisher<sensor_msgs::msg::JointState>(
    "gimbal_node",           // 节点名称
    "/serial/gimbal_joint_state",  // 话题名称
    rclcpp::QoS(10)          // 队列大小
);

// 在主循环中发布消息
sensor_msgs::msg::JointState gimbal_state;
gimbal_state.header.stamp = rclcpp::Clock().now();
gimbal_state.name = {"gimbal_yaw", "gimbal_pitch"};
gimbal_state.position = {yaw_angle, pitch_angle};
gimbal_state.velocity = {yaw_velocity, pitch_velocity};

gimbal_joint_publisher->publish(gimbal_state);
```

### 方案 2：扩展 io::ROS2 类（更好的工程实践）

如果需要长期支持云台功能，建议在 `io/ros2/ros2.hpp` 中添加专门的方法：

```cpp
class ROS2
{
public:
  // ... 现有方法 ...
  
  // 发布云台关节状态（新增方法）
  void publish_gimbal_joint_state(
    const std::string & topic_name,
    const std::vector<std::string> & joint_names,
    const std::vector<double> & positions,
    const std::vector<double> & velocities);
    
private:
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::JointState>> gimbal_publisher_;
};
```

对应的实现（`io/ros2/ros2.cpp`）：

```cpp
void ROS2::publish_gimbal_joint_state(
    const std::string & topic_name,
    const std::vector<std::string> & joint_names,
    const std::vector<double> & positions,
    const std::vector<double> & velocities)
{
  if (!gimbal_publisher_) {
    // 首次调用时创建发布器
    gimbal_publisher_ = create_publisher<sensor_msgs::msg::JointState>(
        "gimbal_node", topic_name, rclcpp::QoS(10));
  }
  
  auto msg = std::make_unique<sensor_msgs::msg::JointState>();
  msg->header.stamp = rclcpp::Clock().now();
  msg->name = joint_names;
  msg->position = positions;
  msg->velocity = velocities;
  
  gimbal_publisher_->publish(std::move(msg));
}
```

## sentry.cpp 中的集成示例

### 完整的集成代码片段

```cpp
#include "sensor_msgs/msg/joint_state.hpp"
#include "rclcpp/rclcpp.hpp"

// 在 main() 中
int main(int argc, char** argv) {
  // ... 现有代码 ...
  
  io::ROS2 ros2;
  
  // 创建云台关节状态发布器
  auto gimbal_joint_pub = ros2.create_publisher<sensor_msgs::msg::JointState>(
      "gimbal_joint_publisher",
      "/serial/gimbal_joint_state",
      rclcpp::QoS(10));
  
  // ... 串口读取线程启动 ...
  
  while (/* main loop condition */) {
    // ... 现有处理逻辑 ...
    
    // 发布云台关节状态
    {
      std::lock_guard<std::mutex> lock(gimbal_data_mutex);
      
      sensor_msgs::msg::JointState gimbal_msg;
      gimbal_msg.header.frame_id = "gimbal_base";
      gimbal_msg.header.stamp = rclcpp::Clock().now();
      gimbal_msg.name = {"gimbal_yaw", "gimbal_pitch"};
      gimbal_msg.position = {gimbal_data.yaw, gimbal_data.pitch};
      gimbal_msg.velocity = {gimbal_data.yaw_velocity, gimbal_data.pitch_velocity};
      
      gimbal_joint_pub->publish(gimbal_msg);
    }
  }
  
  return 0;
}
```

## 关键注意事项

### 线程安全
- `io::ROS2::create_publisher()` 在后台启动线程 spin 节点，确保消息发送
- 如果从多个线程访问发布器，使用互斥锁保护共享数据（如 `gimbal_data_mutex`）

### QoS 策略
```cpp
rclcpp::QoS(10)  // 队列大小为 10
```
- 云台数据一般是实时性强的传感器数据，QoS 10 适合
- 可根据需要调整：更重要的数据用更大的队列

### 话题命名规范
- 始终使用完整的话题路径：`/serial/gimbal_joint_state`（而非相对路径）
- 便于后续与其他 ROS2 节点集成

### 消息字段映射
```
sensor_msgs/msg/JointState 字段说明：
├── header.stamp: 消息时间戳（使用 rclcpp::Clock().now()）
├── header.frame_id: 参考坐标系（推荐 "gimbal_base"）
├── name: 关节名称数组 ["gimbal_yaw", "gimbal_pitch"]
├── position: 关节位置数组（弧度制）
├── velocity: 关节速度数组（弧度/秒）
└── effort: 关节力矩数组（可选，云台一般不填）
```

## 测试发布结果

在 ROS2 环境中：

```bash
# 列出所有话题
ros2 topic list | grep gimbal

# 实时查看云台关节状态消息
ros2 topic echo /serial/gimbal_joint_state

# 检查消息频率和统计信息
ros2 topic info /serial/gimbal_joint_state -v
```

## 现有代码查看位置

| 文件位置 | 功能 |
|---------|------|
| `io/ros2/ros2.hpp` | ROS2 类接口定义 |
| `io/ros2/publish2nav.hpp` | 发布到导航系统的实现 |
| `io/ros2/subscribe2nav.hpp` | 订阅导航系统的实现 |
| `src/sentry.cpp` | 主程序中的使用示例 |

## 故障排除

### 编译错误：找不到 rclcpp/sensor_msgs

```bash
# 确保 ROS2 环境已配置
source /opt/ros/humble/setup.bash

# 重新编译
colcon build --packages-select sp_vision
```

### 运行时：话题未发布

- 检查 ROS2 master 是否运行
- 确认发布器初始化时 ROS2 环境可用
- 查看 sentry 日志确认无异常

### 消息顺序混乱

- 增加 QoS 队列大小
- 确保时间戳正确赋值
