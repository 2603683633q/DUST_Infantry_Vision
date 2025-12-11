#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <chrono>
#include <memory>
#include <thread>
#include <atomic>
#include <cstring>
#include <vector>

#include "serial/serial.h"
#include "tools/logger.hpp"
#include "tools/crc.hpp"

// 与下位机 GimbalToVision 结构完全一致
struct __attribute__((packed)) GimbalToVision
{
    uint8_t head[2];        // 'S', 'P'
    uint8_t mode;           // 0: 空闲, 1: 自瞄, 2: 小符, 3: 大符
    float q[4];             // wxyz顺序
    float yaw;              // yaw角度（弧度）
    float yaw_vel;          // yaw角速度（弧度/秒）
    float pitch;            // pitch角度（弧度）
    float pitch_vel;        // pitch角速度（弧度/秒）
    float bullet_speed;
    uint16_t bullet_count;
    uint16_t crc16;
};

static_assert(sizeof(GimbalToVision) == 48, "GimbalToVision size must be 48 bytes");

const uint8_t FRAME_HEADER_0 = 'S';
const uint8_t FRAME_HEADER_1 = 'P';
const size_t FRAME_SIZE = sizeof(GimbalToVision);

class GimbalJointPublisher : public rclcpp::Node
{
public:
    GimbalJointPublisher()
        : Node("gimbal_joint_publisher"), 
          stop_thread_(false)
    {
        // 声明参数
        this->declare_parameter<std::string>("serial_port", "/dev/gimbal");
        this->declare_parameter<int>("baudrate", 115200);
        
        // 获取参数
        serial_port_ = this->get_parameter("serial_port").as_string();
        baudrate_ = this->get_parameter("baudrate").as_int();
        
        // 创建JointState发布器
        joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>(
            "serial/gimbal_joint_state", 10);
        
        // 初始化串口
        init_serial();
        
        // 启动串口读取线程
        read_thread_ = std::thread(&GimbalJointPublisher::read_thread, this);
        
        RCLCPP_INFO(this->get_logger(), "Gimbal Joint State Publisher initialized");
    }
    
    ~GimbalJointPublisher()
    {
        stop_thread_ = true;
        if (read_thread_.joinable()) {
            read_thread_.join();
        }
        if (serial_.isOpen()) {
            serial_.close();
        }
        RCLCPP_INFO(this->get_logger(), "Gimbal Joint State Publisher shutting down");
    }

private:
    void init_serial()
    {
        try {
            serial_.setPort(serial_port_);
            serial_.setBaudrate(baudrate_);
            serial_.setFlowcontrol(serial::flowcontrol_none);
            serial_.setParity(serial::parity_none);
            serial_.setStopbits(serial::stopbits_one);
            serial_.setBytesize(serial::eightbits);
            
            // 设置超时
            serial::Timeout timeout = serial::Timeout::simpleTimeout(100);
            serial_.setTimeout(timeout);
            
            serial_.open();
            RCLCPP_INFO(this->get_logger(), "Serial port %s opened at %d baud", 
                       serial_port_.c_str(), baudrate_);
            
        } catch (const std::exception & e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial port: %s", e.what());
            rclcpp::shutdown();
        }
    }
    
    void read_thread()
    {
        std::vector<uint8_t> buffer(512);
        size_t buffer_pos = 0;
        size_t frame_count = 0;
        size_t error_count = 0;
        
        while (!stop_thread_ && rclcpp::ok()) {
            try {
                if (!serial_.isOpen()) {
                    RCLCPP_WARN(this->get_logger(), "Serial port is not open, attempting to reconnect...");
                    std::this_thread::sleep_for(std::chrono::seconds(1));
                    init_serial();
                    continue;
                }
                
                // 读取可用数据
                size_t bytes_available = serial_.available();
                if (bytes_available == 0) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(5));
                    continue;
                }
                
                // 限制单次读取大小，防止缓冲区溢出
                size_t bytes_to_read = std::min(bytes_available, size_t(256) - buffer_pos);
                if (bytes_to_read == 0) {
                    // 缓冲区已满，重置
                    RCLCPP_WARN(this->get_logger(), "Buffer overflow, resetting");
                    buffer_pos = 0;
                    continue;
                }
                
                size_t bytes_read = serial_.read(buffer.data() + buffer_pos, bytes_to_read);
                buffer_pos += bytes_read;
                
                // 在缓冲区中查找并解析帧
                while (buffer_pos >= FRAME_SIZE) {
                    // 查找帧头
                    int header_pos = -1;
                    for (size_t i = 0; i <= buffer_pos - FRAME_SIZE; ++i) {
                        if (buffer[i] == FRAME_HEADER_0 && buffer[i + 1] == FRAME_HEADER_1) {
                            header_pos = i;
                            break;
                        }
                    }
                    
                    if (header_pos < 0) {
                        // 没有找到帧头，丢弃缓冲区中的数据
                        buffer_pos = 0;
                        break;
                    }
                    
                    if (header_pos > 0) {
                        // 移除帧头前的数据
                        std::memmove(buffer.data(), buffer.data() + header_pos, buffer_pos - header_pos);
                        buffer_pos -= header_pos;
                    }
                    
                    // 再次检查是否有完整的帧
                    if (buffer_pos < FRAME_SIZE) {
                        break;
                    }
                    
                    // 复制帧数据
                    GimbalToVision frame_data;
                    std::memcpy(&frame_data, buffer.data(), FRAME_SIZE);
                    
                    // CRC校验 - 校验的是除了CRC字段外的所有数据
                    uint16_t calculated_crc = tools::get_crc16(
                        buffer.data(),
                        FRAME_SIZE - sizeof(frame_data.crc16));
                    
                    if (calculated_crc == frame_data.crc16) {
                        // CRC校验通过，发布数据
                        publish_joint_state(frame_data);
                        frame_count++;
                        
                        RCLCPP_DEBUG(this->get_logger(),
                            "Frame %lu - Mode: %d, Yaw: %.4f rad, Pitch: %.4f rad",
                            frame_count, frame_data.mode, frame_data.yaw, frame_data.pitch);
                    } else {
                        error_count++;
                        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                            "CRC mismatch: expected 0x%04x, got 0x%04x (Error count: %lu)",
                            calculated_crc, frame_data.crc16, error_count);
                    }
                    
                    // 移除已处理的帧
                    std::memmove(buffer.data(), buffer.data() + FRAME_SIZE, buffer_pos - FRAME_SIZE);
                    buffer_pos -= FRAME_SIZE;
                }
                
            } catch (const std::exception & e) {
                RCLCPP_ERROR(this->get_logger(), "Serial read error: %s", e.what());
                buffer_pos = 0;
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        }
        
        RCLCPP_INFO(this->get_logger(), 
            "Serial read thread stopped. Total frames: %lu, Errors: %lu",
            frame_count, error_count);
    }
    
    void publish_joint_state(const GimbalToVision& data)
    {
        auto joint_state_msg = sensor_msgs::msg::JointState();
        
        // 设置时间戳和frame_id
        joint_state_msg.header.stamp = this->now();
        joint_state_msg.header.frame_id = "gimbal_base";
        
        // 设置关节名称
        joint_state_msg.name = {"gimbal_yaw", "gimbal_pitch"};
        
        // 设置关节位置（弧度）
        joint_state_msg.position = {data.yaw, data.pitch};
        
        // 设置关节速度（弧度/秒）
        joint_state_msg.velocity = {data.yaw_vel, data.pitch_vel};
        
        // 发布消息
        joint_state_publisher_->publish(joint_state_msg);
    }
    
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
    serial::Serial serial_;
    std::thread read_thread_;
    std::atomic<bool> stop_thread_;
    
    std::string serial_port_;
    int baudrate_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    
    try {
        auto node = std::make_shared<GimbalJointPublisher>();
        RCLCPP_INFO(rclcpp::get_logger("main"), "Gimbal Joint State Publisher is running...");
        rclcpp::spin(node);
    } catch (const std::exception & e) {
        RCLCPP_ERROR(rclcpp::get_logger("main"), "Exception: %s", e.what());
    }
    
    rclcpp::shutdown();
    return 0;
}
