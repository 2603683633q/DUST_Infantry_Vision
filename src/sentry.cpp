#include <fmt/core.h>

#include <chrono>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <thread>

#include "io/camera.hpp"
#include "io/gimbal/gimbal.hpp"
#include "io/cboard.hpp"  // for ShootMode enum
#include "io/ros2/publish2nav.hpp"
#include "io/ros2/ros2.hpp"
#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/shooter.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tasks/omniperception/decider.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

using namespace std::chrono;

const std::string keys =
  "{help h usage ? |                        | 输出命令行参数说明}"
  "{@config-path   | configs/sentry.yaml | 位置参数，yaml配置文件路径 }";

int main(int argc, char * argv[])
{
  tools::Exiter exiter;

  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }
  auto config_path = cli.get<std::string>(0);

  io::ROS2 ros2;
  io::Gimbal gimbal(config_path);
  io::Camera front_camera(config_path, 1);   // 前置相机
  io::Camera back_camera(config_path, 0);    // 后置相机

  auto_aim::YOLO yolo(config_path, false);       // 前置相机YOLO
  auto_aim::YOLO back_yolo(config_path, false);  // 后置相机YOLO
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path, solver);
  auto_aim::Aimer aimer(config_path);
  auto_aim::Shooter shooter(config_path);

  omniperception::Decider decider(config_path);

  cv::Mat img;
  std::chrono::steady_clock::time_point timestamp;

  cv::Mat front_img, back_img;
  std::chrono::steady_clock::time_point front_ts, back_ts;
  
  // 后置相机参数
  double back_fov_h = 54.2;  // 后置相机水平视场角
  double back_fov_v = 44.5;  // 后置相机垂直视场角
  double back_yaw_offset = 180.0;  // 后置相机相对于前置的偏航角偏移（度）
  
  while (!exiter.exit()) {
    front_camera.read(front_img, front_ts);
    back_camera.read(back_img, back_ts);
    img = front_img;
    timestamp = front_ts;
    Eigen::Quaterniond q = gimbal.q(timestamp - 1ms);

    /// 自瞄核心逻辑
    solver.set_R_gimbal2world(q);

    Eigen::Vector3d gimbal_pos = tools::eulers(solver.R_gimbal2world(), 2, 1, 0);

    auto armors = yolo.detect(img);

    decider.get_invincible_armor(ros2.subscribe_enemy_status());

    decider.armor_filter(armors);

    decider.get_auto_aim_target(armors, ros2.subscribe_autoaim_target());

    decider.set_priority(armors);

    auto targets = tracker.track(armors, timestamp);

    io::Command command{false, false, 0, 0};
    
    /// 后置相机检测逻辑
    auto back_armors = back_yolo.detect(back_img);
    decider.armor_filter(back_armors);
    
    // 计算后置相机检测到的目标转向角度
    io::Command back_command{false, false, 0, 0};
    if (!back_armors.empty()) {
      auto& back_armor = back_armors.front();
      
      double delta_yaw_deg = back_yaw_offset + (back_fov_h / 2) - back_armor.center_norm.x * back_fov_h;
      double delta_pitch_deg = back_armor.center_norm.y * back_fov_v - back_fov_v / 2;
      
      back_command.control = true;
      back_command.yaw = tools::limit_rad(gimbal_pos[0] + delta_yaw_deg / 57.3);
      back_command.pitch = tools::limit_rad(delta_pitch_deg / 57.3);
    }

    /// 前置相机优先的自瞄逻辑
    if (tracker.state() != "lost") {
      // 前置相机有跟踪目标，使用精确瞄准
      command = aimer.aim(targets, timestamp, gimbal.state().bullet_speed, io::ShootMode::both_shoot, solver.R_gimbal2world());
    } else if (!armors.empty()) {
      // 前置相机检测到但未跟踪，使用全向感知
      command = decider.decide(back_yolo, gimbal_pos, front_camera);
    } else if (back_command.control) {
      // 后置相机检测到目标，只提供转向方向（不射击）
      command = back_command;
      command.shoot = false;
    }

    /// 发射逻辑
    command.shoot = shooter.shoot(command, aimer, targets, gimbal_pos);

    gimbal.send(command.control, command.shoot, command.yaw, 0, 0, command.pitch, 0, 0);

    /// ROS2通信
    Eigen::Vector4d target_info = decider.get_target_info(armors, targets);

    ros2.publish(target_info);
  }
  return 0;
}
