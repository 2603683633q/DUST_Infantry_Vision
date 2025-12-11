#include <fmt/core.h>

#include <atomic>
#include <chrono>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <thread>

#include "io/camera.hpp"
#include "io/gimbal/gimbal.hpp"
#include "io/cboard.hpp"  // for ShootMode enum
#include "io/ros2/publish2nav.hpp"
#include "io/ros2/ros2.hpp"
#include "tasks/auto_aim/planner/planner.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tasks/omniperception/decider.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
#include "tools/recorder.hpp"
#include "tools/thread_safe_queue.hpp"

using namespace std::chrono;
using namespace std::chrono_literals;

const std::string keys =
  "{help h usage ? |                        | 输出命令行参数说明}"
  "{@config-path   | configs/sentry.yaml | 位置参数，yaml配置文件路径 }";

// 用于在主线程和规划线程之间传递数据的结构体
struct PlanInput {
  std::optional<auto_aim::Target> target;
  io::Command back_command;  // 后置相机命令
  bool has_front_armors;     // 前置相机是否检测到装甲板
};

int main(int argc, char * argv[])
{
  tools::Exiter exiter;
  tools::Plotter plotter;
  tools::Recorder recorder;

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

  auto_aim::YOLO yolo(config_path, true);       // 前置相机YOLO
  auto_aim::YOLO back_yolo(config_path, true);  // 后置相机YOLO
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path, solver);
  auto_aim::Planner planner(config_path);

  omniperception::Decider decider(config_path);

  // 线程安全队列用于传递数据给规划线程
  tools::ThreadSafeQueue<PlanInput, true> plan_queue(1);
  plan_queue.push({std::nullopt, {false, false, 0, 0}, false});

  std::atomic<bool> quit = false;
  auto t0 = std::chrono::steady_clock::now();

  // 规划线程
  auto plan_thread = std::thread([&]() {
    uint16_t last_bullet_count = 0;

    while (!quit) {
      auto input = plan_queue.front();
      auto gs = gimbal.state();

      auto_aim::Plan plan;
      
      // 优先级：前置跟踪 > 后置引导
      if (input.target.has_value()) {
        // 前置相机有跟踪目标，使用 planner 精确瞄准
        plan = planner.plan(input.target, gs.bullet_speed, solver.R_gimbal2world());
      } else if (input.back_command.control) {
        // 后置相机检测到目标，只提供转向方向（不射击）
        plan.control = true;
        plan.fire = false;
        plan.yaw = input.back_command.yaw;
        plan.pitch = input.back_command.pitch;
        plan.yaw_vel = 0;
        plan.yaw_acc = 0;
        plan.pitch_vel = 0;
        plan.pitch_acc = 0;
        plan.target_yaw = input.back_command.yaw;
        plan.target_pitch = input.back_command.pitch;
      } else {
        // 没有目标
        plan = planner.plan(std::nullopt, gs.bullet_speed, solver.R_gimbal2world());
      }

      gimbal.send(
        plan.control, plan.fire, plan.yaw, plan.yaw_vel, plan.yaw_acc, 
        plan.pitch, plan.pitch_vel, plan.pitch_acc);

      auto fired = gs.bullet_count > last_bullet_count;
      last_bullet_count = gs.bullet_count;

      // plotter 数据
      nlohmann::json data;
      data["t"] = tools::delta_time(std::chrono::steady_clock::now(), t0);

      data["gimbal_yaw"] = gs.yaw;
      data["gimbal_yaw_vel"] = gs.yaw_vel;
      data["gimbal_pitch"] = gs.pitch;
      data["gimbal_pitch_vel"] = gs.pitch_vel;

      data["target_yaw"] = plan.target_yaw;
      data["target_pitch"] = plan.target_pitch;

      data["plan_yaw"] = plan.yaw;
      data["plan_yaw_vel"] = plan.yaw_vel;
      data["plan_yaw_acc"] = plan.yaw_acc;

      data["plan_pitch"] = plan.pitch;
      data["plan_pitch_vel"] = plan.pitch_vel;
      data["plan_pitch_acc"] = plan.pitch_acc;

      data["fire"] = plan.fire ? 1 : 0;
      data["fired"] = fired ? 1 : 0;

      if (input.target.has_value()) {
        Eigen::VectorXd x = input.target->ekf_x();
        data["x"] = x[0];
        data["vx"] = x[1];
        data["y"] = x[2];
        data["vy"] = x[3];
        data["z"] = x[4];
        data["vz"] = x[5];
        data["a"] = x[6];
        data["w"] = x[7];
        data["r"] = x[8];
        data["l"] = x[9];
        data["h"] = x[10];
      }

      data["back_camera_active"] = input.back_command.control ? 1 : 0;

      plotter.plot(data);

      std::this_thread::sleep_for(10ms);
    }
  });

  cv::Mat img;
  std::chrono::steady_clock::time_point timestamp;

  cv::Mat front_img, back_img;
  std::chrono::steady_clock::time_point front_ts, back_ts;
  
  // 后置相机参数
  double back_fov_h = 54.2;   // 后置相机水平视场角
  double back_fov_v = 44.5;   // 后置相机垂直视场角
  double back_yaw_offset = 180.0;  // 后置相机相对于前置的偏航角偏移（度）
  
  while (!exiter.exit()) {
    front_camera.read(front_img, front_ts);
    back_camera.read(back_img, back_ts);
    img = front_img.clone();
    timestamp = front_ts;
    Eigen::Quaterniond q = gimbal.q(timestamp - 1ms);

    /// 自瞄核心逻辑
    solver.set_R_gimbal2world(q);

    Eigen::Vector3d gimbal_pos = tools::eulers(solver.R_gimbal2world(), 2, 1, 0);

    auto armors = yolo.detect(img);
    spdlog::info("YOLO detected {} armors (front)", armors.size());

    decider.get_invincible_armor(ros2.subscribe_enemy_status());
    decider.armor_filter(armors);
    spdlog::info("After filter: {} armors (front)", armors.size());

    decider.get_auto_aim_target(armors, ros2.subscribe_autoaim_target());
    decider.set_priority(armors);

    auto targets = tracker.track(armors, timestamp);

    /// 后置相机检测逻辑
    auto back_armors = back_yolo.detect(back_img);
    spdlog::info("Back camera YOLO detected {} armors (before filter)", back_armors.size());
    
    // 在过滤前绘制所有YOLO检测到的装甲板（黄色）
    for (const auto& armor : back_armors) {
      cv::rectangle(back_img, armor.box, cv::Scalar(0, 255, 255), 2);
      tools::draw_text(back_img, fmt::format("c:{} n:{}", static_cast<int>(armor.color), static_cast<int>(armor.name)), 
                       {armor.box.x, armor.box.y - 5}, {0, 255, 255});
    }
    
    decider.armor_filter(back_armors);
    spdlog::info("Back camera detected {} armors after filter", back_armors.size());
    
    // 计算后置相机检测到的目标转向角度
    io::Command back_command{false, false, 0, 0};
    if (!back_armors.empty()) {
      auto& back_armor = back_armors.front();
      
      double delta_yaw_deg = back_yaw_offset + (back_fov_h / 2) - back_armor.center_norm.x * back_fov_h;
      double delta_pitch_deg = back_armor.center_norm.y * back_fov_v - back_fov_v / 2;
      
      back_command.control = true;
      back_command.yaw = tools::limit_rad(gimbal_pos[0] + delta_yaw_deg / 57.3);
      back_command.pitch = tools::limit_rad(delta_pitch_deg / 57.3);
      
      spdlog::info("Back camera target: delta_yaw={:.1f}deg, delta_pitch={:.1f}deg", delta_yaw_deg, delta_pitch_deg);
      
      tools::draw_text(back_img, fmt::format("yaw:{:.1f}", delta_yaw_deg), 
                       cv::Point(static_cast<int>(back_armor.center.x), static_cast<int>(back_armor.center.y) - 30), 
                       {0, 255, 0});
    }

    // 推送数据给规划线程
    PlanInput input;
    if (!targets.empty()) {
      input.target = targets.front();
    } else {
      input.target = std::nullopt;
    }
    input.back_command = back_command;
    input.has_front_armors = !armors.empty();
    plan_queue.push(input);

    /// ROS2通信
    Eigen::Vector4d target_info = decider.get_target_info(armors, targets);
    ros2.publish(target_info);

    /// debug 绘制
    tools::draw_text(img, fmt::format("[{}] armors:{} targets:{}", tracker.state(), armors.size(), targets.size()), 
                     {10, 30}, {255, 255, 255});

    if (!targets.empty()) {
      auto target = targets.front();

      // 绘制所有装甲板位置
      std::vector<Eigen::Vector4d> armor_xyza_list = target.armor_xyza_list();
      for (const Eigen::Vector4d & xyza : armor_xyza_list) {
        auto image_points = solver.reproject_armor(xyza.head(3), xyza[3], target.armor_type, target.name);
        tools::draw_points(img, image_points, {0, 255, 0});
      }

      // planner瞄准位置
      Eigen::Vector4d aim_xyza = planner.debug_xyza;
      auto image_points = solver.reproject_armor(aim_xyza.head(3), aim_xyza[3], target.armor_type, target.name);
      tools::draw_points(img, image_points, {0, 0, 255});
    }
    //
    cv::resize(img, img, {}, 0.5, 0.5);
    tools::draw_text(back_img, fmt::format("Back armors: {}", back_armors.size()), {10, 30}, {0, 255, 255});
    cv::resize(back_img, back_img, {}, 0.5, 0.5);
    cv::imshow("front camera", img);
    cv::imshow("back camera", back_img);
    auto key = cv::waitKey(1);
    if (key == 'q') break;
  }

  quit = true;
  if (plan_thread.joinable()) plan_thread.join();
  gimbal.send(false, false, 0, 0, 0, 0, 0, 0);

  return 0;
}
