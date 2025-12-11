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
#include "tools/plotter.hpp"
#include "tools/recorder.hpp"

using namespace std::chrono;

// 简单的灯条检测函数（用于后置相机，不需要数字识别）
std::vector<cv::RotatedRect> detect_lightbars(const cv::Mat& bgr_img, int threshold = 150) {
  std::vector<cv::RotatedRect> lightbars;
  
  // 转换到HSV并提取红色
  cv::Mat hsv, mask1, mask2, mask;
  cv::cvtColor(bgr_img, hsv, cv::COLOR_BGR2HSV);
  
  // 红色范围 (两段，因为红色在HSV中跨越0度)
  cv::inRange(hsv, cv::Scalar(0, 100, 100), cv::Scalar(10, 255, 255), mask1);
  cv::inRange(hsv, cv::Scalar(160, 100, 100), cv::Scalar(180, 255, 255), mask2);
  mask = mask1 | mask2;
  
  // 形态学操作
  cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
  cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
  cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);
  
  // 查找轮廓
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
  
  for (const auto& contour : contours) {
    if (contour.size() < 5) continue;
    
    cv::RotatedRect rect = cv::minAreaRect(contour);
    float ratio = std::max(rect.size.width, rect.size.height) / 
                  std::min(rect.size.width, rect.size.height);
    
    // 灯条应该是细长的
    if (ratio > 2.0 && rect.size.area() > 50) {
      lightbars.push_back(rect);
    }
  }
  
  return lightbars;
}

// 从两个灯条配对成装甲板中心
cv::Point2f pair_lightbars_to_armor(const std::vector<cv::RotatedRect>& lightbars) {
  if (lightbars.size() < 2) return cv::Point2f(-1, -1);
  
  // 找到最近的两个灯条
  double min_dist = 1e9;
  int idx1 = 0, idx2 = 1;
  
  for (size_t i = 0; i < lightbars.size(); ++i) {
    for (size_t j = i + 1; j < lightbars.size(); ++j) {
      double dist = cv::norm(lightbars[i].center - lightbars[j].center);
      if (dist < min_dist && dist > 20) {  // 至少要有一定距离
        min_dist = dist;
        idx1 = i;
        idx2 = j;
      }
    }
  }
  
  // 返回两个灯条的中点作为装甲板中心
  return (lightbars[idx1].center + lightbars[idx2].center) / 2.0f;
}

const std::string keys =
  "{help h usage ? |                        | 输出命令行参数说明}"
  "{@config-path   | configs/sentry.yaml | 位置参数，yaml配置文件路径 }";

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

  auto_aim::YOLO yolo(config_path, true);  // 前置相机YOLO
  auto_aim::YOLO back_yolo(config_path, true); // 后置相机YOLO (无debug窗口)
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path, solver);
  auto_aim::Aimer aimer(config_path);
  auto_aim::Shooter shooter(config_path);

  omniperception::Decider decider(config_path);

  cv::Mat img;

  std::chrono::steady_clock::time_point timestamp;
  io::Command last_command;

  cv::Mat front_img, back_img;
  std::chrono::steady_clock::time_point front_ts, back_ts;
  
  // 后置相机参数
  double back_fov_h = 54.2;  // 后置相机水平视场角
  double back_fov_v = 44.5;  // 后置相机垂直视场角
  double back_yaw_offset = 180.0;  // 后置相机相对于前置的偏航角偏移（度）
  
  while (!exiter.exit()) {
    front_camera.read(front_img, front_ts);
    back_camera.read(back_img, back_ts);
    img = front_img.clone();  // 主相机用于自瞄 (深拷贝以便绘制标注)
    timestamp = front_ts;
    Eigen::Quaterniond q = gimbal.q(timestamp - 1ms);
    // recorder.record(img, q, timestamp);

    /// 自瞄核心逻辑
    solver.set_R_gimbal2world(q);

    Eigen::Vector3d gimbal_pos = tools::eulers(solver.R_gimbal2world(), 2, 1, 0);

    auto armors = yolo.detect(img);
    spdlog::info("YOLO detected {} armors (front)", armors.size());
    for (auto& a : armors) {
      spdlog::info("  armor: name={}, color={}", static_cast<int>(a.name), static_cast<int>(a.color));
    }

    decider.get_invincible_armor(ros2.subscribe_enemy_status());

    decider.armor_filter(armors);
    spdlog::info("After filter: {} armors (front)", armors.size());

    decider.get_auto_aim_target(armors, ros2.subscribe_autoaim_target());

    decider.set_priority(armors);

    auto targets = tracker.track(armors, timestamp);

    io::Command command{false, false, 0, 0};
    
    /// 后置相机检测逻辑
    spdlog::info("Back img size: {}x{}", back_img.cols, back_img.rows);
    auto back_armors = back_yolo.detect(back_img);
    spdlog::info("Back camera YOLO detected {} armors (before filter)", back_armors.size());
    
    // 在过滤前绘制所有YOLO检测到的装甲板（黄色）
    for (const auto& armor : back_armors) {
      cv::rectangle(back_img, armor.box, cv::Scalar(0, 255, 255), 2);  // 黄色
      tools::draw_text(back_img, fmt::format("c:{} n:{}", static_cast<int>(armor.color), static_cast<int>(armor.name)), 
                       {armor.box.x, armor.box.y - 5}, {0, 255, 255});
    }
    
    decider.armor_filter(back_armors);
    spdlog::info("Back camera detected {} armors after filter", back_armors.size());
    
    // 计算后置相机检测到的目标转向角度
    io::Command back_command{false, false, 0, 0};
    if (!back_armors.empty()) {
      // 选择最近/最大的装甲板
      auto& back_armor = back_armors.front();
      
      // 根据装甲板在图像中的位置计算转向角度
      double delta_yaw_deg = back_yaw_offset + (back_fov_h / 2) - back_armor.center_norm.x * back_fov_h;
      double delta_pitch_deg = back_armor.center_norm.y * back_fov_v - back_fov_v / 2;
      
      back_command.control = true;
      back_command.yaw = tools::limit_rad(gimbal_pos[0] + delta_yaw_deg / 57.3);
      back_command.pitch = tools::limit_rad(delta_pitch_deg / 57.3);
      
      spdlog::info("Back camera target: delta_yaw={:.1f}deg, delta_pitch={:.1f}deg, center=({:.0f},{:.0f})", 
                   delta_yaw_deg, delta_pitch_deg, back_armor.center.x, back_armor.center.y);
      
      // 在后置图像上绘制信息
      tools::draw_text(back_img, fmt::format("yaw:{:.1f}", delta_yaw_deg), 
                       cv::Point(static_cast<int>(back_armor.center.x), static_cast<int>(back_armor.center.y) - 30), 
                       {0, 255, 0});
    }

    /// 前置相机优先的自瞄逻辑
    if (tracker.state() != "lost") {
      // 前置相机有跟踪目标，使用精确瞄准
      command = aimer.aim(targets, timestamp, gimbal.state().bullet_speed, io::ShootMode::both_shoot, solver.R_gimbal2world());
    } else if (!armors.empty()) {
      // 前置相机检测到但未跟踪，使用全向感知
      command = decider.decide(yolo, gimbal_pos, front_camera);
    } else if (back_command.control) {
      // 后置相机检测到目标，只提供转向方向（不射击）
      command = back_command;
      command.shoot = false;  // 后置相机不触发射击
      spdlog::info("Using back camera guidance");
    }

    /// 发射逻辑
    command.shoot = shooter.shoot(command, aimer, targets, gimbal_pos);

    gimbal.send(command.control, command.shoot, command.yaw, 0, 0, command.pitch, 0, 0);

    /// ROS2通信
    Eigen::Vector4d target_info = decider.get_target_info(armors, targets);

    ros2.publish(target_info);

    /// debug
    tools::draw_text(img, fmt::format("[{}] armors:{} targets:{}", tracker.state(), armors.size(), targets.size()), {10, 30}, {255, 255, 255});

    nlohmann::json data;

    // 装甲板原始观测数据
    data["armor_num"] = armors.size();
    if (!armors.empty()) {
      auto min_x = 1e10;
      auto & armor = armors.front();
      for (auto & a : armors) {
        if (a.center.x < min_x) {
          min_x = a.center.x;
          armor = a;
        }
      }  //always left
      solver.solve(armor);
      data["armor_x"] = armor.xyz_in_world[0];
      data["armor_y"] = armor.xyz_in_world[1];
  data["armor_yaw"] = armor.ypr_in_world[0];
  data["armor_yaw_raw"] = armor.yaw_raw;
    }

    if (!targets.empty()) {
      auto target = targets.front();

      // 当前帧target更新后
      std::vector<Eigen::Vector4d> armor_xyza_list = target.armor_xyza_list();
      for (const Eigen::Vector4d & xyza : armor_xyza_list) {
        auto image_points =
          solver.reproject_armor(xyza.head(3), xyza[3], target.armor_type, target.name);
        tools::draw_points(img, image_points, {0, 255, 0});
      }

      // aimer瞄准位置
      auto aim_point = aimer.debug_aim_point;
      Eigen::Vector4d aim_xyza = aim_point.xyza;
      auto image_points =
        solver.reproject_armor(aim_xyza.head(3), aim_xyza[3], target.armor_type, target.name);
      if (aim_point.valid)
        tools::draw_points(img, image_points, {0, 0, 255});
      else
        tools::draw_points(img, image_points, {255, 0, 0});

      // 观测器内部数据
      Eigen::VectorXd x = target.ekf_x();
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
      data["last_id"] = target.last_id;

      // 卡方检验数据
      data["residual_yaw"] = target.ekf().data.at("residual_yaw");
      data["residual_pitch"] = target.ekf().data.at("residual_pitch");
      data["residual_distance"] = target.ekf().data.at("residual_distance");
      data["residual_angle"] = target.ekf().data.at("residual_angle");
      data["nis"] = target.ekf().data.at("nis");
      data["nees"] = target.ekf().data.at("nees");
      data["nis_fail"] = target.ekf().data.at("nis_fail");
      data["nees_fail"] = target.ekf().data.at("nees_fail");
      data["recent_nis_failures"] = target.ekf().data.at("recent_nis_failures");
    }

    // 云台响应情况
  data["gimbal_yaw"] = gimbal_pos[0];
  data["gimbal_pitch"] = -gimbal_pos[1];
    data["shootmode"] = io::ShootMode::both_shoot;
    if (command.control) {
  data["cmd_yaw"] = command.yaw;
  data["cmd_pitch"] = command.pitch;
      data["cmd_shoot"] = command.shoot;
    }

    data["bullet_speed"] = gimbal.state().bullet_speed;

    plotter.plot(data);

    cv::resize(img, img, {}, 0.5, 0.5);  // 显示时缩小图片尺寸 (带标注的前置相机)
    tools::draw_text(back_img, fmt::format("Back armors: {}", back_armors.size()), {10, 30}, {0, 255, 255});
    cv::resize(back_img, back_img, {}, 0.5, 0.5);
    cv::imshow("front camera", img);
    cv::imshow("back camera", back_img);
    auto key = cv::waitKey(1);
    if (key == 'q') break;
  }
  return 0;
}
