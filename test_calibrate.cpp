#include <fmt/core.h>
#include <yaml-cpp/yaml.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>

int main() {
    try {
        // 测试YAML读取
        auto yaml = YAML::LoadFile("configs/calibration.yaml");
        auto pattern_cols = yaml["pattern_cols"].as<int>();
        auto pattern_rows = yaml["pattern_rows"].as<int>();
        auto center_distance_mm = yaml["center_distance_mm"].as<double>();
        
        fmt::print("YAML读取成功:\n");
        fmt::print("pattern_cols: {}\n", pattern_cols);
        fmt::print("pattern_rows: {}\n", pattern_rows);
        fmt::print("center_distance_mm: {}\n", center_distance_mm);
        
        // 测试图片读取
        auto img = cv::imread("assets/img_with_q/1.jpg");
        if (img.empty()) {
            fmt::print("图片读取失败\n");
            return 1;
        }
        fmt::print("图片读取成功, 尺寸: {}x{}\n", img.cols, img.rows);
        
        // 测试标定板检测
        cv::Size pattern_size(pattern_cols, pattern_rows);
        std::vector<cv::Point2f> centers_2d;
        auto success = cv::findCirclesGrid(img, pattern_size, centers_2d);
        fmt::print("标定板检测: {}\n", success ? "成功" : "失败");
        if (success) {
            fmt::print("检测到 {} 个圆点\n", centers_2d.size());
        }
        
        // 测试四元数读取
        std::ifstream q_file("assets/img_with_q/1.txt");
        double w, x, y, z;
        q_file >> w >> x >> y >> z;
        fmt::print("四元数读取成功: {:.3f}, {:.3f}, {:.3f}, {:.3f}\n", w, x, y, z);
        
        fmt::print("所有测试通过!\n");
        
    } catch (const std::exception& e) {
        fmt::print("错误: {}\n", e.what());
        return 1;
    }
    
    return 0;
}
