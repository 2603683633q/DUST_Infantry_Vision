#include <iostream>
#include <thread>
#include <chrono>
#include <csignal>

#include "io/cboard.hpp"
#include "tools/yaml.hpp"

volatile bool running = true;

void signal_handler(int signal) {
    running = false;
}

int main() {
    signal(SIGINT, signal_handler);
    
    try {
        // 创建CBoard实例
        io::CBoard cboard("configs/sentry.yaml");
        
        std::cout << "CBoard initialized. Waiting for IMU data..." << std::endl;
        std::cout << "Press Ctrl+C to exit." << std::endl;
        
        // 等待一段时间让CBoard接收数据
        while (running) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        
        std::cout << "Exiting..." << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}
