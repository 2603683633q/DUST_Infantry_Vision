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
        std::cout << "Starting CBoard with angle printing..." << std::endl;
        std::cout << "Press Ctrl+C to exit." << std::endl;
        
        // 创建CBoard实例，它会在内部线程中打印偏角信息
        io::CBoard cboard("../configs/sentry.yaml");
        
        // 等待一段时间让CBoard接收数据并打印
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
