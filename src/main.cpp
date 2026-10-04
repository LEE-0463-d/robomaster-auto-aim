#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include "auto_aim/aim_node.h"
#include "auto_aim/serial_turret.h"

namespace {

constexpr int kSlaveWaitTimeoutS = 600;  // 等待游戏虚拟串口的超时（s）
constexpr int kSlaveHintIntervalS = 10;  // 等待提示的间隔（s）

}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  std::string slave;
  // 游戏可能比本程序晚启动，轮询等待虚拟串口（最多 600 秒）。
  for (int waited = 0;; ++waited) {
    slave = auto_aim::SerialTurret::FindSlave();
    if (!slave.empty()) {
      break;
    }
    if (waited == 0) {
      std::cout << "未找到游戏串口，等待游戏启动..." << std::endl;
    } else if (waited % kSlaveHintIntervalS == 0) {
      std::cout << "仍在等待游戏串口 (" << waited << "s)" << std::endl;
    }
    if (waited >= kSlaveWaitTimeoutS) {
      std::cerr << "等待超时：未找到游戏串口，请先启动游戏" << std::endl;
      rclcpp::shutdown();
      return 1;
    }
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
  const auto node = std::make_shared<auto_aim::AimNode>(slave);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}