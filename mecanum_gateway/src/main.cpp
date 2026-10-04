#include <csignal>
#include <cstdio>
#include <memory>
#include <thread>
#include "mecanum_gateway/gateway_node.hpp"

namespace {
volatile std::sig_atomic_t interrupted = 0;
void interrupt(int) { interrupted = 1; }
}
int main(int argc, char ** argv) {
  // Publish the final manual stop while the ROS context is still valid.
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT, interrupt);
  std::signal(SIGTERM, interrupt);
  int status = 0;
  try {
    auto node = std::make_shared<mecanum_gateway::GatewayNode>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    while (rclcpp::ok() && !interrupted) {
      executor.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    node->Shutdown();
  } catch (const std::exception & error) {
    std::fprintf(stderr, "Gateway error: %s\n", error.what());
    status = 1;
  }
  rclcpp::shutdown();
  return status;
}
