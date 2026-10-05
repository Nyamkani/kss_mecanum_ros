#pragma once

#include <string>
#include <nlohmann/json.hpp>

namespace mecanum_gateway::protocol {
struct Command {
  nlohmann::json name = nullptr;
  nlohmann::json request_id = nullptr;
  nlohmann::json map_name = nullptr;
  nlohmann::json vx = nullptr, vy = nullptr, wz = nullptr;
  nlohmann::json x = nullptr, y = nullptr, yaw = nullptr;
  std::string error;
};
struct RobotState {
  double timestamp = 0.0;
  std::string mode = "BASE";
  bool base_ready = false;
  double x = 0.0, y = 0.0, yaw = 0.0;
  double vx = 0.0, vy = 0.0, wz = 0.0;
  nlohmann::json map_pose = nullptr;
  std::string motion_state = "UNKNOWN";
  std::string navigation_state = "IDLE";
  std::string error;
};
Command parse_command(const std::string & frame);
std::string command_result(const Command & command);
std::string command_result(const Command & command, bool success, const std::string & message);
std::string robot_state(const RobotState & state);
std::string map_packet(std::uint64_t sequence, double resolution, std::uint32_t width,
  std::uint32_t height, double x, double y, double qz, double qw, const std::string & data);
}  // namespace mecanum_gateway::protocol
