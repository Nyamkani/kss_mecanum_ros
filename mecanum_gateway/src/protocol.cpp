#include "mecanum_gateway/protocol.hpp"

namespace mecanum_gateway::protocol {
using nlohmann::json;
Command parse_command(const std::string & frame) {
  Command result;
  try {
    const auto value = json::parse(frame);
    if (!value.is_object()) {
      result.error = "command must be a JSON object";
      return result;
    }
    result.x = value.value("x", json(nullptr));
    result.y = value.value("y", json(nullptr));
    result.yaw = value.value("yaw", json(nullptr));
    result.vx = value.value("vx", json(nullptr));
    result.vy = value.value("vy", json(nullptr));
    result.wz = value.value("wz", json(nullptr));
    result.map_name = value.value("map_name", json(nullptr));
    result.name = value.value("command", json(nullptr));
    result.request_id = value.value("request_id", json(nullptr));
    if (!(result.request_id.is_null() || result.request_id.is_number_integer() ||
          result.request_id.is_string())) {
      result.request_id = nullptr;
      result.error = "request_id must be an integer, string, or null";
    }
    if (!result.name.is_string()) {
      result.name = nullptr;
      result.error = "command must be a string";
    }
  } catch (const json::exception &) {
    result = Command{};
    result.error = "invalid JSON";
  }
  return result;
}
std::string command_result(const Command & command) {
  const bool success = command.error.empty() && command.name == "get_status";
  return command_result(command, success,
    command.error.empty() ? (success ? "ok" : "unknown command") : command.error);
}
std::string command_result(const Command & command, bool success, const std::string & message) {
  return json({{"type", "command_result"}, {"request_id", command.request_id},
    {"command", command.name}, {"success", success},
    {"message", message}
  }).dump(-1, ' ', true, json::error_handler_t::replace) + '\n';
}
std::string robot_state(const RobotState & s) {
  return json({{"type", "robot_state"}, {"timestamp", s.timestamp}, {"mode", s.mode},
    {"base_ready", s.base_ready}, {"pose", {{"x", s.x}, {"y", s.y}, {"yaw", s.yaw}}},
    {"velocity", {{"vx", s.vx}, {"vy", s.vy}, {"wz", s.wz}}},
    {"navigation_state", s.navigation_state}, {"map_pose", s.map_pose}, {"motion_state", s.motion_state}, {"error", s.error}
  }).dump(-1, ' ', true, json::error_handler_t::replace) + '\n';
}
std::string map_packet(std::uint64_t sequence, double resolution, std::uint32_t width,
  std::uint32_t height, double x, double y, double qz, double qw, const std::string & data) {
  return json({{"type", "map"}, {"sequence", sequence}, {"resolution", resolution},
    {"width", width}, {"height", height}, {"origin", {{"x", x}, {"y", y}, {"qz", qz}, {"qw", qw}}},
    {"encoding", "zlib+base64:int8"}, {"data", data}}).dump() + '\n';
}
}  // namespace mecanum_gateway::protocol
