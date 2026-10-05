#include "mecanum_gateway/gateway_node.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <stdexcept>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Matrix3x3.h>

namespace mecanum_gateway {
GatewayNode::GatewayNode(const rclcpp::NodeOptions & options) : Node("gateway_node", options) {
  const auto address = declare_parameter<std::string>("bind_address", "0.0.0.0");
  const auto port = declare_parameter<int64_t>("port", 8765);
  const auto rate = declare_parameter<double>("telemetry_rate", 10.0);
  odom_timeout_ = declare_parameter<double>("odom_timeout", 1.0);
  map_tf_future_tolerance_ = declare_parameter<double>("map_tf_future_tolerance", 1.5);
  map_tf_stale_timeout_ = declare_parameter<double>("map_tf_stale_timeout", 1.0);
  linear_threshold_ = declare_parameter<double>("linear_motion_threshold", 0.01);
  angular_threshold_ = declare_parameter<double>("angular_motion_threshold", 0.01);
  if (port < 1 || port > 65535) throw std::invalid_argument("port must be between 1 and 65535");
  if (!std::isfinite(rate) || rate <= 0.0) throw std::invalid_argument("telemetry_rate must be finite and positive");
  if (!std::isfinite(odom_timeout_) || odom_timeout_ <= 0.0) throw std::invalid_argument("odom_timeout must be finite and positive");
  if (!std::isfinite(map_tf_future_tolerance_) || map_tf_future_tolerance_ < 0.0)
    throw std::invalid_argument("map_tf_future_tolerance must be finite and nonnegative");
  if (!std::isfinite(map_tf_stale_timeout_) || map_tf_stale_timeout_ <= 0.0)
    throw std::invalid_argument("map_tf_stale_timeout must be finite and positive");
  if (!std::isfinite(linear_threshold_) || linear_threshold_ < 0.0 ||
      !std::isfinite(angular_threshold_) || angular_threshold_ < 0.0) {
    throw std::invalid_argument("motion thresholds must be finite and nonnegative");
  }
  mode_manager_ = std::make_unique<ModeManager>(
    declare_parameter<std::string>("map_dir", "~/mecanum_maps"),
    declare_parameter<double>("map_save_timeout", 30.0));
  max_manual_linear_ = declare_parameter<double>("max_manual_linear_mps", 0.30);
  max_manual_angular_ = declare_parameter<double>("max_manual_angular_rps", 1.00);
  manual_timeout_ = declare_parameter<double>("manual_cmd_timeout", 0.30);
  for (double value : {max_manual_linear_, max_manual_angular_, manual_timeout_})
    if (!std::isfinite(value) || value <= 0) throw std::invalid_argument("manual limits/timeouts must be finite and positive");
  server_ = std::make_unique<TcpServer>(address, static_cast<int>(port));
  velocity_publisher_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  manual_timer_ = create_wall_timer(std::chrono::milliseconds(20),
    std::bind(&GatewayNode::check_manual_timeout, this));
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
  map_stream_ = std::make_unique<MapStream>([this](std::string packet) { server_->publish_map(std::move(packet)); });
  map_subscription_ = create_subscription<nav_msgs::msg::OccupancyGrid>("/map",
    rclcpp::QoS(1).reliable().transient_local(),
    [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr map) { map_stream_->Update(std::move(map)); });
  subscription_ = create_subscription<nav_msgs::msg::Odometry>("/odometry/filtered", 10,
    [this](nav_msgs::msg::Odometry::ConstSharedPtr message) { on_odometry(*message); });
  timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / rate),
    std::bind(&GatewayNode::tick, this));
  tick();
  server_->start();
  RCLCPP_INFO(get_logger(), "TCP gateway listening on %s:%ld", address.c_str(), static_cast<long>(port));
}
GatewayNode::~GatewayNode() { Shutdown(); }
void GatewayNode::Shutdown() {
  if (shutdown_) return;
  shutdown_ = true;
  if (timer_) timer_->cancel();
  if (manual_timer_) manual_timer_->cancel();
  if (rclcpp::ok(get_node_base_interface()->get_context()) && velocity_publisher_) {
    zero_manual(mode_manager_ && mode_manager_->GetMode() == "MAPPING");
    velocity_publisher_->wait_for_all_acked(std::chrono::milliseconds(100));
  }
  if (map_stream_) map_stream_->Stop();
  if (server_) server_->stop();
  mode_manager_.reset();
}
void GatewayNode::zero_manual(bool force) {
  if (manual_active_ || force) velocity_publisher_->publish(geometry_msgs::msg::Twist{});
  manual_active_ = false;
}
void GatewayNode::check_manual_timeout() {
  const auto disconnected = server_->disconnect_count();
  if (disconnected != disconnect_count_) {
    zero_manual(mode_manager_->GetMode() == "MAPPING");
    disconnect_count_ = disconnected;
  }
  if (manual_active_ && (mode_manager_->GetMode() != "MAPPING" || mode_manager_->IsStopping() ||
      server_->active_session() != manual_session_ ||
      std::chrono::duration<double>(std::chrono::steady_clock::now() - last_manual_).count() >= manual_timeout_)) {
    zero_manual();
  }
}
void GatewayNode::manual_command(std::uint64_t session, const protocol::Command & command) {
  auto reply = [&](bool success, const std::string & message) {
    server_->send_result(session, protocol::command_result(command, success, message));
  };
  if (mode_manager_->GetMode() != "MAPPING" || mode_manager_->IsStopping()) {
    reply(false, "manual velocity is only allowed in MAPPING mode"); return;
  }
  if (session != server_->active_session()) { reply(false, "control client disconnected"); return; }
  for (const auto * value : {&command.vx, &command.vy, &command.wz}) {
    if (!value->is_number() || !std::isfinite(value->get<double>())) {
      reply(false, "vx, vy and wz must be finite numbers"); return;
    }
  }
  geometry_msgs::msg::Twist twist;
  twist.linear.x = std::clamp(command.vx.get<double>(), -max_manual_linear_, max_manual_linear_);
  twist.linear.y = std::clamp(command.vy.get<double>(), -max_manual_linear_, max_manual_linear_);
  twist.angular.z = std::clamp(command.wz.get<double>(), -max_manual_angular_, max_manual_angular_);
  velocity_publisher_->publish(twist);
  manual_active_ = twist.linear.x != 0 || twist.linear.y != 0 || twist.angular.z != 0;
  if (manual_active_) { last_manual_ = std::chrono::steady_clock::now(); manual_session_ = session; }
  reply(true, "manual velocity applied");
}
void GatewayNode::on_odometry(const nav_msgs::msg::Odometry & message) {
  const auto & p = message.pose.pose.position;
  const auto & q = message.pose.pose.orientation;
  const auto & t = message.twist.twist;
  const std::array<double, 9> values{p.x, p.y, q.x, q.y, q.z, q.w, t.linear.x, t.linear.y, t.angular.z};
  if (!std::all_of(values.begin(), values.end(), [](double v) { return std::isfinite(v); })) return;
  const double norm = std::hypot(std::hypot(q.x, q.y), std::hypot(q.z, q.w));
  if (!std::isfinite(norm) || norm < 1e-12) return;
  const double x = q.x / norm, y = q.y / norm, z = q.z / norm, w = q.w / norm;
  Snapshot sample;
  sample.frame_id = message.header.frame_id;
  sample.x = p.x;
  sample.y = p.y;
  sample.yaw = std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
  sample.vx = t.linear.x;
  sample.vy = t.linear.y;
  sample.wz = t.angular.z;
  sample.received = std::chrono::steady_clock::now();
  std::lock_guard<std::mutex> lock(snapshot_mutex_);
  snapshot_ = sample;
}
void GatewayNode::tick() {
  check_manual_timeout();
  for (const auto & reply : mode_manager_->Update()) {
    server_->send_result(reply.session, protocol::command_result(reply.command, reply.success, reply.message));
  }
  TcpServer::CommandFrame command;
  for (int i = 0; i < 128 && server_->take_command(command); ++i) {
    const auto parsed = protocol::parse_command(command.frame);
    if (!parsed.error.empty() || parsed.name == "get_status") {
      server_->send_result(command.session, protocol::command_result(parsed));
    } else if (parsed.name == "manual_velocity") {
      manual_command(command.session, parsed);
    } else {
      if (parsed.name == "stop_mode" && mode_manager_->GetMode() == "MAPPING") zero_manual(true);
      if (auto reply = mode_manager_->Handle(command.session, parsed))
        server_->send_result(reply->session, protocol::command_result(reply->command, reply->success, reply->message));
    }
  }
  Snapshot sample;
  {
    std::lock_guard<std::mutex> lock(snapshot_mutex_);
    sample = snapshot_;
  }
  protocol::RobotState state;
  state.timestamp = now().seconds();
  state.mode = mode_manager_->GetMode();
  map_stream_->SetEnabled(state.mode != "BASE");
  check_manual_timeout();
  state.x = sample.x; state.y = sample.y; state.yaw = sample.yaw;
  state.vx = sample.vx; state.vy = sample.vy; state.wz = sample.wz;
  state.base_ready = sample.received &&
    std::chrono::duration<double>(std::chrono::steady_clock::now() - *sample.received).count() <= odom_timeout_;
  if (state.base_ready) {
    state.motion_state = (std::hypot(sample.vx, sample.vy) > linear_threshold_ ||
      std::abs(sample.wz) > angular_threshold_) ? "MOVING" : "STOPPED";
  } else {
    state.error = sample.received ? "odometry stale" : "odometry unavailable";
  }
  if (state.base_ready && !sample.frame_id.empty() && state.mode != "BASE") {
    try {
      // Nonblocking lookup; preserve the original odom pose in robot_state.pose.
      auto transform = tf_buffer_->lookupTransform("map", sample.frame_id, tf2::TimePointZero);
      const auto & t = transform.transform;
      const double age = (now() - rclcpp::Time(transform.header.stamp)).seconds();
      // Latest lookup also works before TF has history spanning the odometry stamp.
      // AMCL future-dates map->odom by transform_tolerance. These ROS-time bounds
      // are independent of odometry's steady-clock reception timeout.
      // Zero-stamped static transforms are timeless.
      if ((transform.header.stamp.sec == 0 && transform.header.stamp.nanosec == 0) ||
          (age >= -map_tf_future_tolerance_ && age <= map_tf_stale_timeout_)) {
        tf2::Transform mapping(tf2::Quaternion(t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w),
          tf2::Vector3(t.translation.x, t.translation.y, t.translation.z));
        tf2::Quaternion body; body.setRPY(0, 0, sample.yaw);
        const auto point = mapping * tf2::Vector3(sample.x, sample.y, 0);
        double roll, pitch, yaw;
        tf2::Matrix3x3(mapping.getRotation() * body).getRPY(roll, pitch, yaw);
        if (std::isfinite(point.x()) && std::isfinite(point.y()) && std::isfinite(yaw))
          state.map_pose = {{"x", point.x()}, {"y", point.y()}, {"yaw", yaw}};
      }
    } catch (const tf2::TransformException &) { /* TF unavailable: map_pose stays null. */ }
  }
  if (!mode_manager_->GetError().empty()) {
    if (!state.error.empty()) state.error += "; ";
    state.error += mode_manager_->GetError();
  }
  server_->publish_state(protocol::robot_state(state));
}
}  // namespace mecanum_gateway
