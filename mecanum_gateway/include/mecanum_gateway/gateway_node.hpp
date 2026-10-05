#pragma once

#include <chrono>
#include <memory>
#include <mutex>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include "mecanum_gateway/map_stream.hpp"
#include "mecanum_gateway/navigation_client.hpp"
#include "mecanum_gateway/protocol.hpp"
#include "mecanum_gateway/mode_manager.hpp"
#include "mecanum_gateway/tcp_server.hpp"

namespace mecanum_gateway {
class GatewayNode : public rclcpp::Node {
public:
  explicit GatewayNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~GatewayNode() override;
  void Shutdown();
private:
  struct Snapshot {
    double x = 0.0, y = 0.0, yaw = 0.0;
    double vx = 0.0, vy = 0.0, wz = 0.0;
    std::string frame_id;
    std::optional<std::chrono::steady_clock::time_point> received;
  };
  void on_odometry(const nav_msgs::msg::Odometry & message);
  void tick();
  void check_manual_timeout();
  void zero_manual(bool force = false);
  void initial_pose_command(std::uint64_t session, const protocol::Command & command);
  void manual_command(std::uint64_t session, const protocol::Command & command);
  std::unique_ptr<ModeManager> mode_manager_;
  std::unique_ptr<NavigationClient> navigation_;
  double odom_timeout_, linear_threshold_, angular_threshold_;
  double initial_pose_xy_variance_, initial_pose_yaw_variance_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_publisher_;
  double map_tf_future_tolerance_, map_tf_stale_timeout_;
  std::mutex snapshot_mutex_;
  Snapshot snapshot_;
  std::unique_ptr<TcpServer> server_;
  std::unique_ptr<MapStream> map_stream_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscription_;
  rclcpp::TimerBase::SharedPtr manual_timer_;
  double max_manual_linear_, max_manual_angular_, manual_timeout_;
  bool manual_active_ = false, shutdown_ = false;
  std::uint64_t manual_session_ = 0, disconnect_count_ = 0;
  std::chrono::steady_clock::time_point last_manual_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace mecanum_gateway
