#pragma once
#include <functional>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include "mecanum_gateway/protocol.hpp"

namespace mecanum_gateway {
// Called only on the Gateway's single-threaded executor.
class NavigationClient {
public:
  using Action = nav2_msgs::action::NavigateToPose;
  using Handle = rclcpp_action::ClientGoalHandle<Action>;
  using Reply = std::function<void(std::uint64_t, std::string)>;
  NavigationClient(rclcpp::Node * node, Reply reply);
  void Command(std::uint64_t session, const protocol::Command & command, bool allowed);
  void Reset();
  const std::string & State() const { return state_; }
private:
  rclcpp::Node * node_;
  Reply reply_;
  rclcpp_action::Client<Action>::SharedPtr client_;
  Handle::SharedPtr handle_;
  std::string state_ = "IDLE";
  std::uint64_t generation_ = 0, cancel_generation_ = 0;
  std::function<void(bool, const std::string &)> pending_goal_reply_, pending_cancel_reply_;
};
}
