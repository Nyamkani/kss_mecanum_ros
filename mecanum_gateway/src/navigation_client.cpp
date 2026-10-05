#include "mecanum_gateway/navigation_client.hpp"
#include <cmath>

namespace mecanum_gateway {
NavigationClient::NavigationClient(rclcpp::Node * node, Reply reply)
: node_(node), reply_(std::move(reply)),
  client_(rclcpp_action::create_client<Action>(node, "/navigate_to_pose")) {}

void NavigationClient::Reset() {
  if (pending_goal_reply_) {
    auto reply = std::move(pending_goal_reply_);
    pending_goal_reply_ = nullptr;
    reply(false, "navigation mode ended");
  }
  if (pending_cancel_reply_) {
    auto reply = std::move(pending_cancel_reply_);
    pending_cancel_reply_ = nullptr;
    reply(false, "navigation mode ended");
  }
  ++generation_;  // Late responses/results from the previous mode cannot overwrite IDLE.
  if (handle_) {
    try { client_->async_cancel_goal(handle_); } catch (const std::exception &) {}
  }
  handle_.reset();
  state_ = "IDLE";
}

void NavigationClient::Command(std::uint64_t session, const protocol::Command & command, bool allowed) {
  auto reply = [this, session, command](bool success, const std::string & message) {
    reply_(session, protocol::command_result(command, success, message));
  };
  if (!allowed) { reply(false, "navigation goal is only allowed in NAVIGATION mode"); return; }
  if (command.name == "cancel_navigation_goal") {
    if (!handle_) { reply(false, "no active navigation goal"); return; }
    if (pending_cancel_reply_) { reply(false, "navigation cancel already pending"); return; }
    pending_cancel_reply_ = reply;
    const auto generation = generation_;
    cancel_generation_ = generation;
    try {
      client_->async_cancel_goal(handle_, [this, generation, reply](auto response) {
        if (!pending_cancel_reply_ || generation != cancel_generation_) return;
        pending_cancel_reply_ = nullptr;
        const bool accepted = response->return_code == 0 && !response->goals_canceling.empty();
        reply(accepted, accepted ? "navigation cancel accepted" : "navigation cancel rejected");
        // CANCELED is set only by the final action result.
      });
    } catch (const std::exception &) {
      pending_cancel_reply_ = nullptr;
      reply(false, "navigation cancel failed");
    }
    return;
  }
  if (state_ == "PENDING" || state_ == "EXECUTING") {
    reply(false, "navigation goal already active"); return;
  }
  for (const auto * value : {&command.x, &command.y, &command.yaw}) {
    if (!value->is_number() || !std::isfinite(value->get<double>())) {
      reply(false, "x, y and yaw must be finite numbers"); return;
    }
  }
  if (!client_->action_server_is_ready()) {
    state_ = "REJECTED"; reply(false, "navigation action server unavailable"); return;
  }
  Action::Goal goal;
  goal.pose.header.frame_id = "map";
  goal.pose.header.stamp = node_->now();
  goal.pose.pose.position.x = command.x.get<double>();
  goal.pose.pose.position.y = command.y.get<double>();
  const double yaw = std::remainder(command.yaw.get<double>(), 2.0 * std::acos(-1.0));
  goal.pose.pose.orientation.z = std::sin(yaw / 2.0);
  goal.pose.pose.orientation.w = std::cos(yaw / 2.0);
  state_ = "PENDING";
  pending_goal_reply_ = reply;
  const auto generation = ++generation_;
  rclcpp_action::Client<Action>::SendGoalOptions options;
  options.goal_response_callback = [this, generation, reply](Handle::SharedPtr handle) {
    if (generation != generation_) {
      if (handle) {
        try { client_->async_cancel_goal(handle); } catch (const std::exception &) {}
      }
      return;
    }
    pending_goal_reply_ = nullptr;
    handle_ = handle;
    state_ = handle ? "EXECUTING" : "REJECTED";
    reply(bool(handle), handle ? "navigation goal accepted" : "navigation goal rejected");
  };
  options.result_callback = [this, generation](const Handle::WrappedResult & result) {
    if (generation != generation_) return;
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED: state_ = "SUCCEEDED"; break;
      case rclcpp_action::ResultCode::CANCELED: state_ = "CANCELED"; break;
      default: state_ = "ABORTED"; break;
    }
    handle_.reset();
  };
  try { client_->async_send_goal(goal, options); }
  catch (const std::exception &) {
    pending_goal_reply_ = nullptr;
    state_ = "REJECTED"; reply(false, "navigation goal send failed");
  }
}
}
