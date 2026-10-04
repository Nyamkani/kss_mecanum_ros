#pragma once

#include <atomic>
#include <cstdint>
#include <deque>
#include <mutex>
#include <string>
#include <thread>

namespace mecanum_gateway {
class TcpServer {
public:
  struct CommandFrame { std::uint64_t session; std::string frame; };
  TcpServer(const std::string & address, int port);
  ~TcpServer();
  TcpServer(const TcpServer &) = delete;
  TcpServer & operator=(const TcpServer &) = delete;
  void start();
  void stop();
  bool take_command(CommandFrame & command);
  void send_result(std::uint64_t session, std::string packet);
  void publish_state(std::string packet);
  void publish_map(std::string packet);
  std::uint64_t active_session() const { return active_session_.load(); }
  std::uint64_t disconnect_count() const { return disconnect_count_.load(); }

private:
  static constexpr std::size_t kQueueLimit = 128;
  static constexpr std::size_t kLineLimit = 16384;
  static constexpr std::size_t kPendingLimit = 262144;
  void run();
  int listener_ = -1;
  std::atomic<bool> stopping_{false};
  std::thread worker_;
  std::mutex mutex_;
  std::deque<CommandFrame> commands_;
  std::deque<CommandFrame> results_;
  std::string state_, map_;
  std::uint64_t map_version_ = 0;
  std::atomic<std::uint64_t> active_session_{0}, disconnect_count_{0};
  std::uint64_t state_version_ = 0;
};
}  // namespace mecanum_gateway
