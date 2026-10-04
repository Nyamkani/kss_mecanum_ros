#pragma once
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>
#include <vector>
#include <sys/types.h>
#include "mecanum_gateway/protocol.hpp"

namespace mecanum_gateway {
// Called only by the Gateway executor. No blocking waits or ROS/socket operations.
class ModeManager {
public:
  struct Completion {
    std::uint64_t session;
    protocol::Command command;
    bool success;
    std::string message;
  };
  explicit ModeManager(const std::string & map_dir, double save_timeout = 30.0);
  ~ModeManager();
  ModeManager(const ModeManager &) = delete;
  ModeManager & operator=(const ModeManager &) = delete;
  std::optional<Completion> Handle(std::uint64_t session, const protocol::Command & command);
  std::vector<Completion> Update();
  const std::string & GetMode() const { return mode_; }
  bool IsStopping() const { return expected_stop_; }
  const std::string & GetError() const { return error_; }
private:
  using Clock = std::chrono::steady_clock;
  struct Process {
    pid_t pid = -1;
    bool exited = false;
    int status = 0;
    int stop_stage = 0;
    int stderr_fd = -1;
    std::string stderr_text;
    Clock::time_point deadline;
  };
  Process Spawn(const std::vector<std::string> & args, bool capture_stderr);
  void Stop(Process & process);
  bool Poll(Process & process);
  bool ValidName(const std::string & name) const;
  std::filesystem::path MapPrefix(const std::string & name) const;
  std::string ExitMessage(const Process & process) const;
  std::filesystem::path map_dir_;
  double save_timeout_;
  std::string mode_ = "BASE", error_;
  std::optional<Process> mode_process_, save_process_;
  std::optional<Completion> stop_request_, save_request_;
  bool expected_stop_ = false;
  std::string save_failure_;
  Clock::time_point save_deadline_;
};
}
