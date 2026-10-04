#include "mecanum_gateway/mode_manager.hpp"
#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <pwd.h>
#include <signal.h>
#include <spawn.h>
#include <stdexcept>
#include <sys/prctl.h>
#include <sys/wait.h>
#include <thread>
#include <unistd.h>
extern char ** environ;

namespace mecanum_gateway {
ModeManager::ModeManager(const std::string & directory, double save_timeout)
: save_timeout_(save_timeout) {
  if (!std::isfinite(save_timeout) || save_timeout <= 0)
    throw std::invalid_argument("map_save_timeout must be finite and positive");
  std::string expanded = directory;
  if (expanded == "~" || expanded.rfind("~/", 0) == 0) {
    const char * home = std::getenv("HOME");
    if (!home) { const auto * user = getpwuid(getuid()); if (user) home = user->pw_dir; }
    if (!home) throw std::runtime_error("cannot resolve home directory");
    expanded = std::string(home) + expanded.substr(1);
  }
  if (expanded.empty()) throw std::invalid_argument("map_dir must not be empty");
  map_dir_ = std::filesystem::weakly_canonical(std::filesystem::absolute(expanded));
  // Adopt/reap descendants when ros2 launch dies before its children.
  if (prctl(PR_SET_CHILD_SUBREAPER, 1) < 0) throw std::runtime_error(std::strerror(errno));
}
ModeManager::~ModeManager() {
  expected_stop_ = true;
  if (mode_process_) Stop(*mode_process_);
  if (save_process_) Stop(*save_process_);
  while (mode_process_ || save_process_) {
    Update();
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
}
ModeManager::Process ModeManager::Spawn(const std::vector<std::string> & args, bool capture) {
  int pipefd[2] = {-1, -1};
  if (capture && pipe2(pipefd, O_CLOEXEC) < 0) throw std::runtime_error(std::strerror(errno));
  posix_spawnattr_t attr;
  posix_spawn_file_actions_t actions;
  posix_spawnattr_init(&attr);
  posix_spawn_file_actions_init(&actions);
  sigset_t empty, defaults;
  sigemptyset(&empty);
  sigemptyset(&defaults);
  sigaddset(&defaults, SIGINT); sigaddset(&defaults, SIGTERM); sigaddset(&defaults, SIGPIPE);
  posix_spawnattr_setsigmask(&attr, &empty);
  posix_spawnattr_setsigdefault(&attr, &defaults);
  posix_spawnattr_setpgroup(&attr, 0);
  posix_spawnattr_setflags(&attr, POSIX_SPAWN_SETPGROUP | POSIX_SPAWN_SETSIGMASK | POSIX_SPAWN_SETSIGDEF);
  posix_spawn_file_actions_addopen(&actions, STDIN_FILENO, "/dev/null", O_RDONLY, 0);
  if (capture) {
    posix_spawn_file_actions_adddup2(&actions, pipefd[1], STDERR_FILENO);
    posix_spawn_file_actions_addclose(&actions, pipefd[0]);
    posix_spawn_file_actions_addclose(&actions, pipefd[1]);
  }
  std::vector<char *> argv;
  for (const auto & arg : args) argv.push_back(const_cast<char *>(arg.c_str()));
  argv.push_back(nullptr);
  Process process;
  const int result = posix_spawnp(&process.pid, argv[0], &actions, &attr, argv.data(), environ);
  posix_spawn_file_actions_destroy(&actions);
  posix_spawnattr_destroy(&attr);
  if (capture) close(pipefd[1]);
  if (result != 0) {
    if (capture) close(pipefd[0]);
    throw std::runtime_error(std::strerror(result));
  }
  if (capture) {
    process.stderr_fd = pipefd[0];
    fcntl(pipefd[0], F_SETFL, O_NONBLOCK);
  }
  return process;
}
void ModeManager::Stop(Process & process) {
  if (process.stop_stage != 0) return;
  kill(-process.pid, SIGINT);
  process.stop_stage = 1;
  process.deadline = Clock::now() + std::chrono::seconds(3);
}
bool ModeManager::Poll(Process & process) {
  if (process.stderr_fd >= 0) {
    char buffer[4096];
    for (int i = 0; i < 4; ++i) {
      const auto count = read(process.stderr_fd, buffer, sizeof(buffer));
      if (count <= 0) break;
      process.stderr_text.append(buffer, static_cast<std::size_t>(count));
      if (process.stderr_text.size() > 8192) process.stderr_text.erase(0, process.stderr_text.size() - 8192);
    }
  }
  if (!process.exited) {
    const auto result = waitpid(process.pid, &process.status, WNOHANG);
    if (result == process.pid || (result < 0 && errno == ECHILD)) process.exited = true;
  }
  if (process.exited) {
    int status;
    while (waitpid(-process.pid, &status, WNOHANG) > 0) {}  // adopted descendants only
  }
  const bool alive = kill(-process.pid, 0) == 0 || errno == EPERM;
  if (process.exited && !alive) {
    if (process.stderr_fd >= 0) { close(process.stderr_fd); process.stderr_fd = -1; }
    return true;
  }
  if (process.stop_stage && Clock::now() >= process.deadline) {
    kill(-process.pid, process.stop_stage == 1 ? SIGTERM : SIGKILL);
    ++process.stop_stage;
    process.deadline = Clock::now() + std::chrono::seconds(2);
  }
  return false;
}
bool ModeManager::ValidName(const std::string & name) const {
  if (name.empty() || name == "." || name == ".." || name.size() > 200) return false;
  for (const unsigned char c : name) {
    if (!((c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') ||
          (c >= '0' && c <= '9') || c == '_' || c == '-' || c == '.')) return false;
  }
  return true;
}
std::filesystem::path ModeManager::MapPrefix(const std::string & name) const {
  if (!ValidName(name)) throw std::invalid_argument("invalid map_name");
  auto prefix = map_dir_ / name;
  for (const auto * extension : {".yaml", ".pgm", ".png"}) {
    if (std::filesystem::is_symlink(prefix.string() + extension))
      throw std::invalid_argument("map file must not be a symbolic link");
  }
  return prefix;
}
std::string ModeManager::ExitMessage(const Process & process) const {
  if (WIFEXITED(process.status)) return "exit code " + std::to_string(WEXITSTATUS(process.status));
  if (WIFSIGNALED(process.status)) return "signal " + std::to_string(WTERMSIG(process.status));
  return "unknown exit status";
}
std::optional<ModeManager::Completion> ModeManager::Handle(
  std::uint64_t session, const protocol::Command & command) {
  Completion reply{session, command, false, "unknown command"};
  try {
    if (command.name == "start_mapping" || command.name == "start_navigation") {
      if (mode_ != "BASE" || mode_process_ || save_process_) {
        reply.message = "another mode is already running";
        return reply;
      }
      const bool mapping = command.name == "start_mapping";
      std::vector<std::string> args{"ros2", "launch", "mecanum_bringup",
        mapping ? "mecanum.mapping.launch.py" : "mecanum.navigation.launch.py", "use_sim_time:=false"};
      if (!mapping) {
        if (!command.map_name.is_string()) throw std::invalid_argument("map_name must be a string");
        const auto map = MapPrefix(command.map_name.get<std::string>()).string() + ".yaml";
        if (!std::filesystem::is_regular_file(map)) throw std::invalid_argument("map not found");
        args.push_back("map:=" + map);
      }
      mode_process_ = Spawn(args, false);
      mode_ = mapping ? "MAPPING" : "NAVIGATION";
      error_.clear();
      expected_stop_ = false;
      reply.success = true;
      reply.message = mapping ? "mapping started" : "navigation started";
    } else if (command.name == "stop_mode") {
      if (stop_request_) { reply.message = "mode stop already in progress"; return reply; }
      if (!mode_process_ && !save_process_) { reply.success = true; reply.message = "already in BASE"; return reply; }
      expected_stop_ = true;
      if (mode_process_) Stop(*mode_process_);
      if (save_process_) { save_failure_ = "map save cancelled"; Stop(*save_process_); }
      stop_request_ = reply;
      return std::nullopt;
    } else if (command.name == "save_map") {
      if (mode_ != "MAPPING" || expected_stop_ || !mode_process_ || mode_process_->stop_stage) {
        reply.message = "save_map requires MAPPING mode"; return reply;
      }
      if (save_process_) { reply.message = "map save already in progress"; return reply; }
      if (!command.map_name.is_string()) throw std::invalid_argument("map_name must be a string");
      const auto prefix = MapPrefix(command.map_name.get<std::string>());
      std::filesystem::create_directories(map_dir_);
      save_process_ = Spawn({"ros2", "run", "nav2_map_server", "map_saver_cli", "-f", prefix.string()}, true);
      save_deadline_ = Clock::now() + std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(save_timeout_));
      save_failure_.clear();
      save_request_ = reply;
      return std::nullopt;
    }
  } catch (const std::exception & exception) { reply.message = exception.what(); }
  return reply;
}
std::vector<ModeManager::Completion> ModeManager::Update() {
  std::vector<Completion> completed;
  if (mode_process_) {
    const bool done = Poll(*mode_process_);
    if (mode_process_->exited && !expected_stop_) {
      if (error_.empty()) error_ = (mode_ == "MAPPING" ? "mapping" : "navigation") +
        std::string(" process exited: ") + ExitMessage(*mode_process_);
      if (!done) Stop(*mode_process_);
      if (save_process_) { save_failure_ = "mapping process exited during map save"; Stop(*save_process_); }
    }
    if (done) { mode_process_.reset(); mode_ = "BASE"; }
  }
  if (save_process_) {
    const bool done = Poll(*save_process_);
    if (!done && Clock::now() >= save_deadline_ && save_failure_.empty()) {
      save_failure_ = "map save timed out";
      Stop(*save_process_);
    }
    if (save_process_->exited && !done) Stop(*save_process_);
    if (done) {
      if (save_request_) {
        auto reply = *save_request_;
        reply.success = save_failure_.empty() && WIFEXITED(save_process_->status) && WEXITSTATUS(save_process_->status) == 0;
        reply.message = reply.success ? "map saved" : (save_failure_.empty() ? "map save failed: " + ExitMessage(*save_process_) : save_failure_);
        if (!reply.success && !save_process_->stderr_text.empty()) reply.message += ": " + save_process_->stderr_text;
        completed.push_back(std::move(reply));
      }
      save_request_.reset(); save_process_.reset();
    }
  }
  if (stop_request_ && !mode_process_ && !save_process_) {
    auto reply = *stop_request_;
    reply.success = true; reply.message = "mode stopped";
    completed.push_back(std::move(reply)); stop_request_.reset();
  }
  return completed;
}
}
