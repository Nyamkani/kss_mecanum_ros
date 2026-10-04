#include "mecanum_gateway/mode_manager.hpp"
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <thread>
#include <sys/stat.h>
#include <unistd.h>

using mecanum_gateway::ModeManager;
using mecanum_gateway::protocol::Command;
void require(bool condition, const char * message) {
  if (!condition) throw std::runtime_error(message);
}
Command command(const std::string & name, const std::string & map = "room") {
  Command value; value.name = name; value.request_id = 1; value.map_name = map; return value;
}
ModeManager::Completion await(ModeManager & manager, const std::string & name) {
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(9);
  while (std::chrono::steady_clock::now() < deadline) {
    for (auto & reply : manager.Update()) if (reply.command.name == name) return reply;
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  throw std::runtime_error("completion timeout");
}
int main() {
  char pattern[] = "/tmp/mecanum-mode-test-XXXXXX";
  const auto * dir = mkdtemp(pattern);
  if (!dir) return 1;
  const std::filesystem::path root(dir);
  const std::string old_path = std::getenv("PATH") ? std::getenv("PATH") : "";
  try {
    // A real OS process group with a stubborn descendant exercises escalation/reaping.
    std::ofstream(root / "ros2") << R"(#!/usr/bin/python3
import os, signal, subprocess, sys, time
from pathlib import Path
root=Path(os.environ['MODE_TEST_ROOT'])
(root/'argv').write_text('\n'.join(sys.argv[1:]))
signal.signal(signal.SIGINT, signal.SIG_IGN)
signal.signal(signal.SIGTERM, signal.SIG_IGN)
child=subprocess.Popen(['/usr/bin/python3','-c','import time; time.sleep(60)'])
(root/('save_pid' if sys.argv[1]=='run' else 'mode_pid')).write_text(str(os.getpid()))
(root/('save_child' if sys.argv[1]=='run' else 'mode_child')).write_text(str(child.pid))
if sys.argv[1]=='run': print('test saver stderr', file=sys.stderr, flush=True)
while True: time.sleep(.05)
)";
    chmod((root / "ros2").c_str(), 0700);
    setenv("PATH", (root.string() + ":" + old_path).c_str(), 1);
    setenv("MODE_TEST_ROOT", root.c_str(), 1);
    {
      ModeManager manager((root / "maps").string(), 0.3);
      require(manager.GetMode() == "BASE", "initial mode");
      require(!manager.Handle(1, command("start_navigation"))->success, "missing map accepted");
      require(manager.Handle(1, command("start_mapping"))->success, "mapping spawn failed");
      require(!manager.Handle(1, command("start_mapping"))->success, "duplicate mapping accepted");
      require(!manager.Handle(1, command("start_navigation"))->success, "duplicate navigation accepted");
      require(!manager.Handle(1, command("save_map", "../escape"))->success, "traversal accepted");
      std::this_thread::sleep_for(std::chrono::milliseconds(150));
      require(!manager.Handle(7, command("save_map")), "save not asynchronous");
      require(!manager.Handle(8, command("save_map"))->success, "duplicate save accepted");
      auto saved = await(manager, "save_map");
      require(!saved.success && saved.session == 7, "save timeout result/session");
      require(saved.message.find("timed out") != std::string::npos, "timeout missing");
      require(saved.message.find("test saver stderr") != std::string::npos, "stderr missing");
      int pid; std::ifstream(root / "save_pid") >> pid;
      require(!std::filesystem::exists("/proc/" + std::to_string(pid)), "save process survived");
      std::ifstream(root / "save_child") >> pid;
      require(!std::filesystem::exists("/proc/" + std::to_string(pid)), "save descendant survived");
      require(!manager.Handle(1, command("stop_mode")), "stop not asynchronous");
      require(await(manager, "stop_mode").success, "stop failed");
      require(manager.GetMode() == "BASE" && manager.GetError().empty(), "normal stop state/error");
      std::ifstream(root / "mode_child") >> pid;
      require(!std::filesystem::exists("/proc/" + std::to_string(pid)), "mode descendant survived");
    }
    std::cout << "PASS timeout, async stop, SIGKILL escalation, descendant reaping, validation\n";
  } catch (const std::exception & error) {
    std::cerr << error.what() << '\n';
    setenv("PATH", old_path.c_str(), 1);
    std::filesystem::remove_all(root);
    return 1;
  }
  setenv("PATH", old_path.c_str(), 1);
  std::filesystem::remove_all(root);
  return 0;
}
