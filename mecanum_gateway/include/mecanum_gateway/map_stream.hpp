#pragma once
#include <condition_variable>
#include <functional>
#include <mutex>
#include <thread>
#include <nav_msgs/msg/occupancy_grid.hpp>

namespace mecanum_gateway {
// Snapshot-only callback; compression and equality checks run outside the ROS executor.
class MapStream {
public:
  static constexpr std::size_t kMaxCells = 1024 * 1024;
  explicit MapStream(std::function<void(std::string)> publish);
  ~MapStream();
  void Update(nav_msgs::msg::OccupancyGrid::ConstSharedPtr map);
  void SetEnabled(bool enabled);
  void Stop();
private:
  void Run();
  std::function<void(std::string)> publish_;
  std::mutex mutex_;
  std::condition_variable changed_;
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr latest_;
  bool enabled_ = false, stopping_ = false;
  std::uint64_t generation_ = 0, epoch_ = 0;
  std::thread worker_;
};
}
