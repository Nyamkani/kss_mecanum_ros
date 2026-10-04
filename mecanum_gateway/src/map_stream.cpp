#include "mecanum_gateway/map_stream.hpp"
#include "mecanum_gateway/protocol.hpp"
#include <chrono>
#include <cmath>
#include <zlib.h>

namespace mecanum_gateway {
namespace {
std::string Base64(const std::vector<unsigned char> & bytes) {
  constexpr char alphabet[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  std::string result;
  result.reserve((bytes.size() + 2) / 3 * 4);
  for (std::size_t i = 0; i < bytes.size(); i += 3) {
    const unsigned a = bytes[i], b = i + 1 < bytes.size() ? bytes[i+1] : 0,
      c = i + 2 < bytes.size() ? bytes[i+2] : 0;
    result += alphabet[a >> 2]; result += alphabet[((a & 3) << 4) | (b >> 4)];
    result += i + 1 < bytes.size() ? alphabet[((b & 15) << 2) | (c >> 6)] : '=';
    result += i + 2 < bytes.size() ? alphabet[c & 63] : '=';
  }
  return result;
}
bool Valid(const nav_msgs::msg::OccupancyGrid & map) {
  const auto & o = map.info.origin;
  const auto cells = static_cast<std::uint64_t>(map.info.width) * map.info.height;
  if (map.header.frame_id != "map" || !cells || cells > MapStream::kMaxCells || cells != map.data.size() ||
      !std::isfinite(map.info.resolution) || map.info.resolution <= 0 ||
      !std::isfinite(o.position.x) || !std::isfinite(o.position.y) ||
      !std::isfinite(o.orientation.z) || !std::isfinite(o.orientation.w) ||
      std::abs(o.orientation.x) > 1e-6 || std::abs(o.orientation.y) > 1e-6 ||
      !std::isfinite(o.orientation.x) || !std::isfinite(o.orientation.y) ||
      !std::isfinite(std::hypot(o.orientation.z, o.orientation.w)) ||
      std::hypot(o.orientation.z, o.orientation.w) < 1e-12) return false;
  for (const auto value : map.data) if (value < -1 || value > 100) return false;
  return true;
}
}
MapStream::MapStream(std::function<void(std::string)> publish) : publish_(std::move(publish)), worker_(&MapStream::Run, this) {}
MapStream::~MapStream() { Stop(); }
void MapStream::Stop() {
  { std::lock_guard<std::mutex> lock(mutex_); stopping_ = true; }
  changed_.notify_one();
  if (worker_.joinable()) worker_.join();
}
void MapStream::Update(nav_msgs::msg::OccupancyGrid::ConstSharedPtr map) {
  { std::lock_guard<std::mutex> lock(mutex_); latest_ = std::move(map); ++generation_; }
  changed_.notify_one();
}
void MapStream::SetEnabled(bool enabled) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (enabled_ == enabled) return;
  enabled_ = enabled; ++epoch_; ++generation_;
  if (!enabled) { latest_.reset(); publish_(""); }
  changed_.notify_one();
}
void MapStream::Run() {
  using Clock = std::chrono::steady_clock;
  std::uint64_t handled = 0, sequence = 0, previous_epoch = 0;
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr previous;
  auto next_send = Clock::now();
  std::unique_lock<std::mutex> lock(mutex_);
  while (!stopping_) {
    changed_.wait(lock, [&] { return stopping_ || (enabled_ && latest_ && generation_ != handled); });
    if (stopping_) break;
    if (Clock::now() < next_send) {
      changed_.wait_until(lock, next_send, [&] { return stopping_ || !enabled_; });
      continue;
    }
    auto map = latest_;
    const auto generation = generation_, epoch = epoch_;
    lock.unlock();
    std::string packet;
    if (Valid(*map) && !(previous && previous_epoch == epoch &&
        previous->info.resolution == map->info.resolution && previous->info.width == map->info.width &&
        previous->info.height == map->info.height && previous->info.origin == map->info.origin && previous->data == map->data)) {
      uLongf length = compressBound(map->data.size());
      std::vector<unsigned char> compressed(length);
      if (compress2(compressed.data(), &length,
          reinterpret_cast<const Bytef *>(map->data.data()), map->data.size(), Z_BEST_SPEED) == Z_OK) {
        compressed.resize(length);
        packet = protocol::map_packet(sequence + 1, map->info.resolution, map->info.width, map->info.height,
          map->info.origin.position.x, map->info.origin.position.y,
          map->info.origin.orientation.z, map->info.origin.orientation.w, Base64(compressed));
      }
    }
    lock.lock();
    handled = generation;
    if (!packet.empty() && enabled_ && epoch == epoch_ && !stopping_) {
      publish_(std::move(packet)); ++sequence;
      previous = std::move(map); previous_epoch = epoch;
      next_send = Clock::now() + std::chrono::seconds(1);
    }
  }
}
}
