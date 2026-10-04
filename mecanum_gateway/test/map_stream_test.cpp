#include "mecanum_gateway/map_stream.hpp"
#include <condition_variable>
#include <iostream>
#include <stdexcept>
#include <nlohmann/json.hpp>
#include <zlib.h>
using namespace std::chrono_literals;
int main() {
  try {
    std::mutex mutex;
    std::condition_variable changed;
    std::vector<nlohmann::json> packets;
    mecanum_gateway::MapStream stream([&](std::string packet) {
      if (packet.empty()) return;
      { std::lock_guard<std::mutex> lock(mutex); packets.push_back(nlohmann::json::parse(packet)); }
      changed.notify_one();
    });
    auto wait_count = [&](std::size_t count) {
      std::unique_lock<std::mutex> lock(mutex);
      if (!changed.wait_for(lock, 3s, [&] {return packets.size() >= count;}))
        throw std::runtime_error("map worker timeout");
    };
    auto map = std::make_shared<nav_msgs::msg::OccupancyGrid>();
    map->header.frame_id = "map";
    map->info.width = 3; map->info.height = 2; map->info.resolution = 0.05;
    map->info.origin.orientation.w = 1;
    map->data = {-1, 0, 100, 25, 50, 75};
    stream.SetEnabled(true); stream.Update(map); wait_count(1);
    nlohmann::json first;
    { std::lock_guard<std::mutex> lock(mutex); first = packets.front(); }
    const auto text = first["data"].get<std::string>();
    const std::string alphabet = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    std::vector<unsigned char> compressed;
    unsigned accumulator = 0; int bits = 0;
    for (char c : text) {
      if (c == '=') break;
      accumulator = (accumulator << 6) | alphabet.find(c); bits += 6;
      if (bits >= 8) { bits -= 8; compressed.push_back((accumulator >> bits) & 255); }
    }
    unsigned char raw[6]; uLongf size = sizeof(raw);
    if (uncompress(raw, &size, compressed.data(), compressed.size()) != Z_OK || size != 6 ||
        raw[0] != 255 || raw[1] != 0 || raw[2] != 100 || first["width"] != 3)
      throw std::runtime_error("map bytes/metadata mismatch");
    auto duplicate = std::make_shared<nav_msgs::msg::OccupancyGrid>(*map);
    duplicate->header.stamp.sec = 123;
    stream.Update(duplicate);
    std::this_thread::sleep_for(1200ms);
    { std::lock_guard<std::mutex> lock(mutex); if (packets.size() != 1) throw std::runtime_error("duplicate map sent"); }
    auto update = std::make_shared<nav_msgs::msg::OccupancyGrid>(*map);
    update->data[0] = 0;
    stream.Update(update); wait_count(2);
    { std::lock_guard<std::mutex> lock(mutex); if (packets.back()["sequence"] <= first["sequence"]) throw std::runtime_error("sequence did not increase"); }
    stream.SetEnabled(false); stream.Update(map);
    std::this_thread::sleep_for(1100ms);
    { std::lock_guard<std::mutex> lock(mutex); if (packets.size() != 2) throw std::runtime_error("map sent in BASE"); }
    stream.Stop();
    std::cout << "PASS map compression, metadata, duplicate suppression, sequence and BASE gating\n";
  } catch (const std::exception & e) { std::cerr << e.what() << '\n'; return 1; }
}
