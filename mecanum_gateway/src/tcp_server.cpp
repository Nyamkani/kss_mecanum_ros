#include "mecanum_gateway/tcp_server.hpp"

#include <cerrno>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <utility>
#include <netdb.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

namespace mecanum_gateway {
TcpServer::TcpServer(const std::string & address, int port) {
  addrinfo hints{};
  hints.ai_family = AF_INET;
  hints.ai_socktype = SOCK_STREAM;
  hints.ai_flags = AI_PASSIVE;
  addrinfo * raw = nullptr;
  const auto service = std::to_string(port);
  const int status = getaddrinfo(address.empty() ? nullptr : address.c_str(),
    service.c_str(), &hints, &raw);
  if (status != 0) throw std::runtime_error(gai_strerror(status));
  std::unique_ptr<addrinfo, decltype(&freeaddrinfo)> addresses(raw, freeaddrinfo);
  listener_ = socket(AF_INET, SOCK_STREAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
  if (listener_ < 0) throw std::runtime_error(std::strerror(errno));
  const int yes = 1;
  if (setsockopt(listener_, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes)) < 0 ||
      bind(listener_, raw->ai_addr, raw->ai_addrlen) < 0 || listen(listener_, 4) < 0) {
    const std::string error = std::strerror(errno);
    close(listener_);
    listener_ = -1;
    throw std::runtime_error(error);
  }
}
TcpServer::~TcpServer() { stop(); }
void TcpServer::start() { worker_ = std::thread(&TcpServer::run, this); }
void TcpServer::stop() {
  stopping_ = true;
  // poll waits at most 50 ms; recv/send are nonblocking. Never hold mutex_ here.
  if (worker_.joinable()) worker_.join();
  if (listener_ >= 0) { close(listener_); listener_ = -1; }
}
bool TcpServer::take_command(CommandFrame & command) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (commands_.empty() || results_.size() >= kQueueLimit) return false;
  command = std::move(commands_.front());
  commands_.pop_front();
  return true;
}
void TcpServer::send_result(std::uint64_t session, std::string packet) {
  // One executor producer calls this after take_command; the worker only removes results.
  std::lock_guard<std::mutex> lock(mutex_);
  results_.push_back({session, std::move(packet)});
}
void TcpServer::publish_state(std::string packet) {
  std::lock_guard<std::mutex> lock(mutex_);
  state_ = std::move(packet);
  ++state_version_;
}
void TcpServer::publish_map(std::string packet) {
  std::lock_guard<std::mutex> lock(mutex_);
  map_ = std::move(packet);
  ++map_version_;
}
void TcpServer::run() {
  int client = -1;
  std::uint64_t session = 0, sent_version = 0;
  std::string incoming, outgoing;
  std::uint64_t sent_map_version = 0;
  std::size_t map_allowance = 0;
  auto disconnect = [&]() {
    if (client >= 0) { close(client); active_session_ = 0; ++disconnect_count_; }
    client = -1;
    incoming.clear();
    outgoing.clear();
  };
  while (!stopping_) {
    pollfd fds[2] = {{listener_, POLLIN, 0},
      {client, static_cast<short>(POLLIN | (outgoing.empty() ? 0 : POLLOUT)), 0}};
    const int result = poll(fds, 2, 50);
    if (result < 0) { if (errno == EINTR) continue; break; }
    if (fds[0].revents & POLLIN) {
      const int candidate = accept4(listener_, nullptr, nullptr, SOCK_NONBLOCK | SOCK_CLOEXEC);
      if (candidate >= 0) {
        if (client >= 0) close(candidate);
        else {
          client = candidate;
          ++session;
          active_session_ = session;
          sent_map_version = 0;
          map_allowance = 0;
          sent_version = 0;
          incoming.clear();
          outgoing.clear();
        }
      }
    }
    if (client >= 0 && fds[1].fd == client && (fds[1].revents & (POLLERR | POLLHUP | POLLNVAL))) {
      disconnect();
    }
    if (client >= 0 && fds[1].fd == client && (fds[1].revents & POLLIN)) {
      char buffer[4096];
      const auto count = recv(client, buffer, sizeof(buffer), 0);
      if (count == 0 || (count < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR)) {
        disconnect();
      } else if (count > 0) {
        incoming.append(buffer, static_cast<std::size_t>(count));
        std::size_t newline;
        while ((newline = incoming.find('\n')) != std::string::npos) {
          if (newline > kLineLimit) { disconnect(); break; }
          bool full;
          {
            std::lock_guard<std::mutex> lock(mutex_);
            full = commands_.size() >= kQueueLimit;
            if (!full) commands_.push_back({session, incoming.substr(0, newline)});
          }
          if (full) { disconnect(); break; }
          incoming.erase(0, newline + 1);
        }
        if (incoming.size() > kLineLimit) disconnect();
      }
    }
    const bool can_start_map = outgoing.empty();
    if (can_start_map) map_allowance = 0;
    // Move queued packets under the mutex; never hold it across socket I/O.
    {
      std::lock_guard<std::mutex> lock(mutex_);
      for (const auto & response : results_) {
        if (client >= 0 && response.session == session) outgoing += response.frame;
      }
      results_.clear();
      if (client >= 0 && sent_version != state_version_) {
        outgoing += state_;
        sent_version = state_version_;
      }
      // A map is one NDJSON frame: never interleave it or queue map history.
      if (client >= 0 && can_start_map && sent_map_version != map_version_) {
        outgoing += map_;
        map_allowance = map_.size();
        sent_map_version = map_version_;
      }
    }
    if (outgoing.size() > kPendingLimit + map_allowance) disconnect();
    if (client >= 0 && fds[1].fd == client && (fds[1].revents & POLLOUT) && !outgoing.empty()) {
      const auto count = send(client, outgoing.data(), outgoing.size(), MSG_NOSIGNAL);
      if (count > 0) outgoing.erase(0, static_cast<std::size_t>(count));
      else if (count == 0 || (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR)) disconnect();
    }
  }
  disconnect();
}
}  // namespace mecanum_gateway
