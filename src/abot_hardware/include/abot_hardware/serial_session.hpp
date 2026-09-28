#pragma once

#include "abot_hardware/protocol.hpp"

#include <chrono>
#include <optional>
#include <string>
#include <vector>

namespace abot_hardware {

class SerialSession {
public:
  SerialSession() = default;
  ~SerialSession();
  SerialSession(const SerialSession &) = delete;
  SerialSession &operator=(const SerialSession &) = delete;

  bool open_device(const std::string &path, int baud_rate, std::string &error);
  void close_device();
  bool is_open() const { return fd_ >= 0; }
  bool send(protocol::Id id, const std::vector<uint8_t> &payload,
    std::chrono::milliseconds timeout);
  std::optional<protocol::Frame> request(protocol::Id id, size_t expected_length,
    std::chrono::milliseconds timeout);
  uint64_t rejected_frames() const { return rejected_frames_; }

private:
  int fd_{-1};
  bool exclusive_{false};
  uint64_t rejected_frames_{};
  bool write_all(const std::vector<uint8_t> &bytes,
    std::chrono::steady_clock::time_point deadline);
};

}  // namespace abot_hardware
