#pragma once

#include "abot_accessory/protocol.hpp"

#include <chrono>
#include <cstddef>
#include <string>

namespace abot_accessory {

class SerialDevice {
 public:
  SerialDevice() = default;
  ~SerialDevice();

  SerialDevice(const SerialDevice &) = delete;
  SerialDevice &operator=(const SerialDevice &) = delete;

  bool open_device(const std::string &path, int baud_rate, std::string &error);
  bool write_frame(const protocol::Frame &frame, std::chrono::milliseconds timeout,
                   std::string &error, std::size_t *bytes_written = nullptr);
  void close_device() noexcept;
  bool is_open() const noexcept;

 private:
  int fd_{-1};
};

}  // namespace abot_accessory
