#include "abot_accessory/serial_device.hpp"

#include <cerrno>
#include <chrono>
#include <cstring>
#include <fcntl.h>
#include <poll.h>
#include <string>
#include <sys/file.h>
#include <termios.h>
#include <unistd.h>

namespace abot_accessory {
namespace {

std::string system_error(const std::string &operation, int error_number) {
  return operation + ": " + std::strerror(error_number);
}

}  // namespace

SerialDevice::~SerialDevice() { close_device(); }

bool SerialDevice::open_device(const std::string &path, int baud_rate, std::string &error) {
  close_device();
  if (path.empty() || path.front() != '/') {
    error = "port must be a non-empty absolute path";
    return false;
  }
  if (baud_rate != 9600) {
    error = "the shoot accessory protocol supports only 9600 baud";
    return false;
  }

  const int fd = ::open(path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
  if (fd < 0) {
    error = system_error("open " + path, errno);
    return false;
  }
  if (::flock(fd, LOCK_EX | LOCK_NB) != 0) {
    const int saved_errno = errno;
    ::close(fd);
    error = system_error("lock " + path, saved_errno);
    return false;
  }

  termios settings{};
  if (::tcgetattr(fd, &settings) != 0) {
    const int saved_errno = errno;
    ::flock(fd, LOCK_UN);
    ::close(fd);
    error = system_error("read serial settings for " + path, saved_errno);
    return false;
  }
  ::cfmakeraw(&settings);
  settings.c_cflag &= static_cast<tcflag_t>(~(CSIZE | PARENB | CSTOPB));
#ifdef CRTSCTS
  settings.c_cflag &= static_cast<tcflag_t>(~CRTSCTS);
#endif
  settings.c_cflag |= static_cast<tcflag_t>(CS8 | CLOCAL | CREAD);
  settings.c_cc[VMIN] = 0;
  settings.c_cc[VTIME] = 0;
  if (::cfsetispeed(&settings, B9600) != 0 || ::cfsetospeed(&settings, B9600) != 0 ||
      ::tcsetattr(fd, TCSANOW, &settings) != 0) {
    const int saved_errno = errno;
    ::flock(fd, LOCK_UN);
    ::close(fd);
    error = system_error("configure 9600 8N1 serial port " + path, saved_errno);
    return false;
  }

  fd_ = fd;
  error.clear();
  return true;
}

bool SerialDevice::write_frame(const protocol::Frame &frame,
                               std::chrono::milliseconds timeout,
                               std::string &error,
                               std::size_t *bytes_written) {
  error.clear();
  if (bytes_written != nullptr) {
    *bytes_written = 0;
  }
  if (fd_ < 0) {
    error = "serial port is not open";
    return false;
  }

  const auto deadline = std::chrono::steady_clock::now() + timeout;
  std::size_t offset = 0;
  while (offset < frame.size()) {
    const auto count = ::write(fd_, frame.data() + offset, frame.size() - offset);
    if (count > 0) {
      offset += static_cast<std::size_t>(count);
      continue;
    }
    if (count < 0 && errno == EINTR) {
      continue;
    }
    if (count < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
      const auto now = std::chrono::steady_clock::now();
      if (now >= deadline) {
        error = "serial write timed out";
        break;
      }
      const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now);
      pollfd descriptor{fd_, POLLOUT, 0};
      const int ready = ::poll(&descriptor, 1, static_cast<int>(remaining.count() + 1));
      if (ready < 0 && errno == EINTR) {
        continue;
      }
      if (ready < 0) {
        error = system_error("poll serial port", errno);
        break;
      }
      if (ready == 0) {
        error = "serial write timed out";
        break;
      }
      if ((descriptor.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
        error = "serial device disconnected during write";
        break;
      }
      continue;
    }

    const int saved_errno = count < 0 ? errno : EIO;
    error = system_error("write serial frame", saved_errno);
    break;
  }

  if (offset == frame.size() && ::tcdrain(fd_) != 0) {
    error = system_error("drain serial frame", errno);
  }
  if (bytes_written != nullptr) {
    *bytes_written = offset;
  }
  if (offset != frame.size() || !error.empty()) {
    const std::string failure = error.empty() ? "incomplete serial frame" : error;
    close_device();
    error = failure + " after " + std::to_string(offset) + "/" +
            std::to_string(frame.size()) + " bytes; serial port closed";
    return false;
  }
  return true;
}

void SerialDevice::close_device() noexcept {
  if (fd_ >= 0) {
    ::flock(fd_, LOCK_UN);
    ::close(fd_);
    fd_ = -1;
  }
}

bool SerialDevice::is_open() const noexcept { return fd_ >= 0; }

}  // namespace abot_accessory
