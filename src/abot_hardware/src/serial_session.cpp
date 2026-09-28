#include "abot_hardware/serial_session.hpp"

#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <poll.h>
#include <sys/file.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

namespace abot_hardware {
namespace {

std::optional<speed_t> baud_constant(int baud) {
  switch (baud) {
    case 115200: return B115200;
    case 230400: return B230400;
    case 460800: return B460800;
    case 500000: return B500000;
    case 576000: return B576000;
    case 921600: return B921600;
    case 1000000: return B1000000;
    default: return std::nullopt;
  }
}

int remaining_ms(std::chrono::steady_clock::time_point deadline) {
  const auto now = std::chrono::steady_clock::now();
  if (now >= deadline) return 0;
  return static_cast<int>(std::chrono::duration_cast<std::chrono::milliseconds>(
    deadline - now).count()) + 1;
}

}  // namespace

SerialSession::~SerialSession() { close_device(); }

bool SerialSession::open_device(const std::string &path, int baud_rate, std::string &error) {
  close_device();
  const auto speed = baud_constant(baud_rate);
  if (!speed) { error = "unsupported baud rate"; return false; }
  fd_ = ::open(path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
  if (fd_ < 0) { error = std::strerror(errno); return false; }
  if (flock(fd_, LOCK_EX | LOCK_NB) != 0) {
    error = "serial port already locked";
    close_device();
    return false;
  }
  if (ioctl(fd_, TIOCEXCL) != 0) {
    error = std::strerror(errno);
    close_device();
    return false;
  }
  exclusive_ = true;
  termios attributes{};
  if (tcgetattr(fd_, &attributes) != 0) {
    error = std::strerror(errno);
    close_device();
    return false;
  }
  cfmakeraw(&attributes);
  attributes.c_cflag |= CLOCAL | CREAD;
  attributes.c_cflag &= ~(CRTSCTS | CSTOPB);  // Protocol uses 8N1, including one stop bit.
  attributes.c_cc[VMIN] = 0;
  attributes.c_cc[VTIME] = 0;
  if (cfsetispeed(&attributes, *speed) != 0 || cfsetospeed(&attributes, *speed) != 0 ||
      tcsetattr(fd_, TCSANOW, &attributes) != 0 || tcflush(fd_, TCIOFLUSH) != 0) {
    error = std::strerror(errno);
    close_device();
    return false;
  }
  return true;
}

void SerialSession::close_device() {
  if (fd_ >= 0) {
    // TIOCEXCL is a property of the tty, not just this file descriptor. A PTY
    // master (or another descriptor) can keep it set after close, preventing
    // our own reconnect from opening the slave again.
    if (exclusive_) ioctl(fd_, TIOCNXCL);
    ::close(fd_);
    fd_ = -1;
    exclusive_ = false;
  }
}

bool SerialSession::write_all(const std::vector<uint8_t> &bytes,
  std::chrono::steady_clock::time_point deadline) {
  size_t offset = 0;
  while (offset < bytes.size()) {
    const int wait_ms = remaining_ms(deadline);
    if (!wait_ms) return false;
    pollfd pfd{fd_, POLLOUT, 0};
    const int ready = ::poll(&pfd, 1, wait_ms);
    if (ready < 0 && errno == EINTR) continue;
    if (ready <= 0 || (pfd.revents & (POLLERR | POLLHUP | POLLNVAL))) return false;
    const ssize_t count = ::write(fd_, bytes.data() + offset, bytes.size() - offset);
    if (count < 0 && (errno == EINTR || errno == EAGAIN)) continue;
    if (count <= 0) return false;
    offset += static_cast<size_t>(count);
  }
  return true;
}

bool SerialSession::send(protocol::Id id, const std::vector<uint8_t> &payload,
  std::chrono::milliseconds timeout) {
  if (!is_open()) return false;
  return write_all(protocol::encode(id, payload), std::chrono::steady_clock::now() + timeout);
}

std::optional<protocol::Frame> SerialSession::request(protocol::Id id,
  size_t expected_length, std::chrono::milliseconds timeout) {
  if (!is_open()) return std::nullopt;
  // Responses carry no sequence number. Discard any response left by the previous
  // timed-out transaction before sending the next request.
  if (tcflush(fd_, TCIFLUSH) != 0) return std::nullopt;
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  if (!write_all(protocol::encode(id), deadline)) return std::nullopt;
  protocol::Parser parser;
  while (true) {
    const int wait_ms = remaining_ms(deadline);
    if (!wait_ms) break;
    pollfd pfd{fd_, POLLIN, 0};
    const int ready = ::poll(&pfd, 1, wait_ms);
    if (ready < 0 && errno == EINTR) continue;
    if (ready <= 0 || (pfd.revents & (POLLERR | POLLHUP | POLLNVAL))) break;
    uint8_t buffer[128];
    const ssize_t count = ::read(fd_, buffer, sizeof(buffer));
    if (count < 0 && (errno == EINTR || errno == EAGAIN)) continue;
    if (count <= 0) break;
    for (ssize_t i = 0; i < count; ++i) {
      auto frame = parser.feed(buffer[i]);
      if (!frame) continue;
      if (frame->id == id && frame->payload.size() == expected_length) {
        rejected_frames_ += parser.rejected();
        return frame;
      }
      ++rejected_frames_;
    }
  }
  rejected_frames_ += parser.rejected();
  return std::nullopt;
}

}  // namespace abot_hardware
