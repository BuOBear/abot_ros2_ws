#include "abot_hardware/serial_session.hpp"

#include <gtest/gtest.h>

#include <array>
#include <algorithm>
#include <chrono>
#include <fcntl.h>
#include <poll.h>
#include <pty.h>
#include <thread>
#include <termios.h>
#include <unistd.h>

using namespace abot_hardware;
using namespace std::chrono_literals;

namespace {
bool read_bytes(int fd, std::vector<uint8_t> &output, size_t expected) {
  while (output.size() < expected) {
    pollfd pfd{fd, POLLIN, 0};
    if (poll(&pfd, 1, 500) <= 0) return false;
    uint8_t buffer[64];
    const auto count = read(fd, buffer, sizeof(buffer));
    if (count <= 0) return false;
    output.insert(output.end(), buffer, buffer + count);
  }
  return true;
}
}  // namespace

TEST(SerialPty, LocksDeviceChecksResponseAndTimesOut) {
  int master = -1, slave = -1;
  char path[128]{};
  ASSERT_EQ(openpty(&master, &slave, path, nullptr, nullptr), 0);
  termios inherited{};
  ASSERT_EQ(tcgetattr(slave, &inherited), 0);
  inherited.c_cflag |= CSTOPB;
  ASSERT_EQ(tcsetattr(slave, TCSANOW, &inherited), 0);
  close(slave);
  SerialSession session;
  std::string error;
  ASSERT_TRUE(session.open_device(path, 921600, error)) << error;
  termios configured{};
  ASSERT_EQ(tcgetattr(master, &configured), 0);
  EXPECT_EQ(configured.c_cflag & CSTOPB, 0u);
  EXPECT_EQ(configured.c_cflag & CSIZE, static_cast<tcflag_t>(CS8));
  EXPECT_EQ(configured.c_cflag & (PARENB | CRTSCTS), 0u);
  SerialSession second;
  EXPECT_FALSE(second.open_device(path, 921600, error));

  bool saw_request = false;
  std::thread responder([&]() {
    std::vector<uint8_t> bytes;
    saw_request = read_bytes(master, bytes, 4) &&
      std::equal(bytes.begin(), bytes.begin() + 4,
        std::array<uint8_t, 4>{0x5a, 0, 0, 0x5a}.begin());
    auto bad = protocol::encode(protocol::Id::version, std::vector<uint8_t>(32, 'a'));
    bad.back() ^= 1;
    write(master, bad.data(), bad.size());
    auto good = protocol::encode(protocol::Id::version, std::vector<uint8_t>(32, 'v'));
    write(master, good.data(), 7);
    std::this_thread::sleep_for(5ms);
    write(master, good.data() + 7, good.size() - 7);
  });
  const auto frame = session.request(protocol::Id::version, 32, 200ms);
  responder.join();
  EXPECT_TRUE(saw_request);
  ASSERT_TRUE(frame);
  EXPECT_EQ(frame->payload[0], 'v');
  EXPECT_GE(session.rejected_frames(), 1u);

  const auto start = std::chrono::steady_clock::now();
  EXPECT_FALSE(session.request(protocol::Id::get_odom, 16, 30ms));
  EXPECT_LT(std::chrono::steady_clock::now() - start, 200ms);
  session.close_device();
  EXPECT_TRUE(second.open_device(path, 921600, error)) << error;
  second.close_device();
  close(master);
}
