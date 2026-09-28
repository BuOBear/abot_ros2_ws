#include "abot_accessory/serial_device.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cerrno>
#include <fcntl.h>
#include <poll.h>
#include <pty.h>
#include <string>
#include <sys/file.h>
#include <termios.h>
#include <unistd.h>

namespace {

bool read_frame(int fd, abot_accessory::protocol::Frame &frame) {
  std::size_t offset = 0;
  while (offset < frame.size()) {
    pollfd descriptor{fd, POLLIN, 0};
    const int ready = ::poll(&descriptor, 1, 500);
    if (ready <= 0 || (descriptor.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
      return false;
    }
    const auto count = ::read(fd, frame.data() + offset, frame.size() - offset);
    if (count <= 0) {
      return false;
    }
    offset += static_cast<std::size_t>(count);
  }
  return true;
}

}  // namespace

TEST(AccessorySerial, SendsFramesAt9600AndHoldsExclusiveLock) {
  int master = -1;
  int slave = -1;
  char path[128]{};
  ASSERT_EQ(::openpty(&master, &slave, path, nullptr, nullptr), 0);
  ::close(slave);

  abot_accessory::SerialDevice device;
  std::string error;
  ASSERT_TRUE(device.open_device(path, 9600, error)) << error;
  EXPECT_TRUE(device.is_open());
  abot_accessory::SerialDevice second_owner;
  EXPECT_FALSE(second_owner.open_device(path, 9600, error));
  EXPECT_NE(error.find("lock"), std::string::npos);

  termios settings{};
  ASSERT_EQ(::tcgetattr(master, &settings), 0);
  EXPECT_EQ(settings.c_cflag & CSIZE, static_cast<tcflag_t>(CS8));
  EXPECT_EQ(settings.c_cflag & (PARENB | CSTOPB), 0u);
  EXPECT_EQ(::cfgetospeed(&settings), static_cast<speed_t>(B9600));

  abot_accessory::protocol::Frame actual{};
  ASSERT_TRUE(device.write_frame(abot_accessory::protocol::kShootFrame,
                                 std::chrono::milliseconds(250), error)) << error;
  ASSERT_TRUE(read_frame(master, actual));
  EXPECT_EQ(actual, abot_accessory::protocol::kShootFrame);

  ASSERT_TRUE(device.write_frame(abot_accessory::protocol::kStopFrame,
                                 std::chrono::milliseconds(250), error)) << error;
  ASSERT_TRUE(read_frame(master, actual));
  EXPECT_EQ(actual, abot_accessory::protocol::kStopFrame);
  device.close_device();
  EXPECT_FALSE(device.is_open());
  EXPECT_TRUE(second_owner.open_device(path, 9600, error)) << error;
  second_owner.close_device();
  ::close(master);
}

TEST(AccessorySerial, RejectsInvalidParametersAndReportsDisconnect) {
  abot_accessory::SerialDevice device;
  std::string error;
  EXPECT_FALSE(device.open_device("relative", 9600, error));
  EXPECT_NE(error.find("absolute"), std::string::npos);
  EXPECT_FALSE(device.open_device("/dev/null", 115200, error));
  EXPECT_NE(error.find("9600"), std::string::npos);

  int master = -1;
  int slave = -1;
  char path[128]{};
  ASSERT_EQ(::openpty(&master, &slave, path, nullptr, nullptr), 0);
  ::close(slave);
  ASSERT_TRUE(device.open_device(path, 9600, error)) << error;
  ::close(master);
  EXPECT_FALSE(device.write_frame(abot_accessory::protocol::kShootFrame,
                                  std::chrono::milliseconds(100), error));
  EXPECT_FALSE(error.empty());
  EXPECT_FALSE(device.is_open());
}
