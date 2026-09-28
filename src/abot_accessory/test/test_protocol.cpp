#include "abot_accessory/protocol.hpp"

#include <gtest/gtest.h>

TEST(AccessoryProtocol, LegacyShootAndStopFramesAreExactEightBytes) {
  using abot_accessory::protocol::kShootFrame;
  using abot_accessory::protocol::kStopFrame;
  EXPECT_EQ(kShootFrame, (std::array<std::uint8_t, 8>{0x55, 0x01, 0x12, 0, 0, 0, 1, 0x69}));
  EXPECT_EQ(kStopFrame, (std::array<std::uint8_t, 8>{0x55, 0x01, 0x11, 0, 0, 0, 1, 0x68}));
}
