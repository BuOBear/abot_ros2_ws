#include "abot_hardware/protocol.hpp"

#include <gtest/gtest.h>

#include <array>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>

using namespace abot_hardware::protocol;

namespace {
void write16(std::vector<uint8_t> &data, int16_t value) {
  const auto bits = static_cast<uint16_t>(value);
  data.push_back(static_cast<uint8_t>(bits));
  data.push_back(static_cast<uint8_t>(bits >> 8));
}
void write32(std::vector<uint8_t> &data, int32_t value) {
  const auto bits = static_cast<uint32_t>(value);
  for (int i = 0; i < 4; ++i) data.push_back(static_cast<uint8_t>(bits >> (8 * i)));
}
void write_float(std::vector<uint8_t> &data, float value) {
  uint32_t bits;
  std::memcpy(&bits, &value, sizeof(bits));
  write32(data, static_cast<int32_t>(bits));
}
}  // namespace

TEST(Protocol, VelocityLittleEndianAndRange) {
  const auto velocity = encode_velocity({1.23, -0.45, 2.5});
  ASSERT_TRUE(velocity);
  EXPECT_EQ(*velocity, (std::array<uint8_t, 6>{0x7b, 0x00, 0xd3, 0xff, 0xfa, 0x00}));
  const auto frame = encode(Id::set_velocity, {velocity->begin(), velocity->end()});
  EXPECT_EQ(frame, (std::vector<uint8_t>{0x5a, 4, 6, 0x7b, 0, 0xd3, 0xff, 0xfa, 0, 0xab}));
  EXPECT_FALSE(encode_velocity({std::numeric_limits<double>::quiet_NaN(), 0, 0}));
  EXPECT_FALSE(encode_velocity({328.0, 0, 0}));
}

TEST(Protocol, OdomPreservesLateralVelocity) {
  std::vector<uint8_t> body;
  write16(body, 120); write16(body, -34); write16(body, 150);
  write32(body, 12345); write32(body, -6789); write16(body, -314);
  ASSERT_EQ(body.size(), 16u);
  Parser parser;
  std::optional<Frame> parsed;
  for (uint8_t byte : encode(Id::get_odom, body)) parsed = parser.feed(byte);
  ASSERT_TRUE(parsed);
  const auto odom = decode_odom(*parsed);
  ASSERT_TRUE(odom);
  EXPECT_NEAR(odom->velocity.x_mps, 1.2, 1e-9);
  EXPECT_NEAR(odom->velocity.y_mps, -0.34, 1e-9);
  EXPECT_NEAR(odom->velocity.yaw_radps, 1.5, 1e-9);
  EXPECT_NEAR(odom->x_m, 123.45, 1e-9);
  EXPECT_NEAR(odom->y_m, -67.89, 1e-9);
  EXPECT_NEAR(odom->yaw_rad, -3.14, 1e-9);
}

TEST(Protocol, ImuFieldsAndNonfiniteRejection) {
  std::vector<uint8_t> body;
  for (float value : {1.f, -2.f, 9.81f, 0.1f, -0.2f, 0.3f, 100.f, -200.f, 300.f}) {
    write_float(body, value);
  }
  const auto imu = decode_imu({Id::get_imu, body});
  ASSERT_TRUE(imu);
  EXPECT_NEAR(imu->acceleration_mps2[2], 9.81, 1e-5);
  EXPECT_NEAR(imu->angular_velocity_radps[1], -0.2, 1e-6);
  EXPECT_NEAR(imu->magnetic_field_tesla[0], 1e-5, 1e-12);
  write_float(body, std::numeric_limits<float>::quiet_NaN());
  std::copy(body.end() - 4, body.end(), body.begin());
  body.resize(36);
  EXPECT_FALSE(decode_imu({Id::get_imu, body}));
}

TEST(Protocol, RejectsTruncatedChecksumLengthAndRecovers) {
  Parser parser;
  const auto good = encode(Id::get_odom, std::vector<uint8_t>(16, 0));
  for (size_t i = 0; i + 1 < good.size(); ++i) EXPECT_FALSE(parser.feed(good[i]));
  parser.reset();  // transaction deadline discards a partial frame
  auto bad_checksum = good;
  bad_checksum.back() ^= 1;
  for (uint8_t byte : bad_checksum) EXPECT_FALSE(parser.feed(byte));
  EXPECT_EQ(parser.rejected(), 1u);
  for (uint8_t byte : {0x5a, 0x05, 0xff}) EXPECT_FALSE(parser.feed(byte));
  for (uint8_t byte : {0x5a, 0x05, 0x0f}) EXPECT_FALSE(parser.feed(byte));
  EXPECT_EQ(parser.rejected(), 3u);
  std::optional<Frame> parsed;
  for (uint8_t byte : good) parsed = parser.feed(byte);
  ASSERT_TRUE(parsed);
  EXPECT_EQ(parsed->payload.size(), 16u);
}
