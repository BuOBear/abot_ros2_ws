#include "abot_hardware/protocol.hpp"

#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>

namespace abot_hardware::protocol {
namespace {

bool valid_response_length(uint8_t id, uint8_t length) {
  switch (static_cast<Id>(id)) {
    case Id::version: return length == 32;
    case Id::set_parameters: return length == 0 || length == 64;
    case Id::get_parameters: return length == 64;
    case Id::init_odom: return length == 0;
    case Id::set_velocity: return length == 0 || length == 6;
    case Id::get_odom: return length == 16;
    case Id::get_pid: return length == 32;
    case Id::get_imu: return length == 36;
  }
  return false;
}

int16_t read_i16(const std::vector<uint8_t> &data, size_t offset) {
  const uint16_t value = static_cast<uint16_t>(data[offset]) |
    (static_cast<uint16_t>(data[offset + 1]) << 8);
  return static_cast<int16_t>(value);
}

int32_t read_i32(const std::vector<uint8_t> &data, size_t offset) {
  const uint32_t value = static_cast<uint32_t>(data[offset]) |
    (static_cast<uint32_t>(data[offset + 1]) << 8) |
    (static_cast<uint32_t>(data[offset + 2]) << 16) |
    (static_cast<uint32_t>(data[offset + 3]) << 24);
  return static_cast<int32_t>(value);
}

float read_f32(const std::vector<uint8_t> &data, size_t offset) {
  static_assert(sizeof(float) == sizeof(uint32_t) && std::numeric_limits<float>::is_iec559,
    "Firmware IMU requires IEEE-754 binary32");
  const uint32_t bits = static_cast<uint32_t>(data[offset]) |
    (static_cast<uint32_t>(data[offset + 1]) << 8) |
    (static_cast<uint32_t>(data[offset + 2]) << 16) |
    (static_cast<uint32_t>(data[offset + 3]) << 24);
  float value;
  std::memcpy(&value, &bits, sizeof(value));
  return value;
}

void write_i16(std::array<uint8_t, 6> &data, size_t offset, int16_t value) {
  const auto bits = static_cast<uint16_t>(value);
  data[offset] = static_cast<uint8_t>(bits);
  data[offset + 1] = static_cast<uint8_t>(bits >> 8);
}

}  // namespace

std::vector<uint8_t> encode(Id id, const std::vector<uint8_t> &payload) {
  if (static_cast<uint8_t>(id) > static_cast<uint8_t>(Id::get_imu) ||
      payload.size() > kMaxPayload) {
    throw std::invalid_argument("invalid abot frame");
  }
  std::vector<uint8_t> bytes{kHeader, static_cast<uint8_t>(id),
    static_cast<uint8_t>(payload.size())};
  bytes.insert(bytes.end(), payload.begin(), payload.end());
  uint8_t sum = 0;
  for (uint8_t byte : bytes) sum = static_cast<uint8_t>(sum + byte);
  bytes.push_back(sum);
  return bytes;
}

std::optional<std::array<uint8_t, 6>> encode_velocity(const Velocity &velocity) {
  const double values[] = {velocity.x_mps, velocity.y_mps, velocity.yaw_radps};
  std::array<uint8_t, 6> data{};
  for (size_t i = 0; i < 3; ++i) {
    const double centi = std::trunc(values[i] * 100.0);  // matches ROS1 C++ short assignment
    if (!std::isfinite(centi) || centi < -32768.0 || centi > 32767.0) return std::nullopt;
    write_i16(data, i * 2, static_cast<int16_t>(centi));
  }
  return data;
}

std::optional<Odom> decode_odom(const Frame &frame) {
  if (frame.id != Id::get_odom || frame.payload.size() != 16) return std::nullopt;
  const auto &data = frame.payload;
  Odom odom;
  odom.velocity.x_mps = read_i16(data, 0) * 0.01;
  odom.velocity.y_mps = read_i16(data, 2) * 0.01;
  odom.velocity.yaw_radps = read_i16(data, 4) * 0.01;
  odom.x_m = read_i32(data, 6) * 0.01;
  odom.y_m = read_i32(data, 10) * 0.01;
  odom.yaw_rad = read_i16(data, 14) * 0.01;
  return odom;
}

std::optional<Imu> decode_imu(const Frame &frame) {
  if (frame.id != Id::get_imu || frame.payload.size() != 36) return std::nullopt;
  Imu imu;
  for (size_t i = 0; i < 9; ++i) {
    const double value = read_f32(frame.payload, i * 4);
    if (!std::isfinite(value)) return std::nullopt;
    if (i < 3) imu.acceleration_mps2[i] = value;
    else if (i < 6) imu.angular_velocity_radps[i - 3] = value;
    else imu.magnetic_field_tesla[i - 6] = value * 1e-7;  // legacy mG -> T
  }
  return imu;
}

void Parser::reset() {
  state_ = State::header;
  id_ = length_ = checksum_ = 0;
  payload_.clear();
}

void Parser::reject(uint8_t byte) {
  ++rejected_;
  reset();
  if (byte == kHeader) {
    checksum_ = byte;
    state_ = State::id;
  }
}

std::optional<Frame> Parser::feed(uint8_t byte) {
  switch (state_) {
    case State::header:
      if (byte == kHeader) { checksum_ = byte; state_ = State::id; }
      break;
    case State::id:
      if (byte > static_cast<uint8_t>(Id::get_imu)) { reject(byte); break; }
      id_ = byte;
      checksum_ = static_cast<uint8_t>(checksum_ + byte);
      state_ = State::length;
      break;
    case State::length:
      if (byte > kMaxPayload || !valid_response_length(id_, byte)) { reject(byte); break; }
      length_ = byte;
      checksum_ = static_cast<uint8_t>(checksum_ + byte);
      payload_.reserve(length_);
      state_ = length_ == 0 ? State::checksum : State::payload;
      break;
    case State::payload:
      payload_.push_back(byte);
      checksum_ = static_cast<uint8_t>(checksum_ + byte);
      if (payload_.size() == length_) state_ = State::checksum;
      break;
    case State::checksum:
      if (byte == checksum_) {
        Frame frame{static_cast<Id>(id_), std::move(payload_)};
        reset();
        return frame;
      }
      reject(byte);
      break;
  }
  return std::nullopt;
}

}  // namespace abot_hardware::protocol
