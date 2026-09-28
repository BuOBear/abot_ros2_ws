#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

namespace abot_hardware::protocol {

// ROS1 simple_dataframe.h/dataframe.h: 5a, ID, length, payload, sum modulo 256.
constexpr uint8_t kHeader = 0x5a;
constexpr size_t kMaxPayload = 64;

enum class Id : uint8_t {
  version = 0,
  set_parameters = 1,
  get_parameters = 2,
  init_odom = 3,
  set_velocity = 4,
  get_odom = 5,
  get_pid = 6,
  get_imu = 7,
};

struct Frame {
  Id id;
  std::vector<uint8_t> payload;
};

struct Velocity {
  double x_mps{};
  double y_mps{};
  double yaw_radps{};
};

struct Odom {
  double x_m{};
  double y_m{};
  double yaw_rad{};
  Velocity velocity;
};

struct Imu {
  std::array<double, 3> acceleration_mps2{};
  std::array<double, 3> angular_velocity_radps{};
  std::array<double, 3> magnetic_field_tesla{};
};

// Throws std::invalid_argument for an invalid ID or payload size.
std::vector<uint8_t> encode(Id id, const std::vector<uint8_t> &payload = {});
// Converts SI to the firmware's signed, little-endian centi-units; returns
// nullopt instead of truncating, wrapping, or accepting non-finite values.
std::optional<std::array<uint8_t, 6>> encode_velocity(const Velocity &velocity);
std::optional<Odom> decode_odom(const Frame &frame);
std::optional<Imu> decode_imu(const Frame &frame);

class Parser {
public:
  std::optional<Frame> feed(uint8_t byte);
  void reset();
  uint64_t rejected() const { return rejected_; }

private:
  enum class State { header, id, length, payload, checksum } state_{State::header};
  uint8_t id_{};
  uint8_t length_{};
  uint8_t checksum_{};
  std::vector<uint8_t> payload_;
  uint64_t rejected_{};
  void reject(uint8_t byte);
};

}  // namespace abot_hardware::protocol
