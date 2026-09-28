#pragma once

#include <array>
#include <cstdint>

namespace abot_accessory::protocol {

using Frame = std::array<std::uint8_t, 8>;

inline constexpr Frame kShootFrame{{0x55, 0x01, 0x12, 0x00, 0x00, 0x00, 0x01, 0x69}};
inline constexpr Frame kStopFrame{{0x55, 0x01, 0x11, 0x00, 0x00, 0x00, 0x01, 0x68}};

}  // namespace abot_accessory::protocol
