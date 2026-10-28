#pragma once
/// @file fc.hpp
/// @brief FcStatus: what the FcBridge reports about the flight controller.

#include <cstdint>

namespace branes::anatomy {

/// Flight mode as reported by the FC's mode manager (architecture §7).
enum class FlightMode : std::uint8_t {
    disarmed = 0,
    manual = 1,
    assisted = 2,
    autonomous = 3,
    hold = 4,
    land = 5,
    kill = 6,
};

struct FcStatus {
    FlightMode mode{FlightMode::disarmed};
    bool armed{false};
    bool rc_ok{false};    ///< RC link healthy
    bool link_ok{false};  ///< companion ↔ FC link healthy

    friend constexpr bool operator==(const FcStatus&, const FcStatus&) = default;
};

}  // namespace branes::anatomy
