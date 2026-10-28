#pragma once
/// @file time.hpp
/// @brief Timestamp: the one time type of the data contracts; TimeBase: its
///        intervals in the stack's scalar type.
///
/// All contract timestamps are nanoseconds on the companion's monotonic clock.
/// FC-side samples (IMU) are mapped to that clock by the FcBridge before they
/// enter the stack. Integer nanoseconds keep replays bit-exact: no rounding
/// differences between platforms when times are compared or subtracted.

#include <branes/anatomy/result.hpp>

#include <cmath>
#include <compare>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <optional>
#include <string>

namespace branes::anatomy {

struct Timestamp {
    std::int64_t ns{0};

    friend constexpr auto operator<=>(const Timestamp&, const Timestamp&) = default;
};

/// Period in nanoseconds of a rate in hertz, rounded to the nearest ns.
/// Pre: hz > 0.
[[nodiscard]] constexpr std::int64_t period_ns(double hz) noexcept {
    return static_cast<std::int64_t>(1e9 / hz + 0.5);
}

/// Rates the framework accepts for sensors and loops. The bounds keep every
/// period a positive, representable number of nanoseconds (≥ 1 µs, ≤ 1000 s).
inline constexpr double kMinRateHz = 1e-3;
inline constexpr double kMaxRateHz = 1e6;

/// Longest duration (SITL scenario) the framework accepts: ~11.6 days, far inside int64 nanoseconds.
inline constexpr double kMaxDurationS = 1e6;

/// period_ns(hz) if hz is finite and within [kMinRateHz, kMaxRateHz], else nullopt.
[[nodiscard]] inline std::optional<std::int64_t> checked_period_ns(double hz) noexcept {
    if (!std::isfinite(hz) || hz < kMinRateHz || hz > kMaxRateHz) {
        return std::nullopt;
    }
    return period_ns(hz);
}

/// Seconds as a double (for logging and arithmetic in the scalar type).
[[nodiscard]] constexpr double to_seconds(Timestamp t) noexcept {
    return static_cast<double>(t.ns) * 1e-9;
}

/// TimeBase<T, Wide>: a tick interval in integer nanoseconds as seconds in the
/// stack's scalar type T, with a checked error (racing-drone docs/plan/time-base/).
///
/// The conversion is computed in `Wide`, a type the slot names: dt = T(Wide(Δns)
/// / Wide(10⁹)), one rounding in Wide and one into T. Rounding each factor into T instead can
/// fail badly: in posit<16,1>, 4e6 rounds to 2^22 and 1e-9 up to minpos, so a
/// 4 ms tick becomes 15.6 ms. The slot states the relative accuracy it needs
/// of dt; a T that can't meet it fails construction, before flight.
///
/// Model
/// -----
///   nominal_ns_ : int64 — the period the slot expects
///   max_ns_     : int64 — the longest interval converted, 2 × nominal (one held tick)
///   rel_tol_    : Wide  — the relative accuracy the slot requires of dt
///
/// Invariants
/// ----------
///   I1: Wide has at least T's significand digits (compile time), and holds
///       every interval up to max_ns_ exactly    — constrains max_ns_
///   I2: 0 < nominal_ns_ ≤ max_ns_; rel_tol_ > 0 and finite
///                                               — constrains nominal_ns_, max_ns_, rel_tol_
///   I3: the nominal and the longest interval convert to a positive, finite,
///       normal T within rel_tol_ of their value in Wide
///                                               — constrains all members
///
/// Errors are relative to the interval's value in Wide, Wide(Δns) / Wide(10⁹):
/// both operands are exact (I1), so it is the exact Δns·10⁻⁹ correctly rounded,
/// within ε_Wide/2 of it.
template <typename T, typename Wide>
class TimeBase {
    static_assert(std::numeric_limits<Wide>::digits >= std::numeric_limits<T>::digits,
                  "TimeBase<T, Wide>: Wide has fewer significand digits than T; name a wider Wide");
    static_assert(std::numeric_limits<Wide>::digits >= 30, "TimeBase<T, Wide>: Wide must hold 10^9 exactly");

public:
    /// Pre: none.
    /// Post: on success I1–I3. Otherwise the reason, naming the period and,
    ///       when the type misses the bound, the error and the bound.
    [[nodiscard]] static result<TimeBase, std::string> make(std::int64_t nominal_ns, Wide rel_tol) {
        if (!(nominal_ns > 0 && nominal_ns <= std::numeric_limits<std::int64_t>::max() / 2)) {
            return std::string("TimeBase: the period must be a positive number of nanoseconds");
        }
        if (!(rel_tol > Wide(0) && rel_tol < std::numeric_limits<Wide>::infinity())) {
            return std::string("TimeBase: the required relative accuracy must be positive and finite");
        }
        const TimeBase b(nominal_ns, rel_tol);
        constexpr int digits = std::numeric_limits<Wide>::digits;
        if constexpr (digits < 63) {
            if (b.max_ns_ > (std::int64_t{1} << digits)) {  // I1
                return "TimeBase: a " + ms(nominal_ns) + " period is longer than the conversion type holds exactly";
            }
        }
        for (const std::int64_t dt : {b.nominal_ns_, b.max_ns_}) {  // I3
            const Wide w = b.wide(dt);
            const T t = static_cast<T>(w);
            if (!b.normal(t)) {
                return "TimeBase: a " + ms(dt) + " interval is not a positive, normal number in this scalar type";
            }
            const Wide e = relative_error(t, w);
            if (!(e <= rel_tol)) {
                return "TimeBase: a " + ms(dt) + " interval converts with relative error " + num(e) +
                       " in this scalar type, above the " + num(rel_tol) + " required";
            }
        }
        return b;
    }

    /// Pre:  none on the argument.
    /// Post: dt_ns · 10⁻⁹ in T, within rel_tol_ of its value in Wide, if
    ///       0 < dt_ns ≤ max_ns_ and the conversion meets the bound; else
    ///       nullopt (a timing fault in the caller, or an interval T can't
    ///       resolve).
    [[nodiscard]] std::optional<T> seconds(std::int64_t dt_ns) const {
        if (!(0 < dt_ns && dt_ns <= max_ns_)) {
            return std::nullopt;
        }
        const Wide w = wide(dt_ns);
        const T t = static_cast<T>(w);
        if (!normal(t) || !(relative_error(t, w) <= rel_tol_)) {
            return std::nullopt;
        }
        return t;
    }

    [[nodiscard]] std::int64_t nominal_ns() const noexcept {
        return nominal_ns_;
    }

    [[nodiscard]] std::int64_t max_ns() const noexcept {
        return max_ns_;
    }

    [[nodiscard]] Wide rel_tol() const noexcept {
        return rel_tol_;
    }

private:
    TimeBase(std::int64_t nominal_ns, Wide rel_tol)
        : nominal_ns_(nominal_ns), max_ns_(2 * nominal_ns), rel_tol_(rel_tol) {}

    [[nodiscard]] Wide wide(std::int64_t dt_ns) const {
        return static_cast<Wide>(dt_ns) / static_cast<Wide>(1'000'000'000);
    }

    [[nodiscard]] static bool normal(T t) {
        const Wide back = static_cast<Wide>(t);
        return back >= static_cast<Wide>(std::numeric_limits<T>::min()) &&
               back <= static_cast<Wide>(std::numeric_limits<T>::max());
    }

    [[nodiscard]] static Wide relative_error(T t, Wide w) {
        const Wide d = static_cast<Wide>(t) - w;
        return (d < Wide(0) ? -d : d) / w;
    }

    [[nodiscard]] static std::string num(Wide x) {
        char buf[32];
        std::snprintf(buf, sizeof buf, "%.3g", static_cast<double>(x));
        return buf;
    }

    [[nodiscard]] static std::string ms(std::int64_t ns) {
        char buf[32];
        std::snprintf(buf, sizeof buf, "%g ms", static_cast<double>(ns) * 1e-6);
        return buf;
    }

    std::int64_t nominal_ns_;
    std::int64_t max_ns_;
    Wide rel_tol_;
};

/// TimeBase<T, Wide>::make.
template <typename T, typename Wide>
[[nodiscard]] result<TimeBase<T, Wide>, std::string> make_time_base(std::int64_t nominal_ns, Wide rel_tol) {
    return TimeBase<T, Wide>::make(nominal_ns, rel_tol);
}

}  // namespace branes::anatomy
