#pragma once
/// @file slot_context.hpp
/// @brief How a slot is built from a mission's context: make_slot, ContextBuilt, parameter_in.
///
/// A slot type with a factory
///
///   static result<S, std::string> from_context(const Ctx&)
///
/// is built from the context by the mission's stack; any other slot is
/// default-constructed. The choice is made per slot at compile time, so the
/// null slots stay as they are. The context is the mission's: what it holds is
/// the profile's and the mission's choice (a roaming vehicle's:
/// BasicSlotContext, slots/context.hpp). Every context carries the baseline
/// file, whose `parameters:` block parameter_in() reads.

#include <branes/anatomy/config/baseline.hpp>
#include <branes/anatomy/result.hpp>

#include <concepts>
#include <string>
#include <string_view>

namespace branes::anatomy {

/// A slot type built by `S::from_context(ctx)` from a context `Ctx`.
template <class S, class Ctx>
concept ContextBuilt = requires(const Ctx& ctx) {
    { S::from_context(ctx) } -> std::same_as<result<S, std::string>>;
};

/// Build slot S from `ctx` if it is ContextBuilt from Ctx, else default-construct it.
template <class S, class Ctx>
[[nodiscard]] result<S, std::string> make_slot(const Ctx& ctx) {
    if constexpr (ContextBuilt<S, Ctx>) {
        return S::from_context(ctx);
    } else {
        (void)ctx;
        return S{};
    }
}

/// A baseline parameter as a number: `fallback` if absent, an error if present
/// but not a number or outside [lo, hi].
[[nodiscard]] inline result<double, std::string>
parameter_in(const config::BaselineConfig& b, std::string_view key, double fallback, double lo, double hi) {
    double v = fallback;
    if (b.parameters.count(std::string(key)) != 0) {
        const auto n = config::numeric_parameter(b, key);
        if (!n) {
            return std::string("baseline parameter " + std::string(key) + " is not a number");
        }
        v = *n;
    }
    if (!(v >= lo && v <= hi)) {
        return std::string("baseline parameter " + std::string(key) + " must be in [" + std::to_string(lo) + ", " +
                           std::to_string(hi) + "]");
    }
    return v;
}

}  // namespace branes::anatomy
