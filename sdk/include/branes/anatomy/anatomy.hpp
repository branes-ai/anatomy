#pragma once
/// @file anatomy.hpp
/// @brief branes::anatomy: the Branes.AI mission framework.
///
/// The body plan every mission composes its vehicle with: the data contracts,
/// the slot concepts and their composition, configs, the replay log, the
/// harness, and the closed-loop simulator, all mission-neutral
/// (racing-drone ADR 0008). The mission repos (racing-drone, scout-drone, ...)
/// supply their task: their world, the slots that see it, their stack and
/// their scorer.
///
/// The framework came here from racing-drone (racing-drone ADR 0008; this
/// repo's docs/plan/move-from-racing-drone/). This header names the library;
/// include the headers of the parts you use.

#include <string_view>

namespace branes::anatomy {

/// The library's name, as mission repos and logs refer to it.
inline constexpr std::string_view kLibraryName = "anatomy";

}  // namespace branes::anatomy
