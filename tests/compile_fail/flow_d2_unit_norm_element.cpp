// Must not compile (docs/plan/computational-domain, C5, W17): a unit-norm vector of integers,
// whose norm has no tolerance. ctest builds this target and expects the D2 static_assert's message.

#include <branes/anatomy/flow/domain.hpp>

#include <cstdint>

namespace fl = branes::anatomy::flow;

[[maybe_unused]] constexpr auto n = fl::max_elements_v<fl::domain<std::int32_t, fl::structure::unit_norm, fl::dim<4>>>;
