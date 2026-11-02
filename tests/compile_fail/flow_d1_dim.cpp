// Must not compile (docs/plan/flow-domains, acceptance criterion 2): an extent of size 0.
// ctest builds this target and expects the D1 static_assert's message.

#include <branes/anatomy/flow/domain.hpp>

namespace fl = branes::anatomy::flow;

[[maybe_unused]] constexpr auto n = fl::max_elements_v<fl::vector<double, fl::dim<0>>>;
