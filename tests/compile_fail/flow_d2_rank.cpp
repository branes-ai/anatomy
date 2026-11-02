// Must not compile (docs/plan/flow-domains, acceptance criterion 2): a symmetric domain of rank 1.
// ctest builds this target and expects the D2 static_assert's message.

#include <branes/anatomy/flow/domain.hpp>

namespace fl = branes::anatomy::flow;

[[maybe_unused]] constexpr auto n = fl::max_elements_v<fl::domain<double, fl::structure::symmetric, fl::dim<3>>>;
