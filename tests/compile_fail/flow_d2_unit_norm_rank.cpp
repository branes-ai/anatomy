// Must not compile (docs/plan/computational-domain, C5, W17): a unit-norm domain of rank 2.
// ctest builds this target and expects the D2 static_assert's message.

#include <branes/anatomy/flow/domain.hpp>

namespace fl = branes::anatomy::flow;

[[maybe_unused]] constexpr auto n =
    fl::max_elements_v<fl::matrix<double, fl::dim<2>, fl::dim<2>, fl::structure::unit_norm>>;
