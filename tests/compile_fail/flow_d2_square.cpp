// Must not compile (docs/plan/flow-domains, acceptance criterion 2): a symmetric domain that is not square.
// ctest builds this target and expects the D2 static_assert's message.

#include <branes/anatomy/flow/domain.hpp>

namespace fl = branes::anatomy::flow;

[[maybe_unused]] constexpr auto n =
    fl::max_elements_v<fl::matrix<double, fl::dim<3>, fl::dim<4>, fl::structure::symmetric>>;
