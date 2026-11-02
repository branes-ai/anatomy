// Must not compile (docs/plan/flow-domains, acceptance criterion 2): a domain whose element type is not one.
// ctest builds this target and expects the D3 static_assert's message.

#include <branes/anatomy/flow/domain.hpp>

namespace fl = branes::anatomy::flow;

struct NotAnElement {};

[[maybe_unused]] constexpr auto n = fl::max_elements_v<fl::vector<NotAnElement, fl::dim<3>>>;
