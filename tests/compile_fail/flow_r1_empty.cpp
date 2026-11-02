// Must not compile (docs/plan/flow-domains, acceptance criterion 2): a record with no fields.
// ctest builds this target and expects the R1 static_assert's message.

#include <branes/anatomy/flow/domain.hpp>

namespace fl = branes::anatomy::flow;

[[maybe_unused]] constexpr bool r = fl::is_element_v<fl::record<>>;
