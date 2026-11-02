// Must not compile (docs/plan/flow-domains, acceptance criterion 2): a record with two fields of one name.
// ctest builds this target and expects the R1 static_assert's message.

#include <branes/anatomy/flow/domain.hpp>

namespace fl = branes::anatomy::flow;

using R = fl::record<fl::field<"x", fl::scalar<double>>, fl::field<"x", fl::scalar<float>>>;
[[maybe_unused]] constexpr bool r = fl::is_element_v<R>;
