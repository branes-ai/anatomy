// Must not compile (docs/plan/flow-domains, acceptance criterion 2): a signature with no output and no state.
// ctest builds this target and expects the S2 static_assert's message.

#include <branes/anatomy/flow/domain.hpp>
#include <branes/anatomy/flow/signature.hpp>

namespace fl = branes::anatomy::flow;

using Sig = fl::signature<fl::ports<fl::port<"x", fl::scalar<double>, fl::direction::in>>>;
[[maybe_unused]] constexpr bool s = fl::is_signature_v<Sig>;
