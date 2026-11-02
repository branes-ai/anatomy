// Must not compile (docs/plan/flow-domains, acceptance criterion 2): a port and a state of one name.
// ctest builds this target and expects the S1 static_assert's message.

#include <branes/anatomy/flow/domain.hpp>
#include <branes/anatomy/flow/signature.hpp>

namespace fl = branes::anatomy::flow;

using D = fl::scalar<double>;
using Sig = fl::signature<fl::ports<fl::port<"x", D, fl::direction::out>>, fl::states<fl::state<"x", D>>>;
[[maybe_unused]] constexpr bool s = fl::is_signature_v<Sig>;
