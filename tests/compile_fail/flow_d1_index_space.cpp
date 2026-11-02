// Must not compile (docs/plan/flow-domains, acceptance criterion 2): a
// signature whose index space has an extent of size 0.
// ctest builds this target and expects the D1 static_assert's message.

#include <branes/anatomy/flow/domain.hpp>
#include <branes/anatomy/flow/signature.hpp>

namespace fl = branes::anatomy::flow;

using Sig = fl::signature<fl::ports<fl::port<"x", fl::scalar<double>, fl::direction::out>>,
                          fl::states<>,
                          fl::index_space<fl::dim<0>>>;
[[maybe_unused]] constexpr bool s = fl::is_signature_v<Sig>;
