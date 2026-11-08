// Must not compile (docs/plan/workbench, B2, W7): an accumulate edge of
// capacity zero. ctest builds this target and expects the G7 static_assert's message.

#include <branes/anatomy/flow/builder.hpp>

using namespace branes::anatomy::flow;

struct M {};

template <>
struct branes::anatomy::flow::domain_of<M> {
    using type = scalar<double>;
};

void describe(builder& g) {
    const auto x = input<M>(g, "x");
    (void)accumulate<0>(x);
}
