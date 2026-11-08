// Must not compile (docs/plan/workbench, B2, W6): an accumulate edge of a
// message whose domain has a shape, which a window doesn't hold until a
// pattern needs it. ctest builds this target and expects the static_assert's message.

#include <branes/anatomy/flow/builder.hpp>
#include <branes/anatomy/flow/data.hpp>

using namespace branes::anatomy::flow;

using Vec = data<vector<double, dim<4>>>;

void describe(builder& g) {
    const auto x = input<Vec>(g, "x");
    using D = domain_of_t<window_t<Vec, 2>>;
    (void)sizeof(D);
    (void)accumulate<2>(x);
}
