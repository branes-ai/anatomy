// Must not compile (docs/plan/workbench, B3): the MLP's layer 2, f64, fed
// layer 1's f32 output with no Cast between them. ctest builds this target and
// expects the G4 static_assert's message.

#include <branes/anatomy/flow/builder.hpp>
#include <branes/anatomy/workbench/patterns/mlp.hpp>

namespace fl = branes::anatomy::flow;
namespace mlp = branes::anatomy::workbench::mlp;

void describe(fl::builder& g) {
    const auto x = fl::input<mlp::Hidden>(g, "x");
    const auto w2 = fl::constant<mlp::W2>(g, "w2");
    const auto linear2 = fl::instance<mlp::Linear2>(g, "linear2");
    fl::scope r = g.region(x);
    (void)fl::fire<"apply">(r, linear2, w2, x);
}
