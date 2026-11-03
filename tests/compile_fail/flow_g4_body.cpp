// Must not compile (docs/plan/flow-graph, 2a): a body whose handler returns another domain.
// ctest builds this target and expects the G4 static_assert's message.

#include "../fixtures/flow/test_mission.hpp"

namespace fl = branes::anatomy::flow;

struct Echo {
    static constexpr std::string_view name = "Echo";
    using binding = test::ControllerBinding;
    static void build(fl::body<test::ControllerBinding>& b) {
        fl::on<"step">(b, [](fl::scope&, auto estimate, auto, auto, auto) { return std::tuple{estimate}; });
    }
};

void describe(fl::builder& g) {
    (void)fl::composite<test::ControllerBinding, Echo>(g, "controller");
}
