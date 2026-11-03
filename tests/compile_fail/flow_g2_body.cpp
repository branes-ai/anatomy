// Must not compile (docs/plan/flow-graph, 2a): a composite filled by an operator of another binding.
// ctest builds this target and expects the G2 static_assert's message.

#include "../fixtures/flow/test_mission.hpp"

namespace fl = branes::anatomy::flow;

void describe(fl::builder& g) {
    (void)fl::composite<test::ControllerBinding, test::NullMonitorOp>(g, "controller");
}
