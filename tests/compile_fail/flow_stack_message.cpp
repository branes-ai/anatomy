// Must not compile (docs/plan/flow-graph, 2b): a stack of an operator whose
// ports are given as domains, which carry no message for the stack to hold.
// ctest builds this target and expects the stack's static_assert's message.

#include "../fixtures/flow/test_mission.hpp"

namespace fl = branes::anatomy::flow;

struct Context {};

void describe(fl::stack_builder<Context>& g) {
    (void)fl::composite<test::MonitorBinding, test::NullMonitorOp>(g, "monitor");
}
