// Must not compile (docs/plan/flow-graph, 2a): a handler that reads a port its signature doesn't have.
// ctest builds this target and expects the G2 static_assert's message.

#include "../fixtures/flow/test_mission.hpp"

namespace fl = branes::anatomy::flow;

struct Binding {
    using signature = test::MonitorBinding::signature;
    using handlers = fl::handlers<fl::handler<"update", fl::ins<"estimate", "limits">>>;
};

using Op = fl::registered<"Monitor", test::NullMonitor, Binding>;
[[maybe_unused]] constexpr auto name = Op::name;
