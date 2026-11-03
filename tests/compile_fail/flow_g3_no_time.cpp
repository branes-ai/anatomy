// Must not compile (docs/plan/flow-graph, 2a): a handler that takes no time, fired at an input.
// ctest builds this target and expects the G3 static_assert's message.

#include "../fixtures/flow/test_mission.hpp"

namespace fl = branes::anatomy::flow;

void describe(fl::builder& g) {
    const test::Ports p = test::ports(g);
    const test::Composites c = test::composites<test::Bodies<>>(g);
    fl::scope imu = g.region(p.imu);
    fl::fire<"imu">(imu, c.estimator, fl::at(p.imu), p.imu);
}
