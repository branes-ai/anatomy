// Must not compile (docs/plan/flow-graph, 2a): an operator whose name is not [A-Za-z0-9_]+.
// ctest builds this target and expects the G1 static_assert's message.

#include "../fixtures/flow/test_mission.hpp"

namespace fl = branes::anatomy::flow;

[[maybe_unused]] constexpr auto name =
    fl::registered<"Test Estimator", test::TestEstimator, test::EstimatorBinding>::name;
