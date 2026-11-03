// Must not compile (docs/plan/flow-graph, 2b): a stack of an operator whose
// binding has a handler with no call().
// ctest builds this target and expects the G2 static_assert's message.

#include "../fixtures/flow/test_mission.hpp"

namespace fl = branes::anatomy::flow;

struct Binding {
    using signature = fl::signature<fl::ports<fl::port<"imu", test::Imu, fl::direction::in>,
                                              fl::port<"estimate", test::Estimate, fl::direction::out>>>;
    struct imu : fl::handler<"imu", fl::ins<"imu">, fl::outs<>, fl::state_role::update> {};
    using handlers = fl::handlers<imu>;
};

struct Context {};

void describe(fl::stack_builder<Context>& g) {
    (void)fl::instance<fl::registered<"Silent", test::TestEstimator, Binding>>(g, "estimator");
}
