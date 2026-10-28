// Must not compile (docs/plan/time-base/, acceptance criterion 3): a TimeBase
// whose conversion type has fewer significand digits than the scalar type.
// ctest builds this target and expects TimeBase's static_assert message.

#include <branes/anatomy/msg/time.hpp>

int main() {
    const auto b = branes::anatomy::make_time_base<double, float>(4'000'000, 0.1f);
    return b ? 0 : 1;
}
