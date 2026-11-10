// Must not compile (docs/plan/computational-domain, C1, K2): a handler whose
// computational domain binds a parameter to an extent its port hasn't.
// ctest builds this target and expects the K2 static_assert's message.

#include <branes/anatomy/flow/builder.hpp>
#include <branes/anatomy/flow/compute.hpp>
#include <branes/anatomy/flow/data.hpp>

namespace fl = branes::anatomy::flow;
namespace cd = branes::anatomy::flow::cd;

using Vec = fl::data<fl::vector<double, fl::dim<4>>>;

struct Op {};
struct Binding {
    using signature =
        fl::signature<fl::ports<fl::port<"x", Vec, fl::direction::in>, fl::port<"y", Vec, fl::direction::out>>>;
    struct apply : fl::handler<"apply", fl::ins<"x">, fl::outs<"y">> {
        static constexpr cd::system compute = [] {
            cd::system s;
            const auto i = s.index("i", 0, s.extent("x", 1));  // a vector has no extent 1
            s.write(s.out("y").at(i), s.in("x")(i));
            return s;
        }();
    };
    using handlers = fl::handlers<apply>;
};

using Registered = fl::registered<"Bad", Op, Binding>;
[[maybe_unused]] constexpr bool r = sizeof(typename Registered::binding) > 0;
