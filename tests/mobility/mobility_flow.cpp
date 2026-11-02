// Unit tests for the mobility profile's domains and composite signatures
// (branes/anatomy/mobility/flow/): each message's domain against its struct,
// and each neutral composite's signature against its slot concept.

#include <branes/anatomy/flow/domain.hpp>
#include <branes/anatomy/flow/signature.hpp>
#include <branes/anatomy/mobility/flow/domains.hpp>
#include <branes/anatomy/mobility/flow/signatures.hpp>
#include <branes/anatomy/mobility/msg/control.hpp>
#include <branes/anatomy/mobility/msg/daa.hpp>
#include <branes/anatomy/mobility/msg/estimate.hpp>
#include <branes/anatomy/mobility/msg/fc.hpp>
#include <branes/anatomy/mobility/msg/imu.hpp>
#include <branes/anatomy/mobility/msg/intruders.hpp>
#include <branes/anatomy/mobility/msg/region.hpp>
#include <branes/anatomy/mobility/msg/truth.hpp>
#include <branes/anatomy/mobility/slots/neutral.hpp>
#include <branes/anatomy/msg/time.hpp>

#include <catch2/catch_template_test_macros.hpp>
#include <catch2/catch_test_macros.hpp>

#include <array>
#include <cstddef>
#include <optional>
#include <tuple>
#include <type_traits>
#include <utility>

using namespace branes::anatomy;
namespace fl = branes::anatomy::flow;

namespace {

/// The number of elements of an array member.
template <class A>
inline constexpr std::size_t array_size_v = std::tuple_size_v<std::remove_cvref_t<A>>;

/// A field's single extent's size, or bound.
template <class R, fl::fixed_string Name>
inline constexpr std::size_t extent_v = fl::max_elements_v<fl::field_t<R, Name>>;

}  // namespace

// ── Every message's domain matches its struct ───────────────────────

TEMPLATE_TEST_CASE("Domains: every message has a domain or a record", "[flow]", float, double) {
    using T = TestType;
    STATIC_REQUIRE(fl::is_record_v<fl::domain_of_t<ReferenceSample<T>>>);
    STATIC_REQUIRE(fl::is_domain_v<fl::domain_of_t<Reference<T>>>);
    STATIC_REQUIRE(fl::is_record_v<fl::domain_of_t<ThrustRates<T>>>);
    STATIC_REQUIRE(fl::is_record_v<fl::domain_of_t<Conflict<T>>>);
    STATIC_REQUIRE(fl::is_record_v<fl::domain_of_t<ConflictSet<T>>>);
    STATIC_REQUIRE(fl::is_record_v<fl::domain_of_t<StateEstimate<T>>>);
    STATIC_REQUIRE(fl::is_record_v<fl::domain_of_t<EstimatorHealth<T>>>);
    STATIC_REQUIRE(fl::is_record_v<fl::domain_of_t<FcStatus>>);
    STATIC_REQUIRE(fl::is_record_v<fl::domain_of_t<ImuSample<T>>>);
    STATIC_REQUIRE(fl::is_record_v<fl::domain_of_t<IntruderDetection<T>>>);
    STATIC_REQUIRE(fl::is_domain_v<fl::domain_of_t<IntruderDetections<T>>>);
    STATIC_REQUIRE(fl::is_record_v<fl::domain_of_t<Geofence<T>>>);
    STATIC_REQUIRE(fl::is_record_v<fl::domain_of_t<StartPose<T>>>);
    STATIC_REQUIRE(fl::is_record_v<fl::domain_of_t<TruthState<T>>>);
}

TEMPLATE_TEST_CASE("Domains: StateEstimate, and its symmetric covariance", "[flow]", float, double) {
    using T = TestType;
    using M = StateEstimate<T>;
    using R = fl::domain_of_t<M>;
    STATIC_REQUIRE(R::size == 8);
    STATIC_REQUIRE(std::is_same_v<fl::field_t<R, "t_valid">, fl::scalar<Timestamp>>);
    STATIC_REQUIRE(extent_v<R, "position"> == array_size_v<decltype(M::position)>);
    STATIC_REQUIRE(extent_v<R, "velocity"> == array_size_v<decltype(M::velocity)>);
    STATIC_REQUIRE(extent_v<R, "attitude"> == sizeof(Quat<T>) / sizeof(T));  // w, x, y, z
    STATIC_REQUIRE(extent_v<R, "body_rates"> == array_size_v<decltype(M::body_rates)>);
    STATIC_REQUIRE(extent_v<R, "accel_bias"> == array_size_v<decltype(M::accel_bias)>);
    STATIC_REQUIRE(extent_v<R, "gyro_bias"> == array_size_v<decltype(M::gyro_bias)>);

    using Cov = fl::field_t<R, "covariance">;
    STATIC_REQUIRE(Cov::kind == fl::structure::symmetric);
    STATIC_REQUIRE(fl::rank_v<Cov> == 2);
    STATIC_REQUIRE(fl::max_elements_v<Cov> == array_size_v<decltype(M::covariance)>);
    STATIC_REQUIRE(fl::stored_elements_v<Cov> == kErrorStateDim * (kErrorStateDim + 1) / 2);
    STATIC_REQUIRE(std::tuple_element_t<0, typename Cov::extents>::name == "n");
    STATIC_REQUIRE(std::is_same_v<typename Cov::element_type, T>);

    using H = EstimatorHealth<T>;
    STATIC_REQUIRE(fl::domain_of_t<H>::size == 3);
}

TEMPLATE_TEST_CASE("Domains: Reference is a bounded vector of sample records", "[flow]", float, double) {
    using T = TestType;
    using M = Reference<T>;
    using D = fl::domain_of_t<M>;
    STATIC_REQUIRE(fl::rank_v<D> == 1);
    STATIC_REQUIRE(fl::is_bounded_v<D>);
    STATIC_REQUIRE(fl::max_elements_v<D> == array_size_v<decltype(M::samples)>);
    STATIC_REQUIRE(std::is_same_v<typename D::element_type, fl::domain_of_t<ReferenceSample<T>>>);

    using S = ReferenceSample<T>;
    using R = fl::domain_of_t<S>;
    STATIC_REQUIRE(R::size == 6);
    STATIC_REQUIRE(extent_v<R, "position"> == array_size_v<decltype(S::position)>);
    STATIC_REQUIRE(extent_v<R, "velocity"> == array_size_v<decltype(S::velocity)>);
    STATIC_REQUIRE(extent_v<R, "acceleration"> == array_size_v<decltype(S::acceleration)>);
    STATIC_REQUIRE(std::is_same_v<fl::field_t<R, "progress">, fl::scalar<T>>);
}

TEMPLATE_TEST_CASE("Domains: ThrustRates, ImuSample and TruthState", "[flow]", float, double) {
    using T = TestType;
    using U = ThrustRates<T>;
    using RU = fl::domain_of_t<U>;
    STATIC_REQUIRE(RU::size == 5);
    STATIC_REQUIRE(std::is_same_v<fl::field_t<RU, "kind">, fl::scalar<CommandKind>>);
    STATIC_REQUIRE(extent_v<RU, "body_rates"> == array_size_v<decltype(U::body_rates)>);
    STATIC_REQUIRE(extent_v<RU, "motors"> == array_size_v<decltype(U::motors)>);

    using I = ImuSample<T>;
    using RI = fl::domain_of_t<I>;
    STATIC_REQUIRE(RI::size == 5);
    STATIC_REQUIRE(extent_v<RI, "gyro"> == array_size_v<decltype(I::gyro)>);
    STATIC_REQUIRE(extent_v<RI, "accel"> == array_size_v<decltype(I::accel)>);
    STATIC_REQUIRE(std::is_same_v<fl::field_t<RI, "saturated">, fl::scalar<decltype(I::saturated)>>);

    using X = TruthState<T>;
    using RX = fl::domain_of_t<X>;
    STATIC_REQUIRE(RX::size == 7);
    STATIC_REQUIRE(extent_v<RX, "attitude"> == sizeof(Quat<T>) / sizeof(T));
    STATIC_REQUIRE(extent_v<RX, "rotor_speeds"> == array_size_v<decltype(X::rotor_speeds)>);
    STATIC_REQUIRE(std::is_same_v<fl::field_t<RX, "on_ground">, fl::scalar<bool>>);
}

TEMPLATE_TEST_CASE("Domains: the DAA messages are bounded by their capacities", "[flow]", float, double) {
    using T = TestType;
    using C = ConflictSet<T>;
    using RC = fl::domain_of_t<C>;
    STATIC_REQUIRE(RC::size == 3);
    using Conflicts = fl::field_t<RC, "conflicts">;
    STATIC_REQUIRE(fl::is_bounded_v<Conflicts>);
    STATIC_REQUIRE(fl::max_elements_v<Conflicts> == array_size_v<decltype(C::conflicts)>);
    STATIC_REQUIRE(fl::domain_of_t<Conflict<T>>::size == 6);

    using D = IntruderDetections<T>;
    using DD = fl::domain_of_t<D>;
    STATIC_REQUIRE(fl::is_bounded_v<DD>);
    STATIC_REQUIRE(fl::max_elements_v<DD> == array_size_v<decltype(D::detections)>);
    STATIC_REQUIRE(fl::domain_of_t<IntruderDetection<T>>::size == 4);
}

TEMPLATE_TEST_CASE("Domains: FcStatus, Geofence and StartPose", "[flow]", float, double) {
    using T = TestType;
    STATIC_REQUIRE(fl::domain_of_t<FcStatus>::size == 4);
    STATIC_REQUIRE(std::is_same_v<fl::field_t<fl::domain_of_t<FcStatus>, "mode">, fl::scalar<FlightMode>>);
    using G = Geofence<T>;
    STATIC_REQUIRE(fl::domain_of_t<G>::size == 2);
    STATIC_REQUIRE(extent_v<fl::domain_of_t<G>, "min"> == array_size_v<decltype(G::min)>);
    STATIC_REQUIRE(extent_v<fl::domain_of_t<G>, "max"> == array_size_v<decltype(G::max)>);
    STATIC_REQUIRE(fl::domain_of_t<StartPose<T>>::size == 2);
}

// ── Every composite's signature matches its slot concept ────────────
//
// A probe implements the slot from the message types each port is the domain
// of, in the signature's port order. The probe satisfying the concept, and
// each message's domain being its port's, ties the signature to the concept.

namespace {

template <class T, class Estimate, class Ref, class Conflicts, class Command>
struct ControllerProbe {
    Command step(const Estimate&, const Ref&, const Conflicts&) {
        return {};
    }
};

template <class T, class Estimate, class Command, class Conflicts, class Fence, class Safe>
struct SafetyFilterProbe {
    Safe filter(const Estimate&, const Command&, const Conflicts&, const Fence&) {
        return {};
    }
};

template <class T, class Detections, class Estimate, class Conflicts>
struct DaaTrackerProbe {
    void update(const Detections&, const Estimate&) {}
    Conflicts conflicts(const Estimate&) const {
        return {};
    }
};

template <class T, class Command, class Imu, class Status>
struct FcBridgeProbe {
    void send(const Command&, Timestamp) {}
    std::optional<Imu> poll_imu() {
        return std::nullopt;
    }
    Status fc_status() const {
        return {};
    }
};

/// Each port of Sig, in order, is the domain of its message, with its direction.
template <class Sig, class Messages, std::size_t... I>
constexpr bool ports_are(std::index_sequence<I...>, const std::array<fl::direction, sizeof...(I)>& dirs) {
    using List = typename Sig::port_list;
    return List::size == sizeof...(I) && Sig::state_list::size == 0 &&
           ((std::is_same_v<typename std::tuple_element_t<I, typename List::types>::type,
                            fl::domain_of_t<std::tuple_element_t<I, Messages>>> &&
             std::tuple_element_t<I, typename List::types>::dir == dirs[I]) &&
            ...);
}

constexpr fl::direction kIn = fl::direction::in;
constexpr fl::direction kOut = fl::direction::out;

}  // namespace

TEMPLATE_TEST_CASE("Signatures: controller", "[flow]", float, double) {
    using T = TestType;
    using Messages = std::tuple<StateEstimate<T>, Reference<T>, ConflictSet<T>, ThrustRates<T>>;
    STATIC_REQUIRE(
        ports_are<fl::controller_signature<T>, Messages>(std::make_index_sequence<4>{}, {kIn, kIn, kIn, kOut}));
    STATIC_REQUIRE(Controller<ControllerProbe<T, StateEstimate<T>, Reference<T>, ConflictSet<T>, ThrustRates<T>>, T>);
}

TEMPLATE_TEST_CASE("Signatures: safety filter", "[flow]", float, double) {
    using T = TestType;
    using Messages = std::tuple<StateEstimate<T>, ThrustRates<T>, ConflictSet<T>, Geofence<T>, ThrustRates<T>>;
    STATIC_REQUIRE(
        ports_are<fl::safety_filter_signature<T>, Messages>(std::make_index_sequence<5>{}, {kIn, kIn, kIn, kIn, kOut}));
    STATIC_REQUIRE(SafetyFilter<
                   SafetyFilterProbe<T, StateEstimate<T>, ThrustRates<T>, ConflictSet<T>, Geofence<T>, ThrustRates<T>>,
                   T>);
}

TEMPLATE_TEST_CASE("Signatures: DAA tracker", "[flow]", float, double) {
    using T = TestType;
    using Messages = std::tuple<IntruderDetections<T>, StateEstimate<T>, ConflictSet<T>>;
    STATIC_REQUIRE(ports_are<fl::daa_tracker_signature<T>, Messages>(std::make_index_sequence<3>{}, {kIn, kIn, kOut}));
    STATIC_REQUIRE(DaaTracker<DaaTrackerProbe<T, IntruderDetections<T>, StateEstimate<T>, ConflictSet<T>>, T>);
}

TEMPLATE_TEST_CASE("Signatures: FC bridge", "[flow]", float, double) {
    using T = TestType;
    using Messages = std::tuple<ThrustRates<T>, ImuSample<T>, FcStatus>;
    STATIC_REQUIRE(ports_are<fl::fc_bridge_signature<T>, Messages>(std::make_index_sequence<3>{}, {kIn, kOut, kOut}));
    STATIC_REQUIRE(FcBridge<FcBridgeProbe<T, ThrustRates<T>, ImuSample<T>, FcStatus>, T>);
}
