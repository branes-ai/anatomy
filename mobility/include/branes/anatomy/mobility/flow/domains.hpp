#pragma once
/// @file domains.hpp
/// @brief The domain of each message of the mobility profile (anatomy
/// docs/adr/0004, step 1; docs/plan/flow-domains).
///
/// One domain_of per message struct in mobility/msg/, field by field, in
/// declaration order, with the struct's member names. A struct is a record.
/// A count and the fixed array it counts are one bounded vector of the
/// element's record, since a bounded extent carries its count in the data:
/// the whole struct, when that is all it holds (Reference, IntruderDetections),
/// or one field (ConflictSet's `conflicts`, which covers `count`).
///
/// The struct is a layout, and the domain says what the data is: the
/// estimate's covariance is a symmetric 15 × 15 matrix, whose stored elements
/// are 120 of the 225 the struct's row-major array holds.

#include <branes/anatomy/flow/domain.hpp>
#include <branes/anatomy/mobility/msg/control.hpp>
#include <branes/anatomy/mobility/msg/daa.hpp>
#include <branes/anatomy/mobility/msg/estimate.hpp>
#include <branes/anatomy/mobility/msg/fc.hpp>
#include <branes/anatomy/mobility/msg/imu.hpp>
#include <branes/anatomy/mobility/msg/intruders.hpp>
#include <branes/anatomy/mobility/msg/region.hpp>
#include <branes/anatomy/mobility/msg/truth.hpp>
#include <branes/anatomy/msg/time.hpp>

#include <cstdint>

namespace branes::anatomy::flow {

/// A 3-vector (Vec3).
template <class T>
using vec3 = vector<T, dim<3>>;
/// A quaternion (Quat): w, x, y, z.
template <class T>
using quat = vector<T, dim<4>>;
/// A Timestamp.
using timestamp = scalar<Timestamp>;

// ── control.hpp ─────────────────────────────────────────────────────

template <class T>
struct domain_of<ReferenceSample<T>> {
    using type = record<field<"t", timestamp>,
                        field<"position", vec3<T>>,
                        field<"velocity", vec3<T>>,
                        field<"acceleration", vec3<T>>,
                        field<"yaw", scalar<T>>,
                        field<"progress", scalar<T>>>;
};

template <class T>
struct domain_of<Reference<T>> {
    using type = vector<domain_of_t<ReferenceSample<T>>, bounded<kMaxReferenceSamples>>;
};

template <class T>
struct domain_of<ThrustRates<T>> {
    using type = record<field<"t", timestamp>,
                        field<"kind", scalar<CommandKind>>,
                        field<"thrust", scalar<T>>,
                        field<"body_rates", vec3<T>>,
                        field<"motors", vector<T, dim<4>>>>;
};

// ── daa.hpp ─────────────────────────────────────────────────────────

template <class T>
struct domain_of<Conflict<T>> {
    using type = record<field<"intruder_id", scalar<std::int32_t>>,
                        field<"position", vec3<T>>,
                        field<"velocity", vec3<T>>,
                        field<"tube_radius", scalar<T>>,
                        field<"time_to_closest", scalar<T>>,
                        field<"min_separation", scalar<T>>>;
};

template <class T>
struct domain_of<ConflictSet<T>> {
    using type = record<field<"valid_at", timestamp>,
                        field<"tracker_ok", scalar<bool>>,
                        field<"conflicts", vector<domain_of_t<Conflict<T>>, bounded<kMaxConflicts>>>>;
};

// ── estimate.hpp ────────────────────────────────────────────────────

template <class T>
struct domain_of<StateEstimate<T>> {
    using type = record<
        field<"t_valid", timestamp>,
        field<"position", vec3<T>>,
        field<"velocity", vec3<T>>,
        field<"attitude", quat<T>>,
        field<"body_rates", vec3<T>>,
        field<"accel_bias", vec3<T>>,
        field<"gyro_bias", vec3<T>>,
        field<"covariance", matrix<T, dim<kErrorStateDim, "n">, dim<kErrorStateDim, "n">, structure::symmetric>>>;
};

template <class T>
struct domain_of<EstimatorHealth<T>> {
    using type =
        record<field<"nis", scalar<T>>, field<"staleness_ns", scalar<std::int64_t>>, field<"ok", scalar<bool>>>;
};

// ── fc.hpp ──────────────────────────────────────────────────────────

template <>
struct domain_of<FcStatus> {
    using type = record<field<"mode", scalar<FlightMode>>,
                        field<"armed", scalar<bool>>,
                        field<"rc_ok", scalar<bool>>,
                        field<"link_ok", scalar<bool>>>;
};

// ── imu.hpp ─────────────────────────────────────────────────────────

template <class T>
struct domain_of<ImuSample<T>> {
    using type = record<field<"t", timestamp>,
                        field<"gyro", vec3<T>>,
                        field<"accel", vec3<T>>,
                        field<"saturated", scalar<std::uint8_t>>,
                        field<"temperature", scalar<T>>>;
};

// ── intruders.hpp ───────────────────────────────────────────────────

template <class T>
struct domain_of<IntruderDetection<T>> {
    using type = record<field<"bearing", vec3<T>>,
                        field<"angular_size", scalar<T>>,
                        field<"range", scalar<T>>,
                        field<"range_sigma", scalar<T>>>;
};

template <class T>
struct domain_of<IntruderDetections<T>> {
    using type = vector<domain_of_t<IntruderDetection<T>>, bounded<kMaxDetections>>;
};

// ── region.hpp ──────────────────────────────────────────────────────

template <class T>
struct domain_of<Geofence<T>> {
    using type = record<field<"min", vec3<T>>, field<"max", vec3<T>>>;
};

template <class T>
struct domain_of<StartPose<T>> {
    using type = record<field<"position", vec3<T>>, field<"yaw", scalar<T>>>;
};

// ── truth.hpp ───────────────────────────────────────────────────────

template <class T>
struct domain_of<TruthState<T>> {
    using type = record<field<"t", timestamp>,
                        field<"position", vec3<T>>,
                        field<"velocity", vec3<T>>,
                        field<"attitude", quat<T>>,
                        field<"body_rates", vec3<T>>,
                        field<"rotor_speeds", vector<T, dim<4>>>,
                        field<"on_ground", scalar<bool>>>;
};

}  // namespace branes::anatomy::flow
