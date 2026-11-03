#pragma once
/// @file domains.hpp
/// @brief The domain of each message of the core (msg/) (anatomy
/// docs/adr/0004, step 2; docs/plan/flow-graph).
///
/// The camera frame is opaque: its image is a buffer sized at run time, with
/// no bound in the data, so a frame is a scalar whose element is the frame.
/// Bounded image extents, bound from the vehicle file, come with the first
/// operator that reads pixels.

#include <branes/anatomy/flow/domain.hpp>
#include <branes/anatomy/msg/camera.hpp>

namespace branes::anatomy::flow {

template <>
struct element_traits<CameraFrame> {
    static constexpr bool is_element = true;
};

template <>
struct domain_of<CameraFrame> {
    using type = scalar<CameraFrame>;
};

}  // namespace branes::anatomy::flow
