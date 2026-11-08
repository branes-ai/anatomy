#pragma once
/// @file codecs.hpp
/// @brief Trace codecs for flow::data<D> and flow::window<M, N>
/// (docs/plan/workbench, "The workbench"), so trace_compare can compare a
/// pattern's runs edge by edge.
///
/// Encoding (little-endian, util/bytes.hpp):
///   data<D>       count u64, then every stored element, in storage order. Rows
///                 past the count are value-initialized (A2), so two values of
///                 the same rows in use encode alike.
///   an element    its bit pattern, of its own width: an integer or an
///                 enumeration as its value, a float as u32, a double as u64,
///                 a complex as its real then its imaginary part, a Timestamp
///                 as i64. A long double's width varies by compiler, so it has
///                 no codec.
///   window<M, N>  count u64, dropped u64, then each of the first `count`
///                 items as a string (u32 length, then M's encoding).

#include <branes/anatomy/flow/data.hpp>
#include <branes/anatomy/flow/trace.hpp>
#include <branes/anatomy/flow/window.hpp>
#include <branes/anatomy/msg/time.hpp>
#include <branes/anatomy/util/bytes.hpp>

#include <bit>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <span>
#include <type_traits>
#include <utility>
#include <vector>

namespace branes::anatomy::workbench {

namespace detail {
template <class T>
inline constexpr bool is_complex_v = false;
template <class T>
inline constexpr bool is_complex_v<std::complex<T>> = true;
}  // namespace detail

/// Write element `e`'s bit pattern.
template <class E>
void put_element(ByteWriter& w, const E& e) {
    if constexpr (detail::is_complex_v<E>) {
        put_element(w, e.real());
        put_element(w, e.imag());
    } else if constexpr (std::is_same_v<E, Timestamp>) {
        w.i64(e.ns);
    } else if constexpr (std::is_enum_v<E>) {
        put_element(w, static_cast<std::underlying_type_t<E>>(e));
    } else if constexpr (std::is_same_v<E, bool>) {
        w.boolean(e);
    } else if constexpr (std::is_same_v<E, float>) {
        w.u32(std::bit_cast<std::uint32_t>(e));
    } else if constexpr (std::is_same_v<E, double>) {
        w.u64(std::bit_cast<std::uint64_t>(e));
    } else if constexpr (std::is_integral_v<E>) {
        static_assert(sizeof(E) <= 8, "codecs: an integer of at most 64 bits");
        if constexpr (sizeof(E) == 1) {
            w.u8(static_cast<std::uint8_t>(e));
        } else if constexpr (sizeof(E) == 2) {
            w.u16(static_cast<std::uint16_t>(e));
        } else if constexpr (sizeof(E) == 4) {
            w.u32(static_cast<std::uint32_t>(e));
        } else {
            w.u64(static_cast<std::uint64_t>(e));
        }
    } else {
        static_assert(!std::is_same_v<E, E>, "codecs: no encoding for this element type (a long double has none)");
    }
}

/// A data<D>'s encoding.
template <class D>
[[nodiscard]] std::vector<std::byte> encode(const flow::data<D>& d) {
    ByteWriter w;
    w.u64(d.count());
    for (const auto& e : d.elements) {
        put_element(w, e);
    }
    return w.take();
}

/// A window's encoding, its items encoded by `item(const M&) -> std::vector<std::byte>`.
template <class M, std::size_t N, class F>
[[nodiscard]] std::vector<std::byte> encode(const flow::window<M, N>& win, const F& item) {
    ByteWriter w;
    w.u64(win.count);
    w.u64(win.dropped);
    for (std::size_t k = 0; k < win.count; ++k) {
        const std::vector<std::byte> b = item(win.items[k]);
        w.u32(static_cast<std::uint32_t>(b.size()));
        w.bytes(std::span<const std::byte>(b));
    }
    return w.take();
}

/// Trace messages of type data<D>.
template <class D>
void add_data(flow::trace_codecs& c) {
    c.add<flow::data<D>>([](const flow::data<D>& d) { return encode(d); });
}

/// Trace messages of type window<M, N>, with M's encoder.
template <class M, std::size_t N, class F>
void add_window(flow::trace_codecs& c, F item) {
    c.add<flow::window<M, N>>([item = std::move(item)](const flow::window<M, N>& w) { return encode(w, item); });
}

}  // namespace branes::anatomy::workbench
