#pragma once
/// @file mtl5.hpp
/// @brief MTL5 views of flow::data<D>, and packing for its structured domains
/// (docs/plan/workbench, "The MTL5 adapter", B4; design choice 7).
///
/// data<D> is what an edge carries; it does no arithmetic. A pattern's
/// operators do theirs in MTL5, through this adapter. The SDK doesn't link
/// MTL5: the workbench does, at reflex's pin.
///
/// Model:
///   view(d)        : a ref over a dense data<D> of rank 1 or 2: an MTL5
///                    dense_vector, or a row-major dense2D, built with MTL5's
///                    external-pointer constructors, of d.count() rows. It
///                    neither copies nor allocates.
///   ref<M, W>      : holds the MTL5 object. It can't be copied or assigned,
///                    because in MTL5 5.20.0 copying an external-pointer view,
///                    or copy-assigning one to another, detaches it: it
///                    allocates storage of its own, and the data<D> behind it
///                    isn't written. Assigning an expression of other
///                    dimensions resizes, which detaches too. So a ref gives
///                    its MTL5 object only by const reference (get()), to read
///                    and to use in expressions, and writes go through it.
///   assign, add    : dst = expr and dst += expr, element by element. Return
///                    false and write nothing if expr's dimensions differ from
///                    dst's.
///   mult           : dst = a·b, by mtl::mult, which writes into dst. Returns
///                    false and writes nothing if the dimensions don't agree.
///   unpack, pack   : a symmetric, triangular or diagonal data<D> to a
///                    fixed-size dense MTL5 matrix (stack storage, no
///                    allocation), and back.
///
/// Contracts:
///   assign(dst, e), add(dst, e) — Pre: e doesn't read dst's elements but
///       (r, c) as it writes (r, c). Post: true and dst's data<D> written in
///       place; or false and nothing written.
///   pack(m, d, tol) — Pre, checked by assert in debug builds: for a symmetric
///       domain, m(i, j) and m(j, i) agree to tol·max(1, |m(i, j)|); for a
///       triangular or diagonal one, m is zero outside it, to tol. Post: d holds
///       m's upper triangle (symmetric), its triangle, or its diagonal.

#include <branes/anatomy/flow/data.hpp>
#include <branes/anatomy/flow/domain.hpp>

#include <mtl/mat/dense2D.hpp>
#include <mtl/mat/operators.hpp>
#include <mtl/operation/mult.hpp>
#include <mtl/operation/trans.hpp>
#include <mtl/vec/dense_vector.hpp>

#include <cassert>
#include <cmath>
#include <cstddef>
#include <type_traits>
#include <utility>

namespace branes::anatomy::workbench::mtl5 {

namespace fl = branes::anatomy::flow;

/// A row-major matrix over memory it doesn't own, when built from a pointer.
template <class E>
using matrix = mtl::mat::dense2D<E>;
/// A vector over memory it doesn't own, when built from a pointer.
template <class E>
using vector = mtl::vec::dense_vector<E>;
/// An R × C row-major matrix on the stack.
template <class E, std::size_t R, std::size_t C>
using fixed_matrix = mtl::mat::dense2D<
    E,
    mtl::mat::
        parameters<mtl::tag::row_major, mtl::detail::c_index, mtl::mat::fixed::dimensions<R, C>, mtl::tag::on_stack>>;
/// An N-vector on the stack.
template <class E, std::size_t N>
using fixed_vector = mtl::vec::
    dense_vector<E, mtl::vec::parameters<mtl::tag::col_major, mtl::vec::fixed::dimension<N>, mtl::tag::on_stack>>;

/// An MTL5 view of a data<D>'s elements, which writes through to them.
/// `Writable` is false for a view of a const data<D>.
template <class M, bool Writable>
class ref {
public:
    using mtl_type = M;
    using value_type = typename M::value_type;
    static constexpr bool is_matrix = !std::is_same_v<M, vector<value_type>>;

    explicit ref(M m) noexcept : m_(std::move(m)) {}
    ref(const ref&) = delete;
    ref& operator=(const ref&) = delete;
    ref(ref&&) = delete;
    ref& operator=(ref&&) = delete;
    ~ref() = default;

    /// The MTL5 object, to read and to use in expressions.
    [[nodiscard]] const M& get() const noexcept {
        return m_;
    }

    [[nodiscard]] std::size_t rows() const noexcept {
        if constexpr (is_matrix) {
            return static_cast<std::size_t>(m_.num_rows());
        } else {
            return static_cast<std::size_t>(m_.size());
        }
    }
    [[nodiscard]] std::size_t cols() const noexcept {
        if constexpr (is_matrix) {
            return static_cast<std::size_t>(m_.num_cols());
        } else {
            return 1;
        }
    }

    [[nodiscard]] value_type& operator()(std::size_t i) noexcept
        requires(Writable && !is_matrix)
    {
        return m_(i);
    }
    [[nodiscard]] value_type& operator()(std::size_t i, std::size_t j) noexcept
        requires(Writable && is_matrix)
    {
        return m_(i, j);
    }
    [[nodiscard]] value_type operator()(std::size_t i) const noexcept
        requires(!is_matrix)
    {
        return m_(i);
    }
    [[nodiscard]] value_type operator()(std::size_t i, std::size_t j) const noexcept
        requires(is_matrix)
    {
        return m_(i, j);
    }

private:
    friend struct access;
    M m_;
};

/// A ref's MTL5 object, writable, for the writes below; or a stack temporary itself.
struct access {
    template <class M>
    [[nodiscard]] static M& target(ref<M, true>& r) noexcept {
        return r.m_;
    }
    template <class M>
        requires(!requires(const M& x) { x.get(); })  // a stack temporary, not a ref
    [[nodiscard]] static M& target(M& m) noexcept {
        return m;
    }
};

namespace detail {
template <class D>
inline constexpr bool dense_view_v =
    fl::is_domain_v<D> && D::kind == fl::structure::dense && (D::rank == 1 || D::rank == 2);

template <class X>
struct is_ref : std::false_type {};
template <class M, bool W>
struct is_ref<ref<M, W>> : std::true_type {};

/// The MTL5 object behind x: a ref's, or x itself.
template <class X>
[[nodiscard]] const auto& unwrap(const X& x) noexcept {
    if constexpr (is_ref<X>::value) {
        return x.get();
    } else {
        return x;
    }
}

/// A matrix, or a matrix expression: indexed by two (MTL5's vectors have num_rows() too).
template <class X>
inline constexpr bool is_matrix_v = requires(const X& x) { x(std::size_t{}, std::size_t{}); };

template <class X>
[[nodiscard]] std::size_t rows_of(const X& x) {
    if constexpr (is_matrix_v<X>) {
        return static_cast<std::size_t>(x.num_rows());
    } else {
        return static_cast<std::size_t>(x.size());
    }
}
template <class X>
[[nodiscard]] std::size_t cols_of(const X& x) {
    if constexpr (is_matrix_v<X>) {
        return static_cast<std::size_t>(x.num_cols());
    } else {
        return 1;
    }
}
}  // namespace detail

/// A writable view of a dense data<D> of rank 1 or 2: d.count() rows.
template <class D>
    requires detail::dense_view_v<D>
[[nodiscard]] auto view(fl::data<D>& d) {
    using E = typename D::element_type;
    if constexpr (D::rank == 1) {
        return ref<vector<E>, true>(vector<E>(d.count(), d.elements.data()));
    } else {
        return ref<matrix<E>, true>(matrix<E>(d.count(), fl::data<D>::cols, d.elements.data()));
    }
}

/// A read-only view of a dense data<D> of rank 1 or 2.
template <class D>
    requires detail::dense_view_v<D>
[[nodiscard]] auto view(const fl::data<D>& d) {
    using E = typename D::element_type;
    // MTL5's external constructors take a non-const pointer; the ref writes nothing through it.
    E* p = const_cast<E*>(d.elements.data());
    if constexpr (D::rank == 1) {
        return ref<vector<E>, false>(vector<E>(d.count(), p));
    } else {
        return ref<matrix<E>, false>(matrix<E>(d.count(), fl::data<D>::cols, p));
    }
}

namespace detail {
/// t = e (or t += e), element by element, if their dimensions agree.
template <class T, class X>
[[nodiscard]] bool write(T& t, const X& e, bool accumulate) noexcept {
    const auto& src = unwrap(e);
    if (rows_of(src) != rows_of(t) || cols_of(src) != cols_of(t)) {
        return false;
    }
    using V = typename T::value_type;
    for (std::size_t i = 0; i < rows_of(t); ++i) {
        if constexpr (is_matrix_v<T>) {
            for (std::size_t j = 0; j < cols_of(t); ++j) {
                const auto v = static_cast<V>(src(i, j));
                t(i, j) = accumulate ? static_cast<V>(t(i, j) + v) : v;
            }
        } else {
            const auto v = static_cast<V>(src(i));
            t(i) = accumulate ? static_cast<V>(t(i) + v) : v;
        }
    }
    return true;
}

template <class R>
concept destination = requires(R& r) { access::target(r); };
}  // namespace detail

/// dst = e: an MTL5 expression, matrix, vector, or a view. dst is a writable
/// view, or a stack temporary (fixed_matrix, fixed_vector).
template <class R, class X>
    requires detail::destination<R>
[[nodiscard]] bool assign(R& dst, const X& e) noexcept {
    return detail::write(access::target(dst), e, false);
}

/// dst += e.
template <class R, class X>
    requires detail::destination<R>
[[nodiscard]] bool add(R& dst, const X& e) noexcept {
    return detail::write(access::target(dst), e, true);
}

/// dst = a·b, by mtl::mult into dst's own elements. a and b are MTL5 objects or views.
template <class A, class B, class R>
    requires detail::destination<R>
[[nodiscard]] bool mult(const A& a, const B& b, R& dst) {
    const auto& x = detail::unwrap(a);
    const auto& y = detail::unwrap(b);
    auto& t = access::target(dst);
    if (detail::cols_of(x) != detail::rows_of(y) || detail::rows_of(x) != detail::rows_of(t) ||
        detail::cols_of(y) != detail::cols_of(t)) {
        return false;
    }
    mtl::mult(x, y, t);
    return true;
}

/// A symmetric, triangular or diagonal n × n data<D> as a dense matrix on the stack.
template <class D>
[[nodiscard]] auto unpack(const fl::data<D>& d) {
    static_assert(fl::is_domain_v<D> && D::rank == 2 && D::kind != fl::structure::dense,
                  "unpack: a symmetric, triangular or diagonal domain; view() a dense one");
    using E = typename D::element_type;
    constexpr std::size_t n = fl::data<D>::cols;
    fixed_matrix<E, n, n> m;
    for (std::size_t i = 0; i < n; ++i) {
        for (std::size_t j = 0; j < n; ++j) {
            m(i, j) = d.at(i, j);
        }
    }
    return m;
}

/// m into d, by d's structure (see the contract above).
template <class D, class M>
void pack(const M& m, fl::data<D>& d, [[maybe_unused]] double tol = 0.0) {
    static_assert(fl::is_domain_v<D> && D::rank == 2 && D::kind != fl::structure::dense,
                  "pack: a symmetric, triangular or diagonal domain");
    using E = typename D::element_type;
    constexpr std::size_t n = fl::data<D>::cols;
    assert(detail::rows_of(m) == n && detail::cols_of(m) == n);
    for (std::size_t i = 0; i < n; ++i) {
        for (std::size_t j = 0; j < n; ++j) {
            const E v = static_cast<E>(m(i, j));
            if (fl::data<D>::inside(i, j)) {
                if constexpr (D::kind == fl::structure::symmetric) {
                    assert(std::abs(static_cast<double>(v - static_cast<E>(m(j, i)))) <=
                           tol * std::fmax(1.0, std::abs(static_cast<double>(v))));
                    if (i > j) {
                        continue;  // the upper triangle is stored
                    }
                }
                d(i, j) = v;
            } else {
                assert(std::abs(static_cast<double>(v)) <= tol);
            }
        }
    }
}

}  // namespace branes::anatomy::workbench::mtl5
