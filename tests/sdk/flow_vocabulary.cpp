// Unit tests for the domain flow vocabulary (branes/anatomy/flow/): the
// queries on domains, records and signatures, and the matching rule. The
// invariants' violations are the compile-fail tests (tests/compile_fail/flow_*).

#include <branes/anatomy/flow/domain.hpp>
#include <branes/anatomy/flow/signature.hpp>
#include <branes/anatomy/msg/time.hpp>

#include <catch2/catch_test_macros.hpp>

#include <cstdint>
#include <type_traits>

using namespace branes::anatomy;
using namespace branes::anatomy::flow;

namespace {

enum class Mode : std::uint8_t { a, b };

using Cov = matrix<float, dim<15, "n">, dim<15, "n">, structure::symmetric>;
using Meas = vector<double, bounded<8, "m">>;
using Sample = record<field<"t", scalar<Timestamp>>, field<"x", vector<double, dim<3>>>, field<"mode", scalar<Mode>>>;

struct Scale {
    using signature = flow::signature<ports<port<"x", Meas, direction::in>, port<"y", Meas, direction::out>>>;
};

struct Counter {};

}  // namespace

// signature_of is specialized for an operator without a member signature.
template <>
struct branes::anatomy::flow::signature_of<Counter> {
    using type = signature<ports<port<"tick", scalar<Timestamp>, direction::in>>, states<state<"n", scalar<int>>>>;
};

TEST_CASE("Domain: rank, elements and bounds", "[flow]") {
    STATIC_REQUIRE(rank_v<scalar<double>> == 0);
    STATIC_REQUIRE(max_elements_v<scalar<double>> == 1);
    STATIC_REQUIRE(rank_v<vector<double, dim<3>>> == 1);
    STATIC_REQUIRE(max_elements_v<vector<double, dim<3>>> == 3);
    STATIC_REQUIRE(rank_v<Cov> == 2);
    STATIC_REQUIRE(max_elements_v<Cov> == 15 * 15);
    STATIC_REQUIRE_FALSE(is_bounded_v<Cov>);
    STATIC_REQUIRE(is_bounded_v<Meas>);
    STATIC_REQUIRE(max_elements_v<Meas> == 8);
    STATIC_REQUIRE(max_elements_v<domain<int, structure::dense, dim<2>, bounded<3>, dim<5>>> == 2 * 3 * 5);
}

TEST_CASE("Domain: the structure sets the stored elements", "[flow]") {
    using X = dim<4>;
    STATIC_REQUIRE(stored_elements_v<matrix<double, X, X>> == 4 * 4);
    STATIC_REQUIRE(stored_elements_v<matrix<double, X, X, structure::symmetric>> == 4 * 5 / 2);
    STATIC_REQUIRE(stored_elements_v<matrix<double, X, X, structure::lower_triangular>> == 4 * 5 / 2);
    STATIC_REQUIRE(stored_elements_v<matrix<double, X, X, structure::upper_triangular>> == 4 * 5 / 2);
    STATIC_REQUIRE(stored_elements_v<matrix<double, X, X, structure::diagonal>> == 4);
    STATIC_REQUIRE(stored_elements_v<Cov> == 15 * 16 / 2);
    STATIC_REQUIRE(stored_elements_v<Meas> == 8);
}

TEST_CASE("Domain: element types", "[flow]") {
    STATIC_REQUIRE(is_element_v<double>);
    STATIC_REQUIRE(is_element_v<bool>);
    STATIC_REQUIRE(is_element_v<Mode>);
    STATIC_REQUIRE(is_element_v<Timestamp>);
    STATIC_REQUIRE(is_element_v<Sample>);
    STATIC_REQUIRE_FALSE(is_element_v<const double>);
    STATIC_REQUIRE_FALSE(is_element_v<double*>);
    STATIC_REQUIRE_FALSE(is_element_v<Counter>);
    // A vector of records is a domain.
    STATIC_REQUIRE(is_domain_v<vector<Sample, bounded<64>>>);
    STATIC_REQUIRE(max_elements_v<vector<Sample, bounded<64>>> == 64);
}

TEST_CASE("Domain: two domains match when they are the same type", "[flow]") {
    STATIC_REQUIRE(std::is_same_v<vector<double, dim<3>>, domain<double, structure::dense, dim<3>>>);
    STATIC_REQUIRE_FALSE(std::is_same_v<vector<double, dim<3>>, vector<float, dim<3>>>);
    STATIC_REQUIRE_FALSE(std::is_same_v<vector<double, dim<15, "n">>, vector<double, dim<15>>>);
    STATIC_REQUIRE_FALSE(std::is_same_v<vector<double, dim<8>>, vector<double, bounded<8>>>);
    STATIC_REQUIRE_FALSE(
        std::is_same_v<matrix<double, dim<4>, dim<4>>, matrix<double, dim<4>, dim<4>, structure::symmetric>>);
    STATIC_REQUIRE(dim<15, "n">::name == "n");
    STATIC_REQUIRE(dim<15>::name.empty());
}

TEST_CASE("Record: fields by name", "[flow]") {
    STATIC_REQUIRE(Sample::size == 3);
    STATIC_REQUIRE(std::is_same_v<field_t<Sample, "t">, scalar<Timestamp>>);
    STATIC_REQUIRE(std::is_same_v<field_t<Sample, "x">, vector<double, dim<3>>>);
    STATIC_REQUIRE(std::is_same_v<field_t<Sample, "mode">, scalar<Mode>>);
    STATIC_REQUIRE(Sample::names[1] == "x");
    // A field may hold a record.
    using Outer = record<field<"sample", Sample>, field<"ok", scalar<bool>>>;
    STATIC_REQUIRE(std::is_same_v<field_t<Outer, "sample">, Sample>);
}

TEST_CASE("Signature: ports, directions and state", "[flow]") {
    using Sig = signature_of_t<Scale>;
    STATIC_REQUIRE(is_signature_v<Sig>);
    STATIC_REQUIRE(std::is_same_v<port_t<Sig, "x">, Meas>);
    STATIC_REQUIRE(direction_v<Sig, "x"> == direction::in);
    STATIC_REQUIRE(direction_v<Sig, "y"> == direction::out);
    STATIC_REQUIRE(Sig::port_list::size == 2);
    STATIC_REQUIRE(Sig::state_list::size == 0);
    STATIC_REQUIRE(std::is_same_v<Sig::space, no_index_space>);

    // Specialized: no out port, but a state domain (S2).
    using CounterSig = signature_of_t<Counter>;
    STATIC_REQUIRE(CounterSig::state_list::size == 1);
    STATIC_REQUIRE(direction_v<CounterSig, "tick"> == direction::in);

    // An inout port alone satisfies S2; a leaf names its index space.
    using Leaf = signature<ports<port<"a", Cov, direction::inout>>, states<>, index_space<dim<15>, dim<15>>>;
    STATIC_REQUIRE(is_signature_v<Leaf>);
    STATIC_REQUIRE(Leaf::space::rank == 2);
}
