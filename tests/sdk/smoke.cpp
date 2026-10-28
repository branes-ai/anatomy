// The skeleton's smoke test: the library builds, links, and its header is
// self-contained. The framework's own tests arrive with the move from
// racing-drone (docs/plan/move-from-racing-drone/).

#include <branes/anatomy/anatomy.hpp>

#include <catch2/catch_test_macros.hpp>

TEST_CASE("anatomy: the library builds and names itself", "[smoke]") {
    STATIC_REQUIRE(branes::anatomy::kLibraryName == "anatomy");
}
