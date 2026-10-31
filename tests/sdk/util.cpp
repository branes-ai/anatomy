// Unit tests for the utilities: SHA-256 against FIPS 180-4 test vectors, and
// SplitMix64 against the reference output and its own determinism.

#include <branes/anatomy/util/sha256.hpp>
#include <branes/anatomy/util/splitmix64.hpp>

#include <catch2/catch_test_macros.hpp>

#include <string>

using namespace branes::anatomy;

TEST_CASE("sha256: FIPS 180-4 test vectors", "[util]") {
    REQUIRE(sha256_hex("") == "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855");
    REQUIRE(sha256_hex("abc") == "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad");
    REQUIRE(sha256_hex("abcdbcdecdefdefgefghfghighijhijkijkljklmklmnlmnomnopnopq") ==
            "248d6a61d20638b8e5c026930c3e6039a33ce45964ff2167f6ecedd419db06c1");
}

TEST_CASE("sha256: inputs around the block boundary", "[util]") {
    // 55, 56, and 64 bytes exercise the one- and two-block padding paths.
    REQUIRE(sha256_hex(std::string(55, 'a')) == "9f4390f8d30c2dd92ec9f095b65e2b9ae9b0a925a5258e241c9f1e910f734318");
    REQUIRE(sha256_hex(std::string(56, 'a')) == "b35439a4ac6f0948b6d6f9e3c6af0f5f590ce20f1bde7090ef7970686ec6738a");
    REQUIRE(sha256_hex(std::string(64, 'a')) == "ffe054fe7ae0cb6dc65c3af9b61d5209f439851db43d0ba5997337df154668eb");
}

TEST_CASE("SplitMix64: reference output for seed 0", "[util]") {
    SplitMix64 rng(0);
    REQUIRE(rng.next() == 0xE220A8397B1DCDAFull);
}

TEST_CASE("stream_seed: the derivation the simulator's streams have always used", "[util]") {
    // The simulator seeds its IMU, randomization and camera streams this way
    // (sim/basic_simulator.hpp), so a log's seed means what it always meant.
    for (const std::uint64_t seed : {0ull, 1ull, 1700ull, 20261024ull}) {
        for (const std::uint64_t stream : {0x494D55ull, 0x52414E44ull, 0x43414Dull}) {
            REQUIRE(stream_seed(seed, stream) == SplitMix64(seed ^ (stream * 0x9E3779B97F4A7C15ull)).next());
        }
    }
    STATIC_REQUIRE(stream_seed(1700, 0x494D55) == stream_seed(1700, 0x494D55));
}

TEST_CASE("stream_seed: different streams and seeds give different seeds", "[util]") {
    REQUIRE(stream_seed(1700, 1) != stream_seed(1700, 2));
    REQUIRE(stream_seed(1700, 1) != stream_seed(1701, 1));
}

TEST_CASE("SplitMix64: same seed, same stream; uniform01 in [0, 1)", "[util]") {
    SplitMix64 a(20261024);
    SplitMix64 b(20261024);
    for (int i = 0; i < 1000; ++i) {
        const double u = a.uniform01();
        REQUIRE(u == b.uniform01());
        REQUIRE(u >= 0.0);
        REQUIRE(u < 1.0);
    }
}
