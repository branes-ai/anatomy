---
status: in-progress
issue: branes-ai/anatomy#40
---
# Plan: domain types and operator signatures (ADR 0004, step 1)

Issue: [anatomy#40](https://github.com/branes-ai/anatomy/issues/40).

## Goal

This is step 1 of [ADR 0004](../../adr/0004-domain-flow-composition.md): the vocabulary of layers 1 and 2 of the [domain flow model](../../arch/domain-flow-model.md), in C++, alongside the slot concepts. It adds:

- **in the core:** the types that describe a domain, a record of domains, a port, a state domain, a computational domain, and an operator signature, with their invariants checked at compile time;
- **in the mobility profile:** the domain of each message a roaming vehicle passes, and the signatures of the four neutral composites (controller, safety filter, DAA tracker, FC bridge), each matching its slot concept.

Nothing uses them yet. Step 2 builds the graph format and the generator on them.

**No change in behavior:**
- no existing header changes, except to add the new files to the documentation map;
- the slot concepts, the stacks, the log and the harness are untouched;
- the mission repos are untouched, and their outputs can't change because nothing they build includes the new headers.

## Model

Everything is a type, and every check is a `static_assert` or a concept, so the generator of step 2 can check a graph's edges with `std::is_same`. Nothing is evaluated at run time. The headers are `flow/domain.hpp` and `flow/signature.hpp` in the core (`sdk/include/branes/anatomy/flow/`, namespace `branes::anatomy::flow`), and `mobility/flow/domains.hpp` and `mobility/flow/signatures.hpp` in the mobility profile.

```cpp
namespace branes::anatomy::flow {

// Extents (model §3): a static size, or an upper bound with a count in the data.
template <std::size_t N, fixed_string Name = "">   struct dim;
template <std::size_t Max, fixed_string Name = ""> struct bounded;

enum class structure : std::uint8_t { dense, symmetric, lower_triangular, upper_triangular, diagonal };

// A domain: an element type, a structure, and its extents (rank = their number).
template <class E, structure S, class... Extents> struct domain;
template <class E>                                using scalar = domain<E, structure::dense>;
template <class E, class X>                       using vector = domain<E, structure::dense, X>;
template <class E, class R, class C, structure S = structure::dense>
                                                  using matrix = domain<E, S, R, C>;

// A record: named fields, each a domain or a record. A record is also an element type,
// so an array of records is a domain (a bounded vector of reference samples).
template <fixed_string Name, class D> struct field;
template <class... Fields>            struct record;

// What a message type is, as a domain; specialized per message.
template <class M> struct domain_of;
template <class M> using domain_of_t = typename domain_of<M>::type;

// Ports, state and the computational domain (model §5).
enum class direction : std::uint8_t { in, out, inout };
template <fixed_string Name, class D, direction Dir> struct port;
template <fixed_string Name, class D>                struct state;
template <class... Extents>                          struct index_space;
struct no_index_space;

template <class... P> struct ports;
template <class... S> struct states;
template <class Ports, class States = states<>, class Space = no_index_space> struct signature;

// An operator's signature, without touching the operator's type: either
// Op::signature, or a specialization of signature_of.
template <class Op> struct signature_of;
}
```

**Element types** are the arithmetic types, enumerations, `Timestamp`, and records, without cv-qualifiers. Any other type `T` is an element type only if `element_traits<T>` is specialized. The scalar types of the precision sweep (Universal's, in mobility_sim) will be specialized when the sweep first describes a graph, in step 2. The core names no Universal type.

**Queries,** all `constexpr` and used by the tests and by step 2:

| Query | Meaning |
|---|---|
| `rank_v<D>` | the number of extents |
| `max_elements_v<D>` | the product of the extents' sizes or bounds |
| `stored_elements_v<D>` | the elements the structure needs: `n(n+1)/2` for symmetric and triangular, `n` for diagonal |
| `is_bounded_v<D>` | some extent is bounded |
| `field_t<R, "name">` | a record field's domain |
| `port_t<Sig, "name">`, `direction_v<Sig, "name">` | a port's domain and direction |

**Two domains match when they are the same type.** The extents' names count, so `dim<15, "n">` doesn't match `dim<15>`. That is the stricter rule, and it costs nothing while every message's domain is written once, in `domain_of`.

## Invariants

All of them are checked at compile time. A violation is a `static_assert` with a message that names the invariant.

| Id | Invariant | Constrains |
|---|---|---|
| D1 | every extent has a size or a bound of at least 1 | `dim`, `bounded` |
| D2 | a structure other than dense needs rank 2 and the same extent twice (a square domain) | `domain` |
| D3 | the element type is an element type | `domain` |
| R1 | a record has at least one field, and its field names are non-empty and distinct | `record` |
| S1 | a signature's port and state names are non-empty and distinct, across ports and state together | `signature` |
| S2 | a signature has at least one `out` or `inout` port, or a state domain | `signature` |

The checks are in the types' bodies, so they run when a type is instantiated: by a query, or by a record, port or signature that holds it, which instantiates what it holds. An invalid domain can be named in an alias, but it can't be used, or held by a signature that is used, without failing to compile.

**Each container also checks what it holds,** under its invariant's id: a field, a port and a state hold a domain or a record (R1, S1), `ports<>` holds ports and `states<>` states (S1), and a domain's or an index space's extents are `dim` or `bounded` (D1).

## Contracts

The types carry no operations, only queries.

- **`field_t<R, Name>`, `port_t<Sig, Name>`, `direction_v<Sig, Name>`.** Pre: `Name` is one of the record's fields or the signature's ports, or the code fails to compile with a message naming it. Post: the field's or port's domain or direction.
- **`domain_of<M>`.** Pre: specialized for `M`. Post: `domain_of_t<M>` is a domain (D1–D3), or a record (R1). A port, a state or a field holds either.
- **`signature_of<Op>`.** Pre: `Op::signature` exists, or `signature_of<Op>` is specialized. Post: a signature (S1, S2).

## The mobility domains

`domain_of` for every message struct in `mobility/msg/`, field by field, in their declaration order, with the struct's member names. That includes the element records (`ReferenceSample`, `Conflict`, `IntruderDetection`) and the two structs no slot passes (`EstimatorHealth`, `StartPose`):
- `StateEstimate<T>` is a record: `t_valid` a `scalar<Timestamp>`; position, velocity, body rates and the two biases each a `vector<T, dim<3>>`; the attitude a `vector<T, dim<4>>` (w, x, y, z); and the covariance a symmetric `matrix<T, dim<15, "n">, dim<15, "n">, symmetric>`, so its stored elements are 120 of the 225 the struct holds.
- `Reference<T>` is a `vector` of `ReferenceSample<T>` records, `bounded<kMaxReferenceSamples>`, whose count is the struct's `count`.
- `IntruderDetections<T>` is the same: a `vector` of `IntruderDetection<T>` records, `bounded<kMaxDetections>`. In `ConflictSet<T>`, the conflicts are a field, a `vector` of `Conflict<T>` records, `bounded<kMaxConflicts>`, beside `valid_at` and `tracker_ok`.
- `ThrustRates<T>`, `ImuSample<T>`, `Geofence<T>`, `FcStatus`, `TruthState<T>`, `EstimatorHealth<T>` and `StartPose<T>` are records of their members. Enumerations such as `CommandKind` and `FlightMode` are scalar fields, and a quaternion is a `vector<T, dim<4>>`.

**The covariance is symmetric in the domain, and full in the struct.** The struct's row-major array is a layout, which the model leaves to layer 5. The domain says what the data is: a symmetric matrix, whose storage the compiler may halve.

## The neutral composites

`mobility/flow/signatures.hpp` gives the signature of each neutral slot as a composite (ADR 0004, decision 6):

| Composite | Ports |
|---|---|
| controller | in `estimate`, in `reference`, in `conflicts`; out `command` |
| safety filter | in `estimate`, in `command`, in `conflicts`, in `geofence`; out `safe_command` |
| DAA tracker | in `detections`, in `estimate`; out `conflicts` |
| FC bridge | in `command`; out `imu`, out `fc_status` |

**Each port's domain is the `domain_of` of the type its slot concept takes or returns.** A test builds a probe of each slot from the message types its ports are the domains of, in port order, and checks that the probe satisfies the concept. So a signature can't drift from its slot while both exist.

**A composite declares no state.** Its state belongs to whichever graph fills it: a Kalman tracker has state, and a pass-through filter has none. Leaf operators declare their state from step 4 on, when the first leaf is lowered.

## Not in this step

- **Faces, and leaf operators' targets.** `index_space` names a leaf's computational domain, but which ports are its faces, and the targets it has implementations for, arrive in step 4 with the first leaf.
- **Banded, block and sparse structures.** They need parameters (bandwidths, block sizes, a pattern) and have no message to describe yet. They come with the first operator that needs them.
- **Binding symbolic extents from config.** A name like `n` is carried, but binding it from the vehicle or baseline file is step 2's generator.
- **Triggers, regions and the graph format.** Those are step 2.

## Acceptance

1. **The core vocabulary** is in `flow/domain.hpp` and `flow/signature.hpp` as above, with no dependency beyond the standard library.
2. **Every invariant D1–S2 has a compile-fail test** that expects its message (tests/compile_fail, as `compile_fail.time_base.narrow_wide` does). The compile-fail tests build in the one build tree, so they hold a CTest lock and run one at a time; the time base's test takes the lock too.
3. **Every message in `mobility/msg/` has a `domain_of`.** Its tests check each struct against its domain: each array's size equals its extent's size or bound, and the covariance's 225 elements equal the matrix's `max_elements_v`.
4. **The four neutral composites' signatures** each match their slot concept's parameter and return types, by test.
5. **Each new header compiles alone** against its own library (the header check), and no core header names a mobility type.
6. **No behavior change:** every existing test passes, unchanged, and no existing source file changes. `tests/CMakeLists.txt` changes to register the compile-fail tests.
7. **Builds and tests:** GCC and Clang with no warnings, debug and release, with every test passing. MSVC builds and tests in CI, which also checks the C++20 class-type template parameters (`fixed_string`).
8. **Docs:** this plan; ADR 0004's current state; records in the model's section 3; the repository layout and the documentation map.

## Progress

| Step | PR | State |
|---|---|---|
| ADR 0004 accepted; this plan | [#41](https://github.com/branes-ai/anatomy/pull/41) | merged |
| The vocabulary, the mobility domains and signatures, and the tests | [#43](https://github.com/branes-ai/anatomy/pull/43) | in review |
