---
status: in-progress
issue: branes-ai/anatomy#89
---
# Plan: the workbench, and the MLP, Kalman and DSP patterns (ADR 0005, step 1)

Issue: [anatomy#89](https://github.com/branes-ai/anatomy/issues/89).

## Goal

This is step 1 of [ADR 0005](../../adr/0005-application-tier.md). It adds a workbench in anatomy where the graph machinery is developed on graph patterns rather than on missions. The workbench is a driver, three patterns and their oracles, with no vehicle, world, simulator or scorer. It runs on the C++ builder and the CPU stack of [ADR 0004](../../adr/0004-domain-flow-composition.md), step 2 ([plan](../flow-graph/README.md)).

Until now, every feature of the graph machinery was pulled by a mission and accepted on byte-equality with a mission's slot stack. The flow tests ([plan](../flow-testing/README.md)) added oracles of their own, but on toy operators over a time and an unsigned integer. So the parts of the [domain flow model](../../arch/domain-flow-model.md) that matter most to the compiler have never run:

- domains with real shapes and a structure: a 32 × 16 matrix, a symmetric covariance;
- constants with values: an MLP's weights, a filter's taps;
- an element type that differs from one edge to the next;
- complex elements;
- any rate rule but `hold`.

This plan adds what the first three patterns need of those, and the patterns themselves:

- **in the core** (`branes::anatomy`):
  - `flow::data<D>`, which stores the value of a domain;
  - complex element types;
  - the `accumulate` rate rule, through the builder, the stack, the canonical form, the Mermaid view, the reference interpreter and the mutations;
- **the workbench** (`workbench/`, `branes::anatomy_workbench`): the driver, its synthetic sources, and three patterns with their oracles:
  - **the MLP:** two layers in different element types, with weights as constants;
  - **the Kalman filter:** constant velocity, with a symmetric covariance as state, predicting on a clock and updating on a bounded measurement;
  - **the DSP spectral pipeline:** a FIR, `accumulate` into frames, a window, an FFT and a magnitude;
- **`workbench_run`:** a tool that runs a pattern, reports its oracle, and writes its trace and its graph. It is the first graph tool that needs no mission.

**What doesn't change:**
- the builder's API for what exists today, and the invariants G1–G9, except G7, which admits `accumulate` beside `hold`;
- the canonical form and the hash of every graph that has no `accumulate` edge, so every mission's `composition_hash`;
- the log format, the trace format, and every gate.

ADR 0004, decision 7 holds: racing-drone's and scout-drone's outputs must not change.

**The workbench is not linked by any mission.** Its operators are reference implementations, written plainly to be checked, not to be fast (ADR 0005, decision 1). It isn't installed or exported, so no mission can link it.

## What the tests must establish

Each property has an id, which the tests name.

| Id | Property |
|---|---|
| W1 | **Fidelity:** each pattern's stack writes, bit for bit, what its operators write when called by hand in the order the graph declares, on the same inputs. The oracle is exact: it shows that the graph runs what it says |
| W2 | **Correctness:** each pattern's outputs meet its oracle, to the tolerance the pattern states. This shows that the operators compute what the pattern means |
| W3 | **Determinism:** a pattern's run, with its seed, writes byte-identical traces run to run. Across element types, and across GCC, Clang and MSVC, the outputs agree with the oracle to its tolerance, not byte for byte |
| W4 | **Shapes and structure:** `data<D>` stores `stored_elements_v<D>` elements; a symmetric domain's `(i, j)` and `(j, i)` are one element; a bounded extent's count never exceeds its bound. An edge between domains of different shapes, structures or element types doesn't compile (G4) |
| W5 | **Constants with values:** a constant's value reaches every firing that reads it, unchanged, and isn't copied per firing |
| W6 | **`accumulate`:** when its region begins, a reader sees, oldest first, the values its producer wrote since that region last began: the latest N of them, and how many were dropped. Before the producer first writes, it reads an empty window. `flow::stack` and the reference interpreter agree on random graphs with `accumulate` edges |
| W7 | **The checks refuse what `accumulate` breaks:** an `accumulate` edge within a region or from a constant is refused, naming G7. A capacity of zero doesn't compile |
| W8 | **No allocation once built** holds with `accumulate` edges and with the patterns' messages |
| W9 | **The missions are unchanged:** every existing graph's canonical form and hash are unchanged, and racing-drone's and scout-drone's outputs are byte-identical to their main's |

## Model

### Domains that run: `flow::data<D>`

Step 1 of ADR 0004 made domains types, and the missions' messages give a domain to their own structs (`domain_of`). A pattern's edges are shaped values: a 16-vector, a 32 × 16 matrix, a symmetric 4 × 4 covariance, 256 complex samples. Writing a struct for each would be noise. `flow/data.hpp` adds the storage of a domain:

```cpp
namespace branes::anatomy::flow {

// The value of domain D, whose element type is arithmetic, complex, an enum or Timestamp.
template <class D>
struct data {
    using domain = D;
    std::array<typename D::element_type, stored_elements_v<D>> elements{};

    // A bounded leading extent's count; its bound otherwise.
    [[nodiscard]] constexpr std::size_t count() const noexcept;
    // A bounded leading extent: sets the count if n is at most the bound. A static extent:
    // accepts only its size. Otherwise returns false and leaves the count unchanged.
    [[nodiscard]] constexpr bool resize(std::size_t n) noexcept;

    // Row-major for dense. Symmetric and upper triangular: packed, the upper triangle row by row.
    // Lower triangular: the lower triangle, row by row.
    // Diagonal: the diagonal.
    [[nodiscard]] constexpr E& operator()(std::size_t i) noexcept;                  // rank 1
    [[nodiscard]] constexpr E& operator()(std::size_t i, std::size_t j) noexcept;   // rank 2
    // const access to an element outside a triangle or a diagonal reads zero
};

template <class D> struct domain_of<data<D>> { using type = D; };
}
```

- A message of type `data<D>` is its own domain. Two ports that carry `data<D>` match on domain and on message type, and G4 refuses a port of any other shape, structure or element type at compile time.
- The storage is a `std::array` of fixed size, so a `data<D>` holds nothing on the heap, and a slot of it allocates once, when the stack is built.
- **Bounded extents:** only the leading extent may be bounded, and `count()` is the number of its rows in use. The count is private, and `resize(n)` refuses a count above the bound, so the bound holds for every `data<D>` (W4). A bounded value starts empty (count 0), and rows past the count hold value-initialized elements, so two values with the same rows in use compare equal. A domain with no bounded extent has its full count, and `resize` accepts only that. More than one bounded extent is refused by a `static_assert`, until a pattern needs it.
- **Structure:** `(i, j)` of a symmetric matrix is `(j, i)`: one stored element. Writing outside a triangle is a precondition violation, checked in debug builds.
- A record's element type has no C++ type to store, so `data<D>` takes domains of the element types above only. A message that is a record stays a struct with its `domain_of`, as the missions' are.

The layout is fixed here because step 3's document and step 6's lowering will read it. It is the layout a CPU kernel sees, not one a target must use. The model's layouts are the lowering's (section 4).

### Complex elements

`std::complex<float>`, `std::complex<double>` and `std::complex<long double>` become element types (`element_traits`, in `flow/domain.hpp`). The DSP pattern's FFT is their first client.

### The `accumulate` rate rule

`hold` gives a reader the latest value its producer wrote. The DSP pattern needs the other rule the model names for edges between regions of different rates (section 8): **accumulate**, an accumulator domain. Here, the producer's region runs once per sample and the reader's once per frame of N samples.

```cpp
// In the frame region, read the samples the sample region wrote since the frame region last began.
auto [w] = flow::fire<"apply">(frame, window_op, flow::accumulate<256>(filtered));
```

- **What the reader reads:** `flow::window<M, N>`. It holds the values written since the reader's region last began, oldest first: `items` (a `std::array<M, N>`), their `count`, and `dropped`, how many older values didn't fit. Its domain is `record<field<"items", vector<E, bounded<N>>>, field<"dropped", scalar<std::uint64_t>>>`, where `E` is the element type of `M`'s domain if that is a scalar, or `M`'s record. A message whose domain has a shape is refused, until a pattern needs it.
- **When:** the stack latches each accumulator as the reader's region begins, and empties it for the producer. So every firing of the region reads the same window, and the window doesn't depend on the order of firings within the region. Before the producer first writes, the window is empty: `count` and `dropped` are 0, as a `hold` reads its message value-initialized (flow-testing, P4).
- **Overflow:** a producer that writes more than N values between two reader regions keeps the latest N and counts the rest in `dropped`. That is deterministic, allocates nothing, and lets the reader see what it lost. The DSP pattern's driver writes exactly N per frame, and its oracle requires `dropped == 0`.
- **In the graph:** `Ref` gains `rate_rule::accumulate` and a `capacity`. `canonical()` writes ` accumulate <N>` where it writes ` hold`, and nothing for any other edge, so the canonical form and the hash of every graph without an `accumulate` edge are unchanged (W9). `mermaid()` draws the edge dotted, labelled `accumulate N`.
- **G7** now reads: an edge between regions has a rate rule, `hold` or `accumulate`; an edge within a region, or from a constant, has none.
- **Each `accumulate` edge has its own accumulator.** One out port read by two regions fills two windows.

The model's other uses of an accumulator, such as IMU samples integrated between frames, are an operator applied to the window, not another rule. `queue` stays for when a pattern or a mission needs it.

### The workbench

```text
workbench/
├── include/branes/anatomy/workbench/
│   ├── driver.hpp      # sources at declared rates, merged into one stream; runs a pattern's stack
│   ├── sources.hpp     # a clock, a sequence, a noisy constant-velocity trajectory, a sum of tones
│   ├── codecs.hpp      # trace codecs for data<D> and window<M, N>
│   ├── pattern.hpp     # what a pattern provides (below)
│   └── patterns/       # mlp.hpp, kalman.hpp, dsp.hpp: operators, bindings, graph, oracle
└── apps/workbench_run/
```

`branes::anatomy_workbench` is an interface library over the core. It is built when `ANATOMY_BUILD_WORKBENCH` is on, which defaults to `ANATOMY_BUILD_TESTS`, and it is not installed or exported. Its tests are in `tests/workbench/`.

**A pattern** provides:

- its name;
- its configuration, a struct with defaults: shapes, rates, parameters, a seed;
- its operators, registered under names that carry their element type and shape (`Linear_f32_32x16`);
- its graph: a template over its builder, as a mission's is, so `flow::builder` checks it and `flow::stack_builder` runs it;
- its sources;
- its hand-wired run (W1);
- its oracle (W2): given the configuration and the outputs, a report of each check, with its value, its tolerance, and whether it passed.

The operators are built from a workbench context that holds the configuration, with `from_context`, as a mission's are.

**The driver** merges the sources into one stream in time order, hands each input to the stack, and collects the outputs after each. Two inputs at the same time go in the order the pattern declares its sources: the fixed order between simultaneous triggers that the model requires (section 8). The noise is drawn from `std::mt19937_64`'s raw output, mapped by the workbench's own code, as the flow tests' generator does, because the standard distributions differ between GCC, Clang and MSVC.

### The patterns

**The MLP.** `y = W₂·tanh(W₁·x + b₁) + b₂`, with `x` a 16-vector, a hidden layer of 32, and an output of 8.

- Layer 1 is `f32` and layer 2 is `f64`, joined by a `Cast` operator, so the element type is per edge.
- The weights and biases are constants with values, drawn from the seed.
- Its operators are `Linear` (`y = W·x`), `BiasAdd`, `Tanh` and `Cast`, each a template over its element type and shape. One region, triggered by each input `x`.
- **Oracle:** a direct evaluation in `long double`, written without the operators. The output agrees with it to a relative 1e-5, which layer 1's `f32` sets. Layer 2, given layer 1's output, agrees to 1e-12. The operation count in closed form, `32·16 + 8·32` multiply-adds plus the epilogues, is written beside the pattern for step 5's cost record to check against.
- **A compile-fail test:** an `f32` vector into an `f64` port doesn't compile (G4).

**The Kalman filter.** Constant velocity in the plane: state `x = (px, py, vx, vy)`, covariance `P` a symmetric 4 × 4 `data`.

- `Kalman` is one operator with four handlers. Two update its state: `predict`, a time handler fired by a 100 Hz clock, and `update`, fired by `fixes`. Two read it: `prior` and `posterior`.
- `fixes` is a measurement of up to 4 position fixes with a count (`data<matrix<double, bounded<4>, dim<2>>>`), arriving at 10 Hz on clock ticks, and sometimes with none.
- At a time shared by the clock and the fixes, the clock goes first, so the update applies to the predicted state.
- The update is in Joseph form. `prior` runs in the clock region after `predict` and writes `x` and `P` to the `prior` output. `posterior` runs in the fixes region after `update` and writes them to the `posterior` output, which the oracle's checks of the update read.
- The signature declares `x` and `P` as its state domains.
- **Oracle:**
  - `P` is positive definite after every firing: its Cholesky factorization succeeds;
  - with one fix per update, `P` after the update converges to the steady-state solution of the discrete Riccati equation, which the oracle iterates in `long double`, to a relative 1e-9 within 500 updates;
  - over fixed seeds, the normalized estimation error squared (NEES) of 2000 updates averages within its 95 % chi-square bounds;
  - the `f32` and `f64` instantiations agree to 1e-4 relative.

**The DSP spectral pipeline.** Samples at 8192 Hz, frames of N = 256.

- The **sample region**, fired by each sample, runs a band-pass FIR of 63 taps. Its delay line is its state, and its taps are a constant with values: a windowed sinc, designed from the configuration when the graph is built.
- The **frame region**, fired by a frame tick at every 256th sample's time, after the sample, reads `accumulate<256>` of the filtered samples. It then runs a Hann window (a constant), a cast to `complex<double>`, a radix-2 FFT and a magnitude.
- Its outputs are the spectrum (256 complex bins) and the magnitude of the first 129 bins.
- The source is a sum of tones at bin centers, two in the pass band and one in the stop band, plus seeded noise.
- **Oracle:**
  - every frame's window has `count == 256` and `dropped == 0`;
  - the largest magnitudes are at the pass-band tones' bins;
  - each pass-band tone's magnitude is within 1 % of `A·|H(f)|·Σw/2`, with `H(f)` evaluated directly from the taps;
  - the stop-band tone's magnitude is within 1 % of the same prediction, which the design puts at least 40 dB down;
  - Parseval holds between the windowed frame and the spectrum, to a relative 1e-12.

A first frame that begins before 256 samples have arrived isn't possible with this driver. W6's tests cover short and overfull windows on the toy graphs.

### `workbench_run`

```text
workbench_run <mlp|kalman|dsp> [--inputs N] [--seed S] [--trace run.brtrace] [--mermaid g.mmd] [--canonical g.txt]
```

`workbench_run` runs a pattern with its default configuration and prints its oracle's report. It writes the trace (with the workbench's codecs) and the graph's Mermaid or canonical form when asked. It exits 0 if every check passes, 1 if one fails, and 2 on a usage error, as `trace_compare` does. `trace_compare` reads its traces, so two runs, or two element types, can be compared edge by edge. The graph host, which runs a graph document without a rebuild, is step 4's. This tool runs only the three patterns compiled into it.

## Acceptance

Six PRs, each accepted on its own criteria. ADR 0005 was accepted with this plan's PR (B0).

**B0, this plan.** The plan, its design choices confirmed, and ADR 0005 accepted.

**B1, core: `flow::data<D>` and complex elements.**
1. `flow/data.hpp`, and complex element types in `flow/domain.hpp`.
2. Tests of each structure's storage size, index mapping and symmetric access, and of a bounded extent's count (W4).
3. Compile-fail tests: an edge between `data` of different shapes, of different structures, and of different element types (W4).
4. Only headers are added, so no mission's build changes.

**B2, core: `accumulate`.**
1. `flow::accumulate<N>`, `flow::window<M, N>`, `rate_rule::accumulate` and `Ref::capacity`. Also the stack's accumulators, `canonical()`, `mermaid()` and G7.
2. The reference interpreter and the generator of the flow tests draw `accumulate` edges, and `flow_differential` runs them (W6).
3. Tests of an empty window, a short window, an overfull window with its `dropped`, and two readers of one out port (W6). A counting-allocator test with `accumulate` edges (W8).
4. `flow_mutations` refuses an `accumulate` within a region and from a constant, naming G7 (W7), and a compile-fail test refuses a capacity of zero.
5. A test that the canonical forms and hashes of `test_mission.hpp`'s graphs and of 200 generated graphs without `accumulate` equal main's, which are recorded as goldens in this PR before the change (W9).
6. racing-drone's and scout-drone's `tools/compare_outputs.sh`, built against this branch: byte-identical to their main's (W9).

**B3, the workbench and the MLP.**
1. `workbench/`, `ANATOMY_BUILD_WORKBENCH`, the driver, its sources and codecs, and the MLP pattern.
2. `workbench_mlp` (`tests/workbench/`): W1, W2 and W3 on the MLP, and W5, a constant read by reference by every firing.
3. `workbench_run mlp`, with a test that runs the tool on each exit code.

**B4, the Kalman filter.** The pattern, and `workbench_kalman`: W1, W2 and W3. B4 has the first operator with two handlers that update its state in two regions, fired by two triggers at shared times.

**B5, the DSP spectral pipeline.** The pattern, and `workbench_dsp`: W1, W2 and W3. B5 is the first client of `accumulate` and of complex elements.

**B6, docs.** This plan's progress, ADR 0005's current state, the repository layout, the documentation map, and a guide to the workbench: how to run a pattern, read its graph and trace, and add one.

**In every PR:** GCC and Clang with no warnings, debug and release, and every test passing; MSVC in CI. The workbench's tests run in under 10 s in a debug build.

## Design choices

To be confirmed with this plan's PR.

1. **The workbench is a directory and a library of its own,** not part of `tests/`. Step 4's graph host and step 5's evaluation command will link it, and `workbench_run` already does. It isn't installed or exported, so ADR 0005's rule holds: no mission links a reference operator.
2. **Two oracles per pattern, one exact and one mathematical.** The hand-wired run (W1) is bit-exact on the same compiler, so any difference is a fault in the graph's execution, as in the flow tests. The closed-form oracle (W2) is to a tolerance, and judges the operators. A failure says which of the two it is.
3. **`data<D>` is in the core.** A pattern's edges need it now, and step 3's document, step 6's lowering, and a mission's first shaped operator (a camera's pixels, ADR 0004) will need one storage, not one per repo. The missions' message structs stay as they are.
4. **`accumulate` latches when the reader's region begins, and drops the oldest on overflow.** Latching makes every firing of the reader see one window, whatever their order. Dropping the oldest and counting them is deterministic and allocates nothing, and the reader can tell that it lost values. The alternative, refusing to run, would make a run's result depend on whether a rate assumption held, with no record of it.
5. **No interpreter support for the patterns.** The reference interpreter knows the toy operators, and gains `accumulate` on them (B2). A second implementation of an FFT or a Kalman update in the interpreter would be a third oracle. The closed-form one is a stronger check, and the hand-wired run already checks the graph's execution.
6. **Tolerances, not byte-identity, across compilers.** Floating point differs between GCC, Clang and MSVC, and between their libms. The missions' determinism gates are byte-exact on one compiler, and so is W3 run to run. Across compilers, the oracle's tolerance is the check.

## Not in this plan

- **The six other patterns** (attitude, PID, convolution, stencils, SOR/SSOR, QR) and **the computational domain of a leaf**: step 2.
- **The graph document, the reader, the graph host, the cost record and the evaluation:** steps 3 to 5. `workbench_run` runs only compiled patterns.
- **The `queue` rate rule, conditional regions, and iteration to convergence.**
- **Checking an operator's declared state domains against its handlers' state roles.** The Kalman filter declares its state, but nothing checks the declaration, which is step 3's to carry in the document.
- **Any change to a mission**, other than the gate in B2.
- **Performance.** The reference operators are written to be checked, not to be fast.
- **Optical flow, a ResNet18 CNN, and a transformer (a ViT or DETR).** These are the patterns closest to an Embodied AI pipeline's perception, and they follow in a step of their own, with an amendment to ADR 0005, whose table doesn't list them. They need three things step 1 doesn't have:
  - **A subgraph defined once and used many times.** A ResNet is a residual block, repeated per stage with its own shapes and weights, and stages of blocks; a transformer is an encoder layer repeated N times. Today a body is a C++ type that fills one composite of a scoreboard graph. What these patterns need is a subgraph with ports, which a graph instantiates as often as it needs, nested, with its own constants, in a research graph or a pattern as well as in a composite. The checks must see through it: an instance's edges, state and rate rules are checked as if it were written out, and the canonical form, the trace and the Mermaid view name each instance's firings by its path (`stage2.block1.conv1`). This is the feature these patterns are the client for.
  - **Tensor domains of rank 3 and 4,** for images and feature maps. `data<D>` here stores rank 1 and 2. The convolution pattern of step 2 brings rank 3 and the sliding-window computational domain, and these patterns build on it.
  - **Operators and weights from synapse.** ResNet18 and a transformer come with GGUF weights, and their kernels are synapse's on MTL5. The workbench's operators are reference implementations that no mission links (ADR 0005, decision 1). Whether the workbench may link synapse's operators, or registers its own reference ones and checks synapse's against them, is the amendment's to decide. Optical flow is the bridge between the two: a classical pattern (Lucas–Kanade over an image pyramid) whose oracle is a known translation or rotation of a synthetic image.

## Progress

| Step | PR | State |
|---|---|---|
| B0: this plan | [#90](https://github.com/branes-ai/anatomy/pull/90) | merged |
| B1: `flow::data<D>` and complex elements | [#91](https://github.com/branes-ai/anatomy/pull/91) | open |
| B2: `accumulate` | | |
| B3: the workbench and the MLP | | |
| B4: the Kalman filter | | |
| B5: the DSP spectral pipeline | | |
| B6: docs | | |
