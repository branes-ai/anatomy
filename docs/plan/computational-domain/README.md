---
status: draft
issue: branes-ai/anatomy#98
---
# Plan: the computational domain of a leaf, and the six other patterns (ADR 0005, step 2)

Issue: [anatomy#98](https://github.com/branes-ai/anatomy/issues/98).

## Goal

This is step 2 of [ADR 0005](../../adr/0005-application-tier.md). Step 1 ([plan](../workbench/README.md)) built the workbench and its first three patterns: the MLP, the Kalman filter and the DSP spectral pipeline. Their graphs run, but each node is still an operator's name and its handlers. What a handler computes is hidden in a C++ function, so a compiler, a cost model or an analysis can see only its ports.

Decision 2 of the ADR makes a leaf describe its compute: its computational domain, the dependences between its points, the faces that are its ports, its state and its structure. Anatomy captures this, checks it against the node's ports, and carries it in the document (step 3). It never writes the kernel. Today the core has a placeholder, `flow::index_space<Extents...>`. Nothing reads it, it has no dependences, and it isn't tied to the ports.

This plan adds:

- **in the core** (`branes::anatomy`, `flow/compute.hpp`):
  - the computational domain of a handler, a constexpr value in its operator's binding: an index space with extents bound from the ports, recurrence equations whose bodies are expressions, the faces where each port enters and leaves, and where the operator's state is read and written;
  - checks K1–K8, at compile time where they are cheap and by enumeration otherwise, each reporting the invariant broken;
  - the operation count derived from a declaration, which step 5's cost record will check against;
- **in the workbench:**
  - an evaluator that runs a declaration point by point, in any order its dependences allow. It is a third oracle beside W1 and W2: for every leaf, the declaration, evaluated on a run's inputs, gives what the handler gives;
  - the declarations of the leaves of the MLP, the Kalman filter and the DSP pipeline. The FFT's butterflies are the first dependences that aren't affine;
  - the six other patterns of the ADR's table, in its order. Each lands with the feature it exercises and an oracle in closed form:

    | Pattern | The feature it lands with |
    |---|---|
    | PID control | a **carried edge** in the graph, and a closed loop through it |
    | attitude filter | a **unit-norm** structure on a vector domain |
    | convolution | **rank-3** `data<D>`, and a sliding-window computational domain |
    | stencils | the **halos** of a divided computational domain, derived from its dependences |
    | SOR/SSOR | the **dependence depth** of a computational domain, derived, which says what can run in parallel |
    | QR | a **triangular** computational domain, and an `inout` port |

**What doesn't change:**
- the builder's API for what exists today, and the invariants G1–G9, except G7, which also admits a carried edge (C4);
- the canonical form and the hash of every graph without a carried edge, so every mission's `composition_hash`. A computational domain isn't part of the canonical form. It reaches the document in step 3, and the document's hash covers it;
- the log format, the trace format, and every gate;
- `flow::index_space` and `flow::no_index_space`. racing-drone's generated bridge re-exports them, so they stay until a mission regenerates its bridge against a release that marks them deprecated. Their removal is a later release's.

ADR 0004, decision 7 holds: racing-drone's and scout-drone's outputs must not change.

## What the tests must establish

Each property has an id, which the tests name. They continue step 1's W1–W9, which hold for every new pattern.

| Id | Property |
|---|---|
| W10 | **The checks hold:** every computational domain declared in the workbench passes K1–K7, and K8 classifies it as its pattern states. Each of K1–K7 refuses a declaration that breaks it, naming its id: a compile-fail test for K1 and K2, and a mutation test for K3–K7. K8 refuses nothing, so its test holds the toys' and the leaves' classifications to the expected ones |
| W11 | **Fidelity of the declaration:** for every leaf of every pattern, its declaration evaluated on the inputs of each of its firings in a run gives the handler's outputs and state. The tolerance comes from the arithmetic. It is exact where the declaration fixes the order of every operation the handler does |
| W12 | **Orders agree:** a declaration evaluated in its deterministic order and in 20 random orders that its dependences allow gives bit-identical results. Every allowed order computes the same operations on the same values, and the evaluator keeps a port's or a state's old and new values apart, so a difference is a fault in the evaluator. It is a sampling test of the evaluator. A read of a variable before it is written is reported as K3, not computed |
| W13 | **Operation counts:** the count derived from each leaf's declaration equals its closed form: `R·C` multiply-adds for `Linear`, 63 per sample for the FIR, `(N/2)·log₂N` butterflies for the FFT, and so on for each pattern below |
| W14 | **Carried edges:** a read through a carried edge sees what its producer wrote in the region's previous firing, and its message value-initialized before the first. `flow::stack` and the reference interpreter agree on random graphs with carried edges. The checks refuse a carried edge between regions, or from a constant or an input, naming G7 |
| W15 | **Divided domains:** the halo derived for a tiling of a stencil's domain is the reach of its dependences. The domain evaluated tile by tile, each reading only its tile and its halo, equals the undivided evaluation bit for bit |
| W16 | **Dependence depth:** the longest chain of dependences derived from a declaration equals its closed form: per sweep, `2(n − 1) + 1` for lexicographic Gauss–Seidel on an `n × n` grid, and 2 for red-black |
| W17 | **New storage:** a rank-3 `data<D>` stores its elements row-major, and the unit-norm structure stores what dense does. W4 holds for both |
| W18 | **The missions are unchanged:** every existing graph's canonical form and hash are unchanged, and racing-drone's and scout-drone's outputs are byte-identical to their main's |

## Model

### The computational domain of a handler

A firing is one call of a handler, and that is what a lowering turns into a hardware subroutine. So the computational domain belongs to a handler, not to its operator. The Kalman filter's `predict` and `update` compute different things over the same state, and its `prior` and `posterior` only copy it out.

A handler declares its computational domain as a `static constexpr` member `compute` of its handler struct, in the operator's binding. A handler without one is opaque, as every handler is today. A composite's handlers never have one: their compute is their body's.

The description follows the system of recurrences of domain_flow's `.sure` specifications (`domain_flow/docs/SURE`): an index space, variables defined by equations over it, inputs that enter on faces, and outputs that leave on faces. Anatomy's form stops before uniformization, which is the lowering's (design choice 3). A body may read a port at an affine function of the point, as `W(i, j)` and `x(j)` below, rather than pipelining `x` across the rows on a face, as `gemv.sure` does. The domain_flow compiler takes the form here and uniformizes it, projects it and schedules it.

`Linear`, `y = W·x` with `W` of `R × C`:

```cpp
struct apply : fl::handler<"apply", fl::ins<"w", "x">, fl::outs<"y">> {
    static vec<E, R> call(const Linear<E, R, C>&, const mat<E, R, C>& w, const vec<E, C>& x);

    static constexpr fl::cd::system compute = [] {
        fl::cd::system s;
        const auto rows = s.extent("w", 0);                         // bound from the ports
        const auto cols = s.extent("w", 1);
        const auto i = s.index("i", 0, rows);                       // 0 <= i < rows
        const auto j = s.index("j", 0, cols);                       // 0 <= j < cols
        const auto acc = s.var("acc", fl::cd::etype_of<E>);
        s.init(acc, s.literal(0, fl::cd::etype_of<E>), s.face(j == -1));   // the reduction's seed
        s.eq(acc, acc(i, j - 1) + s.in("w")(i, j) * s.in("x")(j));
        s.write(s.out("y").at(i), acc(i, j), s.where(j == cols - 1));     // y leaves on the face j = cols − 1
        return s;
    }();
};
```

This is the spelling C1 settled (`flow/compute.hpp`). What a `system` holds is fixed here, because step 3's document and step 6's lowering read it:

- **Parameters,** each bound to an extent of a port or a state, or to an integer. One declaration serves every instantiation of its operator: the `Linear` above describes both `Linear_f32_32x16` and `Linear_f64_8x32`.
- **Counts:** a bounded extent has two parameters, its bound (`s.extent`) and its count (`s.count`). The count is known only at run time: the index space over a bounded extent runs to its count, and the checks enumerate every count from 0 to the bound (the Kalman filter's fixes). A bounded out port or state declares its count as an affine expression of parameters, `s.out("y").count(s.count("x"))`, and its output face writes only the rows below it. The evaluator sets the count before it writes the rows, so an empty input gives an empty output, as `CastFixes` gives one today.
- **The index space:** named indices, each with affine bounds in the parameters and the indices before it, plus further affine constraints. That admits a box, a triangle (QR, `j ≤ i`) and a band.
- **Variables and equations:** `v(p) = e` over the whole space, or over a subdomain given by affine constraints. `e` is an expression of literals, parameters, reads, and the operations below. A variable read at a point outside the subdomain where it is defined must be given there by an `init`: a literal, a port's element, or a state's.
- **Reads,** each with its index function:
  - **uniform:** a variable at the point plus a constant offset, `acc(i, j − 1)`;
  - **affine:** a variable or a port at an affine function of the point, `x(j)`, or `a(i, k, k − 1)` as in `cholesky.sure`;
  - **non-affine:** an index function that uses `xor`, shifts or `mod` of the point, such as the FFT's butterfly partner `p xor 2ˢ`. It is allowed, and the description marks it, so the compiler knows the system isn't a SURE (design choice 6).
- **Faces:**
  - **an input face** is where a port or a state is read: the set of points that read it, and the index function from a point to the element read;
  - **a record** is read and written by field path: `s.in("w", "items")(k)` and `s.in("w", "dropped")()` for the window `Framer` reads. A parameter binds an extent or a count of a field path in the same way, `s.count("w", "items")`. A field is a domain with faces of its own, so each check treats it as a port;
  - **an output face** is where a port or a state is written: `s.out("y")(i) = s.on(j == C − 1, …)` writes `y(i)` from the points of the face `j = C − 1`;
  - **an `inout` port or a state** has both. Its input faces read its value before the firing, and its output faces write its value after it. The two are kept apart: no write is seen by a read of the same firing. A lowering that updates the port or the state in place adds its own orderings: each write after every read of the old value it overwrites. It reads them from the description, which names every element each read and write touches.
- **Operations:** `+`, `−`, `×` and `÷`; `sqrt`, `tanh`, `exp`, `abs`; a comparison and `select`; complex multiply, conjugate, real and imaginary parts and magnitude; and `cast` to an element type. Each operation is counted by kind for W13. A transcendental is one operation of its kind, not an expansion. An operation the description lacks is added with the first pattern that needs it.
- **Operator parameters:** a scalar or a table that the operator holds from its context. Examples are the Kalman filter's `q` and `r`, and the FFT's twiddle factors. A declaration names it, `s.param("twiddle")(k)`, and the evaluator reads it from the operator through a hook the binding provides. Step 3 records the values in the document, as it records constants'.

A handler whose work is a sequence of steps, such as the Kalman update in Joseph form (`S = H·P·Hᵀ + R`, then the gain, then `x` and `P`), declares a **program**: systems in order, joined by named intermediate domains. Each system's output faces write an intermediate that a later system's input faces read. The intermediates are internal to the handler. A lowering may fuse the systems or keep them apart (the model's section 5: a body of leaf operations inside one leaf).

### The checks

Like G1–G9, the checks report the invariant broken and where: the operator, the handler, the variable or face, and for an enumerated check the first point that breaks it. K1 and K2 are `static_assert`s on the declaration and its binding, which `fl::registered` instantiates. K3–K8 enumerate the bound index space, so they run when a test or a tool asks for them (`fl::cd::check<Op, "handler">()`), not in every build. The workbench's tests run them on every declared handler (W10), and `workbench_run --check` runs them on a pattern's.

| Id | Invariant |
|---|---|
| K1 | **Names:** the indices and variables of a system have distinct, non-empty names. Every port and state it names is the binding's, and the handler's `ins` and `outs` include each port it reads or writes. An `in` port is only read, an `out` port is only written, and an `inout` port or a state is read before it is written. Each read's arity is its target's rank |
| K2 | **Parameters:** each parameter is bound to an integer, or to an extent of a port or state the system names, of a rank that has it |
| K3 | **Every read is defined:** at every point of the space, every variable read is defined there by an equation at a point the dependences order before it, or by an `init` |
| K4 | **Reads stay inside their ports:** every port, state or intermediate read, at every point that reads it, is at an index inside its domain. For a bounded extent, that means inside its count: the check runs for every count from 0 to the bound, so a read is accepted only if it is guarded for each one |
| K5 | **Every output is written exactly once:** every element an out port or an intermediate stores is written by exactly one point of its output faces. An `inout` port, and a state the handler writes, have each element written at most once, and an element no point writes keeps its old value. A state the handler only reads has no output faces, and K5 doesn't apply to it (K7). For all of these, no point writes an index outside its destination. A declared count lies in `[0, bound]` for every count of the inputs, or it is refused before any row is checked. For a bounded extent, the rows checked are those below the declared count. For a symmetric or triangular domain, it means once per stored element, and nothing outside the stored triangle |
| K6 | **The dependences are acyclic:** the dependence graph has a node for each definition at a point: an equation's, an `init`'s, or an output face's write. Every read of a variable or an intermediate adds an edge from the definition it reads. So a face that reads `acc(i, j)` at the point whose equation defines it depends on that equation, not on the point. A read of a port or a state sees its value before the firing, so it adds no edge. The graph is acyclic, so some order evaluates every definition after every one it reads. No schedule is declared (design choice 5) |
| K7 | **State and state roles:** a handler whose state role is `update` writes every state it reads through its output faces. A handler whose role is `read` writes none, and one whose role is `none` names none. This is the check of declared state against state roles that step 1 left out |
| K8 | **Classification:** each read is classified as uniform, affine or non-affine, and so is each system: a SURE if every read is uniform, otherwise affine or non-affine. A classification is reported, not refused. A test holds each pattern's leaves to the class it states |

### What the description gives

These are derived from a checked declaration, and each is checked against a closed form:

- **operation counts** by kind, summed over the space: the count `fl::cd::count` (W13). Step 5's cost record checks a measured record's operations against it. Step 1's MLP plan recorded `32·16 + 8·32` multiply-adds beside the pattern, and this is where that number is derived;
- **the dependence depth:** the longest chain of dependences through the space, and the number of points at each depth. It is the critical path a parallel schedule can't beat, and the parallelism at each step (W16);
- **the halo of a tiling:** for a division of the index space into boxes, the region each tile reads outside itself, from the reach of its dependences (W15).

### The evaluator

`workbench/evaluate.hpp` runs a declaration on the values of a firing's in ports, its time input for a time handler, its state and its operator's parameters. It returns the out ports and the new state. It enumerates the points in an order, evaluates each equation in the system's element types, and records every variable in a dense store over the space. That makes it slow, but these are reference implementations of small domains.

- **The order** is a topological order of the dependence graph, whose nodes are definitions at points (K6). The deterministic one takes, among the definitions whose reads are all written, the one at the lexicographically least point first, and those at one point in the order they are declared. So a backward sweep (SSOR) runs backward. A random one takes any of them, from a seed. A read of a variable not yet written is K3, reported at the point where it is made.
- **Old and new values:** an `inout` port and a state have two stores, the value before the firing, which their input faces read, and the value after it, which their output faces write. An element no point writes is copied from the first to the second.
- **It is an oracle of the declaration, not of the operator** (design choice 7). W11 compares it with the handler. W12 compares it with itself across orders. The dependences come from the reads, so every order honors them, and each point computes the same operations on the same values. Since no read sees a write of the same firing to a port or a state, no order can change a result, and W12 samples orders to check that the evaluator keeps to this.
- **Every pattern's tests run it** on the inputs of every firing of a short run. The trace records a firing's out ports after it runs, so it can't supply the operator's state before a firing, and the trace format is unchanged. Instead, the pattern's run by hand (W1), which calls each handler directly, takes a hook that is called before each handler. The hook receives the handler's in-port values (inputs, constants and windows included), its time, and a copy of its operator.

### Carried edges (C4)

The PID pattern closes a loop: the controller reads the plant's output from the previous tick, and the plant reads the controller's command from this one. The model's section 6 calls the edge that closes it a **carried edge**: from an output in one firing of a region to an input of the next firing of the same region. Today a `Ref` names an earlier firing of its own region, or a firing of another region through a rate rule, so a cycle can't be written.

```cpp
auto [u] = flow::fire<"step">(tick, pid, setpoint, flow::carried(y));  // y from the last tick
auto [y] = flow::fire<"step">(tick, plant, u);
```

- **What the reader reads:** the value its producer wrote in the region's previous firing. Before the first firing, it reads the message value-initialized, as a `hold` does (flow-testing, P4).
- **In the graph:** `rate_rule::carried`. `canonical()` writes ` carried` where it writes ` hold`, so the form and hash of a graph without a carried edge are unchanged (W18). `mermaid()` draws the edge dotted and backward, labelled `carried`.
- **G7** reads: an edge between regions has a rate rule, `hold` or `accumulate`; an edge within a region has none, or `carried`; an edge from a constant or an input has none. A carried edge may name its own firing or a later one, which is what lets it close a cycle. Every cycle crosses a carried edge, because every other `Ref` of a region names an earlier firing.
- **The stack** keeps one slot per carried edge, written at the end of the region's firing. The reference interpreter, the generator and the mutations of the flow tests gain carried edges (W14).

A carried edge isn't an operator's state. The PID's integrator and last error are its state, declared and carried inside it as the FIR's delay line is. The carried edge is the loop between two operators, which the graph can see.

### The patterns

Each pattern has a header in `workbench/patterns/`, the interface of `pattern.hpp`, a run by hand (W1), an oracle (W2), a determinism test (W3), tests in `tests/workbench/`, and an entry in `workbench_run`. Each of its leaves declares its computational domain (W10–W13).

**The first three, declared (C2, C3).**
- **MLP:**
  - `Linear` is a reduction along `j`, a SURE but for its affine reads of `W` and `x`, with `R·C` multiply-adds.
  - `BiasAdd`, `Tanh` and `Cast` are elementwise: their space is their data domain, and every read is at the point. The model's section 4 fuses such an operator onto a drain. K8 classifies it, so the lowering can find it.
- **Kalman filter:** the first programs.
  - `predict` is two systems: `x ← F·x` and `P ← F·P·Fᵀ + Q`. `dt` comes from the time input and `t_last`, the time of the last predict, which the handler writes back. `t_last` is a member today, so the binding declares it as a third state beside `x` and `P`. The guard (no predict when `dt ≤ 0`) is a `select`.
  - `update` is a program over the fixes' bounded extent: one fix at a time, as the handler does, in Joseph form.
  - `prior` and `posterior` copy the state out, so their output faces write the out ports and nothing writes the state (K7).
  - Its counts are stated in closed form for a 4-state filter with one fix.
- **DSP:**
  - `Fir` is a 63-point reduction whose taps are a port and whose delay line is a state. Its input face is the previous delay line, and its output face writes the line shifted by one.
  - `Framer`, `Hann`, `ToComplex` and `MagnitudeOp` are elementwise.
  - `Fft` has a space of `log₂N` stages by `N` points, with a non-affine read of each butterfly's partner and a twiddle factor from its table. It has `(N/2)·log₂N` complex multiplies and `N·log₂N` complex adds. It is the first non-affine system (K8).

**PID control (C4).**
- A PID controller and a first-order plant `y' = (K·u − y)/τ`, discretized exactly, as two operators in one region on a 100 Hz clock. The loop is closed through a carried edge.
- The controller's integrator and its last error are its declared state.
- **Oracle:**
  - with the integral and derivative gains zero, the closed loop's step response is the first-order discrete recurrence in closed form, to 1e-12;
  - with the integral term, the steady-state error is below 1e-9 after 20 time constants;
  - the loop's poles, computed from the gains, predict the overshoot to 1 %.

**The attitude filter (C5).**
- A quaternion `q`, `data<vector<double, dim<4>, structure::unit_norm>>`, integrated from gyro samples at 400 Hz and corrected toward gravity from the accelerometer at 50 Hz. It is a complementary filter, with a gain from the configuration.
- `structure::unit_norm` is new: a vector whose norm is 1, stored as dense, with the norm checked in debug builds against a tolerance the domain's element type sets. D2 admits it on rank 1.
- **Oracle:**
  - a constant rate integrates to the known rotation `exp(½·ω·t)`, to 1e-9 after 1 s;
  - from a tilted start, a static attitude converges to gravity at the rate the gain predicts;
  - the norm stays 1 to 1e-12 after every firing.

**Convolution (C6).**
- A 2D convolution of a 3 × 32 × 32 input with 8 kernels of 3 × 3, then a bias and a ReLU, in one region.
- `data<D>` gains rank 3, row-major.
- The convolution's space is `(o, y, x, c, ky, kx)`, a sliding window: its read of the input at `(c, y + ky, x + kx)` is affine, and its input face is a window into the padded input.
- **Oracle:**
  - direct summation in `long double`, to 1e-12;
  - an impulse input returns each kernel;
  - the count is `8·32·32·3·9` multiply-adds.

**Stencils (C7).**
- The 5-point Jacobi step on a 64 × 64 grid, the 7-point step on a 16³ grid, and an explicit heat-equation step. Each is one region per step, with the grid carried through a carried edge.
- The halo derivation (above), and the evaluator run on a divided domain:
  - each tile reads its tile and its halo from the previous step;
  - the tiles run in any order;
  - their writes are joined.
- **Oracle:**
  - the lowest eigenmode of the grid decays by its known factor per step, to 1e-12;
  - the halo of a 16 × 16 tiling is one point on each side;
  - the divided evaluation equals the undivided one bit for bit (W15).

**SOR/SSOR (C8).**
- Successive over-relaxation on the 2D Poisson problem with a manufactured solution. It is done in two orders, lexicographic and red-black, each with a forward sweep and a forward-then-backward (SSOR) sweep. There is a fixed number of sweeps per firing, since iteration to convergence is out of scope.
- One sweep is a system over `(i, j)`. In lexicographic order it reads its new west and south neighbours and its old east and north ones. Red-black is a system over `(color, i, j)`. The dependences, not a schedule, say what runs in parallel, and the derived depth shows it (W16).
- **Oracle:**
  - the error against the manufactured solution falls to the discretization error;
  - at the optimal ω, the residual's rate per sweep agrees with the spectral radius `ω_opt − 1` to 5 %;
  - the depths are as W16 states.

**QR (C9).**
- A Householder factorization of a 32 × 16 matrix, then a least-squares solve. `A` is an `inout` port, overwritten by `R` and the Householder vectors.
- Its space is triangular: `0 ≤ k < n`, `k ≤ i < m`, `k ≤ j < n`. It is the first space with affine bounds in another index. The back substitution is a triangular solve.
- **Oracle:**
  - `QᵀQ = I` to 1e-12, and `R` upper triangular exactly;
  - `QR = A` to 1e-12;
  - the least-squares solution of a system with a known solution, to 1e-10;
  - the count against the exact count of the Householder algorithm as written: a polynomial in `m` and `n`, stated with the pattern, whose leading terms are `2mn² − (2/3)n³`.

Every new pattern runs in `f64`, with `f32` where its oracle says what it costs. The per-edge precision loop is step 5's.

### `workbench_run`

It gains the six patterns, plus two flags:
- `--check` runs K3–K8 on the pattern's declarations.
- `--compute file.txt` writes each leaf's declaration as text: one block per handler, with its parameters bound, its space, its equations, its faces, its classification and its counts. The text is for reading. It isn't a format anyone parses, since the document is step 3's.

## Acceptance

Eleven PRs, each accepted on its own criteria.

**C0, this plan.** The plan, with its design choices confirmed.

**C1, core: the computational domain and its checks.**
1. `flow/compute.hpp`:
   - the description;
   - K1 and K2 as `static_assert`s through `fl::registered`;
   - `fl::cd::check` for K3–K8;
   - `fl::cd::count` and the depth. The halo lands with its client, the stencils (C7);
   - the text form.
2. Toy declarations in `tests/sdk/`: a reduction, an elementwise map, a triangular space, a program of two systems, a state and a non-affine read. Each is checked, counted and printed.
3. Compile-fail tests for K1 and K2, a mutation test that breaks each of K3–K7 in a valid declaration and checks the id reported, and K8's classification of each toy against the expected one (W10).
4. `workbench/evaluate.hpp` and its tests on the toys (W11, W12).
5. racing-drone's and scout-drone's `tools/compare_outputs.sh`, built against this branch, are byte-identical to their main's (W18).

**C2, the MLP's and the Kalman filter's declarations.** Each leaf's declaration, W10–W13 on both patterns, and `workbench_run --check` and `--compute`.

**C3, the DSP pipeline's declarations.** W10–W13, with the FFT as the first non-affine system.

**C4, carried edges and PID.**
1. The core: `flow::carried`, `rate_rule::carried`, the stack's slots, `canonical()`, `mermaid()` and G7.
2. The flow tests: the interpreter, the generator, the mutations and `flow_differential` (W14). A counting-allocator test with carried edges (W8).
3. The canonical forms and hashes of `test_mission.hpp`'s graphs and of 200 generated graphs without carried edges are equal to main's, which are recorded as goldens before the change.
4. The missions' gate (W18).
5. The PID pattern, with W1–W3 and W10–W13.

**C5, the attitude filter.** `structure::unit_norm` (W17), the missions' gate, and the pattern.

**C6, convolution.** Rank-3 `data<D>` (W17), the missions' gate, and the pattern.

**C7, stencils.** The halo of a tiling, the divided evaluation (W15), and the pattern.

**C8, SOR/SSOR.** The pattern, with its depths (W16).

**C9, QR.** The pattern.

**C10, docs.** This plan's progress, ADR 0005's current state, the workbench guide (declaring a computational domain, reading `--compute`, and the six patterns), the repository layout and the documentation map.

**In every PR:**
- GCC and Clang, debug and release, with no warnings and every test passing, and MSVC in CI;
- a PR that changes the core runs the missions' gate.

The workbench's tests run in under 20 s in a debug build. The enumerated checks and the evaluator run on small runs, not on every firing of the oracle's runs.

## Design choices

To be confirmed in this plan's PR.

1. **A computational domain belongs to a handler, and is declared in the binding.** A firing is one call of a handler, and so is what a lowering turns into a hardware subroutine. One operator's handlers can compute different things, as the Kalman filter's do. Declaring it beside the handler keeps it next to the code it describes. The signature's `index_space`, one per operator and with no dependences, is superseded, and is removed once no mission's bridge names it.
2. **The declaration is a constexpr value, not a type and not text.**
   - Domains are types because G4 compares them at compile time.
   - A computational domain is compared with nothing; it is checked and carried. As a value, step 3 can serialize it directly, and its enumerated checks don't instantiate templates per point.
   - Text in the `.sure` language would need a parser in anatomy, and would tie anatomy to a form that is uniformized for a fabric (choice 3).
   - The cost is a fixed capacity for indices, variables and expression nodes, with a `static_assert` when a declaration exceeds it.
3. **Affine recurrences, not uniformized ones.** Anatomy states what each point computes, which points it reads, and where each port enters and leaves. The domain_flow compiler's work is to pipeline a broadcast operand along a free axis, add a feed axis, and choose a projection and a schedule (`gemv.sure`'s commentary). That is layer 5, which ADR 0004 decision 3 keeps outside anatomy. The faces here are where data enters and leaves, not the fabric's boundary.
4. **Bodies are expressions.** The evaluator, the counts, and later the document and the compiler read one description. If bodies were C++ callables, the evaluator could run them, but the counts would be declared by hand and the document couldn't carry the compute. The operation set grows with the patterns.
5. **No schedule is declared.** K6 checks that one exists. Choosing one is the lowering's, and the model's section 9 puts it there. The `.sure` files declare `tau` because they are lowering inputs.
6. **Non-affine dependences are recurrences, marked.** This answers the ADR's open question for the FFT. Its butterflies are written as a recurrence over `(stage, point)`, with a non-affine index function, rather than named as a library kernel. Every check and the evaluator treat it the same way, and K8 tells the compiler it isn't a SURE. A compiler may still recognize the butterfly and use a kernel it has. QR's Householder updates are affine in a triangular space and don't need this.
7. **The evaluator is a third oracle, and it lives in the workbench.** W1 shows that the graph runs its operators, W2 that the operators compute what the pattern means, and W11 that the declaration describes the operator. Without W11, a declaration is metadata nothing checks. The evaluator is a reference implementation, as the patterns' operators are, so it stays out of the libraries a mission links. The checks and the counts are in the core, because step 3 writes and step 5 reads them.
8. **A carried edge is a rule on a `Ref`, within one region.** It is the model's section 6 edge, and the PID pattern is its client. Between regions, `hold` and `accumulate` already serve. Before its first write it reads its message value-initialized, as `hold` does, so a pattern that needs an initial condition other than zero sets it in an operator's state.

## Not in this plan

- **The graph document, the reader, the graph host, the cost record and the evaluation:** steps 3 to 5. The declarations are carried in the document in step 3.
- **A `.sure` writer and a lowering:** step 6. The text form of `--compute` is for reading.
- **Declarations for the missions' operators.** Cortex's, reflex's and synapse's operators stay opaque until a mission needs one lowered. The mission interface is unchanged.
- **Iteration to convergence, conditional regions, and `queue`.** SOR/SSOR runs a fixed number of sweeps.
- **Varying a bounded shape or a symbolic extent at run time:** an ADR open question, which the document's step decides.
- **Optical flow, ResNet18 and a transformer, and the reusable subgraphs they need:** a step of their own (step 1's plan).
- **Performance.**

## Progress

| Step | PR | State |
|---|---|---|
| C0: this plan | [#99](https://github.com/branes-ai/anatomy/pull/99) | merged |
| C1: the computational domain and its checks | [#100](https://github.com/branes-ai/anatomy/pull/100) | merged |
| C2: the MLP's and the Kalman filter's declarations | | |
| C3: the DSP pipeline's declarations | | |
| C4: carried edges and PID | | |
| C5: the attitude filter | | |
| C6: convolution | | |
| C7: stencils | | |
| C8: SOR/SSOR | | |
| C9: QR | | |
| C10: docs | | |
