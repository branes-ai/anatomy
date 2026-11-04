---
status: in-progress
issue: branes-ai/anatomy#64
---
# Plan: testing the flow graph beyond equality with the slot stack

Issue: [anatomy#64](https://github.com/branes-ai/anatomy/issues/64).

## Goal

Steps 2 and 3 of [ADR 0004](../../adr/0004-domain-flow-composition.md) were accepted on one oracle: a graph build's logs equal its slot build's, except `composition_hash` ([plan](../flow-graph/README.md)). The oracle is exact, and it shows that the graph runs today's code faithfully. But it can only judge graphs that a slot stack can also express. Every mission graph today:

- fills each composite with a body of one operator;
- reads across regions with `hold` only;
- logs its external outputs only, not the edges between its operators;
- is a scoreboard graph. Only anatomy's test mission (`tests/fixtures/flow/test_mission.hpp`) has a research graph and a body of several operators, and it checks each against the same operators wired by hand.

So the slot stack can't be the oracle for what the graph adds, and step 4's export will read the checked `flow::Graph` value, which no test runs. This plan adds tests whose oracle isn't the slot stack:

- **a reference interpreter** that runs the checked `flow::Graph` value directly, and random graphs over toy operators, which `flow::stack` and the interpreter must run to the same records;
- **mutations** of valid random graphs, each breaking one invariant, which the builder must refuse, naming the invariant;
- **metamorphic tests:** changes to a graph that must leave its records unchanged;
- **a trace of every edge,** and a comparison that names the first edge two runs differ on;
- **a client:** a racing-drone baseline whose composite has a body of several operators, and a research baseline, each checked against the same operators wired by hand, and traced.

**What doesn't change:** the builder's API, the invariants, the stack's behavior, the log format, and every existing gate. The trace is a separate file, written only when asked for. A test that finds a bug in the builder or the stack fixes it in its own PR, which states the behavior it changes.

## What the tests must establish

Each property has an id, which the tests name.

| Id | Property | Tested today |
|---|---|---|
| P1 | **Execution:** for any checked graph and any input stream, `flow::stack` writes what the graph says: each region's firings in their order, each in port read from its `Ref`, and each `hold` the value its region last wrote | by equality with the slot stack, on the missions' graphs |
| P2 | **The graph value is complete but for its values:** the checked `flow::Graph`, with the operators' definitions and the constants' values, determines the outputs. The export relies on the first two; the constants come from the baseline, as parameters do | no |
| P3 | **Determinism:** the same graph and inputs give the same records, run to run and across GCC, Clang and MSVC | by the missions' determinism gates |
| P4 | **`hold` before its first write** reads the message's value-initialized state, and that is the documented behavior | no |
| P5 | **State order:** a read of an operator's state before its update in a region sees the previous firing's state, and one after it sees the new state | by the builder's G5 check; not run |
| P6 | **The checker is complete for its invariants:** a graph that breaks G1, G3 (an output's producer), or G5–G8 is refused, and the error names the invariant and the node or firing | one hand-written graph per invariant |
| P7 | **Refactorings that preserve meaning preserve the records:** a composite's body against the same operators in a research graph; reordering firings that share no value and no state; renaming nodes. The hash changes in each case, and the records don't | no |
| P8 | **Attribution (ADR 0004, decision 6):** two scoreboard baselines that differ in one composite's body first differ on an edge that composite's body writes | no |
| P9 | **No allocation once built:** after `build_stack()`, running the stack allocates nothing, for messages of fixed size | claimed; not tested. A message that holds a `std::vector` may allocate when `input()` copies it into a slot whose capacity is smaller |

G2 and G4 are checked by the compiler, and stay covered by the compile-fail tests (`tests/compile_fail/`). G9 stays tested by value.

## Model

### Toy operators

The random graphs are built from a small set of operators in `tests/fixtures/flow/toys.hpp`. Each is a registered operator with its own signature and binding, over messages that are a time and a `std::uint64_t`, so that arithmetic is exact and defined on overflow, with no floating point:

| Operator | Handlers | State |
|---|---|---|
| `Unary` | `apply(x) -> y`: a fixed mix of x and the operator's salt | none |
| `Binary` | `apply(x, y) -> z` | none |
| `Counter` | `update(x)` adds x to its sum; `read() -> sum` | update, read |
| `Clock` | `at(t) -> y`, a time handler | none |
| `Pair` | `apply(x) -> (y, z)`, two out ports | none |
| `Rec` | `apply(x) -> r`, a record of two fields, so that a source can project one | none |
| `Acc` | `add(x)` and `scale(x)`, two updates of one sum. For the mutations only: the generator doesn't draw it | update, update |

Each operator is a template over a salt, registered once per salt (`Unary1`, `Unary2`, ...), so that two operators of one kind compute different functions, and an edge read from the wrong instance shows up in the records. The salt is in the operator's registered name, which the `Graph` holds, so it needs no context: the stack builder builds every toy operator from an empty one.

### The reference interpreter

`tests/fixtures/flow/interpreter.hpp` runs a checked `flow::Graph` directly. It takes the graph and the constants' values, which the `Graph` doesn't hold: the test hands the interpreter the values its recipe gave the builder. It keeps every value in a map from `(region, firing, port)`, from an input's name and from a constant's name. On each input it walks that input's region: for each `Firing`, it resolves each `Ref`, projects its path, calls the toy operator, and its salt, that the `Instance` names, and stores the outputs. It implements the toy operators again, from the table above, without including their headers, so a bug shared by the operators and their bindings isn't hidden. It is slow and dynamic on purpose. It knows the toy operators only, and runs no mission's graph.

That the interpreter needs nothing but the `Graph`, the operators' definitions and the constants' values is property P2. T1 found one more fact it needs: the `Graph` names its regions' triggers, but not its inputs, so the interpreter takes an input's region to be the one whose trigger is the input's name. That holds for an input declared with no trigger of its own, as every generated input is.

### The generator

`tests/fixtures/flow/generate.hpp` makes a random valid graph from a seed, as a **recipe**: a plain description of the inputs, constants, regions, instances and firings, and of each in port's source. A test builds the recipe with `flow::stack_builder`, so the same recipe gives the stack and the `Graph` the interpreter reads. The generator:

- draws 1 to 4 inputs, each the trigger of one region or of none, a tick among them, and 0 to 2 constants;
- fires, in each region, 1 to 8 handlers of 1 to 6 instances, each in port's source drawn from the region's inputs, the constants, the region's earlier firings' out ports, or `hold` of another region's out port;
- respects G5 and G6 by construction: an instance's state is updated at most once per region, and every handler of every instance fires somewhere;
- draws research graphs, and scoreboard graphs whose composites have bodies drawn from a fixed set of bodies of one to three toy operators. Bodies are types, so the set of bodies is fixed, and only the wiring is drawn.

The randomness is `std::mt19937_64`'s raw output, mapped to ranges by the generator's own code, because the distributions of the standard library differ between GCC, Clang and MSVC. A failing case prints its seed and its recipe, as text, so it can be rerun.

### Mutations

A mutation changes one thing in a valid recipe so that it breaks one invariant:

| Invariant | Mutations |
|---|---|
| G1 | an empty name; a name that isn't `[a-z_][a-z0-9_]*`; two nodes, or a node and an external port, with one name |
| G3 | an external output with no producer, or with two |
| G5 | a second update of an instance's state in one region: `Acc`'s two, since firing one handler twice is G6 |
| G6 | a second region for a trigger; a time handler fired at an input of another region; a handler that never fires; a handler fired twice at one input |
| G7 | an edge between regions with no rate rule; a rate rule on an edge within a region, or from a constant |
| G8 | an instance outside a body in a scoreboard graph; a composite in a research graph |

The builder must refuse each mutated recipe, and its `ConfigError` must name the invariant and the node or firing the mutation changed. A mutation the builder accepts, or refuses for another reason, fails the test. A mutation that needs what the recipe lacks adds it first, validly: a node and its firing, an input with no region, or a constant. So each seed gets every mutation, but two nodes of one name, which needs two nodes.

G8's other half is the baseline file's, which `check_baseline()` checks against a built graph, not the builder. Those mutations change the file, not the recipe: a file that names a body for a composite the graph doesn't have, or none for one it has, or the wrong class. `check_baseline()` must refuse each, against the valid graph built from the recipe, naming G8 and the composite.

### Metamorphic pairs

Each pair is two recipes from one seed, whose records must be equal and whose hashes must differ:

1. **A body against a research graph:** a scoreboard recipe, and the research recipe that places the bodies' operators as instances of their own, with the same wiring.
2. **Independent firings reordered:** two adjacent firings in a region that share no value and no instance, swapped.
3. **Renamed:** every instance and every region's out port renamed.

A swap of two firings of one instance where one reads its state and the other updates it is checked the other way: the builder accepts both orders, and the records may differ. That's P5, which the interpreter checks.

### The edge trace

The stack gains an optional observer, set before it runs. After each firing, the stack hands the observer the firing's region, its index, the time, and its out ports. `flow::trace` writes these as a separate file, `<run>.brtrace`, beside the log, whose records are the region, the firing, the port, the time, and the message's bytes as the log's codec encodes them. A port whose message has no codec keeps its region, firing, port and time, and leaves out only the bytes, so `trace_compare` still names it, and compares its time. Every toy message has a codec, so the differential test compares every edge's value.

`trace_compare` (`apps/`), like `log_compare`, reads two traces and reports the first edge they differ on, by its `region.firing.port`, or that they are equal. It exits 0, 1 or 2, as `log_compare` does.

The trace is a debugging and testing tool. The log, its format, and every gate stay unchanged, and a stack with no observer runs as it does now.

### The client

racing-drone gains two baselines, built only as graphs:

- **a scoreboard baseline whose controller is a body of two operators,** a setpoint shaper and `CascadedPid`, beside `b0-gt-pid`, whose controller is `CascadedPid` alone;
- **a research baseline** with the same operators placed between the mission's external ports, which is marked as research.

The oracle for each is a test that wires the same operators by hand, in the order the graph declares, and compares the logs. The attribution property P8 is checked on the traces of `b0-gt-pid` and the scoreboard baseline: they must first differ on an edge inside the controller's body.

## Acceptance

Six PRs, each accepted on its own criteria. T1 to T4 are anatomy's, T5 is racing-drone's, on the anatomy release that has T4, and T6 is the docs in both.

**T1, anatomy: the interpreter and the differential test.**
1. The toy operators, the interpreter and the generator (`tests/fixtures/flow/`).
2. `flow_differential` (`tests/sdk/`) runs a fixed set of seeds, at least 500 graphs with 1000 inputs each, through the stack and the interpreter, and requires equal outputs after every input (P1, P2). It runs in under 30 s in a debug build. `ANATOMY_FLOW_SEEDS` sets a longer run, which CI doesn't run.
3. A test for P4, and the stack's documentation of it.
4. A test for P5: a read before and a read after an update, in one region, against the interpreter.
5. A test for P9, with a counting allocator around a running stack of fixed-size messages.

**T2, anatomy: mutations.**
1. The mutations above, applied to each of a fixed set of seeds' recipes, at least one per row of the table per seed (P6).
2. Each recipe mutation refused by the builder, with its invariant named, and the node or firing.
3. Each baseline-file mutation refused by `check_baseline()`, against the valid graph, naming G8 and the composite.

**T3, anatomy: metamorphic pairs.**
1. The three pairs, for a fixed set of seeds (P7).
2. The swap of a read and an update, checked against the interpreter.

**T4, anatomy: the edge trace.**
1. The stack's observer, `flow::trace`, the `.brtrace` file, and `trace_compare`.
2. A unit test for each `trace_compare` outcome, and a test that runs the tool on each exit code.
3. A test that a stack with an observer writes the same outputs as one without, and that the differential test's traces equal the interpreter's values for every edge.

**T5, racing-drone: a body of several operators and a research baseline.**
1. The setpoint shaper, registered as an operator, and the two baselines above, built as `sitl_<name>_graph`, `replay_<name>_graph` and `sitl_batch_<name>_graph`.
2. Their logs on every scenario equal to the hand-wired stacks' logs, as a gate.
3. The determinism gate for each.
4. `--trace <run.brtrace>` on the SITL binary, and the attribution test (P8).
5. Every existing baseline's outputs byte-identical to main's (`tools/compare_outputs.sh`).

**T6, docs.** This plan's progress, ADR 0004's current state, and racing-drone's docs on the new baselines and the trace.

**In every PR:** GCC and Clang with no warnings, debug and release, and every test passing; MSVC in CI.

## Design choices

To be confirmed with this plan's PR.

1. **The interpreter, not a second stack, is the oracle.** It must be different from the stack in every way that could share a bug. It reads the `Graph` value, which the stack doesn't, keeps values in a map, and reimplements the operators. A second compiled stack would share the builder's thunks and slots.
2. **Toy operators over integers.** Floating-point operators would bring rounding into every comparison, and the differences between compilers that the determinism gates already cover. Integers make each comparison exact, so any difference is a fault in the graph's execution.
3. **Random recipes, not random calls of the builder.** A recipe is data, so a failing case is printed and replayed, mutated one field at a time, and shrunk by hand. The builder is driven from it, so the API under test is the one missions use.
4. **The trace is a separate file.** Putting edges in the log would change the log format and every log comparison. A trace is written only when asked for, and the log stays the record a run is judged by.
5. **The client is racing-drone,** because B0 is its graph build, and racing's scoreboard is the one a body of several operators would join. Scout-drone follows when a scout baseline needs one.

## Not in this plan

- **`accumulate` and `queue` rate rules, declared state domains (ADR 0004, decision 4), and conditional regions.** Each is a feature with its own plan. The interpreter is written to take a rate rule at a time, so each gains its oracle with it.
- **The export for the domain flow compiler** (step 4). The `Graph` value names no edge's domain, which the export needs, and holds no constant's value, and no input or its trigger. T1's interpreter checks that it is otherwise complete (P2).
- **A flight build that refuses a graph reading truth (SYS-02).** No mission has a flight target yet.
- **Performance.** The cost of the stack's indirect calls against the slot stack's direct ones is worth measuring, but isn't a correctness property, and gates nothing here.

## Progress

| Step | PR | State |
|---|---|---|
| This plan | [#65](https://github.com/branes-ai/anatomy/pull/65) | merged |
| T1: the interpreter and the differential test | [#67](https://github.com/branes-ai/anatomy/pull/67) | open |
| T2: mutations | | open |
| T3: metamorphic pairs | | |
| T4: the edge trace | | |
| T5: racing-drone's baselines | | |
| T6: docs | | |
