# ADR 0004: missions compose operators over domains in a dependence graph, not slots

**Status:** Proposed
**Date:** 2026-11-01
**Related:** [the domain flow computational model](../arch/domain-flow-model.md) (the model this ADR adopts); [racing-drone ADR 0001](https://github.com/branes-ai/racing-drone/blob/main/docs/adr/0001-baseline-composition-framework.md) (the slots, which this would supersede in part); [racing-drone ADR 0008](https://github.com/branes-ai/racing-drone/blob/main/docs/adr/0008-anatomy-mission-framework.md) (change the core only with a client in hand); [ADR 0001](0001-core-profiles-multi-node.md) (the core, the profiles and the multi-node layer)

## Context

Anatomy composes a mission's stack from **slots** (racing-drone ADR 0001). A slot is a C++20 concept for a component role, templated on a scalar type `T`. A baseline names one implementation per slot, and each mission's hand-written `Stack` class wires the slots into a fixed pipeline. The design kept baselines comparable and the hot path free of virtual dispatch, and both missions run on it today.

It doesn't reach the hardware Branes.AI builds for. A mission capability has to run on a distributed data path, such as the KPU's compute fabric, whose instruction set is made of high-level transformations: linear algebra and constraint solvers (LU, QR, Schur, SVD), and stateful operators such as Kalman filters and ODE solvers. A compiler that maps a capability onto such a fabric needs three things the slot design does not provide:

1. **The dependences between steps, as a graph.** Bufferization and scheduling depend on the read/write dependences being explicit. In a slot stack they are calls inside `Stack::on_tick`.
2. **The size, shape and structure of every input, state and output.** Dividing and conquering an operator that is larger than the fabric, and allocating its buffers, depends on them. A slot's ports are typed by the scalar `T`, which is only the element type of the aggregate actually passed.
3. **State as data.** A Kalman covariance kept on the fabric, or a stationary weight matrix, has to be allocated by the compiler. A slot keeps its state in its own members.

The word "slot" also hides what the parts are: positions in a fixed pipeline, rather than transformations whose composition expresses a capability.

The options considered:

- **Rename the slots, and keep the design.** This clears up the vocabulary, but gives the compiler nothing.
- **A graph of components typed by `T`.** This makes the topology into data, but leaves out shapes, structure and state.
- **The domain flow model:** operators over domains, a dependence graph, and events, with the lowering left to a compiler and a runtime. This is the one chosen.

## Decision

1. **Anatomy adopts the [domain flow computational model](../arch/domain-flow-model.md) for composition.** A mission capability is a graph of **operators** over **domains**, joined by **dependences**, and driven by **triggers** and **notifiers**. It replaces slots as the unit of composition.
2. **Anatomy owns layers 1 to 4 of the model:** domain types, operator descriptors, the graph and the events. It must provide:
   - **domain types** with an index set, an element type per domain, a structure, and static or bounded symbolic shapes;
   - **operator descriptors** with ports (in, out, inout), declared state domains, and a computational domain where a leaf declares one;
   - **a graph format in the baseline file**, which names each operator (the algorithm, such as `CascadedPid`) and its parameters, the edges between them, and the triggers and regions. It must not name a target's implementation of an operator: that choice is the lowering's;
   - **a generator** that checks the graph at configure time: port domains match, every input is connected, every cycle crosses a carried edge, and every operator belongs to a region. It must emit the CPU schedule as static C++, with no virtual dispatch, one build target per baseline, as today;
   - **an export of the checked graph** for the domain flow compiler.
3. **Anatomy must not implement layer 5.** Choosing a leaf operator's implementation for a target, phases, layouts, schedules, placement, tiling, bufferization and the resource contract belong to the compiler and the runtime, which live outside anatomy. A baseline may express a pin, and anatomy passes it on, but a pin must be a hint, never a requirement.
4. **State must be declared.** An operator's state that persists between firings must be a declared state domain, not a hidden member. The CPU implementation may keep it in members, but the descriptor must declare it.
5. **Determinism stays a core invariant.** Carried edges are the boundaries between firings, so a region's graph without them is acyclic. Each firing runs its operators in a fixed topological order of that graph, and triggers that coincide have a fixed order, so a log replays to the same outputs on the CPU schedule, as it does today. Each edge can be logged.
6. **The change is made with both clients in hand.** Each step below is accepted only when racing-drone's and scout-drone's outputs are byte-identical to the slot stack's, as ADR 0008 requires.

### Steps

| Step | Repo | What |
|---|---|---|
| 1 | anatomy | Domain types and operator descriptors, alongside the slot concepts, with no change in behavior |
| 2 | anatomy, scout-drone | The graph format and the generator. scout-drone's `s0-gt-pid` re-expressed as a graph, with logs byte-identical to the slot stack's on every scenario and the batch |
| 3 | racing-drone | B0 re-expressed as a graph, checked the same way |
| 4 | anatomy | The graph export for the domain flow compiler, with a first leaf operator lowered onto the fabric |
| 5 | all | The slots retired: the hand-written `Stack` classes removed, racing-drone ADR 0001 superseded in part, and the docs moved to the new vocabulary |

## Consequences

- **The topology becomes data.** A new stage, such as a detector, a map or a fusion step, is a change to the graph plus an operator, not a new `Stack` class.
- **One baseline runs on every target.** The baseline is the same algorithm on the CPU, the KPU and a TPU, and only the lowering differs, so results stay comparable across targets.
- **Replay can check any edge,** not only the three outputs `replay --verify` compares today.
- **SYS-02 is enforced by the build.** Truth is an external port that only the SITL interface has, so a flight graph that reads it fails to build. `TruthConsumer` goes away.
- **The multi-node layer (ADR 0001, decision 4) gets its model.** An edge between nodes is a message in the inter-node envelope.
- **The cost is a generator that becomes a small compiler:** it parses graphs, checks domains and cycles, and orders regions. The invariants of the hand-written stacks (a valid field, ticks in order) move into the generated code or into the mission interface's contract.
- **The comparability ADR 0001 secured by fixing the slots** now depends on the mission interface and the scorer staying fixed while the graph varies. How much of the graph a baseline may change is an open question of the model (its section 11), and must be answered before step 2.
- **Every existing implementation survives** as a CPU implementation of an operator. `make_slot` and `from_context` remain how an operator is built from the mission's context.

**Current state:** proposed. No step has been started.
