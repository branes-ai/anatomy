---
status: in-progress
issue: branes-ai/anatomy#47
---
# Plan: the graph format, the generator and the baseline classes (ADR 0004, step 2)

Issue: [anatomy#47](https://github.com/branes-ai/anatomy/issues/47).

## Goal

This is step 2 of [ADR 0004](../../adr/0004-domain-flow-composition.md). It adds layers 3 and 4 of the [domain flow model](../../arch/domain-flow-model.md) in anatomy: a mission's stack becomes a graph described in data, which a generator checks and turns into the CPU schedule. It builds on step 1's domain and signature types ([plan](../flow-domains/README.md)). It adds:

- **a mission interface file:** a mission's external ports, its composites with their signatures, the connections between them, and the regions and their triggers;
- **a graph format in the baseline file:** the baseline's class, and either a body for each composite (scoreboard) or a whole graph between the external ports (research);
- **a generator,** which checks a baseline's graph against the mission interface and emits the stack as static C++, with no virtual dispatch, one build target per baseline, as today;
- **the two baseline classes,** with the scoreboard check of ADR 0004, decision 6.

**scout-drone is the first client.** Its `s0-gt-pid` and `null` baselines are re-expressed as graphs and built beside today's slot stacks. ADR 0004, decision 7, accepts a step only on both clients' outputs:
- **scout-drone:** the graph builds' logs match the slot builds' on every scenario and the batch;
- **racing-drone:** its slot baselines, built against the anatomy that contains step 2, write logs byte-identical to its main's. racing-drone gains no graph until step 3, so step 2 must leave it unchanged.

**What doesn't change:**
- the slot concepts, the `Stack` classes, `composition_generate()` and the slot baselines' targets, which stay until step 5;
- racing-drone, until step 3;
- the log format and the profiles.

## Model

A mission's stack is a graph of **operators**. Each operator is fired by **handlers**, each handler by a **region**, and each region by a **trigger**. What the generator reads is in YAML. What it checks against is C++ types.

### The mission interface

Each mission has one interface file, `mission/interface.yaml`. It names C++ types and bindings as strings, and the generated code checks them (G4, below). Scout's would read:

```yaml
mission: scout
profile: branes::scout::ScoutProfile          # the harness contract, unchanged
external:                                     # the external ports
  in:
    imu:       { message: branes::anatomy::ImuSample,          trigger: imu }
    camera:    { message: branes::anatomy::CameraFrame,        trigger: frame, time: header.t_mid }
    obstacles: { message: branes::scout::ObstacleObservations, trigger: obstacles }
    truth:     { message: branes::anatomy::TruthState,         trigger: truth, sitl_only: true }
    tick:      { trigger: tick }                               # carries only its time
  constant:                                   # fixed when the stack is built
    field:      { message: branes::scout::Field, from: field }
    detections: { message: branes::anatomy::IntruderDetections }   # empty: no producer yet
  out:
    estimate:  { message: branes::anatomy::StateEstimate, region: tick, from: estimator.estimate }
    reference: { message: branes::anatomy::Reference,     region: tick, from: planner.reference }
    command:   { message: branes::anatomy::ThrustRates,   region: tick, from: safety_filter.safe_command }
composites:                                   # in the order the stack builds them
  estimator:
    signature: branes::scout::flow::estimator_signature
    binding: branes::scout::flow::EstimatorBinding
    handlers:
      imu:       { in: [imu], state: update }
      frame:     { in: [camera], state: update }
      obstacles: { in: [obstacles], state: update }
      truth:     { in: [truth], state: update }
      estimate:  { out: [estimate], time: true, state: read }
  daa_tracker:
    signature: branes::anatomy::flow::daa_tracker_signature
    binding: branes::anatomy::flow::DaaTrackerBinding
    handlers:
      update:    { in: [detections, estimate], state: update }
      conflicts: { in: [estimate], out: [conflicts], state: read }
  # planner, controller, safety_filter and fc_bridge alike
regions:                                      # each a trigger, its firings in the declared order
  imu:
    - estimator.imu: [imu]
  frame:
    - estimator.estimate@camera
    - estimator.frame: [camera]
    - daa_tracker.update: [detections, estimator.estimate]
  tick:
    - estimator.estimate@tick
    - planner.plan: [field, {from: obstacles, rate: hold}, estimator.estimate]
    - daa_tracker.conflicts: [estimator.estimate]
    - controller.step: [estimator.estimate, planner.reference, daa_tracker.conflicts]
    - safety_filter.filter: [estimator.estimate, controller.command, daa_tracker.conflicts, field.geofence]
    - fc_bridge.send: [safety_filter.safe_command, tick]
  # obstacles and truth alike
```

The grammar, as 2a settled it:

- **A composite declares its handlers.** Each handler is one function of the composite's binding. It names its in and out ports, which are port names of the signature, in the binding's order; whether it takes a time (`time: true`); and what it does with the operator's state (`state: read` or `update`, ADR 0004, decision 4). The generator can't see C++, so it checks the graph on these names, and the compiler checks the names and their domains against the signature (G4).
- **A region is a trigger,** and lists its **firings** in the order they run. A firing is `<composite>.<handler>`, with one source per in port: `- estimator.frame: [camera]`. A handler that takes a time names the input whose time it takes: `estimator.estimate@camera` reads the estimate at the frame's time (`time: header.t_mid`).
- **A source** is an input, a constant, or another firing's out port: `camera`, `field`, `estimator.estimate`. It may project a record's field: `field.geofence`. Composites and external ports share one namespace, so a source names one thing.
- **A source written in another region** carries a rate rule: `{from: obstacles, rate: hold}`, and the planner reads the latest obstacles seen. Step 2 implements `hold` only. `accumulate` and `queue` are refused until a mission needs them.
- **The order of a region's firings** is a topological order of its flow edges: a source written in the region is written by a firing declared before its reader. A read of state before an update is declared by putting it first. The estimate at the frame's time is read before the estimator takes the frame, and the DAA gets that prior, as `Stack::on_frame` does today. Between independent firings, the order is the order of declaration. So the generated order is fixed, and today's `Stack` order can be written down exactly.

The structure is YAML rather than a line syntax such as `a <- (x, y)`, since YAML splits a flow list on its commas and reads `x: hold` as a mapping.

### The baseline file

The file gains two keys, `class` and `graph`. A slot baseline keeps its `composition:` block, and the loader accepts all three, so in step 2 one file holds both forms of a baseline. The loader keeps the graph as text, and `flow::load_graph()` reads and checks it against the interface and the registry.

```yaml
name: s0-gt-pid
class: scoreboard
composition: { ... }                       # the slot form, unchanged until step 5
graph:
  estimator: GroundTruthEstimator          # a body of one operator, the shorthand
  planner: GoalPlanner
  controller: CascadedPid
  safety_filter: NullSafetyFilter
  daa_tracker: NullDaaTracker
  fc_bridge: NullFcBridge
parameters: { ... }
```

**A scoreboard baseline** gives a body for each composite. A body is one operator, as above, or a subgraph of operators whose external ports are the composite's ports. A subgraph gives, for each of the composite's handlers, the firings that implement it and the producer of each of its out ports. A firing's source is one of the handler's in ports, or an earlier firing's out port:

```yaml
  controller:
    operators: { outer: PositionLoop, inner: RateLoop }
    handlers:
      step:
        fire:
          - outer.step: [estimate, reference]
          - inner.step: [estimate, outer.rates, conflicts]
        out: { command: inner.command }
```

**A research baseline** gives `operators:`, `regions:` and `out:` between the mission's external ports, in the interface's grammar, with operator instances in place of composites. Step 2 builds the research path and tests it on a test mission in anatomy. scout-drone ships no research baseline yet.

### Operators and bindings

- **An operator** is a C++ class registered under a name with its header and type, as `composition_register_impl()` does today. It is registered in one of two ways:
  - `flow_register_operator(<registry> <Name> HEADER <h> TYPE <t> COMPOSITE <c>)`: the operator fills composite `c`, and takes its signature, binding and handlers. A single-operator body binds its ports by name to the composite's.
  - `flow_register_operator(<registry> <Name> HEADER <h> TYPE <t> SIGNATURE <s> BINDING <b> HANDLERS <...>)`: an operator of its own, with its own signature (`signature_of`, step 1), binding and handlers. It can be placed in a research graph, or in a scoreboard body that has more than one operator.

  A research graph may use operators of either kind. An operator that fills a composite keeps that composite's signature and binding wherever it is placed. The registry reaches the generator as a YAML file (`operators:`, one entry per operator), which `flow::load_registry()` reads.
- **A binding** is the CPU calling convention of a composite's operators. It is a C++ struct with one static function per handler, for example `EstimatorBinding::frame(est, f)` calls `est.process_frame(f)`. Every implementation of a slot concept has the same methods, so a binding is written once per composite, from its slot concept. The slot concepts survive as calling conventions until step 5.
- **State.** A handler declares whether it reads or updates its operator's state (ADR 0004, decision 4). A composite declares no state domain (step 1); its handlers declare their state role here, so the generator can check that each read is ordered before or after the update.

### The generator

**A C++ host tool, `anatomy_flowgen`, built from anatomy with yaml-cpp.** A baseline's target runs it as a build step: it reads the mission interface, the baseline file and the operator registry, checks the graph, and writes `<build>/generated/<name>/flow_baseline.hpp`. The generated header defines the baseline's name, its graph hash, its class, whether it is SITL-only, and `Graph<T>`: a class satisfying `RunnableStack` with the mission's profile. Its `on_<trigger>()` functions are the regions, in the checked order, calling the bindings, and its `create(field, ctx)` builds each operator with `make_slot`, as `Stack::create` does.

**The checks split by where the facts live.** The graph's structure is checked by the tool, before anything compiles. The domains are C++ types, so they are checked by the C++ compiler, in the generated header, with a `static_assert` per edge that names the edge.

**The graph hash** is SHA-256 of the checked graph's canonical form. It is computed in C++ only, by the same code at build time and at startup, so CMake no longer duplicates the hash, as `composition_generate()` does today.

## Invariants

The loader checks G1–G3 and G5–G8 and refuses a graph that breaks any of them, with a message that names the file, the key and the invariant. G4 is a `static_assert` in the generated header. G9 is derived, not refused: the checked graph says whether it is SITL-only.

| Id | Invariant |
|---|---|
| G1 | names are non-empty, `[a-z_][a-z0-9_]*`, and distinct per kind (composites, operators, regions, ports); a graph's nodes and its external ports share one namespace |
| G2 | every operator is registered; one placed as a composite's single-operator body fills that composite; and every firing names a handler its node declares (its binding has the function: the compiler, 2b) |
| G3 | every in port of every handler has exactly one source, and every source exists; every external out port has one producer |
| G4 | every edge joins two ports of the same domain (`port_t` equality); a projection `x.f` is `field_t` of `x`'s record |
| G5 | within a region, the handlers' flow edges are acyclic, and the declared order is a topological order of them; a region updates a node's state at most once, so each read of it is declared before or after the update |
| G6 | every firing belongs to exactly one region, every region has one trigger the interface declares, a firing takes its time from an input of that trigger, and every handler of the interface's composites fires |
| G7 | an edge between regions has a rate rule, and step 2 accepts `hold` only; an edge within a region, or from a constant, has none |
| G8 | the baseline declares its class; a scoreboard baseline keeps the interface's composites, their ports and their connections, and gives each a body |
| G9 | a graph that reads a `sitl_only` port is SITL-only, and says so in its header |

## Acceptance

Step 2 is four PRs. Each is accepted on its own criteria, and the step on all of them.

**2a, anatomy: the graph description.**
1. `flow/graph.hpp`: the graph as a C++ value (composites, operators, handlers, edges, regions, triggers, rate rules), with a loader for the mission interface, the operator registry, and a baseline's `class:` and `graph:`, and the checks G1–G3 and G5–G9. A checked graph is flattened to operator firings, a body's firings in place of its composite's, in the order the regions run them.
2. The baseline loader accepts `class:` and `graph:`, and every existing baseline file still loads, unchanged.
3. The canonical form and the graph hash, with a round-trip test.
4. A test for each invariant G1–G3 and G5–G8: a file that breaks it, and the message expected. G9 is tested by value: a graph that reads a SITL-only input is SITL-only, and one that doesn't isn't. G4 is tested with the generator (2b).
5. `domain_of<CameraFrame>` (`flow/domains.hpp`). The image is a run-time-sized buffer with no bound in the data, so in step 2 the frame is a scalar of an opaque element type. Bounded image extents, bound from the vehicle file, come with the first operator that reads pixels.

**2b, anatomy: the generator.**
1. `anatomy_flowgen` and `flow_add_baseline()` (cmake/flow.cmake), which generates the header and its targets, in the way `composition_generate()` does now.
2. The neutral composites' bindings (`mobility/flow/bindings.hpp`).
3. A test mission in `tests/` built end to end, with its outputs checked as the harness drives the generated stack. Its baselines cover a scoreboard baseline of single-operator bodies, a scoreboard body of several operators, and a research baseline that uses an operator of its own (`SIGNATURE`, `BINDING`).
4. A compile-fail test for G4.
5. No existing target changes, and every existing test passes.

**2c, scout-drone: the graphs.**
1. `mission/interface.yaml`, scout's bindings (estimator and planner), and `domain_of` for `Field` and `ObstacleObservations`.
2. `s0-gt-pid` and `null` gain `class: scoreboard` and `graph:`, and build `sitl_<name>_graph`, `replay_<name>_graph` and `sitl_batch_<name>_graph` beside the slot targets.
3. **Equality with the slot stack,** on every scenario in `scenarios/` and the batch: the logs of the slot and graph builds have byte-identical records, and headers equal in every field except `composition_hash`. That is the slot hash in one, and the graph hash in the other. A comparison tool in anatomy (`log_compare --except composition_hash`) checks it in a gate test.
4. The determinism gate passes for the graph build as it does for the slot build.

**racing-drone: unchanged.**
1. In a racing-drone branch that moves its anatomy pin to the release with 2a and 2b, every baseline builds and passes its gates. Its SITL logs on its scenarios are byte-identical to its main's. The branch is a check, and isn't merged unless asked: racing-drone's pin moves in its own PR (ADR 0008).

**2d, docs.**
1. This plan; ADR 0004's current state; the model's §10, on what changes in today's code; and scout-drone's docs on the mission interface.

**In every PR:** GCC and Clang with no warnings, debug and release, and every test passing; MSVC in CI.

## Design choices

All six were confirmed on 2026-11-02.

1. **The generator is a C++ host tool run at build time, not CMake at configure time.** ADR 0004, decision 2 says configure time. A graph needs a real YAML parser, cycle checks and a topological order, and the hash should be computed by one implementation. CMake can do the first two only with a hand-written line parser, as today's flat `composition:` block does, and it can't share code with the startup loader. The cost is a host tool, which matters when anatomy is cross-compiled. Agreed: 2a amends decision 2 to "checks the graph before the stack compiles".
2. **The domains are checked by the C++ compiler, in the generated header.** Domains are C++ types (step 1), and the tool can't see them. The alternative is to describe the domains again in YAML, which the step 1 plan ruled out by writing each message's domain once.
3. **The slot concepts become the bindings, the CPU calling convention, until step 5.** Every existing implementation works unchanged. The alternative is to rewrite each implementation as an operator with one method per port, which would change code that the equality criterion compares against.
4. **The region order is declared and checked, not inferred.** The topological order alone doesn't fix the order of independent handlers, or the read of the estimate before the frame update. Declaring the order makes today's order exact and replayable.
5. **The camera frame is opaque in step 2.** It has no bounded extents until an operator reads pixels.
6. **In the scout logs, the headers may differ in `composition_hash` only.** A graph build's log names the graph's hash, so whole logs can't be byte-identical to the slot build's. Everything else, every record included, must be.

## Not in this step

- **Racing-drone's B0 as a graph** (step 3), and the export for the domain flow compiler (step 4).
- **`accumulate` and `queue` rate rules, conditional regions, and notifiers.** Notifiers go to a resource controller, which is layer 5, and the CPU schedule has none.
- **Binding symbolic extents from config.** No step 2 domain has a symbolic extent the generator must size, so this moves to step 4.
- **Retiring the slots** (step 5).

## Progress

| Step | PR | State |
|---|---|---|
| This plan | [#48](https://github.com/branes-ai/anatomy/pull/48) | merged |
| 2a: the graph description | [#50](https://github.com/branes-ai/anatomy/pull/50) | open |
| 2b: the generator | | not started |
| 2c: scout-drone's graphs | | not started |
| 2d: docs | | not started |
