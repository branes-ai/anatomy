---
status: draft
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

**scout-drone is the first client.** Its `s0-gt-pid` and `null` baselines are re-expressed as graphs and built beside today's slot stacks. The step is accepted only when the graph builds' logs match the slot builds' on every scenario and the batch (ADR 0004, decision 7).

**What doesn't change:**
- the slot concepts, the `Stack` classes, `composition_generate()` and the slot baselines' targets, which stay until step 5;
- racing-drone, until step 3;
- the log format and the profiles.

## Model

A mission's stack is a graph of **operators**. Each operator is fired by **handlers**, each handler by a **region**, and each region by a **trigger**. What the generator reads is in YAML. What it checks against is C++ types.

### The mission interface

Each mission has one interface file, `mission/interface.yaml`. It names C++ types and bindings as strings, and the generated code checks them (G4, below).

```yaml
mission: scout
profile: branes::scout::ScoutProfile          # the harness contract, unchanged
external:                                     # the external ports
  in:
    imu:       { message: branes::anatomy::ImuSample,       trigger: imu }
    camera:    { message: branes::anatomy::CameraFrame,     trigger: frame, time: header.t_mid }
    obstacles: { message: branes::scout::ObstacleObservations, trigger: obstacles }
    truth:     { message: branes::anatomy::TruthState,      trigger: truth, sitl_only: true }
    tick:      { trigger: tick }                            # carries only its time
  constant:                                   # fixed when the stack is built
    field:      { message: branes::scout::Field, from: field }
    detections: { message: branes::anatomy::IntruderDetections }   # empty: no producer yet
  out:
    estimate:  { message: branes::anatomy::StateEstimate, region: tick }
    reference: { message: branes::anatomy::Reference,     region: tick }
    command:   { message: branes::anatomy::ThrustRates,   region: tick }
composites:                                   # in the order the stack builds them
  estimator:     { signature: branes::scout::flow::estimator_signature,     binding: branes::scout::flow::EstimatorBinding }
  planner:       { signature: branes::scout::flow::planner_signature,       binding: branes::scout::flow::PlannerBinding }
  controller:    { signature: branes::anatomy::flow::controller_signature,  binding: branes::anatomy::flow::ControllerBinding }
  safety_filter: { signature: branes::anatomy::flow::safety_filter_signature, binding: branes::anatomy::flow::SafetyFilterBinding }
  daa_tracker:   { signature: branes::anatomy::flow::daa_tracker_signature, binding: branes::anatomy::flow::DaaTrackerBinding }
  fc_bridge:     { signature: branes::anatomy::flow::fc_bridge_signature,   binding: branes::anatomy::flow::FcBridgeBinding }
regions:                                      # each fired by one trigger; a handler list, in dependence order
  imu:       [estimator.imu <- imu]
  frame:     [estimator.estimate@camera, estimator.frame <- camera, daa_tracker.update <- (detections, estimator.estimate@camera)]
  obstacles: [estimator.obstacles <- obstacles]
  truth:     [estimator.truth <- truth]
  tick:
    - estimator.estimate@tick
    - planner.plan <- (field, obstacles: hold, estimator.estimate)
    - daa_tracker.conflicts <- estimator.estimate
    - controller.step <- (estimator.estimate, planner.reference, daa_tracker.conflicts)
    - safety_filter.filter <- (estimator.estimate, controller.command, daa_tracker.conflicts, field.geofence)
    - fc_bridge.send <- (safety_filter.safe_command, tick)
```

The handler syntax above is a sketch. Its exact grammar is settled in 2a, below. What it must express:

- **A handler** is one firing of a composite: `estimator.frame` takes the camera frame, and `estimator.estimate@camera` reads the estimate at the frame's time.
- **An edge** joins a source (an external port, a constant, a field of one, `field.geofence`, or another handler's out port) to a handler's in port.
- **`hold`** marks an edge between regions that run at different rates: the planner reads the latest obstacles seen. Step 2 implements hold only. `accumulate` and `queue` are refused until a mission needs them.
- **The order of a region's handlers** is a topological order of its flow edges. A read of an operator's state before a handler that updates it is declared by putting it first: the estimate at the frame's time is read before the estimator takes the frame, and the DAA gets that prior, as `Stack::on_frame` does today. Between independent handlers, the order is the order of declaration. So the generated order is fixed, and today's `Stack` order can be written down exactly.

### The baseline file

The file gains two keys, `class` and `graph`. A slot baseline keeps its `composition:` block, and the loader accepts all three, so in step 2 one file holds both forms of a baseline.

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

**A scoreboard baseline** gives a body for each composite. A body is one operator, as above, or a subgraph of operators whose external ports are the composite's ports. **A research baseline** gives `operators:` and `regions:` between the mission's external ports, in the interface's grammar. Step 2 builds the research path and tests it on a test mission in anatomy. scout-drone ships no research baseline yet.

### Operators and bindings

- **An operator** is a C++ class registered under a name with its header and type, as `composition_register_impl()` does today: `flow_register_operator(<registry> <Name> COMPOSITE <c> HEADER <h> TYPE <t>)`. A registered operator fills one composite, so a single-operator body binds its ports by name to the composite's.
- **A binding** is the CPU calling convention of a composite's operators. It is a C++ struct with one static function per handler, for example `EstimatorBinding::frame(est, f)` calls `est.process_frame(f)`. Every implementation of a slot concept has the same methods, so a binding is written once per composite, from its slot concept. The slot concepts survive as calling conventions until step 5.
- **State.** A binding declares which of its handlers read and which update the operator's state (ADR 0004, decision 4). A composite declares no state (step 1); an operator does here, through its binding, so the generator can check that a read before an update is declared in that order.

### The generator

**A C++ host tool, `anatomy_flowgen`, built from anatomy with yaml-cpp.** A baseline's target runs it as a build step: it reads the mission interface, the baseline file and the operator registry, checks the graph, and writes `<build>/generated/<name>/flow_baseline.hpp`. The generated header defines the baseline's name, its graph hash, its class, whether it is SITL-only, and `Graph<T>`: a class satisfying `RunnableStack` with the mission's profile. Its `on_<trigger>()` functions are the regions, in the checked order, calling the bindings, and its `create(field, ctx)` builds each operator with `make_slot`, as `Stack::create` does.

**The checks split by where the facts live.** The graph's structure is checked by the tool, before anything compiles. The domains are C++ types, so they are checked by the C++ compiler, in the generated header, with a `static_assert` per edge that names the edge.

**The graph hash** is SHA-256 of the checked graph's canonical form. It is computed in C++ only, by the same code at build time and at startup, so CMake no longer duplicates the hash, as `composition_generate()` does today.

## Invariants

The generator checks G1–G3 and G5–G9 and refuses a graph that breaks any of them, with a message that names the file, the key and the invariant. G4 is a `static_assert` in the generated header.

| Id | Invariant |
|---|---|
| G1 | names are non-empty, `[a-z_][a-z0-9_]*`, and distinct per kind (composites, operators, regions, ports) |
| G2 | every operator is registered, fills the composite it is placed in, and its binding has the handler named |
| G3 | every in port of every handler has exactly one source, and every source exists; every external out port has one producer |
| G4 | every edge joins two ports of the same domain (`port_t` equality); a projection `x.f` is `field_t` of `x`'s record |
| G5 | within a region, the handlers' flow edges are acyclic, and the declared order is a topological order of them; a read of state before an update in one region is declared in that order |
| G6 | every handler belongs to exactly one region, and every region has one trigger the interface declares |
| G7 | an edge between regions has a rate rule, and step 2 accepts `hold` only |
| G8 | the baseline declares its class; a scoreboard baseline keeps the interface's composites, their ports and their connections, and gives each a body |
| G9 | a graph that reads a `sitl_only` port is SITL-only, and says so in its header |

## Acceptance

Step 2 is four PRs. Each is accepted on its own criteria, and the step on all of them.

**2a, anatomy: the graph description.**
1. `flow/graph.hpp`: the graph as a C++ value (composites, operators, handlers, edges, regions, triggers, rate rules), with a loader for the mission interface and for a baseline's `class:` and `graph:`, and the checks G1–G3 and G5–G9.
2. The baseline loader accepts `class:` and `graph:`, and every existing baseline file still loads, unchanged.
3. The canonical form and the graph hash, with a round-trip test.
4. A test for each invariant G1–G9: a file that breaks it, and the message expected.
5. `domain_of<CameraFrame>`. The image is a run-time sized buffer with no bound in the data, so in step 2 the frame is a scalar of an opaque element type. Bounded image extents, bound from the vehicle file, come with the first operator that reads pixels.

**2b, anatomy: the generator.**
1. `anatomy_flowgen` and `flow_add_baseline()` (cmake/flow.cmake), which generates the header and its targets, in the way `composition_generate()` does now.
2. The neutral composites' bindings (`mobility/flow/bindings.hpp`).
3. A test mission in `tests/` built end to end: scoreboard and research baselines, the generated stack driven by the harness, and its outputs checked.
4. A compile-fail test for G4.
5. No existing target changes, and every existing test passes.

**2c, scout-drone: the graphs.**
1. `mission/interface.yaml`, scout's bindings (estimator and planner), and `domain_of` for `Field` and `ObstacleObservations`.
2. `s0-gt-pid` and `null` gain `class: scoreboard` and `graph:`, and build `sitl_<name>_graph`, `replay_<name>_graph` and `sitl_batch_<name>_graph` beside the slot targets.
3. **Equality with the slot stack,** on every scenario in `scenarios/` and the batch: the logs of the slot and graph builds have byte-identical records, and headers equal in every field except `composition_hash`. That is the slot hash in one, and the graph hash in the other. A comparison tool in anatomy (`log_compare --except composition_hash`) checks it in a gate test.
4. The determinism gate passes for the graph build as it does for the slot build.

**2d, docs.**
1. This plan; ADR 0004's current state; the model's §10, on what changes in today's code; and scout-drone's docs on the mission interface.

**In every PR:** GCC and Clang with no warnings, debug and release, and every test passing; MSVC in CI.

## Design choices to confirm

1. **The generator is a C++ host tool run at build time, not CMake at configure time.** ADR 0004, decision 2 says configure time. A graph needs a real YAML parser, cycle checks and a topological order, and the hash should be computed by one implementation. CMake can do the first two only with a hand-written line parser, as today's flat `composition:` block does, and it can't share code with the startup loader. The cost is a host tool, which matters when anatomy is cross-compiled. If this is agreed, 2a amends decision 2 to "checks the graph before the stack compiles".
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
| This plan | [#48](https://github.com/branes-ai/anatomy/pull/48) | open |
| 2a: the graph description | | not started |
| 2b: the generator | | not started |
| 2c: scout-drone's graphs | | not started |
| 2d: docs | | not started |
