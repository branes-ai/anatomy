---
status: in-progress
issue: branes-ai/anatomy#47
---
# Plan: the graph builder, the CPU stack and the baseline classes (ADR 0004, step 2)

Issue: [anatomy#47](https://github.com/branes-ai/anatomy/issues/47).

## Goal

This is step 2 of [ADR 0004](../../adr/0004-domain-flow-composition.md). It adds layers 3 and 4 of the [domain flow model](../../arch/domain-flow-model.md) in anatomy: a mission's stack becomes a graph, described in C++ and checked as it is built, which the CPU stack runs. It builds on step 1's domain and signature types ([plan](../flow-domains/README.md)). It adds:

- **a graph builder** (`flow/builder.hpp`). A mission uses it to declare its external ports and its composites, and to fire their handlers in regions, one region per trigger. The builder returns the checked graph, or the first invariant the graph breaks;
- **the two baseline classes.** A scoreboard baseline fills each composite with a body. A research baseline fires operators of its own between the mission's external ports. The scoreboard rule of ADR 0004, decision 6, holds by construction;
- **the baseline file's `class:` and `graph:`.** They name what fills the graph: for a scoreboard baseline, a body for each composite; for a research baseline, a graph registered in C++;
- **a CPU stack built from the checked graph.** It fires the operators in the checked order, with no virtual dispatch, and builds one target per baseline, as today.

**scout-drone is the first client.** Its `s0-gt-pid` and `null` baselines are re-expressed as graphs and built beside today's slot stacks. ADR 0004, decision 7, accepts a step only on both clients' outputs:
- **scout-drone:** the graph builds' logs match the slot builds' on every scenario and the batch;
- **racing-drone:** its slot baselines, built against the anatomy that contains step 2, write logs byte-identical to its main's. racing-drone gains no graph until step 3, so step 2 must leave it unchanged.

**What doesn't change:**
- the slot concepts, the `Stack` classes, `composition_generate()` and the slot baselines' targets, which stay until step 5;
- racing-drone, until step 3;
- the log format and the profiles.

## Model

A mission's stack is a graph of **operators**. Each operator is fired by **handlers**, each handler by a **region**, and each region by a **trigger**. The graph is C++: the domains are already C++ types (step 1), so the compiler checks every edge, and there is no file format to design, parse or keep in step with the types.

### The mission

A mission describes its external ports, its composites and its regions with the builder. The description is a template over its builder: with a `flow::builder` it only describes and checks, and with a `flow::stack_builder` it also builds the operators that run it (2b). Scout's would read:

```cpp
// The external ports.
template <class T>
ScoutPorts<T> scout_ports(flow::builder& g, const Field<T>& field) {
    ScoutPorts<T> p;
    p.imu       = flow::input<ImuSample<T>>(g, "imu");                 // its time: the message's `t`
    p.camera    = flow::input<CameraFrame>(g, "camera",
                      {.trigger = "frame", .time = [](const CameraFrame& f) { return f.header.t_mid; }});
    p.obstacles = flow::input<ObstacleObservations<T>>(g, "obstacles");
    p.truth     = flow::input<TruthState<T>>(g, "truth", {.trigger = "truth", .sitl_only = true});
    p.tick      = g.tick("tick");                                    // carries only its time
    p.field     = flow::constant<Field<T>>(g, "field", field);
    p.detections = flow::constant<IntruderDetections<T>>(g, "detections");  // no producer yet: empty
    p.estimate  = flow::output<StateEstimate<T>>(g, "estimate", p.tick);
    p.reference = flow::output<Reference<T>>(g, "reference", p.tick);
    p.command   = flow::output<ThrustRates<T>>(g, "command", p.tick);
    return p;
}

// A scoreboard graph: the composites, filled by the baseline's bodies, and the regions.
template <class T, class Bodies, class G>
ScoutPorts<T> scout_graph(G& g, const Field<T>& field) {
    const auto p = scout_ports<T>(g, field);
    const auto estimator   = flow::composite<EstimatorBinding<T>, typename Bodies::estimator>(g, "estimator");
    const auto daa_tracker = flow::composite<DaaTrackerBinding<T>, typename Bodies::daa_tracker>(g, "daa_tracker");
    // planner, controller, safety_filter and fc_bridge alike

    flow::scope frame = g.region(p.camera);
    auto [prior] = flow::fire<"estimate">(frame, estimator, flow::at(p.camera));
    flow::fire<"frame">(frame, estimator, p.camera);
    flow::fire<"update">(frame, daa_tracker, p.detections, prior);

    flow::scope tick = g.region(p.tick);
    auto [estimate]  = flow::fire<"estimate">(tick, estimator, flow::at(p.tick));
    auto [reference] = flow::fire<"plan">(tick, planner, p.field, flow::hold(p.obstacles), estimate);
    auto [conflicts] = flow::fire<"conflicts">(tick, daa_tracker, estimate);
    auto [command]   = flow::fire<"step">(tick, controller, estimate, reference, conflicts);
    auto [safe]      = flow::fire<"filter">(tick, safety_filter, estimate, command, conflicts,
                                            flow::project<"geofence", &Field<T>::geofence>(p.field));
    flow::fire<"send">(tick, fc_bridge, flow::at(p.tick), safe);
    g.produce(p.estimate, estimate);
    g.produce(p.reference, reference);
    g.produce(p.command, safe);
    // the imu, obstacles and truth regions alike
    return p;
}
```

The grammar, as 2a and 2b build it:

- **A composite has a binding,** which declares its handlers. A handler is one function of the binding. It names its in and out ports, which are port names of the signature, in the binding's order. It says whether it takes a time (`takes_time::yes`), and what it does with the operator's state (`state_role::read` or `update`, ADR 0004, decision 4). To run, a handler is a struct deriving from `flow::handler<>`, whose static `call()` calls its operator:

  ```cpp
  template <class T>
  struct EstimatorBinding {
      using signature = estimator_signature<T>;
      struct imu : flow::handler<"imu", flow::ins<"imu">, flow::outs<>, flow::state_role::update> {
          static void call(auto& est, const ImuSample<T>& s) { est.process_imu(s); }
      };
      struct frame : flow::handler<"frame", flow::ins<"camera">, flow::outs<>, flow::state_role::update> {
          static void call(auto& est, const CameraFrame& f) { est.process_frame(f); }
      };
      struct estimate : flow::handler<"estimate", flow::ins<>, flow::outs<"estimate">, flow::state_role::read,
                                      flow::takes_time::yes> {
          static StateEstimate<T> call(const auto& est, Timestamp t) { return est.state_at(t); }
      };
      using handlers = flow::handlers<imu, frame, estimate>;
  };
  ```
- **A port carries a message type** (`port<"estimate", StateEstimate<T>, direction::in>`), and holds its `domain_of`; a port given as a domain describes, but doesn't run. A value carries its message type, so an edge between two ports of message types also joins one message type (G4).
- **A region is a trigger,** and its firings run in the order of the calls. `fire<"h">(region, node, sources...)` takes one source per in port, in the handler's order. A time handler is fired at an input, first: `fire<"estimate">(frame, estimator, at(p.camera))` reads the estimate at the frame's time, which the input's `time` accessor gives.
- **A firing's out ports are values,** returned in the handler's order: `auto [command] = fire<"step">(...)`. A value exists only once its firing does, so a region's flow edges are acyclic, and its order a topological order of them, by construction.
- **A source** is an input, a constant, or a firing's out port. It may project a record's field, naming the member that holds it: `project<"geofence", &Field<T>::geofence>(p.field)`.
- **A source written in another region** carries a rate rule: `hold(p.obstacles)`, and the planner reads the latest obstacles seen. Step 2 implements `hold` only. `accumulate` and `queue` come when a mission needs them.
- **A read of state before an update** is declared by putting it first. The estimate at the frame's time is read before the estimator takes the frame, and the DAA gets that prior, as `Stack::on_frame` does today. So the order is fixed, and today's `Stack` order can be written down exactly.

A call that takes a type or a name as a template argument is a free function (`flow::fire<"step">(...)`), so that it reads the same inside a mission's templates, without `.template`.

### Operators and bodies

- **An operator** is a C++ class, registered under a name with its binding: `flow::registered<"CascadedPid", CascadedPid<T>, ControllerBinding<T>>`. An operator that fills a composite has that composite's binding. An operator of its own has its own signature (`signature_of`, step 1) and binding, and can be placed in a research graph, or in a body of several operators.
- **A body of several operators** is a class with a name, its composite's binding, and `build(auto& body)`. That function creates its instances and gives a function for each of the composite's handlers. The function takes the composite handler's in ports as values, fires the body's operators, and returns the handler's out ports:

  ```cpp
  struct GainTrack {
      static constexpr std::string_view name = "GainTrack";
      using binding = ControllerBinding;
      static void build(auto& b) {
          const auto gain  = flow::instance<Gain>(b, "gain");
          const auto track = flow::instance<Track>(b, "track");
          flow::on<"step">(b, [=](flow::scope& s, auto estimate, auto limits, auto alerts, auto marks) {
              auto [setpoint] = flow::fire<"apply">(s, gain, estimate, limits);
              auto [command]  = flow::fire<"step">(s, track, estimate, setpoint, alerts, marks);
              return std::tuple{command};
          });
      }
  };
  ```

  When the mission fires the composite's handler, the body's firings take its place, in the same region. The checked graph is flattened to operator firings.
- **A binding** is the CPU calling convention of a composite's operators: its handlers' `call()`s, for example `EstimatorBinding::frame::call(est, f)` calls `est.process_frame(f)`. Every implementation of a slot concept has the same methods, so a binding is written once per composite, from its slot concept. The slot concepts survive as calling conventions until step 5.
- **State.** A handler declares whether it reads or updates its operator's state (ADR 0004, decision 4). A composite declares no state domain (step 1). Its handlers declare their state role, so the builder can check that each read is ordered before or after the update.
- **The registry** (cmake/flow.cmake). `flow_register_operator(<registry> <Name> HEADER <h> TYPE <t> BINDING <b>)` registers an operator, which a baseline's header names as `registered<"Name", t<T>, b<T>>`, in the way `composition_register_impl()` does today. `flow_register_body()` registers a body of several operators, and `flow_register_graph()` a research graph.

### The baseline file

The file gains two keys, `class` and `graph`, which name what fills the graph. The graph itself is C++. A slot baseline keeps its `composition:` block, and the loader accepts all three, so in step 2 one file holds both forms of a baseline.

```yaml
name: s0-gt-pid
class: scoreboard
composition: { ... }                       # the slot form, unchanged until step 5
graph:                                     # a body for each composite
  estimator: GroundTruthEstimator
  planner: GoalPlanner
  controller: CascadedPid
  safety_filter: NullSafetyFilter
  daa_tracker: NullDaaTracker
  fc_bridge: NullFcBridge
parameters: { ... }
```

A research baseline's `graph:` is the name of a graph registered in C++: `graph: MyResearchGraph`. `flow::check_baseline()` checks the file against the graph built for it (G8).

### The CPU stack (2b)

**`flow::stack`, built from the checked graph, with no generator.**
- **The baseline's header.** `flow_add_baseline(<registry> <name> FILE <yaml> TEMPLATE <in> OUTPUT <h>)` reads the file's `class:` and `graph:` and configures the mission's template with the bodies it names, one `using <composite> = <type>;` each, or the research graph's type. A baseline's target compiles the mission's description with them.
- **The build.** At startup, the mission's stack describes its graph to a `flow::stack_builder<Ctx>`, which builds each operator with `make_slot` from the mission's context, as `Stack::create` does, and `build_stack()` returns the stack, or the first invariant broken, or the first operator that couldn't be built.
- **The run.** The stack holds a value slot per input, constant and out port. For each firing, it records the thunk the binding's `call()` instantiates, with its operator and the addresses of its in and out slots. `stack.input(port, message)` writes the input's slot and runs its trigger's region; `stack.output(port)` reads what the region wrote. Once built, the stack allocates nothing and calls no virtual function: each firing is one indirect call through a table fixed at build time. A value read in another region (`hold`) is the slot as that region last wrote it.
- **The mission's hooks.** The mission writes the class that satisfies `RunnableStack` with its profile: each `on_<input>()` hands the stack its input, and `on_tick()` returns the tick region's outputs. So the harness is unchanged.

**The graph hash** is SHA-256 of the checked graph's canonical form. It is computed in C++ only, from the graph the stack runs, so CMake no longer duplicates the hash, as `composition_generate()` does today.

## Invariants

The compiler checks what the types know, with a `static_assert` that names the invariant. The builder checks the rest as the graph is built, and `build()` returns the first invariant broken, as a `ConfigError` naming the node or the firing. G9 is derived, not refused: the checked graph says whether it is SITL-only.

| Id | Invariant | Checked by |
|---|---|---|
| G1 | names are non-empty, `[a-z_][a-z0-9_]*`, and distinct per kind (composites, instances, regions, ports, a handler's ports); a graph's nodes and its external ports share one namespace; an operator's, a body's and a graph's names are `[A-Za-z0-9_]+` | the builder; the compiler for a binding's and an operator's names |
| G2 | every firing names a handler its node declares; a handler's ports are ports of its signature, of the right direction; a composite's body has its binding; a body implements each of its composite's handlers once; to run, a handler's `call()` takes its operator, its time if it takes one, and its in ports' messages | the compiler; the builder for a body's handlers |
| G3 | every in port of every handler has exactly one source, and a time handler a time; every external out port has one producer | the compiler; the builder for the out ports |
| G4 | every edge joins two ports of the same domain, and of one message type when both carry one; a projection `x.f` is `field_t` of `x`'s record, and a message's projection names the member that holds it; a body's handler returns its composite handler's out ports | the compiler |
| G5 | within a region, the flow edges are acyclic and the order is a topological order of them (by construction); a region updates a node's state at most once, so each read of it is before or after the update; a body's operator updates state only where its composite handler does | the builder |
| G6 | every trigger has at most one region; a firing takes its time from an input of its region's trigger; a handler fires once at each input; an output is written by a firing of its region; every handler of every composite, and of every research instance, fires | the builder |
| G7 | an edge between regions has a rate rule, and step 2 accepts `hold` only; an edge within a region, or from a constant, has none | the builder |
| G8 | a scoreboard graph has composites and no operators outside their bodies; a research graph has operators and no composites; the baseline file declares its class with its graph, and names the body of each of the graph's composites and of no other | the builder; `check_baseline()` for the file |
| G9 | a graph that reads a `sitl_only` input is SITL-only, and says so | derived |

## Acceptance

Step 2 is four PRs. Each is accepted on its own criteria, and the step on all of them.

**2a, anatomy: the graph builder.**
1. `flow/builder.hpp`: handlers, bindings, registered operators, bodies, and the builder, which checks G1–G3 and G5–G8 as a graph is built, and returns the checked graph (`flow/graph.hpp`). A checked graph is flattened to operator firings, a body's firings in place of its composite's, in the order the regions run them.
2. The baseline loader accepts `class:` and `graph:`, and every existing baseline file still loads, unchanged. `check_baseline()` checks a file against its graph.
3. The canonical form and the graph hash, with a test that equal graphs have one form, and that a different operator, body or order changes it.
4. A test for each invariant the builder checks, G1–G3 and G5–G8: a graph that breaks it, and the message expected. G9 is tested by value: a graph that reads a SITL-only input is SITL-only, and one that doesn't isn't. A compile-fail test for each invariant the compiler checks (G1–G4), expecting its `static_assert`'s message.
5. `domain_of<CameraFrame>` and `domain_of<Timestamp>` (`flow/domains.hpp`). The image is a run-time-sized buffer with no bound in the data, so in step 2 the frame is a scalar of an opaque element type. Bounded image extents, bound from the vehicle file, come with the first operator that reads pixels.

**2b, anatomy: the CPU stack.**
1. `flow::stack` and `flow::stack_builder`, built from a checked graph and the bindings; and `flow_register_operator()`, `flow_register_body()`, `flow_register_graph()` and `flow_add_baseline()` (cmake/flow.cmake), which configure a baseline's header, as `composition_generate()` does now.
2. The neutral composites' bindings (`mobility/flow/bindings.hpp`), and their signatures carrying their message types.
3. A test mission in `tests/` built end to end through the CMake functions, with its baselines' SITL logs equal, record for record, to the logs of the same operators wired by hand, as a slot stack is. Its baselines cover a scoreboard baseline of single-operator bodies, one with a stateful operator, a scoreboard body of several operators, and a research graph that uses an operator of its own.
4. A compile-fail test for a binding that lacks a handler's `call()` (G2), for a stack of ports that carry no message type, and for a projection of a message that names no member.
5. No existing target changes, and every existing test passes.

**2c, scout-drone: the graphs.**
1. Scout's ports, composites and regions (`mission/flow/`), scout's bindings (estimator and planner), and `domain_of` for `Field` and `ObstacleObservations`.
2. `s0-gt-pid` and `null` gain `class: scoreboard` and `graph:`, and build `sitl_<name>_graph`, `replay_<name>_graph` and `sitl_batch_<name>_graph` beside the slot targets.
3. **Equality with the slot stack,** on every scenario in `scenarios/` and the batch: the logs of the slot and graph builds have byte-identical records, and headers equal in every field except `composition_hash`. That is the slot hash in one, and the graph hash in the other. A comparison tool in anatomy (`log_compare --except composition_hash`) checks it in a gate test.
4. The determinism gate passes for the graph build as it does for the slot build.

**racing-drone: unchanged.**
1. In a racing-drone branch that moves its anatomy pin to the release with 2a and 2b, every baseline builds and passes its gates. Its SITL logs on its scenarios are byte-identical to its main's. The branch is a check, and isn't merged unless asked: racing-drone's pin moves in its own PR (ADR 0008).

**2d, docs.**
1. This plan; ADR 0004's current state; the model's §10, on what changes in today's code; and scout-drone's docs on the mission's graph.

**In every PR:** GCC and Clang with no warnings, debug and release, and every test passing; MSVC in CI.

## Design choices

Choices 1 to 7 were confirmed on 2026-11-02, after 2a's first version described the graph in YAML. Choices 8 to 11 were made in 2b, to run the graph, and are confirmed with its PR.

1. **The graph is described in C++, not in a file.** A file format would be a new format to design, parse and keep in step with the C++ types, and the alternatives examined don't hold a mission's graph:
   - **GGUF** holds a model's weights and metadata. Its graph is built in code, per architecture, as llama.cpp and synapse do. It remains the container for operators' weights and stationary parameters.
   - **domain_flow's `DomainFlowGraph` and `.dfg`** describe one dataflow call over a closed set of DL operators. They have no regions, triggers, rate rules, persistent state or composites. They are step 4's export target: each region of the checked graph becomes one DFG, and a leaf operator's inside a `.sure` spec.

   C++ is how ggml, llama.cpp and synapse describe their graphs. Step 1's domains and signatures are C++ types already. A baseline file still names what fills the graph (choice 4).
2. **The compiler checks what the types know, and the builder the rest.** Domains, handlers, ports and arity are types, so G2–G4 are `static_assert`s, with no generated code to carry them. The order, the regions, the rate rules, the state and the class are checked when the graph is built.
3. **The region order is the order of the calls.** The topological order alone doesn't fix the order of independent handlers, or the read of the estimate before the frame update. Declaring the order makes today's order exact and replayable, and with values that exist only once fired it is a topological order by construction.
4. **A scoreboard baseline's file names a body per composite, and nothing else.** The mission's code fixes the composites and how they connect, so a scoreboard baseline can't rewire them: decision 6's rule holds by construction, and `check_baseline()` checks the names. A research baseline names a graph registered in C++.
5. **The slot concepts become the bindings, the CPU calling convention, until step 5.** Every existing implementation works unchanged. The alternative is to rewrite each implementation as an operator with one method per port, which would change code that the equality criterion compares against.
6. **The camera frame is opaque in step 2,** with no bounded extents until an operator reads pixels. **In the scout logs, the headers may differ in `composition_hash` only:** a graph build's log names the graph's hash, so whole logs can't be byte-identical to the slot build's. Everything else, every record included, must be.
7. **The CPU stack runs the checked graph, with no generator.** A generator would turn the graph into C++ source. That needs a host tool, which matters when anatomy is cross-compiled, and C++ spellings of the types, which the builder doesn't have. The stack instead records, for each firing, a function instantiated from the binding, so each handler call is static and inlined within its function. What remains is one indirect call per firing, through a table fixed at build time, with no virtual dispatch and no allocation once built. The alternative is to generate the source as `composition_generate()` does, from names the registry gives.

8. **A port carries its message type.** The stack holds a value as its C++ message, which a domain doesn't name: several messages may have one domain. A step 1 port given as a domain still describes, but doesn't run. The alternative is a table from domain to message, which is the same declaration, kept apart from the port.
9. **A handler is a struct with a static `call()`.** C++ can't call a function from its name as a string, so the binding can't be a struct of functions named after the handlers. The struct puts each handler's declaration and its call in one place.
10. **A mission's description is a template over its builder.** The stack builder builds each operator from the mission's context, whose type only the mission knows. The alternative is a builder of type-erased factories, built at startup, which is more code for the same choice.
11. **The mission writes its stack's hooks** (`on_imu()`, `on_frame()`, `on_tick()`), which hand the stack their input and read its outputs. C++ can't name member functions from the graph's triggers. The hooks hold no logic: the order of the firings is the graph's.

## Not in this step

- **Racing-drone's B0 as a graph** (step 3), and the export for the domain flow compiler (step 4).
- **`accumulate` and `queue` rate rules, conditional regions, and notifiers.** Notifiers go to a resource controller, which is layer 5, and the CPU schedule has none.
- **Binding symbolic extents from config.** No step 2 domain has a symbolic extent the stack must size, so this moves to step 4.
- **Retiring the slots** (step 5).

## Progress

| Step | PR | State |
|---|---|---|
| This plan | [#48](https://github.com/branes-ai/anatomy/pull/48) | merged |
| 2a: the graph builder | [#50](https://github.com/branes-ai/anatomy/pull/50) | merged |
| 2b: the CPU stack | | in progress |
| 2c: scout-drone's graphs | | not started |
| 2d: docs | | not started |
