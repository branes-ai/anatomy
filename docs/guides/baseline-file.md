# The Baseline File

A baseline is a named stack: what fills each part of a mission's stack, and the values those parts run with. Each baseline is one file, `baselines/<name>.yaml`, in the mission's repo. Each file becomes its own build targets, so a baseline is chosen when it is built, not when it runs.

This page is the reference: every key, who reads it, and what is checked. [Designing a baseline](designing-a-baseline.md) is about choosing what goes in one, and writing down what it is for.

The file holds no wiring. A mission's stack is C++: the slot stack's `Stack` class, or, in the domain flow form, the graph the mission builds ([ADR 0004](../adr/0004-domain-flow-composition.md), decision 2). The file only names what fills the stack.

## An example

scout-drone's `s0-gt-pid`, in full:

```yaml
# Baseline s0-gt-pid (docs/plan/bring-up/): the true state, a straight line to
# the goal at a cruise speed, and the reflex position/attitude cascade. SITL
# only: the estimator reads the simulated truth (SYS-02). It doesn't avoid
# obstacles, so it is the lower bound every avoiding baseline must beat.
name: s0-gt-pid
composition:
  estimator: GroundTruthEstimator
  planner: GoalPlanner
  controller: CascadedPid
  safety_filter: NullSafetyFilter
  daa_tracker: NullDaaTracker
  fc_bridge: NullFcBridge
class: scoreboard
graph:
  estimator: GroundTruthEstimator
  planner: GoalPlanner
  controller: CascadedPid
  safety_filter: NullSafetyFilter
  daa_tracker: NullDaaTracker
  fc_bridge: NullFcBridge
parameters:
  control_rate_hz: 250     # position and attitude loops; SimFc runs the rate loop at 1 kHz
  cruise_speed_m_s: 3.0    # GoalPlanner
  lookahead_m: 0.5
  switch_radius_m: 1.0
  slowdown_m: 3.0
  max_speed_m_s: 4.5       # CascadedPid: velocity-setpoint cap, above the cruise speed
```

## The two forms

During ADR 0004's steps 2 to 4, a mission's stack can be built in two ways, and one file can give both.

| Form | Keys | What it builds |
|---|---|---|
| **Slots** (racing-drone [ADR 0001](https://github.com/branes-ai/racing-drone/blob/main/docs/adr/0001-baseline-composition-framework.md)) | `composition:` | The mission's hand-written `Stack`, with one implementation per slot |
| **Domain flow** (ADR 0004) | `class:` and `graph:` | The CPU stack built from the mission's checked graph (`flow/graph.hpp`), with a body per composite or a research graph |

Every file has the slot form. The loader requires `composition:` until step 5 retires the slots. A mission that builds a baseline's graph as well (scout-drone's `scout_add_baseline(<name> GRAPH)`) also reads `class:` and `graph:`, and gives the baseline a second set of targets, named with a `_graph` suffix.

In scout-drone, `composition:` and `graph:` list the same names. Scout's six composites are its six slots, and every slot implementation is registered as an operator of its composite, so both forms describe the same stack. Step 2 accepted the graph build only when its logs equalled the slot build's, so the same names in both blocks are what that test requires. Nothing checks that the two blocks agree, but a mission's graph gate does: `gate.<name>.graph` flies both builds and fails if their logs differ. In step 5, `composition:` goes away and `graph:` remains.

## The keys

The file is a mapping with these keys, and no others. The order doesn't matter.

| Key | Required | Value | Checks |
|---|---|---|---|
| `name` | yes | the baseline's name, `[a-z0-9-]+` | matches the file name and the name compiled into the binary. Quote `"null"`, since a bare `null` is YAML's null |
| `composition` | yes | a mapping, slot to implementation | names each of the mission's slots exactly once; each implementation is `[A-Za-z0-9_]+`, and registered for that slot |
| `class` | with `graph` | `scoreboard` or `research` | present exactly when `graph` is (G8) |
| `graph` | with `class` | a scoreboard baseline's: a mapping, composite to body. A research baseline's: the name of a graph | see [The graph](#the-graph) |
| `parameters` | no | a mapping, key to a scalar | each value is a scalar. Which keys are read, and their ranges, is up to the components |

The C++ type is `config::BaselineConfig` (`<branes/anatomy/config/baseline.hpp>`). Its comment lists the invariants, and `load_baseline()` checks all of them before it returns a value.

### The graph

A **scoreboard** baseline's `graph:` maps each of the mission's composites to its body. A composite is `[a-z_][a-z0-9_]*` and a body is `[A-Za-z0-9_]+` (G1). A body is an operator, or a body of several operators, registered in the mission's operator registry. The file must name each of the graph's composites, and no others (G8). It can't add a composite, remove one, or rewire them: the mission's code fixes them.

A **research** baseline's `graph:` is one name, the name of a graph registered in C++:

```yaml
class: research
graph: MyResearchGraph
```

A research graph may connect the mission's external ports in any way. It is scored like any other baseline, but the scoreboard doesn't rank it ([ADR 0004](../adr/0004-domain-flow-composition.md), decision 6).

### The parameters

`parameters:` is one flat namespace that all of a baseline's components share. Each component reads its own keys when it is built from the mission's context (`from_context()`), with `parameter_in(baseline, key, fallback, lo, hi)`:

| The key is | `parameter_in()` returns |
|---|---|
| absent | the fallback, the component's default |
| present, a finite number | the value, if it is in `[lo, hi]`; else an error, and the binary refuses to start |
| present, not a number | an error, and the binary refuses to start |

A key that no component reads is not reported. A misspelled key therefore leaves the component on its default, with no error. The apps read `control_rate_hz` themselves, which defaults to `kDefaultControlRateHz`, 100 Hz.

Values are stored as strings and read with `numeric_parameter()`, which accepts a value only if the whole string is a finite number. `emit_baseline()` writes each value back as the string it was read as, so an emitted file loads back equal.

## Who reads it, and when

| When | Who | Reads | Does |
|---|---|---|---|
| configure | `composition_generate()` (`cmake/composition.cmake`) | `composition:` | checks the slots against the mission's slot registry, hashes the composition, and generates the slot build's header |
| configure | `flow_add_baseline()` (`cmake/flow.cmake`), for a graph build | `class:`, `graph:` | checks each body, or the research graph, against the mission's operator registry, and generates the graph build's header |
| startup, slot build | `harness::check_baseline()` | the whole file | loads it, then refuses to start if the name or the composition hash differs from the binary's |
| startup, graph build | `flow::check_baseline()` | `class:`, `graph:` | refuses to start if the class or a body differs from the graph built (G8) |
| startup | each component's `from_context()` | `parameters:` | reads its keys |

Editing a file reruns CMake, so the generated headers never go stale. Editing only `parameters:` changes neither hash, so the binary needn't be rebuilt.

### The layout CMake reads

CMake reads `composition:` and `graph:` line by line, not as YAML. Write them in block style:

- `composition:` alone on its line, with no comment after it;
- each entry on its own line, indented by two spaces: `  slot: Implementation`, optionally followed by a `# comment`;
- the block ends at the next line that starts in column one.

Flow style, such as `composition: { estimator: ... }`, loads in C++ but fails at configure: CMake finds no entries. `graph:` follows the same rules, except that a comment may follow `graph:`. A research graph's name goes on the same line, `graph: Name`.

## The hashes

A binary records which stack it runs, and replay checks it.

- **The slot build** compiles in the **composition hash**: the SHA-256 of `slot=Implementation\n` for each slot, with the slots sorted (`config::composition_hash()`, and the same computed in CMake). It records the hash in each log's header.
- **The graph build** records the **graph hash** in the same field: the SHA-256 of the checked graph's canonical form (`flow::graph_hash()`). The canonical form includes the wiring, the regions and the order of the firings, so a change to the mission's graph changes the hash, even if no name in the file changes.

So a slot build's log and a graph build's log of the same baseline differ only in `composition_hash`. That's the field `log_compare --except composition_hash` leaves out.

## The checks' messages

A file that fails to load reports the file, the key and the rule. These are the messages the loader and CMake give for the domain flow keys:

| Message | Means |
|---|---|
| `G8: a baseline with a graph declares its class` | `graph:` without `class:` |
| `G8: a baseline with a class gives its graph` | `class:` without `graph:` |
| `must be scoreboard or research` | another value of `class:` |
| `a scoreboard graph maps each composite to its body` | a scoreboard baseline whose `graph:` is a single name |
| `a research graph is the name of a registered graph` | a research baseline whose `graph:` is a mapping |
| `'<Body>' is not a registered operator or body (G2)` | at configure: the body isn't in the operator registry |
| `'<Name>' is not a registered graph (G8)` | at configure: the research graph isn't registered |

The invariants G1 to G9 are listed in full in the [graph builder's plan](../plan/flow-graph/README.md#invariants).
