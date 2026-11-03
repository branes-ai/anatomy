# Designing a Baseline

A baseline answers one question about a mission, such as "can the stack fly the course at all?". It answers by being scored on the mission's scenario suite, beside the other baselines. This guide covers choosing what goes into a baseline, and writing down what it is for. [The baseline file](baseline-file.md) is the reference for the keys.

The file has no key for a baseline's purpose. A file names components and values. What the baseline is for, what it is compared with, and which gate it must pass are written in its header comment, and enforced by the mission's gates. Those three are what make it a baseline, so this guide spends most of its time on them.

## 1. Start from the question

Write down the question first, and what answer would settle it. Every baseline that exists today began that way:

| Baseline | The question | The answer that settles it |
|---|---|---|
| racing-drone `null` | Does the framework run end to end, before any component exists? | Every harness runs, logs and replays. The vehicle stays on the pad |
| racing-drone `b0-gt-pid` | Can the stack fly the course at all, with perfect state? | Gate P1: two laps of `a2rl-2025` at ≤ 5 m/s in ≥ 99 % of 500 seeds |
| scout-drone `s0-gt-pid` | How well does a stack do that doesn't avoid obstacles? | Its score is the lower bound every avoiding baseline must beat |

A baseline answers one question, so changing what it is for usually means making a new baseline.

## 2. Pick its reference

A baseline's result means something only next to another baseline's. Name the baseline it is measured against, and change as little as you can relative to it. A scoreboard baseline that replaces one composite's body can have its difference attributed to that body. One that replaces three can't ([ADR 0004](../adr/0004-domain-flow-composition.md), decision 6).

Perfect-state baselines like `b0-gt-pid` and `s0-gt-pid` exist to give the others a reference. They read the simulated truth, so they bound what any estimator can do. A later baseline that swaps in a real estimator, and keeps everything else, measures what that estimator costs.

## 3. Choose the class

| If the baseline | Its class is |
|---|---|
| fits the mission's composites: it only changes what fills them | `scoreboard` |
| needs a different graph: a new stage between composites, a composite split in two, or other wiring | `research` |

Prefer `scoreboard`. A research baseline is scored on the same suite, but the scoreboard doesn't rank it, and its difference from a scoreboard baseline can't be attributed to any one composite.

A research baseline joins the scoreboard in one of two ways. Its graph can be fitted into the mission's composites, or the mission can change its composites so the graph fits. The second is a change to the mission's interface, and every scoreboard baseline follows it. Treat it as a mission decision, not a baseline's.

## 4. Fill the parts

For each slot (and, in a graph build, each composite), name a registered implementation:

- **A stub where a part is not under test.** `NullSafetyFilter`, `NullDaaTracker` and `NullFcBridge` keep a baseline's question narrow. A stub says "not part of this question"; it isn't a gap to fill later.
- **A new component** is registered before a baseline can name it. In the slot registry (`composition_register_impl()`), it is registered for its slot. In the operator registry (`flow_register_operator()`, or the mission's wrapper such as `scout_register_operator()`), it is registered with its composite's binding. A slot implementation satisfies its slot's concept, so it works as an operator of that composite unchanged.
- **Truth.** A component that reads the simulated true state (`TruthConsumer`, or a graph that reads the `truth` port) makes the baseline SITL-only (SYS-02). The checked graph marks it so (G9). Say so in the header, as both perfect-state baselines do.

## 5. Choose the parameters

- **Name a parameter by what it is, with its unit as a suffix:** `cruise_speed_m_s`, `lookahead_m`, `corner_angle_deg`, `control_rate_hz`. The keys share one namespace, so a key two components read is one value for both. That is right for `control_rate_hz`, and a collision for anything else.
- **Comment the value, not just the key.** Give the reader the reason: `max_speed_m_s: 4.5  # velocity-setpoint cap, under the 5 m/s gate limit` says why 4.5, and what bounds it.
- **Group the keys by the component that reads them,** and name the component in the first key's comment, as `s0-gt-pid` does with `# GoalPlanner`.
- **Check the spelling.** A key no component reads isn't reported, and a misspelled key leaves its component on the default without an error. If a value seems to have no effect, compare its key against the component's `from_context()`.
- **Put ranges in the component, not in the file.** `parameter_in()` refuses a value outside the range the component declares, so the file can't take the component outside what it was tested for.

Parameters are read at startup, so changing one needs no rebuild. That makes them the right place for anything a sweep varies (scout-drone's `sitl_batch --set key=value` overrides a parameter for one batch). It also means a parameter change doesn't change the composition hash. If a value is part of what makes the baseline itself, say so in the header comment.

## 6. Write the header

The header comment is where a baseline's purpose is kept. Give it, in this order:

1. **The baseline's name, and the plan it belongs to:** `# Baseline B0 \`gt-pid\` (docs/plan/b0/):`.
2. **What it is,** in one sentence that names its parts: "the true state, a waypoint path through the gate centers, and the reflex position/attitude cascade".
3. **Where it can run:** "SITL only: the estimator reads the simulated truth (SYS-02)", when it can't fly.
4. **The question, and its reference:** "It answers 'can the stack fly the course at all?' and bounds the later baselines from above".
5. **Its gate, with the numbers:** "Gate P1: two laps of a2rl-2025 at ≤ 5 m/s in ≥ 99 % of 500 nominal seeds".

```yaml
# Baseline B0 `gt-pid` (docs/plan/b0/): perfect state, a waypoint path through
# the gate centers, and the reflex position/attitude cascade. SITL only: the
# estimator reads the simulated truth (SYS-02). It answers "can the stack fly
# the course at all?" and bounds the later baselines from above. Gate P1:
# two laps of a2rl-2025 at <= 5 m/s in >= 99 % of 500 nominal seeds.
name: b0-gt-pid
```

The name follows the mission's convention. Both missions so far use the baseline's id in the mission's plan, then the parts that set it apart: `b0-gt-pid` is B0, ground truth and PID; racing-drone's planned `b2-vio-gate-mpcc` is B2, VIO, gate detection and MPCC.

## 7. Hold it to its gate

A comment can say what a baseline must do, but only a test can hold it to that. Each claim in the header should have a test in the mission's suite:

| The header says | The test |
|---|---|
| its gate, with its numbers | a gate test, run in the nightly suite if it's long (racing-drone's `gate.p1.full`) |
| it is deterministic | a determinism gate: the same seed twice, the same log |
| its graph build flies as its slot build does | the graph gate, `gate.<name>.graph` |
| it is SITL-only | nothing to add: the checked graph of a baseline that reads truth is marked SITL-only (G9) |

## A checklist

- [ ] The question, and the answer that settles it, are in the header.
- [ ] The reference baseline is named, and as little as possible differs from it.
- [ ] The class is `scoreboard`, unless the graph has to change.
- [ ] Every part not under test is a stub.
- [ ] Every component is registered, in the slot registry and, for a graph build, the operator registry.
- [ ] SITL-only is stated, if any component reads truth.
- [ ] Each parameter has its unit in its name, its reader's group, and a reason in its comment, and each key matches its reader's `from_context()`.
- [ ] Each claim in the header has a test.
