# Plan: the core's outputs, truth and events take the mission's types

Issue: [anatomy#7](https://github.com/branes-ai/anatomy/issues/7). Status: done. 1a (#8) released in v0.2.0; 1b ([racing-drone#70](https://github.com/branes-ai/racing-drone/pull/70)) moved racing-drone to it, byte-identical.

## Goal

Step 1 of [ADR 0001](../../adr/0001-core-profiles-multi-node.md) (Decision 2, order of work 5.1). The core's contracts stop assuming a mobile vehicle:

- **Stack outputs.** The Runner no longer encodes a `TickOutput`. The stack's profile names its output type and encodes it.
- **Truth.** The Runner's truth channel decodes, checks and casts the profile's truth type, not `TruthState`.
- **Events.** The IMU and camera events become the mobility profile's input events. The core keeps the events every mission has: the tick, the truth (SITL), and `sim_config`.

What a roaming vehicle needs (`TickOutput`, `TruthState`, IMU, camera) becomes the **mobility profile**, still in today's libraries. Step 2 moves it into `branes::anatomy_mobility`.

**Unchanged:**
- behavior, the log format, the channel ids, and every error message;
- racing-drone's outputs, which stay byte-identical;
- the simulator, the closed loop, the SITL loop and the scorer. The ADR puts them in `mobility_sim`, so they stay mobility-only. They now say so with a constraint.

## The model

### A stack profile (core, new `compose/profile.hpp`)

A profile is a type `P` that says what a stack of one kind of mission takes and produces. It has no state; every member is static.

```text
Model: P (a StackProfile)
  truth_type<T>     : the simulated truth, in scalar type T
  output_type<T>    : what one tick produces
  inputs            : std::array<log::Channel, N> — the profile's input channels
  outputs           : std::array<log::Channel, M> — its output channels, in pipeline order
  decode_truth(b)   : std::optional<truth_type<double>>
  feed(S&, rec)     : result<ok_t, std::string> — decode, check and cast one input record, and hand it to the stack
  encode(y, t, out) : appends one Output per channel of `outputs`, in that order
  drives<S>         : bool — S has the profile's input hooks (mobility: on_imu, on_frame)

Invariants (checked by static_assert where the profile is defined, and by the concept):
  P1: every channel of `inputs` is an input (!is_output), and none is tick, truth or sim_config
  P2: every channel of `outputs` is an output (is_output)
  P3: `inputs` and `outputs` hold no duplicates
```

`valid(truth_type<T>)` and `cast<To>(truth_type<From>)` are required by the concept, found by ADL. They are named conversions, as `TruthState`'s are today.

### Contracts

```text
P::decode_truth(b) -> optional<truth_type<double>>
  Post: engaged iff b is a well-formed encoding

P::feed(s, rec) -> result<ok_t, string>
  Pre:  rec.channel ∈ P::inputs
  Post: success ⇒ exactly one event was handed to s, decoded and cast to s's scalar type,
                  with the checks the profile makes (mobility: today's — an IMU sample must be
                  valid() once cast; a camera frame is passed as decoded)
        error   ⇒ s is unchanged; the message names the record's kind and t (today's messages, verbatim)

P::encode(y, t, out)
  Post: appended exactly |P::outputs| records, the k-th on P::outputs[k], at time t,
        each payload the codec's encoding of cast<double> of that part of y
```

### `RunnableStack<S>` (core)

```text
S names: scalar_type, and profile (a StackProfile)
S has:   on_truth(const profile::truth_type<scalar_type>&)
         on_tick(Timestamp) -> profile::output_type<scalar_type>
         and profile::drives<S>
```

A stack names its profile explicitly (`using profile = MobilityProfile;`). There is no default: a stack that names none is not a `RunnableStack`, and it fails to compile where it is used.

### The Runner (core, `harness/runner.hpp`)

Its model and invariants I1 and I2 are unchanged. It dispatches a record in this order:

1. a pass-through channel: recorded, not consumed (unchanged);
2. `tick`: the ordering check (unchanged), then `on_tick`, then `P::encode`;
3. `truth`: `P::decode_truth`, `valid`, `cast<T>`, `valid` again, then `on_truth` (today's steps and messages);
4. `sim_config`: skipped (unchanged);
5. a channel of `P::inputs`: `P::feed`;
6. anything else: "not an input channel" (unchanged).

One precondition is added: no pass-through channel is one of `P::inputs` (I2 extended, asserted).

`Output` moves to `compose/profile.hpp` as `branes::anatomy::Output`, because a profile's `encode` produces it. `harness::Output` stays as an alias, so racing-drone compiles unchanged there.

### The mobility profile (`compose/tick.hpp`)

`MobilityProfile`: `truth_type = TruthState`, `output_type = TickOutput`, `inputs = {imu, camera}`, `outputs = {state, reference, command}`. Its `feed` and `encode` are the Runner's current IMU, camera and tick code, moved verbatim.

### Truth consumers (`slots/slot_context.hpp`)

- Core: `ConsumesTruth<S, Truth>`, a slot with `process_truth(const Truth&)`.
- Mobility: `TruthConsumer<S, T>` keeps its name and meaning (`ConsumesTruth<S, TruthState<T>>`).

I am not reusing `TruthConsumer` with a truth type as its second parameter. `TruthConsumer<Est, double>` would still compile and quietly be false, and a ground-truth slot would stop getting truth.

### Constraints on the mobility-only loops

`run_closed_loop` and `run_sitl` decode the `command` and `state` channels. They take a `MobileStack` (a `RunnableStack` whose profile is `MobilityProfile`, in `compose/tick.hpp`), so a stack of another profile fails there by name, not deep in a decode. `RunScorer` takes no stack and keeps its mobility types.

### What 1a found

- **The log reader must be told a profile's channels** when they are not `log::Channel` enumerators (the test profile's 8 and 19). The reader's `mission` list names them today, separately from the Runner's pass-through list. Step 2 decides whether a profile's channels join the registry.
- **The bridge generator exports `anatomy::detail`**, as it already exports `codec::detail` and `sim::detail`. 1b's regenerated `anatomy.hpp` gains a `branes::racing::detail` block with the profile checks' helpers.

## Out of scope (step 2 or later)

These are mobility contracts too, but step 1 does not touch them:
- `BasicSlotContext`'s vehicle config and `MissionWorld`'s start pose;
- the codecs' file and namespace;
- `log::Channel`'s enumerators;
- `dump.hpp`'s per-channel printing;
- the namespace or library of `MobilityProfile`.

Step 2 decides them with the library split.

## The PRs

| PR | Repo | Content |
|---|---|---|
| **1a** | anatomy | This plan; ADR 0001's status set to Accepted; `profile.hpp`, `MobilityProfile`, the generic Runner, `ConsumesTruth`, the loops' constraints; the test missions name their profile; a new test profile. Breaking (`feat!`), so release-please proposes v0.2.0 |
| **1b** | racing-drone | Pin anatomy v0.2.0; `Stack` names `using profile = MobilityProfile;`; regenerate the bridge headers; check byte-identity and log compatibility |

## Acceptance (stated before any test is run)

**anatomy (1a):**
1. All 102 existing tests pass. The only edits to them are the test stacks' new `using profile` lines and the include they need.
2. A new test mission with a non-mobile profile: a truth type of two target positions, one input channel (id 8; first written as 7, which `neutral_mission.cpp` already uses for its depth channel) carrying a scalar measurement, and an output type of one assessment on output channel 19. Its `on_tick` output depends on the measurements and the truth it was fed. It passes three checks:
   - the Runner feeds it, and rejects an undecodable and a non-`valid` record of each kind with an error;
   - `replay_log` and `verify_outputs` reproduce a run's outputs record for record;
   - `static_assert`s: the stack is a `RunnableStack`; a stack without `profile` is not; a profile that breaks P1 or P2 is not a `StackProfile`.
3. CI is green on every preset (gcc and clang, Debug and Release, MSVC), with clang-format 18 and commitlint clean, and `framework.names_no_mission` still passes.

**racing-drone (1b):**
4. Its 132 tests pass, on gcc and clang.
5. All 39 harness outputs are byte-identical to the pre-split baseline (main f85ff73), compared with `tools/compare_outputs.sh`.
6. Log compatibility: a log written at f85ff73 replays and verifies on 1b.

## Risks

- **Compile-time genericity across the runner (ADR 0001's risk).** It is bounded here to one concept and one dispatch step. The loops stay concrete.
- **A breaking change for clients.** Every stack must name its profile. racing-drone is the only client today (scout-drone is not built yet), and its change is one line.
