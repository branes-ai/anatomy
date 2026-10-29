# ADR 0001: a mission-neutral core with mobility and sensing profiles, and a multi-node layer

**Status:** Accepted
**Date:** 2026-10-28
**Related:** [racing-drone ADR 0008](https://github.com/branes-ai/racing-drone/blob/main/docs/adr/0008-anatomy-mission-framework.md) (anatomy's origin, and its rule that a new client drives each generalization); [racing-drone ADR 0006](https://github.com/branes-ai/racing-drone/blob/main/docs/adr/0006-hot-path-in-process-zenoh-off-path.md) (in-process hot path, Zenoh off it); the [move from racing-drone](../plan/move-from-racing-drone/README.md)

## Context

Anatomy is the mission framework of the Branes.AI stack. Every mission repo composes its vehicle with it. It was generalized from one mission, racing-drone. The second mission, scout-drone, is mobile too. Both mission types roam: a vehicle with an ego state, actuators, and a closed loop.

A different kind of mission is coming: **Edge AI**. A set of stationary sensor nodes observes part of the world, or part of the spectrum, and produces an assessment. The mission capability is **sensor fusion across nodes and modalities**. The platform tier serves it well: cortex for detection, tracking and fusion, synapse for the models, and the KPU for power-limited nodes. Much of anatomy's mechanism serves it too: slot composition, configs, the deterministic log and replay, the channel registry, the batch runner, and the scalar-type sweep.

But several of anatomy's "neutral" contracts are really the contracts of a **mobile vehicle**:

1. **The stack's outputs are flight outputs.** `RunnableStack` and `TickOutput` are a state estimate, a reference, and a thrust command. An Edge AI stack outputs an *assessment*: tracks, classifications, events.
2. **Truth means the vehicle's own state.** `TruthState`, the simulator's truth, and the scorer's `on_truth` describe one vehicle's pose. For Edge AI, truth is the *world*: the targets, emitters or events under observation.
3. **Sensors are mounted on a body.** `VehicleConfig` puts an IMU and a camera on a body frame. Edge nodes have surveyed poses in a site frame, and many modalities: radar, RF spectrum, acoustic, thermal. Some produce measurements anatomy has no contract for: bearing-only, range-Doppler, spectrogram.
4. **The simulator is a quadrotor closed loop.** `BasicSimulator` steps the reflex dynamics under a simulated flight controller. Edge AI needs an open-loop scene simulator: moving targets, and fixed sensors producing detections with noise, occlusion and false alarms.
5. **Everything assumes one node:** one stack, one clock, one process, one log. Fusion across units needs:
   - time synchronization, and clock offsets between nodes;
   - out-of-sequence measurements;
   - transport with latency and loss;
   - per-node logs that replay together deterministically;
   - a choice of where fusion runs.

Point 5 is not only Edge AI's need. Multi-drone racing, cooperative detect-and-avoid, and `counter-uas` all need it too. Counter-UAS is a hybrid: a network of fixed sensors fused into an air picture, and a mobile interceptor.

## Decision

1. **Anatomy becomes a core and profiles.**

   | Library | Holds | Depends on |
   |---|---|---|
   | `branes::anatomy` (core) | What every mission needs: `result`, utilities, time and `TimeBase`, geometry, the camera and sensor-frame contracts, the log and its channel registry, the codec base, the config machinery (errors, the scenario schema, the baseline loader), slot construction and the composition generator, the harness (runner, replay, batch pool), the scalar-type sweep, and **the multi-node layer** (Decision 4) | yaml-cpp |
   | `branes::anatomy_mobility` | What a roaming vehicle adds: the vehicle config, the ego state estimate, the reference and command contracts, FC status, DAA conflicts, the controller, safety-filter, DAA-tracker and FC-bridge slots, `TickOutput`, the flight harness, the synthetic sensors | core |
   | `branes::anatomy_mobility_sim` | The closed-loop simulator (`BasicSimulator`), the reflex glue, the FC and IMU models, the SITL and batch loops | mobility, reflex |
   | `branes::anatomy_sensing` | What a stationary sensor network adds: node configs (site pose, modality), measurement and assessment contracts, the scene simulator, and tracking and detection metrics (GOSPA, detection latency) | core |

   A mission links the core and the profile it needs. racing-drone and scout-drone link mobility. An Edge AI mission links sensing. counter-uas links both.

2. **The core's contracts take the mission's types where they assumed a vehicle.** This is the same move ADR 0008 made for the world type.
   - **Stack outputs:** `RunnableStack` names its output type, and the Runner encodes it through the mission's codec. Mobility's output is `TickOutput`; sensing's is an assessment.
   - **Truth:** the simulator's truth and the scorer's `on_truth` take the profile's truth type. For mobility that is the ego `TruthState`; for sensing, the world's targets.
   - **Events:** the input events (IMU, camera, tick) become a set the profile defines over a common record type. A sensing node has no IMU. It has measurements of its own modalities.

3. **The sensing profile is built when an Edge AI mission needs it, not before.** ADR 0008's rule stands: the client that needs a generalization drives it. The first Edge AI mission repo (or counter-uas, if it comes first) defines the sensing profile's contracts. racing-drone and scout-drone are the regression clients, and their outputs must stay byte-identical through every change to the core. The split in Decision 1 is the target shape; it is reached one client-driven step at a time.

4. **The multi-node layer is part of the core.** Every kind of mission needs it, mobile or stationary.
   - **Nodes and clocks.** Every node has an identity and its own clock. Every record carries the node, its local time, and its synchronized time. Clock offset and drift are estimated (for example from PTP or two-way exchanges), and the estimate is itself logged.
   - **Messages.** Inter-node messages share one envelope: source node, sequence number, local and synchronized times, the channel, and the payload in the same codecs as the log. Measurements carry their own time, so fusion can handle out-of-sequence arrivals. That handling is the fusion slot's job, and the framework guarantees the times it needs.
   - **Transport behind an interface.**
     - **In SITL:** an in-process, deterministic transport simulates latency, loss and clock drift from the scenario's seed, so a multi-node run is as reproducible as a single-node one.
     - **Deployed:** an adapter (Zenoh, following racing-drone ADR 0006) carries the same envelopes. The hot path inside a node stays in-process.
   - **Logs and replay.** Each node writes its own log. Replay merges them deterministically, in the order of (synchronized time, node, sequence), and can replay one node, a subset, or all of them. A merged replay of a SITL run is byte-identical to the run.
   - **Where fusion runs is the mission's choice:** node-local, central, or hierarchical. It is a slot and a deployment choice, and the framework supports all three through the same envelopes.

5. **The order of work:**
   1. Generalize the core's outputs, truth and events (Decision 2). racing-drone and scout-drone stay byte-identical.
   2. Split the libraries into core, mobility and mobility_sim (Decision 1). The mission repos switch their links, and stay byte-identical.
   3. Build the multi-node layer with the simulated transport and merged replay. Its first client is multi-drone racing (racing-drone B4) or counter-uas.
   4. Build the sensing profile with the first Edge AI mission.

   Steps 1 and 2 can start as soon as this ADR is accepted. Steps 3 and 4 wait for their clients.

## Consequences

- **The framework serves stationary and mobile missions, and hybrids,** with one mechanism for composition, logging, replay, simulation and scoring. Their results line up across mission types.
- **The core gets narrower,** and what a roaming vehicle needs moves to a profile. Mobile missions change their link lines (step 5.2). anatomy is at 0.x, so this ships as minor releases, and each mission repo moves when it bumps its pin.
- **Multi-node replay is a new invariant** the core must keep: merged per-node logs replay deterministically. It costs a log-format extension, adding the node id and synchronized time to records, under a version bump. Old single-node logs must still read, as one node.
- **The sensing profile's contracts are not designed here.** Measurement types, assessment types and the scene simulator are decided with the first Edge AI mission. This ADR fixes only where they go and what they must not break.
- **One risk:** a core that is generic over outputs, truth and events adds compile-time genericity across the runner, simulator and scorer. The mitigation is ADR 0008's: change the core only with a client in hand, and check every change against the existing clients' byte-identical outputs.

**Current state:** accepted. Step 1 is done ([plan](../plan/generalize-core/README.md), [anatomy#7](https://github.com/branes-ai/anatomy/issues/7); anatomy v0.2.0). Step 2 is in progress ([plan](../plan/split-libraries/README.md), [anatomy#10](https://github.com/branes-ai/anatomy/issues/10)).
