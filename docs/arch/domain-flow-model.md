# The domain flow computational model

**Related:** [ADR 0004](../adr/0004-domain-flow-composition.md) (anatomy adopts this model for composition); [ADR 0001](../adr/0001-core-profiles-multi-node.md) (the core, the profiles and the multi-node layer); [racing-drone ADR 0001](https://github.com/branes-ai/racing-drone/blob/main/docs/adr/0001-baseline-composition-framework.md) (the slots this model replaces)

This document describes how a mission capability is expressed so that it can run on a CPU, and be compiled onto a distributed data path such as the KPU's compute fabric or a TPU's systolic array. A capability is a graph of **operators** over **domains**, joined by read/write **dependences**, and driven by **events**. A compiler and a runtime lower that graph onto the hardware. The author of the graph never writes the lowering.

## 1. Why slots are not enough

Today a mission's stack is a fixed pipeline of **slots** (racing-drone ADR 0001). A slot is a C++20 concept for a component role: estimator, planner, controller, safety filter. A baseline fills each slot with one implementation, and the mission's `Stack` class wires the slots together by hand in `on_tick`. Three properties of that design stop it from reaching a parallel fabric:

- **The topology is code.** The dependences between steps are calls inside `on_tick`. A compiler can't see them, so it can't bufferize or schedule them.
- **A port's type is a scalar type `T`.** A matmul, an LU solve or a Kalman update consumes and produces aggregates: vectors, matrices, tensors. To allocate buffers and divide the work, a compiler needs their size, shape and structure, of which `T` is only the element type.
- **State is hidden.** A component keeps its state in its members. A compiler that must place a Kalman covariance on the fabric, or keep a weight matrix stationary, has to see that state as data.

## 2. The layers

| Layer | What it says | Who writes it | Phases visible |
|---|---|---|---|
| 1. Domains | index set, element type, structure, shape | the operator library and the mission | no |
| 2. Operators | transformations over domains, their state, their computational domains | the library (leaf operators); the mission or a baseline (composite operators) | no |
| 3. Dependence graph | the read/write edges between operators | the baseline | no |
| 4. Events | triggers, notifiers and the regions they fire | the mission and the baseline | no |
| 5. Lowering | each domain's phases, layouts, schedules and placement; tiling; bufferization; the resource contract | the compiler and the runtime | yes |

A baseline writes layers 2 to 4. It names operators, the algorithms, and never a target's implementation of one. So a baseline is the same algorithm on every target: only layer 5 differs between the CPU, the KPU and a TPU, where the compiler picks each leaf operator's implementation for the target and lowers it. Baselines stay comparable on any target.

## 3. Domains

A **domain** is an index set with an element type and a structure. The same word names the data and the computation, because in domain flow they are the same kind of object:

- **A data domain** is the index set of a value: a vector's `[0, n)`, a matrix's `[0, m) × [0, n)`, a tensor's box.
- **A computational domain** is the index space of an operator: the `(i, j, k)` cube of a matmul, the triangular domain of an LU factorization.
- **A face** is where a computational domain's wavefront reaches its boundary. An operator's input and output data domains are faces of its computational domain.

The domain is what stays the same as the data moves through memory and the fabric. What changes is its embedding, its **phase** (section 4).

```text
domain State = vector<T>[n]          n = 15
domain Cov   = matrix<T>[n, n]       symmetric
domain Meas  = vector<T>[≤m]         m = 8, with a count
domain Image = tensor<u8>[480, 640, 1]
```

- **Shape** is static or **bounded**. A bounded dimension has an upper limit and a count or a mask. Missions have sizes that vary at run time (detections, obstacles seen, gates in view), and a bound gives the compiler a size to allocate while the data stays variable. Data that is not an index set (a list that grows without bound, a tree) is not a domain, and runs on the CPU.
- **Dimensions are symbolic** (`n`, `m`) and are bound by the vehicle and baseline configs, so one operator serves a 15-state and a 9-state filter.
- **Structure** (dense, symmetric, triangular, banded, block, sparse) is part of the type. It changes the computational domain and the storage, and the compiler can exploit it only if the type says so.
- **The element type is per domain.** A covariance can be `f32` while the IMU path is `posit<16,1>`, so a precision sweep is per edge rather than one `T` for a whole stack.
- **A subdomain** is a tile of a domain, of either kind (section 7).
- **A record** is a list of named fields, each a domain or a record. A message is often a bundle of domains: a state estimate is a timestamp, 3-vectors, a quaternion and a symmetric covariance. A record is also an element type, so a bounded vector of records is a domain (the samples of a reference trajectory). A record's layout is not part of it: the compiler may store its fields apart or together (section 9).

## 4. Domain phases

At each point of a lowered program, a domain is in exactly one of three phases:

| Phase | Form | Embedding | Where it is |
|---|---|---|---|
| **at rest** | flat | a **layout**: index → address | in a flat memory, at a memory level |
| **in flight** | spatial and in time | a **schedule**: index → (processing element, time) | streaming through the fabric, entering and leaving at faces |
| **resident** | spatial, held | an **allocation**: index → processing element | held on the fabric across firings |

**The memory level is an attribute of the at-rest phase.** There are two levels to start with: **on-chip** and **off-chip**. A domain's level follows from the CSP schedules that implement its operators. When an operator is divided into a sequence of subdomains, the sequence creates reuse that only the runtime can resolve, so the level is a plan the runtime can revise. Whether the level stays an attribute or becomes part of the phase depends on what the ABI needs for forward and backward compatibility. That is still open (section 11).

### Transitions

| Transition | From → to | What it does |
|---|---|---|
| **stream in** | at rest → in flight | reads by layout, and enters through an input face in the order the schedule needs |
| **drain** | in flight → at rest | serializes an output face into a layout |
| **pipeline** | in flight → in flight | an output face feeds the next operator's input face directly |
| **park** | in flight → resident | holds a domain on the fabric after a firing |
| **resume** | resident → in flight | the next firing starts from the held domain |
| **evict** / **restore** | resident ↔ at rest | spills a held domain to memory, or brings it back |
| **relayout** | at rest → at rest | transposes, reblocks or reorders a domain in memory |

**The default for a flow edge is drain, then stream in, with a relayout between them if needed.** Each operator has its own schedule and its own face shapes, and dependent operators rarely have congruent faces. Even a direct pipeline would need a transformation in space and time between them, and flat memory performs one for free. **Pipeline** is an optimization the compiler applies when two faces are congruent. **A carried edge** (section 6) keeps its state resident between firings, by park and resume, where the resources allow it. It is evicted to rest, and restored, only when the runtime spills it. Stationary operands are parked the same way.

### Prologues and epilogues

An elementwise operator, whose computational domain is its data domain, is fused into a transition instead of being scheduled in its own right. An MLP layer is the example:

```text
y = act(W·x + b)
```

The linear operator `W·x` is divided into subdomains. Each output subdomain drains to memory with the space-time structure the division gave it, which is hard to describe in terms of the matmul's computational domain. But on its way to memory, the subdomain is exactly the shape the bias add and the activation need. So `+ b` and `act` are applied to each subdomain as it drains, as an **epilogue** of the drain. The matmul's own space-time structure is not useful for fusing them.

An elementwise operator applied to a subdomain as it streams in is a **prologue**. The rule: an elementwise operator fuses onto a transition of the subdomain that feeds it, not onto the computational domain of its producer.

## 5. Operators

**Leaf operators** are the fabric's instruction set as the compiler sees it: matmul; the LU, QR, Cholesky, Schur and SVD factorizations; triangular solves; convolutions; ODE integrator steps; Kalman predict and update. A leaf declares:

- **its ports**, each a domain with a direction: **in**, **out**, or **inout**. An inout port updates in place, which the compiler sees as a write after a read of the same buffer;
- **its computational domain**, whose faces are its ports;
- **its state**, as domains (below);
- **the targets it has implementations for:** a KPU Domain Flow Program, a TPU lowering, a CPU kernel. The library provides the implementations, and which one runs is the lowering's choice (layer 5), never the baseline's. A leaf with no fabric implementation runs on the CPU.

**State is a declared domain.** It is carried from one firing to the next, so its buffer outlives a firing. It can stay on the fabric, it is what a checkpoint saves, and the compiler sees all of this:

```text
operator kf_update
  state  x: State, P: Cov                    # carried between firings
  in     z: Meas, H: matrix<T>[≤m, n], R: matrix<T>[≤m, ≤m] symmetric
  body
    S = H·P·Hᵀ + R                           # matmul, matmul, add
    L = cholesky(S)
    K = solve(L, H·P)ᵀ                       # triangular solves
    x ← x + K·(z − H·x)                      # inout on x
    P ← (I − K·H)·P                          # inout on P
```

**A composite operator's body is a graph.** A mission's estimator is a composite whose leaves are fabric operations. The compiler can keep a composite whole, or open it to fuse and tile its leaves. The composites with a stable signature are what slots were: the units a mission names and a baseline fills. A scoreboard baseline may change only what is inside the mission's composites, while a research baseline may change the whole graph between the mission's external ports ([ADR 0004](../adr/0004-domain-flow-composition.md), decision 6).

## 6. The dependence graph

The nodes are operators. An edge joins an output port to an input port of the same domain. Edges are the read/write dependences the compiler needs to bufferize:

- **A flow edge** is a read after a write within one firing.
- **A carried edge** runs from a state output to the next firing's state input. Every cycle in the graph must cross a carried edge.

Each edge has one producer, and its consumers are explicit. Each edge is a buffer candidate: the compiler decides whether it is pipelined, drained and streamed in, fused into a prologue or an epilogue, or eliminated. Each edge is also a channel that can be logged, so a replay can check any intermediate result, not only the stack's outputs.

```text
 imu ─(trigger 1 kHz)─> [kf_predict]──x, P (state)──┐
                                                    v
 frame ─(trigger 90 Hz)─> [gate_detect]──z, H──> [kf_update]──x (hold)──> control region
```

## 7. Divide and conquer

An operator larger than the fabric is divided into a sequence of **suboperators** over subdomains. A 1M × 1M matmul on a 128 × 128 fabric:

```text
operator matmul C = A·B          computational domain (i, j, k) ∈ [0, 1M)³

lowered
  for each tile (I, J, K) in the tiling order
    suboperator over (I, J, K)          128 × 128 × 128
    A[I, K], B[K, J]   at rest → stream in
    C[I, J]            resident across K (accumulates)       park / resume
                       drained when K is done → at rest
```

Dividing serializes each output face into flat memory for later reuse. The order of the subdomains creates reuse of the at-rest tiles, which the runtime resolves (section 4). The partial sums of `C` are resident across `K`, as Kalman state is across firings: the model needs no new concept for either.

## 8. Events: triggers and notifiers

Events carry no data. They attach to the same nodes as the data:

- **A trigger** starts something: a sensor sample arrives, a clock ticks, a mode changes. A trigger fires a **region**, the subgraph that runs in response. A region's first domains stream in from at rest, or resume from resident.
- **A notifier** signals that something has completed: a region, or an operator. It goes to the runtime's resource controller, and it tells the controller which resources are up for renegotiation. The drained domains' fabric resources are released, but the resident domains' are not.

**An edge between regions that run at different rates is at rest at the boundary.** It carries a rate rule, which determines its buffer:

| Rule | Example | Buffer |
|---|---|---|
| hold | the controller reads the latest estimate | a double buffer |
| accumulate | IMU samples integrated between frames | an accumulator domain |
| queue | detections processed in order | a ring, as deep as the rate ratio times the latency bound |

Determinism comes from a fixed order within each firing of a region, and a fixed order between triggers that arrive at the same time. A region's graph can have cycles, but every cycle crosses a carried edge (section 6), and a carried edge is a boundary between firings. Without its carried edges, a region's graph is acyclic, and its operators run in a fixed topological order of it. That keeps replay exact (anatomy's deterministic log).

```text
region vision   trigger: frame (90 Hz)
  stream in  Image                    → gate_detect → drain z, H
  stream in  z, H;  resume P, x       → kf_update   → park P, x
  drain      x → at rest (hold, read by the control region)
notify vision.done   releases: gate_detect's processing elements and buffers
                     holds:    P, x (resident)
```

## 9. Lowering and the resource contract

Domain flow is about **negotiating in advance** all of the bufferization and the compute fabric, so that a computation streams in a space-time optimal way. The compiler lowers each region in this order:

1. **Tile** each computational domain larger than the fabric into a sequence of suboperators.
2. **Assign each domain its phase sequence:** pipeline where faces are congruent, park state and stationary operands, fuse elementwise operators into prologues and epilogues, and drain the rest.
3. **Choose a layout** for each at-rest phase, so that the stream in that follows it matches its consumer's input face, or add a relayout.
4. **Place** the at-rest phases at a memory level, and the resident domains on processing elements.
5. **Schedule** each region, with its notifiers where it releases resources.

The result is a **resource contract** for each region: the processing elements, the buffers and their shapes, and the memory it needs. The runtime honors the contract, with two exceptions:

- **Pins are suggestions.** A baseline may ask for a domain to stay resident, for latency. Only the compiler and the runtime know whether resources are available to honor the request, so a pin is a hint they weigh, never a requirement.
- **The runtime may evict.** When an operator of higher priority arrives, the runtime can evict a region's pipeline, including its resident domains. The evicted domains are restored before that region next runs, at the cost of a restore.

## 10. What happens to today's code

Step 2 of ADR 0004 runs today's code from the graph, unchanged, and step 5 retires the slots ([plan](../plan/flow-graph/README.md)).

- **Slot concepts and their implementations** become the CPU implementations of operators. A composite's binding maps each method of the slot's concept to a handler, a struct whose static `call()` calls the method, so every implementation is an operator of its composite as it is (`mobility/flow/bindings.hpp` for the neutral composites).
- **`Stack::on_tick`** becomes the control region, its firings in the checked order, and the other `on_*` handlers become the regions their triggers fire. The mission still writes the hooks: each hands the CPU stack its input and reads its outputs, and holds no logic.
- **The profile's channels** become the mission's external ports. The log keeps its channels, so a graph build logs what its slot build logs, and the two logs differ only in `composition_hash`.
- **`TruthConsumer`** disappears: `truth` is an external port that only the SITL interface has, so a graph that reads truth is marked SITL-only, as the checked graph says (G9). Until the slots retire, the estimator's binding hands the truth to an estimator that is a `TruthConsumer`, and does nothing for one that isn't, so a graph with that binding is SITL-only whichever estimator fills it.
- **`make_slot` and `from_context`** remain how an operator's implementation is built from the mission's context: the stack builder builds each operator with `make_slot`.
- **`composition_generate()` and the slot baselines' targets** stay beside the graph builds until step 5, and a gate holds each graph build equal to its slot build.

## 11. Open questions

1. **ABI compatibility.** Is the memory level an attribute of the at-rest phase, or part of the phase? This depends on how the ABI must stay forward and backward compatible.
2. **Conditional execution.** Mode switches, an estimator that falls back when its health degrades, gating a detection that fails a validation test: alternative regions selected by a trigger, or a data-dependent choice within one region?
3. **What a notifier carries.** Only that something completed, or also which resources it releases, so the controller needs no further query?
4. **The unit of comparison.** Decided in [ADR 0004](../adr/0004-domain-flow-composition.md), decision 6. A scoreboard baseline may change any graph inside the mission's composites, but must keep the composites themselves, their ports and how they connect. A research baseline may change the whole graph between the mission's external ports, and its results are marked as research.
5. **Transformations between faces.** When two faces are almost congruent, is a spatial transformation on the fabric a transition, or a small operator in the graph?
