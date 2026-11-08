# Repository Layout

```text
anatomy/
├── apps/         # the tools a mission's gates run: log_compare, two logs equal but for the header fields left out;
│                 #   trace_compare, the first edge two edge traces differ on
├── cmake/        # deps, warnings, test registration (compile_all), composition and flow (a mission's baselines)
├── docs/         # the documents: the source of this site
├── docs-site/    # the Starlight site generated from docs/, published to GitHub Pages
├── glue/         # branes::anatomy_glue: adapters from reflex components to the slots
├── mobility/     # branes::anatomy_mobility: a roaming vehicle's contracts and their domains, profile, configs, slots, dump
├── sdk/          # branes::anatomy (core): time, geometry, camera, domains and signatures, profile, slots, baseline, log and its comparison, harness, util
├── sim/          # branes::anatomy_mobility_sim: the closed-loop simulator on the reflex dynamics
├── tests/        # Catch2 tests, one executable per tests/<layer>/<name>.cpp; fixtures/ holds their data
└── yaml/         # branes::anatomy_yaml: the YAML helpers for config loaders, anatomy's and a mission's
```

The core's headers are under `<branes/anatomy/...>` and the mobility profile's under `<branes/anatomy/mobility/...>` ([ADR 0001](../adr/0001-core-profiles-multi-node.md)). Every header compiles alone against its own library: the `headers_*` object libraries in `tests/` check it as part of the default build when the tests are enabled.

The domain flow vocabulary ([ADR 0004](../adr/0004-domain-flow-composition.md)) is in `<branes/anatomy/flow/...>`: `domain.hpp` (extents, domains, records), `data.hpp` (the value of a domain, stored by its structure), `signature.hpp` (ports, state, signatures), `window.hpp` (what an `accumulate` edge's reader reads), `domains.hpp` (the core messages' domains), `builder.hpp` (a mission's graph described in C++, checked as it is built, and the CPU stack that runs it) and `graph.hpp` (the checked graph, its canonical form and its hash). The mobility profile's message domains, and the neutral composites' signatures and bindings, are in `<branes/anatomy/mobility/flow/...>`.

The [documentation map](../README.md) lists every document.
