# Repository Layout

```text
anatomy/
├── cmake/        # deps, warnings, test registration (compile_all), composition (a mission's baselines)
├── docs/         # the documents: the source of this site
├── docs-site/    # the Starlight site generated from docs/, published to GitHub Pages
├── glue/         # branes::anatomy_glue: adapters from reflex components to the slots
├── mobility/     # branes::anatomy_mobility: a roaming vehicle's contracts and their domains, profile, configs, slots, dump
├── sdk/          # branes::anatomy (core): time, geometry, camera, domains and signatures, profile, slots, baseline, log, harness, util
├── sim/          # branes::anatomy_mobility_sim: the closed-loop simulator on the reflex dynamics
├── tests/        # Catch2 tests, one executable per tests/<layer>/<name>.cpp; fixtures/ holds their data
└── yaml/         # branes::anatomy_yaml: the YAML helpers for config loaders, anatomy's and a mission's
```

The core's headers are under `<branes/anatomy/...>` and the mobility profile's under `<branes/anatomy/mobility/...>` ([ADR 0001](../adr/0001-core-profiles-multi-node.md)). Every header compiles alone against its own library: the `headers_*` object libraries in `tests/` check it as part of the default build when the tests are enabled.

The domain flow vocabulary ([ADR 0004](../adr/0004-domain-flow-composition.md)) is in `<branes/anatomy/flow/...>`: `domain.hpp` (extents, domains, records), `signature.hpp` (ports, state, signatures), `domains.hpp` (the core messages' domains), `builder.hpp` (a mission's graph described in C++, and checked as it is built) and `graph.hpp` (the checked graph, its canonical form and its hash). The mobility profile's message domains and composite signatures are in `<branes/anatomy/mobility/flow/...>`.

The [documentation map](../README.md) lists every document.
