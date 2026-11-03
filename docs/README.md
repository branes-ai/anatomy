# Documentation map

These documents are published at <https://branes-ai.github.io/anatomy/>. The site is generated from this folder on every merge to main: edit the documents here, never the site. [`index.mdx`](index.mdx) is the site's landing page, and this file is its documentation map.

| Folder | Contents |
|---|---|
| [getting-started/](getting-started/introduction.md) | [Introduction](getting-started/introduction.md), [build & test](getting-started/build-and-test.md), [repository layout](getting-started/repository-layout.md) |
| [arch/](arch/domain-flow-model.md) | Architecture: [the domain flow computational model](arch/domain-flow-model.md) |
| [plan/](plan/move-from-racing-drone/README.md) | Per-issue plans, each with its issues and PRs: [the move from racing-drone](plan/move-from-racing-drone/README.md), [domain types and operator signatures](plan/flow-domains/README.md), [the graph builder and the CPU stack](plan/flow-graph/README.md) |
| [adr/](adr/README.md) | Architecture decision records: [0001, a core with mobility and sensing profiles, and a multi-node layer](adr/0001-core-profiles-multi-node.md), [0002, third-party license compatibility](adr/0002-third-party-license-compatibility.md), [0003, no GPL code; clean-room from papers](adr/0003-no-gpl-clean-room-policy.md), [0004, operators over domains in a dependence graph, not slots](adr/0004-domain-flow-composition.md) |

The plans and the decisions are published on the [architecture site](https://branes-ai.github.io/architecture/anatomy/), alongside every other repo's plans and decisions and their status, rather than here.
