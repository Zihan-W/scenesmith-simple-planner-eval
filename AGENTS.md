# AGENTS.md

## Scope

This repository primarily contains Python and robotics code.

Prefer pragmatic, prototype-oriented engineering. Keep implementations simple, explicit, and easy to debug.

## Modification Rules

* Make the smallest coherent change that fully implements the requested behavior.
* Do not perform unrelated refactoring or cleanup.
* Preserve existing APIs, config formats, dataset schemas, and external behavior unless explicitly requested.
* Reuse existing utilities and abstractions instead of creating parallel implementations.
* Do not introduce new hardcoded paths, compatibility layers, fallback behavior, or legacy branches without a concrete requirement.

## Assumptions

Use reasonable judgment for local implementation details.

Ask the user before making assumptions that affect:

* APIs or data schemas,
* coordinate frames or units,
* robot control semantics,
* timing or synchronization semantics,
* filesystem paths,
* backward compatibility,
* default runtime behavior.

Do not block on minor, reversible implementation choices.

## Coding Style

* Follow Google Python Style Guide.
* Add appropriate docstrings to public modules, functions, methods, and classes.
* Prefer clear control flow over unnecessary abstractions.
* Avoid speculative generalization and duplicate helpers.
* Preserve useful existing comments.

## Error Handling

* Fail loudly when the system cannot continue correctly.
* Do not add broad or unnecessary `try/except`.
* Do not silently catch errors.
* Do not add retries, fallbacks, or default substitutions unless explicitly required.

## Robotics

Do not guess coordinate conventions, units, quaternion ordering, control frequency, timestamp semantics, or absolute/relative command semantics when they materially affect behavior.

Do not weaken existing robot safety checks or introduce physical robot motion unless explicitly requested.

## Validation

Validation should be proportional to the change.

Permanent tests are not required for every modification. Add tests mainly for stable contracts, deterministic logic, or meaningful regressions.

For experimental code, prefer lightweight local validation, representative runs, simulation, dry-runs, or hardware validation over large mock-heavy test suites.

Avoid mock-heavy tests that mainly encode implementation details.

Existing tests are evidence of intended behavior, not absolute ground truth.

Do not claim validation that was not performed.

## Definition of Done

A task is complete only when the requested behavior is connected to the intended execution path.

Partial implementations must be reported as partial.

## Communication

Keep final responses concise. State:

* what changed,
* important design decisions,
* validation performed,
* unresolved issues or assumptions.
