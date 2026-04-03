# Contributing to auto-drone

## Workflow

1. **Pick an issue** — work from the [issue tracker](https://github.com/branes-ai/auto-drone/issues). Assign yourself.
2. **Branch** — create a branch from `main` named `<issue#>-<short-description>` (e.g., `3-isaac-sim-setup-docs`).
3. **Commit** — use [Conventional Commits](#conventional-commits) for every commit.
4. **PR** — open a pull request against `main`. The PR title must also follow Conventional Commits format.
5. **CI must pass** — all three platform builds (Linux, macOS, Windows) and the commit lint check must be green before merge.
6. **Review** — at least one approval required.

## Conventional Commits

Every commit message must follow the [Conventional Commits](https://www.conventionalcommits.org/) specification:

```
<type>(<scope>): <subject>

[optional body]

[optional footer(s)]
```

### Types

| Type | When to Use |
|------|-------------|
| `feat` | New feature or capability |
| `fix` | Bug fix |
| `docs` | Documentation only |
| `style` | Formatting, whitespace (no logic change) |
| `refactor` | Code restructuring (no new feature, no bug fix) |
| `perf` | Performance improvement |
| `test` | Adding or updating tests |
| `build` | Build system or dependency changes |
| `ci` | CI/CD configuration changes |
| `chore` | Maintenance (updating .gitignore, etc.) |
| `revert` | Reverting a previous commit |

### Scopes

Scopes map to project components. Use the most specific one that fits:

| Scope | Component |
|-------|-----------|
| `zenoh` | `libs/zenoh_interface/` |
| `data-types` | `libs/data_types/` |
| `control` | `libs/control_algorithms/` |
| `bt` | `libs/behavior_tree/` |
| `perception` | `autonomy_stack/object_tracking/` |
| `avoidance` | `autonomy_stack/obstacle_avoidance/` |
| `airsim` | `sim_interfaces/airsim_*/` |
| `isaac` | `sim_interfaces/isaac_*/` |
| `mission` | `packages/mission_framework/` |
| `demo` | `demos/*/` |
| `ci` | `.github/workflows/` |
| `docs` | `docs/` |
| `cmake` | Build system files |

New scopes are allowed — the linter will warn but not block.

### Examples

```
feat(isaac): add headless multirotor simulation node
fix(control): correct PID anti-windup clamp direction
docs(docs): add virtual world prerequisites guide
test(perception): add unit tests for PerceptionEngine edge cases
ci(ci): add conventional commit linting to PR checks
refactor(mission): extract phase registry into separate module
build(cmake): upgrade Catch2 to v3.6
```

### Referencing Issues

Link commits to issues in the footer:

```
feat(mission): add follow-target phase for solitary sentinel

Implements area-based distance control with yaw and vertical
centering for single-target tracking missions.

Refs: #10
```

Use `Fixes: #N` to auto-close an issue when the PR merges.

## CI Checks

Every PR triggers:

| Check | What It Validates | Required to Merge |
|-------|-------------------|-------------------|
| **Conventional Commits** | PR title and all commit messages follow the format | Yes |
| **Linux Build + Tests** | Builds with GCC, runs CTest suite | Yes |
| **macOS Build + Tests** | Builds with Clang, runs CTest suite | Yes |
| **Windows Build + Tests** | Builds with MSVC, runs CTest suite | Yes |

All checks must pass before a PR can be merged.
