# M3 follow-up: verified output and directed open pockets

Implemented 2026-10-24. C07 is complete for the supported output subset; C02/C03 broader fixture contact and controller compensation remain open in the build plan.

## Changes

`post_checked` now interprets the exact generated program and compares every motion/dwell event with its source toolpaths before the desktop app exposes it for preview, export or machine loading. Checks cover positions, rapid/feed classification, feed units, arc centres, directions and sweep, drilling pecks and return levels, dwell, active tool and spindle direction/speed. Unsupported commands, altered custom-post motions and missing program/spindle termination fail with a line-specific error. Custom pauses and unmodelled macros are deliberately blocked.

The first posted rapid now establishes Z clearance before XY travel. Absolute IJ output explicitly sets G90.1; configurable unit/distance/feed/plane modes are established after the preamble. GRBL and TurboCNC presets reject absolute IJ and canned-cycle configurations. The interpreter now recognizes full circles with centre words and no XYZ words, and rejects unsupported G95 feed-per-revolution semantics. Arc-sweep checks reject rounding that would change a short arc into a complete circle.

Every operation's clearance must be above all setup fixture boxes, covering the height needed for transfers between operations. Existing conservative swept tool/shank/holder tests still apply within each operation.

Open pockets now accept directed multi-segment open walls. Both endpoints must lie on rectangular stock edges; the stock perimeter closes the clearing region on the wall's left side. Reversing the wall selects the opposite region. Closed clearing extents and island loops remain supported. Self-crossing, overlapping or ambiguous closures fail rather than guessing.

## Evidence and reproduction

- `cargo test --workspace`: **75 tests passed**.
- `post_qualification.rs` uses hand-calculated cycle sequences independently of both the generator's cycle expander and interpreter. All six supported drilling cycles run through all five presets in both unit systems.
- Arc tests cover CW/CCW, helices, full circles, linearized output, absolute/incremental centres, modal output and machine-Z tool changes. Deliberately changed endpoints, feeds, arc directions, arc modes, tools, spindle commands, feed modes and custom macro motions must be rejected.
- Every operation menu entry also passes checked posting.
- The native Rhino acceptance job verifies **81,894 motion/dwell events per post**, retains identical regenerated moves after reopening, and reports **zero rapid stock collisions**, with **36.420 mm³ removed** on the 512-cell simulation grid.

```powershell
cargo test -p warp-cam --test post_qualification --test m3
cargo run -p warp-import-3dm --example m3_acceptance
```

The example writes `target/m3-acceptance/report.txt`, five checked programs, `rhino-job.warp`, the dedicated T20 tool library, and `open-wall-job.warp` plus `open-wall.tap`. In the desktop app import the tool library, open either project, generate and choose Preview G-code. Successful output shows a motion-verification status message. No machine connection or execution is part of these tests.

Desktop smoke test: the final release executable opened `open-wall-job.warp`, generated the directed-wall pocket, and displayed checked G-code with **G-code motion verification passed for the current operations**. The preview visibly starts with Z clearance before XY. The app was left open on this result.

## Limits

Verification uses setup-space coordinates with zero external WCS/tool-length tables. It does not certify a physical controller or the machine's initial position. Preset and arbitrary macro behavior beyond the interpreter's supported subset is not claimed. Linearized-arc emission and expected trace use the shared tessellator; separate analytic-cylinder tests check generated arc samples. Canned-cycle expectations use an independent expander and hand-calculated fixtures. Rounding tolerance is derived from output precision; output too coarse to preserve motion semantics is rejected.

The app checks current generation keys before posting and re-verifies newly emitted text on every output action. Editing geometry, setup, fixtures, tools or parameters invalidates generated paths; editing post settings clears the G-code preview. No verification result can authorize a later edited program.

Reference semantics for arc modes and supported canned cycles were checked against the [official LinuxCNC G-code reference](https://www.linuxcnc.org/docs/stable/html/gcode/g-code.html). This reference is not evidence of physical-machine qualification for other presets.
