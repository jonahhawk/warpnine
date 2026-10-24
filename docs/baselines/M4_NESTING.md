# M4 nesting evidence

Updated 2026-10-24.

## N01 - Parts and jobs (done)

**Implementation:** `crates/warp-nest/src/lib.rs` (`PartDefinition`, `PartSource`, `PartInstance`, `Sheet`, `Exclusion`, `NestConstraints`, `Placement`, `NestJob`, `JobReport`), `crates/warp-nest/src/candidates.rs` (block candidates, loose-group suggestions). `Project.nesting` in `crates/warp-cam/src/project.rs` persists the job; `validate_document` rejects a job whose parts reference geometry that no longer exists.

**Model decisions**

- Parts store references and transforms, never copied curves. A block part uses definition-local references (empty instance path); a loose part lists explicit members and its roles must be members.
- Instances derive from quantity with stable IDs `part/index`, so raising a quantity keeps existing placements. Lowering it reports orphaned placements; `prune_orphans` returns what it removed. Nothing is dropped silently.
- An outline is only suggested when exactly one closed planar loop contains all others. Disconnected outer loops, crossing loops, non-planar curves, meshes and nested blocks are reported as ambiguities; open curves are never promoted to an outline.
- Sheets carry size, thickness, edge margin, grain, rectangular exclusions and an available count; placements name a sheet copy.
- `report()` checks rotation set, mirror permission and part/sheet grain alignment per placement. Geometry feasibility (overlap, containment, exclusions) is N02.

**Tests:** `crates/warp-nest/tests/n01.rs` (11) and `nesting_job_persists_and_is_validated_against_the_scene` in `crates/warp-cam/tests/document.rs`. Fixtures: a bracket block placed directly twice and once through a nested assembly block (instance count 3), loose groups with disconnected/crossing/non-planar loops, an island inside a hole, and the `pocket-island` baseline project. Workspace: 87 tests passed.

**Restrictions:** no solver yet. Projects that never nested serialize without a `nesting` field.

## N02 - Feasibility geometry (done)

**Implementation:** `crates/warp-nest/src/footprint.rs` derives a part's footprint from the scene at check time: CCW outline and CW holes in a local frame whose origin is the outline's bounding-box minimum, plus `source_offset`/`source_z` and a `source_to_sheet` matrix for carrying features into CAM. Placement applies mirror (about local Y), then rotation about the local origin, then translation. Non-planar loops fail with a repair message unless the part has an approved `ProjectionPlane` (XY, XZ or YZ). Holes must lie in the outline plane and inside the outline.

`crates/warp-nest/src/feasibility.rs` checks placements: each outline is inflated by its part's `extra_clearance` for sheet/exclusion tests and by `extra_clearance + part_spacing / 2` for pair tests, so the required gap between two parts is `spacing + extra_a + extra_b`. Sheet containment uses the usable rectangle after `edge_margin`. Exclusions are rectangles shrunk by the tolerance so exact touching passes. Pair tests run a bounding-box broad phase, then an exact `geo` polygon intersection with one side shrunk by the tolerance. Instances on different sheet copies never interact. Parts without a footprint are reported as `NoFootprint`, never skipped. `fits_sheet` reports oversize parts per allowed rotation.

**Tests:** `crates/warp-nest/tests/n02.rs` (8). Independent expectations: L-shape area `3600 - 1600 - 16*pi`; a 50x30 plate parked inside the L's notch 6 mm from both arms passes while bounding boxes overlap, and 1 mm closer fails; two interlocking Ls whose closest approach is tip to tip at `sqrt(2) * (p - 90)` pass at 6.08 mm and fail at 5.80 mm; touching exclusions pass, 1 mm overlap fails; a tilted loop needs a projection and is rejected when its projection is degenerate. Workspace: 95 tests passed.

**Restrictions:** exclusions and sheets are rectangles (irregular sheets/remnants are M8 S07). Clearance inflation uses round joins, so a required gap is measured as true Euclidean distance. `allow_part_in_hole` is stored but not honoured; outlines are treated as solid. Tool diameter is not modelled separately: set `part_spacing` to at least the profiling tool diameter plus kerf allowance, and use `extra_clearance` for tabs and lead-ins.

## N03 - Deterministic first solver (done)

**Implementation:** `crates/warp-nest/src/solver.rs`. `solve(job, footprints, cancel)` keeps locked placements (validated, never moved), discards unlocked ones, and places the remaining instances largest bounding box first, ties broken by part ID then copy index. Each instance tries sheet kinds in job order and copies in order, choosing the first sheet copy with room. On a sheet, candidate points are the usable corner plus each placed body's bounding-box corners offset by the required gap (spacing plus both parts' extra clearance) and exclusion corners; the lowest, then leftmost feasible point across all allowed orientations wins. Orientations come from `Rotations::candidates()` (Fixed list or step multiples, capped at 360) times mirror permission, filtered by `grain_aligned` when both part and sheet have grain. Every candidate is checked with the N02 `Body` rules, so solver output has zero violations by construction. Reasons for unplaced instances: no footprint, grain cannot align, oversize for every sheet, no room, cancelled. `Score` orders layouts by sheet count, then net utilization, then the larger free full-width or full-height strip on the last sheet. No randomness; `seed` is reserved for N06.

**Tests:** `crates/warp-nest/tests/n03.rs` (10). Six 50x30 plates on a 200x100 sheet with 10 mm margin and 5 mm spacing land at exactly (10,10), (65,10), (120,10), (10,45), (65,45), (120,45) with utilization 9000/14400 and a 15 mm top strip remnant; a seventh plate reports "no room" or moves to sheet copy 1 at (10,10) when two copies exist; a 45 mm usable width forces 90° rotation and Fixed([0]) reports oversize; part grain 0 on sheet grain 90 forces 90/270 and Fixed([0,180]) reports a grain reason; a locked plate in the middle is kept and the first free bottom-left spot clears it by 5 mm; a clamp exclusion shifts the first placement to x = 30.01; mixed plates and squares fill the bottom row largest first; cancellation after two placements returns a feasible two-part layout with four "cancelled" reasons. Workspace: 105 tests passed.

**Restrictions:** candidate points are bounding-box corners, so concave parts do not interlock and no part is placed inside another's notch or hole; N06 adds no-fit regions and local search. Each candidate builds a clearance buffer, which is quadratic in placed parts per sheet; adequate for sheet-scale jobs, to be measured against the nesting corpus before N06. Sheet kinds are tried in job order rather than by cost.

## N04 - Nesting workspace (done)

**Implementation:** `crates/warp-app/src/nest_ui.rs` adds a **Nesting** tab. Left panel: parts (add from block candidates with instance counts, or from the CAM geometry browser's preselection as a loose group; suggestions and ambiguities go to the status bar), per-part name, quantity, outline and hole pickers (closed curves of the block or the group), rotation set, mirror, grain, extra clearance and projection plane; sheets (size, thickness, margin, grain, available copies, rectangular exclusions); constraints (spacing, tolerance, seed); **Nest all** / **Cancel**, last score, orphaned-placement repair, unplaced list with solver reasons and a **Place** button, and the live violation list. Centre: the selected sheet copy drawn to scale with usable area, exclusions and grain arrow; placements are drawn in blue (locked amber, violating red). Drag moves a part (committed on release, so one undo step per drag), **R**/Shift+R cycles allowed rotations about the part centre, **M** mirrors when permitted, **L** locks, **Delete** unplaces. Every edit re-runs `check_placements` with the same rules as the solver.

`crates/warp-nest/src/job.rs` runs the solver on a background thread (`NestQueue`) keyed by `input_key` (parts, sheets, constraints, locked placements and footprints; unlocked placements excluded). A result is applied only if its epoch, cancel token and key still match the current document; otherwise it is discarded with a status message. Applying records one undo step. Footprints are cached per document revision and dropped by `invalidate_document`, so undo, open and reimport all refresh them. The job persists in the project through `Project.nesting`, so save/reopen and undo/redo reuse the document mechanism.

`warpnine [project.warp] [--tab ...]` opens a project and tab from the command line for smoke tests. `serde_json` now has `float_roundtrip` enabled workspace-wide; without it a saved coordinate could reload one ulp off, which made identical layouts compare unequal.

**Evidence:** `cargo run -p warp-cam --example nest_demo` builds a job from the native `.3dm` acceptance project (the repeated `Part` block ×8 with grain, the loose `Circle` ×3, and the `Periodic nonplanar` curve ×2 with an approved XY projection after being reported as non-planar), nests on a 60×40 mm sheet with a 10 mm clamp exclusion, verifies zero violations, saves `target/m4-nesting/nest-demo.warp`, reopens it with identical placements and writes `report.txt`: 13 instances, 1 sheet, 34.8% utilization, 15% remnant, 0 unplaced. Debug app smoke test on that project: the tab rendered all placements; pressing R on a part rotated it and the grain and overlap violations appeared in red; dragging a disc onto its neighbour reported the 2 mm clearance violations; two Ctrl+Z restored the clean layout; Nest all ran in the background and reproduced the same deterministic layout and score. `background_queue_returns_keyed_results_and_honours_cancellation` covers the queue. Workspace: 106 tests passed.

**Restrictions:** no zoom/pan in the preview (the sheet is fitted to the panel); exclusions are edited numerically; the seed is stored but unused until N06.

**Next:** N05 placements into CAM setups.
