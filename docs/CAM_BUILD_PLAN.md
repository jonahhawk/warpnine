# WarpNine CAM build plan

Status: planned, not implemented. Updated 2026-10-24.

This is the execution plan for [the RhinoCAM feature specification](RHINOCAM_FEATURE_PLAN.md). That specification remains the authority for the complete operation inventory and source references. This document breaks the work into implementable changes with dependencies and release gates.

## Build order

| Milestone | Deliverable | Depends on |
| --- | --- | --- |
| M0 | Baseline fixtures and project migration contract | None |
| M1 | Stable geometry, explicit selection, setups and regeneration | M0 |
| M2 | Rhino/openNURBS curve, mesh and block import | M1 |
| M3 | Usable curve-driven 2.5D CAM | M1; M2 for Rhino acceptance |
| M4 | Sheet nesting connected to CAM | M2, M3 |
| M5 | Qualified solid import and core 3D machining | M2 meshing decision, M3 geometry/linking |
| M6 | High-speed adaptive clearing and rest machining | M3, shared stock queries; M5 for 3D |
| M7 | Advanced 3D finishing and remaining roughing | M5, M6 stock states |
| M8 | Specialty operations and full inventory qualification | M3-M7, machine/post capability extensions |

Ship M2+M3 as the first Rhino CAM preview, M4 as the sheet-production release, M5 as the core 3D release, and M6 as the high-speed release. Continue through M8 to complete the requested scope. Nesting must not wait for advanced surface machining.

Resolve the three largest technical unknowns early: trimmed-Brep meshing, engagement-controlled adaptive paths, and undercut-capable verification. Prototype these behind internal interfaces; release them only after their acceptance tests pass. Estimate calendar dates after the prototypes and baseline measurements, rather than treating research-heavy algorithms as routine UI work.

## Implementation rules

- Keep existing DXF/STL workflows working throughout the migration. Introduce adapters before replacing legacy storage and generators.
- Each numbered task below is a reviewable change or small series of changes. Complete its tests before marking it done. All task checkboxes start unchecked.
- Persist geometry and operation intent; generated paths are revision-keyed derived data. An edit must never leave a stale path looking current.
- Add operation behavior, parameters, selection validation, simulation and post support together. A menu entry alone is not a finished feature.
- Use independent geometric expectations and end-to-end fixtures for tests. Avoid tests that merely repeat an implementation formula.
- No machine-control redesign is required for ordinary 2.5D/3-axis paths. Add capability checks where a new cycle or specialty tool actually requires it.

## M0 - Baseline and contracts

- [x] **B01: Capture current behavior.** Run `cargo test --workspace`; record existing failures separately from changes. Create small synthetic DXF/STL/project fixtures for pocket-with-island, open profile, drill pattern, projected open curve and sculpted surface. Use the existing sample files as optional smoke tests, not the sole test corpus.
- [x] **B02: Define versioned project format.** Document migration from the current unversioned `warp-cam/src/project.rs` JSON and how unknown versions fail in [the format contract](contracts/PROJECT_FORMAT.md). Specify stable object/operation/setup identifiers, unit conversion, source provenance and unresolved-reference handling. Implementation remains in M1.
- [x] **B03: Establish accuracy and benchmark contracts.** Record input units, source tolerance, approximation tolerance, collision sampling and output rounding separately. Add a benchmark manifest with fixed part dimensions, tools, parameters, hardware and measured generation/memory statistics. Do not assign a single tolerance label to unrelated approximations.
- [x] **B04: Create the strategy coverage ledger.** Copy every family from the feature specification into a machine-readable or Markdown checklist with states: missing, prototype, verified, released. Track parameter coverage and machine restrictions alongside names.

Completed 2026-10-24: [M0 evidence and known limitations](baselines/M0_BASELINE.md). Workspace: 38 tests passed. Memory measurements are scoped path/grid storage, not peak RSS. M1 implementation is recorded below.

**Gate:** existing behavior is reproducible; migrations and acceptance fixtures have clear expected outcomes. No production code migration begins without B02.

## M1 - Geometry and CAM workflow foundation

Implemented 2026-10-24. See [M1 acceptance and testing](baselines/M1_WORKFLOW.md) for delivered scope and limitations. Face/edge subelements await the B-rep importer; check-surface/fixture collision and stock-dependent regeneration await their machining milestones.

- [x] **G01: Introduce a scene model.** Add `warp-core/src/scene.rs` and `selection.rs`: object IDs, layer IDs, geometry definitions, groups, instance transforms and references to an instance path plus source object/subelement. Preserve analytic/3D geometry separately from tessellation. Detect cyclic instance definitions.
- [x] **G02: Migrate projects without changing machining intent.** Add versioned serialization and a legacy reader. Convert indexed selections to stable references and preserve historical empty-selection behavior as an explicit mode. Keep legacy-to-generator adapters until all generators consume resolved geometry. Test old project -> new project -> save -> reopen -> equivalent selected geometry and paths.
- [x] **G03: Separate geometry roles.** Replace overloaded `Common.curves` usage with machining regions, containment, avoid regions, ordered drive curves, part surfaces, check surfaces, hole locations and start points. Define `Selected`, `AllEligible` and `StockBoundary` explicitly where valid. Reject invalid role/type combinations before generation.
- [x] **G04: Add setups and transforms.** Introduce `SetupId`, model-to-setup transform, WCS, stock, fixture references and ordered operations. Normalize generation in setup coordinates; apply output transforms once. Migrate the current project to one default setup. Test translated/rotated setups and units against known endpoints.
- [x] **G05: Implement the selection UI.** Update `view3d.rs`, `app.rs` and `cam_ui.rs`; add object/layer browser and role-specific pick mode, additive selection, chain reversal, start arrows, hide/isolate and unresolved-reference repair. Add preselection-to-operation behavior. Put new UI components in modules instead of continuing to enlarge `app.rs`.
- [x] **G06: Implement invalidation and bounded jobs.** Add a dependency/revision model and cancellable job queue. Geometry, tool, setup, parameter or predecessor-stock changes invalidate affected paths. A job result is accepted only if its input revision still matches; reordering/deleting an operation during generation cannot attach output to a different operation.
- [x] **G07: Add undo/redo for document edits.** Support selection assignments, parameters, operation reorder, setups and instance placement. Exclude transient viewport navigation and job progress. Restore dependency state and invalidate derived results correctly.
- [x] **G08: Fix projected-curve selection.** Update `ops3d.rs` so an open drive curve is independent of closed containment. Add a regression with one open drive path, a closed boundary and an avoid island; verify the resulting path follows the intended surface and excludes the island.

**Gate:** a saved profile/pocket job retains correct selections after geometry reordering and reopening. Deleting selected geometry produces a repairable error. An obsolete background result cannot overwrite a newer job.

## M2 - Rhino/openNURBS import

Implemented 2026-10-24. See [M2 import acceptance and testing](baselines/M2_RHINO_IMPORT.md). I05 selects explicit Rhino-generated meshes and a future host-side exporter; a generic standalone solid mesher is not part of this milestone.

- [x] **I01: Establish the native reader build.** Add `crates/warp-import-3dm` with a pinned openNURBS SDK, narrow native bridge, explicit ownership/error handling and reproducible Windows build. Document dependency notices and supported file versions. Add an importer trait/result that returns scene objects and structured diagnostics.
- [x] **I02: Read curves and metadata.** Import points, lines/arcs, polylines, polycurves and NURBS curves, UUIDs, units, model tolerance, names and layers. Preserve geometry in 3D; validate planarity when resolving 2.5D geometry. Test nonuniform weights, closed/periodic curves and inch/mm files.
- [x] **I03: Read blocks and meshes.** Import block definitions, nested instances and mesh objects. Resolve transforms without destructive explosion. Test repeated, mirrored, nonuniformly scaled and nested blocks; flag missing linked definitions and unsupported objects. Preserve an instance path so identical block geometry remains separately selectable.
- [x] **I04: Wire import into the app.** Add `Rhino / openNURBS (*.3dm)` to the import dialog, import summary and layer/object browser. Add reimport matching by source UUID and instance identity; unmatched topology/subelements require repair rather than guessed reassignment.
- [x] **I05: Decide the solid meshing approach.** Prototype trimmed Breps/extrusions with holes, seams and curved faces. Compare candidate meshing integration against known analytic surfaces and trim boundaries. Record accuracy, watertightness, speed, supported types and dependency/build costs in `docs/decisions/3dm-meshing.md`. Cached render meshes may be a preview input; their accuracy must be qualified before production machining.

**Gate:** import a Rhino file containing loose curves, repeated blocks and meshes; select and machine only one instance, save/reopen, and preserve scale and placement. Solid objects with unavailable machining meshes display an actionable diagnostic. I05 is a required decision before M5.

## M3 - First usable 2.5D release

Implemented as a bounded 2.5D preview, 2026-10-24. See [M3 evidence and supported limits](baselines/M3_25D_CAM.md). General controller compensation, nonrectangular-stock openings and full solid/fixture contact are not claimed by this preview. Directed open walls ending on rectangular stock edges are supported.

- [x] **C01: Build robust region resolution.** Extend `region.rs` to validate planarity, closed/open chains, nested islands and disjoint regions; identify self-intersections and duplicate/degenerate geometry. Use true segment/region clipping, including boundary intersections, rather than filtering sampled points.
- [ ] **C02: Consolidate entry/link planning.** Extract shared plunge/ramp/helix, leads, retract and linking behavior from generators into a tested planner. Account for stock, tool envelope, fixtures and clearance. Distance alone cannot authorize a low link across an island.
- [ ] **C03: Qualify profile, pocket and facing.** Implement distinct Face Top and stock-driven prismatic Roughing; complete open/closed pocket behavior, islands, finish passes, wall/floor allowance, climb/conventional, slotting, profile sides and tabs. Add cutter compensation modes only with valid lead lengths and post support.
- [x] **C04: Qualify holes and follow paths.** Finish drilling, Engrave/Follow, distinct Hole Pocketing/Hole Profiling, helical entry and thread-milling controls. Test circular features, selected point locations, handedness, breakthrough and output helix direction.
- [x] **C05: Qualify V-carve finishing and chamfer.** Check actual cutter contact, width/depth, narrow corners and depth limits against independent geometric fixtures. Report features the selected tool cannot reach.
- [x] **C06: Complete operation editing.** Consistent geometry/tool/feed/height/cut/entry sections; duplicate, suppress, reorder and regenerate selected/all. Add machine/post capability metadata and block unsupported output actions.
- [x] **C07: Verify output end to end.** For each existing post, post representative operations, re-interpret supported output and compare endpoints, arcs/helices, units, heights and cycle semantics. Keep generation, verification and posted output tied to the same revision.

**Release gate:** from a `.3dm` fixture, create a profile, island pocket, holes and follow path; preview, simulate, post and reopen the job. Include stock and fixture checks for the supported geometry. Existing DXF/STL workflows remain usable.

Remaining qualification: C02 needs general stock/fixture contact beyond conservative envelopes; C03 retains general controller cutter-compensation coverage. Directed open walls now close along rectangular stock edges with explicit left-side intent. C07 is implemented for the supported posts/cycles with runtime motion verification, independent cycle-depth expectations and corruption tests. See [M3 post qualification](baselines/M3_POST_QUALIFICATION.md). The four-operation Rhino preview gate has automated evidence; C02/C03 remain open.

## M4 - Nesting and sheet production

In progress since 2026-10-24. See [M4 nesting evidence](baselines/M4_NESTING.md).

- [x] **N01: Define parts and jobs.** Add `crates/warp-nest` with `PartDefinition`, `PartInstance`, `Sheet`, `NestConstraints` and `Placement`. Store quantities and part-local feature references. Blocks become candidate parts; loose geometry supports explicit grouping and outer-loop/hole suggestions with ambiguity reporting.
- [x] **N02: Implement feasibility geometry.** Derive part footprints in the chosen plane; account for edge margins, tool/lead/tab clearances and clamp exclusions. Add exact narrow-phase polygon collision/containment checks after broad-phase bounds. For nonplanar parts, require an approved projected footprint.
- [x] **N03: Implement a deterministic first solver.** Bottom-left/best-fit placement over allowed rotations; support quantity, grain direction, locked positions, mirror permission and multiple sheets. Unplaceable parts return reasons. Score feasible layouts by sheet count, utilization and usable remnant shape.
- [x] **N04: Add the nesting workspace.** Sheet preview, quantity/rotation/grain controls, drag/rotate/lock, utilization and unplaced-part list, undo and save/reopen. Run optimization as a cancellable job with a stable seed. Validate manual edits with the same feasibility rules as the solver.
- [ ] **N05: Connect placements to CAM.** Generate sheet setups and transformed part instances; reuse operation definitions with per-instance overrides. Preserve inside-before-outside and retention dependencies when ordering by tool. Revalidate tabs, entry points and handedness after transforms.
- [ ] **N06: Improve packing.** Add no-fit-region support and bounded multi-start/local search after the first solver is stable. Use concave shapes and mixed quantities to demonstrate improvements against the recorded baseline. Keep the best feasible layout available during cancellation.

**Release gate:** nest both block parts and loose-geometry parts across multiple sheets, with quantities and grain constraints; generate internal cuts before releasing outlines; post per sheet and reproduce placements after reopening. Require zero clearance violations and no silently dropped quantities.

## M5 - Reliable 3D geometry and core machining

In progress since 2026-10-24. See [M5 3D evidence](baselines/M5_3D.md). M4's N05/N06 remain open in parallel.

- [x] **D01: Productionize the selected mesher.** Implement I05's decision with import cancellation, memory limits, edge/face provenance and accuracy diagnostics. Handle trims, seams, surface orientation and transformed instances. Define explicit support for extrusions and SubD-derived geometry rather than assuming all solid types work.
- [x] **D01b: Import STEP solids without Rhino.** Add `crates/warp-import-step` on a pinned static Open CASCADE build: read AP203/AP214/AP242 solids, free shells and faces; mesh them at the import tolerance; measure the mesh against the exact B-rep with the D01 policy; keep names and units. Rhino remains optional for users who only receive STEP files.
- [x] **D02: Add surface query services.** Introduce a spatial index and cutter-contact/collision queries behind an interface. Keep Z-map acceleration for appropriate jobs, but use sufficiently accurate mesh/surface queries where vertical walls or local detail require them. Bound interpolation and sampling error. Done 2026-10-25: `warp_cam::surface` (exact facet/edge contact on an indexed mesh for every tool profile, adaptive chord refinement to half the tolerance, measured gouge bound); finishing operations use it by default, 3D roughing keeps the Z-map.
- [ ] **D03: Qualify existing 3D strategies.** Horizontal Roughing with linear/stock-offset/part-offset patterns; Parallel; Horizontal/Waterline; Radial; Spiral; Curve Machining. Add exact cut levels, per-region centers and robust containment/avoid clipping. Check links after final path fitting.
- [ ] **D04: Add Projection Pocketing.** Generate planar pocket offsets and project cutter contact onto selected part surfaces; keep it distinct from constant-3D-spacing offsets.
- [ ] **D05: Extend verification.** Add cutter/shank/holder and fixture collision, model gouge/deviation, remaining-stock display and accuracy reporting. Test swept motions, including arcs and helices, rather than endpoints only.

**Release gate:** import a sculpted Rhino solid, rough and finish selected surfaces inside selected boundaries, and verify measured error on flat, steep, curved and trimmed features. Refuse machining when the selected model representation cannot meet the requested accuracy.

## M6 - High-speed and remaining-stock machining

- [ ] **H01: Introduce shared stock queries.** Add a dependency-neutral `warp-stock` crate or equivalent shared module for stock snapshots and cutter sweeps; both CAM and simulation consume it. Avoid a `warp-cam` <-> `warp-sim` dependency cycle. Support rest-state queries at roughing levels and revision-keyed snapshots.
- [ ] **H02: Prototype planar adaptive clearing early.** On synthetic pockets, develop bounded radial engagement, tangential transitions and narrow-region recovery. Include tool diameter, minimum radius, stepover/engagement and entry constraints. Use an independent engagement analyzer and swept-stock verification; record failures before promoting the algorithm.
- [ ] **H03: Deliver planar adaptive clearing.** Add reliable ramp/helix entry, trochoidal motion where needed, stock-aware stay-down links, retract fallback and bounded path smoothing. Expose progress/cancellation and show infeasible-tool or entry constraints.
- [ ] **H04: Extend adaptive clearing to 3D levels.** Build machinable regions from stock and model slices, preserve wall/floor allowance and handle level transitions. Apply contact and holder checks to the entire resulting motion.
- [ ] **H05: Deliver rest strategies.** 2.5D Re-Machining, Horizontal Re-Roughing, adaptive rest clearing and Clear Flats reference actual previous stock. Editing/suppressing/reordering the predecessor invalidates dependent stock and paths.
- [ ] **H06: Qualify high-speed output.** Fit arcs only within error bounds and machine capabilities. Recheck engagement and collisions after fitting. Benchmark maximum engagement, retract count, cutting/air distance, path length, estimated time, generation time and memory against conventional paths.

**Release gate:** engagement limits hold or violations are explicitly reported on islands, slots, open pockets and cavities. A smaller rest tool removes remaining material without recutting the entire region. Increased feed alone does not pass the adaptive gate. Use configured machine limits for timing estimates and qualify controlled machine trials separately.

## M7 - Advanced 3-axis strategies

- [ ] **A01: Plunge roughing and plunge re-roughing.** Use overlapping Z cuts, suitable cutter geometry, residual-stock filtering and verified approach/retreat motions.
- [ ] **A02: 3D offset pocketing and profiling.** Compute surface-following spacing from selected boundaries/curves; handle disconnected patches, offset collapse and inaccessible areas. Test spacing in 3D, not just in XY.
- [ ] **A03: Pencil tracing and valley re-machining.** Detect tool-radius-dependent double-contact locations; trace stable branches and filter cleanup using previous cutter accessibility or remaining stock. Test small fillets and sharp/rounded valleys.
- [ ] **A04: Flats and steeps families.** Add Flats/Plateau, Steeps/Parallel Hill and Steeps Z/Horizontal Hill with slope classification, appropriate cut directions and controlled overlap at region transitions.
- [ ] **A05: Between-two-curves machining.** Ordered guide selection, direction matching, parallel/normal flow modes, interpolation and projection; detect folded/crossing guides and invalid blends.
- [ ] **A06: Scallop controls.** Add surface-aware spacing/height controls where supported and explain their limits for tool shapes and curvature. Keep this additional capability separate from the reference's named operation inventory.

**Gate:** finish a mold fixture with roughing, re-roughing, clear flats, 3D offsets, steep/shallow finishing and pencil cleanup. Verify both gouges and remaining material; smooth-looking toolpaths alone are insufficient.

## M8 - Specialty strategies and complete coverage

- [ ] **S01: V-carve roughing and filleting.** Coordinate roughing with the intended finishing-tool envelope; add ball/corner-radius edge-rounding paths.
- [ ] **S02: Undercut verification.** Introduce a volumetric or multi-layer stock representation adequate for T-slots and back-side cutting. Extend tool collision beyond a single-valued top profile. Qualify this before releasing T-slotting.
- [ ] **S03: T-slotting, drag knife and deburring.** Add correct tool/head/neck or blade-tip geometry, accessible entries, knife corner compensation, selected-edge deburring and automatic edge candidates. Each strategy receives a dedicated verification fixture.
- [ ] **S04: Saw cutting.** Add blade/holder geometry and machine orientation semantics. Deliver qualified fixed-orientation cases where possible; full documented rotation behavior requires an orientation-capable post/machine extension. Do not label restricted support as full parity.
- [ ] **S05: Complete hole-making cycles.** Standard/peck tapping, CW/CCW, drag/no-drag/manual boring, reverse boring and all user-defined cycle slots listed in the feature specification. Model spindle synchronization/orientation and required operator actions explicitly; unsupported cycles cannot fall back to ordinary feed moves silently.
- [ ] **S06: Reverse Post Milling.** Import supported APT CL and ISO G-code motions, resolve coordinate/modal state, project onto selected surfaces and verify. Preserve a diagnostic for unsupported input commands. Reuse the existing interpreter where its semantics fit.
- [ ] **S07: Advanced nesting.** Irregular sheets/remnants, controlled part-in-hole placement and common-line cutting. Add retention/cut-order constraints before enabling these packing optimizations.
- [ ] **S08: Production workflow.** Reusable operation templates, feature recognition, toolpath transforms/instances, verified edits, post profiles and setup documentation. Edits invalidate applicable stock and verification results.
- [ ] **S09: Final parity audit.** Review every family in the feature specification against parameters, tools, selection roles, entry/link controls, simulation and post behavior. Resolve missing items; publish capability restrictions. All requested families must meet documented acceptance criteria before calling the full scope complete.

## First implementation batch

Execute these in order to produce the smallest useful foundation:

1. B01-B03: baseline, version/migration contract and accuracy fixtures.
2. G01-G02: stable scene IDs and legacy project adapter.
3. G03 + G08: explicit geometry roles and the open projected-path fix.
4. G04-G06: default setup migration, viewport selection and safe background generation.
5. I01-I04: `.3dm` curves, blocks and mesh import wired into the app.
6. C01-C04 + C07: one complete profile/pocket/drill job through simulation and posting.

G07 and the remaining M0-M3 tasks are required before the milestone release, even if they are outside this first demonstration. Begin I05 and H02 prototypes early to expose feasibility constraints; they do not block the planar import demonstration.

**First demo fixture:** a Rhino file containing two instances of a block with an outer perimeter, pocket island and drill holes, plus a loose open engraving curve. Import with correct units; select only one instance; assign boundaries and drive paths independently; generate, simulate, post and save/reopen. Changing the selected geometry must invalidate only affected operations and their dependents.

## Validation and completion tracking

Run focused crate tests for each change, then `cargo test --workspace` at integration gates. Build the Windows release and smoke-test the app at each release milestone. Add native-import packaging checks when I01 lands; do not treat a developer machine with an installed SDK as proof that the application package works.

For each completed task record: implementation files, fixture/tests, measured result, remaining restrictions and the next blocked task, if any. For each release retain a representative project, tool configuration, post configuration, simulation report and generated output. Do not update completion checkboxes merely because a plan or stub was written.

The implementation is complete only when M8's inventory audit passes; the earlier releases deliver usable subsets without losing the full requested scope.
