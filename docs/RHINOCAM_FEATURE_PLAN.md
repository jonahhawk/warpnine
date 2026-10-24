# WarpNine: RhinoCAM-style 2.5D / 3-axis CAM plan

Planning baseline: 2026-10-24. This document defines implementation order and acceptance criteria; it does not claim the planned features are implemented.

For concrete implementation tasks and dependencies, follow the [CAM build plan](CAM_BUILD_PLAN.md).

## Outcome and scope

Build a standalone CAM workflow in which users import Rhino geometry, select curves or surface edges as machining boundaries or drive paths, organize operations into setups, generate and verify toolpaths, and post machine-specific G-code. Include efficient sheet nesting for Rhino block instances and loose geometry.

The target inventory is **all operation families in the supplied RhinoCAM-MILL 2026 reference's 2.5-axis, 3-axis, advanced 3-axis, and non-rotary hole-making categories**, across editions. Implement matching machining behavior and familiar concepts using WarpNine's own UI and algorithms. A matching operation name alone is not parity. Rotary 4/5-axis programming is a separate future scope; specialty operations listed under 2.5-axis that require machine orientation support remain explicitly tracked below.

First useful release: import planar `.3dm` curves and blocks, select machining geometry explicitly, program profile/pocket/drill/follow operations, nest repeated parts on sheets, simulate, save/reopen, and post. Next: reliable 3D import and finishing, adaptive high-speed roughing, rest machining, then the full advanced and specialty inventory.

## Current implementation and gaps

Code inspection, rather than README claims alone, establishes this starting point:

| Area | Existing foundation | Required change |
| --- | --- | --- |
| Geometry | `warp-core`: DXF curves, STL meshes, 2D NURBS evaluation | Persistent object identity, 3D curves, layers, blocks, instances, surface/edge references, `.3dm` reader |
| Project | `warp-cam/src/project.rs`: flat `Vec<Polyline2>`, one optional mesh, stock box, ordered operations | Versioned scene, multiple models, setups, fixtures, stock states, nesting jobs |
| Selection | `Common.curves: Vec<usize>`; viewport click selection | Typed selection roles, named sets, chain editing, robust references through edits/reimport |
| 2.5D CAM | Facing, Profile, Pocket, Drill, Engrave, VCarve, HelicalBore, ThreadMill, Chamfer | Qualify existing behavior and add the missing strategies and controls below |
| 3D CAM | Z-map roughing, Parallel, Waterline, Spiral, Radial, ProjectCurves | Better geometric accuracy, distinct drive/containment geometry, stock-aware roughing, advanced finishing |
| Execution | Background generation threads already exist in `warp-app/src/app.rs` | Bounded job queue, cancellation, progress, dependency tracking, discard stale results |
| Verification | Height-field stock removal, rapid/material warnings; G-code parser/interpreter | Swept cutter/shank/holder collision, model gouge and remaining-stock checks; volumetric support for undercuts |
| Output | Several machine posts, arcs/helices and six drill cycles | Capability-aware cycles, cutter compensation, specialty motions and round-trip qualification |

Important existing issues to resolve early:

- Empty curve selection means all curves in `Project::select_curves`, while 3D setup treats it as whole stock. Replace this ambiguity with explicit modes such as Selected, All eligible, and Stock boundary.
- `ops3d::setup` interprets the same selected curves as closed containment that `ProjectCurves` later uses as drive paths. Separate these roles so open projected paths work independently of a closed boundary.
- `ops3d::clip_polyline` filters sample points rather than computing exact segment/region intersections. Narrow exclusions and links need robust clipping and swept-motion checks.
- A single sampled Z-map is an approximation with limited vertical-wall and undercut representation. Do not claim that its cell size or path simplification tolerance guarantees final machining accuracy.

## Target user workflow

1. **Import and inspect:** choose Rhino `.3dm`, DXF, or STL. Show units, layers, objects, blocks, import diagnostics, and geometry accuracy. Keep source geometry separate from machining representations.
2. **Create a setup:** select machine/post, work coordinate system, stock, material, fixtures and safe heights. Support multiple manually oriented setups without implying rotary machine support.
3. **Select control geometry:** choose curves, curve chains, surface edges, surfaces/meshes, points or part instances in the viewport or object browser. Use role-specific pick modes and visible direction/start arrows.
4. **Create an operation:** choose a strategy, tool, feeds/speeds, cut levels, stepover/stock allowance, entry/exit and linking. Keep Control Geometry, Tool, Feeds & Speeds, Clearance, Cut Parameters, Cut Levels and Engage/Retract consistently organized.
5. **Generate and inspect:** preview the selected region before computing, then show toolpath, errors and warnings. Editing an input marks dependent paths and stock states dirty. Provide regenerate selected/all, suppress, duplicate, reorder and operation groups.
6. **Simulate and post:** inspect stock removal, remaining material, gouges and collisions; post selected operations or a setup using the same geometry revision that was verified.

Selection roles must be independent: machining regions; containment regions; avoid regions; ordered drive curves; part surfaces; check surfaces/fixtures; hole locations; entry/start points. Include inside/outside/on or contact-based containment semantics where appropriate, and preview the cutter envelope. Do not apply a generic radius offset to every strategy without considering contact geometry.

Support window/crossing selection, layer/type filters, hide/isolate, chain direction reversal, start-point editing, closed-loop validation, nested islands and disjoint regions. Missing references must produce a repairable error, never silently select unrelated geometry. Geometry selected before creating an operation should populate the appropriate role with a preview.

## Architecture to establish first

- **`warp-core`:** a scene of stable object IDs, source Rhino UUIDs, analytic/3D curves, meshes, surface/edge provenance, layers, groups, block definitions and transformed instances. Preserve original units and model tolerance; use millimeters internally. Introduce robust planar-region construction and a spatial index.
- **New `warp-import-3dm`:** native openNURBS integration behind a narrow C ABI or equivalent audited Rust/C++ boundary. Own allocations explicitly, convert exceptions to diagnostics, pin the SDK version, and package Windows builds reproducibly. Keep this dependency out of CAM algorithm code.
- **`warp-cam`:** versioned setups and operation IDs, typed geometry references, strategy parameter types, a shared linking/entry planner, strategy capability registry, and stock-state dependency graph. Generation inputs are immutable snapshots. Cache by geometry, setup, tool, parameters, stock revision and algorithm version.
- **New `warp-nest`:** part definitions, quantities, stock sheets, constraints, placements and a cancellable optimizer. Store transforms and references, not destructive copies of source curves.
- **`warp-sim`:** shared stock and swept-tool queries usable by both CAM and verification. Retain the height field for supported top-down jobs; add a more complete representation where the strategy requires it.
- **`warp-app`:** machining job tree plus geometry browser, operation editor, selection modes, nesting workspace, undo/redo and generation status. Avoid turning every strategy into a separate inconsistent dialog.
- **`warp-gcode` / posts / machine capabilities:** represent unsupported actions explicitly. Add cycle/orientation/spindle semantics before enabling a strategy that relies on them.

Migrate existing `.warp` files with a schema version and deterministic IDs. Preserve the old selection meaning during migration, then display the explicit migrated mode. Keep user files intact and cover save/reopen, unresolved references, and transformed instances.

## Rhino `.3dm` import through openNURBS

Use the import label **Rhino / openNURBS (`.3dm`)**. openNURBS is the reader technology, not a separate file extension.

Initial support: points, lines, arcs, polylines, polycurves and NURBS curves; mesh objects; layer hierarchy, visibility, names, UUIDs, groups, block definitions and nested block-instance transforms. Handle inch/mm scaling, mirrored instances, nonuniform scaling, nested instances, and missing linked definitions with explicit diagnostics. Define and test the supported file-version range against the pinned reader.

Keep true geometry and a tolerance-controlled machining approximation. Never flatten arbitrary 3D curves into XY silently; require a setup plane and validate planarity for 2.5D operations. Offer a deliberate projection operation when flattening is intended.

**Critical integration gate:** openNURBS reads geometry and evaluates NURBS but does not supply surface tessellation, surface intersections or a complete CAD kernel. Cached render meshes can be read, but may be absent or too coarse for machining. See McNeel's [openNURBS limitations](https://developer.rhino3d.com/guides/opennurbs/what-is-opennurbs/) and [reading render meshes](https://developer.rhino3d.com/en/guides/opennurbs/reading-render-meshes/).

Run a meshing proof of concept in Phase 1: trimmed Breps, holes, seams, extrusions, units and block transforms. Evaluate a suitable meshing/kernel bridge or a controlled trimmed-surface tessellator against measured deviation and topology. Select the production approach before promising arbitrary Rhino solids. Cached meshes may provide preview and explicitly qualified machining input; never assume an unmeasured render mesh satisfies CAM tolerance. SubD needs its own evaluated representation or supported conversion path.

Report import counts by type, warnings, unsupported objects, unresolved instances and meshing status. Unsupported geometry must remain visible as a diagnostic instead of disappearing without notice. Importing RhinoCAM's proprietary operation data from `.3dm` is not implied by importing Rhino geometry.

## Phased delivery and acceptance gates

Phases are ordered by dependency and user value. Scope estimates should follow the import/meshing and adaptive-kernel prototypes; fixed dates before those gates would be unreliable.

### Phase 0: establish a reproducible baseline

- Run existing workspace tests; preserve representative `.warp`, DXF and STL fixtures.
- Create a strategy coverage ledger from the inventory below, distinguishing present, prototype, verified and released.
- Record geometry error, path length, generation time, memory use and simulation results for representative parts.
- Specify selection semantics, project migration, machining tolerance budget and post capability checks.

**Exit:** repeatable baseline and fixture corpus; no claim that existing strategies have RhinoCAM-equivalent behavior simply because their names match.

### Phase 1: geometry foundation, Rhino curves/blocks and familiar selection

- Implement versioned scene/project storage and stable references; adapt DXF/STL import to it.
- Add openNURBS curve/mesh/block import, layers and instance-aware viewport picking.
- Implement independent control-geometry roles, chain/start/direction controls, explicit selection modes and undo/redo.
- Add setups, operation invalidation and stale-job-result protection.
- Complete the solid meshing proof of concept described above.

**Exit:** open a Rhino file with loose curves and repeated/nested blocks, select one instance's perimeter and holes, save/reopen without losing references, and regenerate a profile after an edit. Open drive curves can be projected independently of closed containment. Invalid/unsupported geometry is reported.

### Phase 2: dependable everyday 2.5D and first sheet-nesting release

- Qualify Face Top/Facing, closed/open Pocketing, Profiling, Drill and Engrave/Follow first.
- Complete islands, region validation, slotting, separate wall/floor finish passes, ramps/helical entries, leads, tabs, finish allowance and cut ordering.
- Separate Hole Pocketing and Hole Profiling behavior; qualify existing helical bore, thread mill, chamfer and V-carve finishing.
- Deliver nesting MVP described below: parts from blocks or loose geometry, quantities, sheet sizes, margins, spacing, allowed rotations, grain, locked placements and multi-sheet results.
- Carry nested transforms into CAM; machine internal features before releasing outer contours. Simulate and post per sheet.

**Exit / first useful release:** import a `.3dm` sheet-parts job, define a loose-geometry part and a block part, nest quantities without collisions, program pockets/holes/outlines with tabs, simulate and post, then reproduce the same result after reopening. Narrow gaps, nested holes, disconnected parts and mirrored blocks have regression coverage.

### Phase 3: reliable core 3D machining

- Deliver qualified Brep/extrusion meshing; support multiple part/check models and edge-derived boundaries.
- Upgrade collision/contact queries and robust path clipping; establish a combined import, contact, path-fitting and post-output error budget.
- Qualify Horizontal Roughing, Parallel, Horizontal/Waterline Finishing, Radial, Spiral and Curve Machining.
- Add roughing patterns, exact user-specified Z levels, boundary contact modes, slope restrictions and projection pocketing.
- Add baseline holder/shank/fixture checks and remaining-stock comparison.

**Exit:** machine an imported sculpted mold fixture from roughing to finish with selected surfaces, containment and avoid regions. Measured gouge/deviation stays within the specified budget on vertical walls, shallow areas, seams and boundary crossings. Missing or unqualified mesh data cannot silently become a production path.

### Phase 4: high-speed machining and stock-aware rest machining

- Build a shared stock-aware adaptive clearing engine: begin with planar regions, then apply it at 3D roughing levels.
- Control radial engagement, smooth corners and tangential transitions; support trochoidal recovery in confined regions, suitable ramp/helix entry, minimum cutting radius, stay-down links and safe retract fallback.
- Track remaining stock after actual swept cuts; use it for 2.5D Re-Machining, Horizontal Re-Roughing, adaptive rest clearing and Clear Flats.
- Add bounded arc fitting and capability-aware output; retain model/fixture collision checks after smoothing and fitting.

**Exit:** benchmark narrow slots, islands, open pockets and deep cavities. Report peak engagement, engagement distribution, path length, cutting/air distance, retracts, estimated cycle time, generation time and memory against conventional roughing. Engagement violations must be reported or resolved, not hidden by labeling ordinary offsets “adaptive.” Feed increases alone do not constitute high-speed machining.

Machine acceleration, jerk/look-ahead and block-processing limits affect achievable feed. Use configured limits for estimates, retain conservative user overrides, and qualify posted output before machine trials. The goal is controlled engagement and smooth motion, not a universal promised speedup.

### Phase 5: advanced 3-axis finishing and remaining roughing families

- Implement Plunge Roughing and Plunge Re-Roughing with tool suitability and retreat checks.
- Implement true 3D Offset Pocketing/Profiling, Pencil Tracing and Valley Re-Machining.
- Implement Flats/Plateau, Steeps/Parallel Hill and Steeps Z/Horizontal Hill finishing.
- Implement Between 2 Curves machining with ordered guide pairing, orientation controls and fold/self-intersection handling.
- Add scallop-height finishing controls as an additional WarpNine capability; distinguish surface-distance stepovers from XY projection offsets.

**Exit:** reproduce the machining intent of the supplied advanced 3-axis tutorial using independently created fixtures: rough, re-rough, clear flats, finish and pencil cleanup. Verify residual material and surface error, including small fillets, valleys, steep/shallow transitions and disconnected surfaces.

### Phase 6: complete specialty inventory and production workflow

- Finish V-Carve Roughing, Filleting, T-Slotting, Knife Cutting, Deburring, Saw Cutting, tapping and advanced boring/custom cycles.
- Implement Reverse Post Milling: read supported APT CL and ISO G-code motions, map coordinates, select projection surfaces, regenerate and verify. Existing G-code simulation alone does not satisfy this strategy.
- Improve nesting with irregular sheets/remnants, part-in-hole placement where allowed, better packing search and constrained common-line cutting.
- Add reusable operation templates, feature recognition, batch regeneration, setup sheets, reusable post profiles and verified toolpath edits/instances.
- Audit every matrix row against strategy parameters, supported tools, entry/link behavior, geometry controls, simulation and post output.

**Exit:** every requested strategy is implemented and qualified for its stated machine/tool/geometry capabilities, or explicitly marked blocked with the missing capability. No unimplemented menu item counts as completion. Specialty support must not silently lower a requested strategy to a different one.

## Complete operation-family coverage ledger

“Existing” below means a corresponding code path exists, not that production parity has been proven. Phase numbers identify primary delivery, with later qualification where noted. Source pages are the printed/PDF page numbers in the local reference.

### 2.5-axis (reference pp. 219-235)

| RhinoCAM family | WarpNine baseline / required behavior | Phase |
| --- | --- | --- |
| Face Top | Add distinct stock/part-driven top-facing behavior | 2 |
| Roughing | Add stock-driven prismatic area clearing; retain part islands | 2; adaptive extension 4 |
| Facing | Existing; qualify boundaries, islands and outside approach | 2 |
| Pocketing | Existing; complete open/closed regions, patterns and finishing | 2 |
| Profiling | Existing; qualify open/closed sides, leads, tabs, multiple finish passes and compensation | 2 |
| V-Carve Roughing | Add clearance coordinated with the finishing V-tool envelope | 6 |
| V-Carve Finishing | Existing VCarve; qualify corner lift, depth limits and cleanup | 2 |
| Engraving | Existing planar follow; add preserved 3D curve following and tool controls | 2-3 |
| Chamfering | Existing; qualify edge geometry, cutter contact and width/depth | 2 |
| Filleting | Add edge rounding with ball/corner-radius tools | 6 |
| Hole Pocketing | Existing HelicalBore is a starting point; qualify helix, area clearing and final circle | 2 |
| Hole Profiling | Add distinct circular profile/depth/helix controls | 2 |
| Thread Milling | Existing; qualify internal/external, handedness, multiple passes and helix output | 2 |
| T-Slotting | Add cutter/head/neck envelopes, side/top/bottom stock and accessible entry | 6 |
| Knife Cutting | Add drag-knife tip offset, corner/swivel compensation and lift behavior | 6 |
| Deburring | Add selected-edge and automatic candidate-edge detection with sharpness/clearance controls | 6 |
| Saw Cutting | Add blade/holder representation and machine-specific orientation/output support | 6, capability gate |
| Re-Machining | Add remaining-stock-driven cleanup after facing/pocket/profile | 4 |

Slotting is also explicitly taught in the supplied 2-axis advanced tutorial (p. 25); include a first-class workflow in Phase 2 even though it is not a separate family in the reference overview. Treat high-speed planar/adaptive clearing as an additional directly accessible 2.5D strategy.

Saw Cutting appears in RhinoCAM's 2.5-axis category but its documented output includes tool-axis rotations. Support fixed-orientation cases only when valid for the configured machine; full behavior requires an explicit orientation-capable machine/post extension. Keep this dependency visible rather than silently excluding the family. T-slotting similarly needs undercut-aware verification that the current height field cannot provide.

### 3-axis basic and advanced (reference pp. 235-252)

| RhinoCAM family | WarpNine baseline / required behavior | Phase |
| --- | --- | --- |
| Horizontal Roughing | Existing Roughing3D; add/qualify linear, stock-offset and part-offset patterns | 3 |
| Parallel Finishing | Existing Parallel; qualify sampling, boundaries and links | 3 |
| Horizontal Finishing | Existing Waterline; qualify exact levels and wall accuracy | 3 |
| Radial Machining | Existing Radial; support per-region center and controlled angular coverage | 3 |
| Spiral Machining | Existing Spiral; qualify region centers, clipping and continuous motion | 3 |
| Adaptive Roughing | Add stock-aware engagement-controlled high-speed clearing | 4 |
| Plunge Roughing | Add overlapping Z-cut pattern, accessible entry and retreat | 5 |
| Horizontal Re-Roughing | Add previous-stock comparison and constant-Z rest clearing | 4 |
| Plunge Re-Roughing | Add stock-filtered plunge cleanup | 5 |
| Clear Flats Machining | Add exact flat-level detection and residual clearing | 4 |
| Projection Pocketing | Add planar pocket offsets projected onto part surfaces | 3 |
| 3D Offset Pocketing | Add offsets with controlled 3D spacing on sculpted surfaces | 5 |
| 3D Offset Profiling | Add surface-following offsets from selected curves | 5 |
| Pencil Tracing | Add tool-radius-aware double-contact valley/corner paths | 5 |
| Valley Re-Machining | Add cleanup of regions inaccessible to previous finishing tools | 5 |
| Flats (Plateau) Machining | Add shallow-slope area finishing with slope thresholds | 5 |
| Steeps (Parallel Hill) Machining | Add steep-region detection and locally appropriate cutting direction | 5 |
| Steeps Z (Horizontal Hill) Finishing | Add slope-restricted constant-Z finishing | 5 |
| Curve Machining | Existing ProjectCurves; separate drive paths, containment and part surfaces | 1 fix; 3 qualification |
| Between 2 Curves Machining | Add blended parallel/normal flow paths between selected guides | 5 |
| Reverse Post Milling | Add APT/ISO path import and surface projection, beyond existing simulation | 6 |

### Non-rotary hole making (reference pp. 267-274)

| Family | Required scope | Phase |
| --- | --- | --- |
| Drilling | Standard, deep peck, countersink/dwell, chip-break and user-defined Drill 1/2 | 2 basic; 6 custom |
| Tapping | Standard and peck, CW/CCW; user-defined Tap 1-4; synchronized feed/spindle semantics | 6 |
| Boring | Drag, no-drag/oriented, manual and user-defined Bore 1/2 | 6 |
| Reverse Boring | Standard and user-defined Reverse Bore 1/2; oriented spindle and clearance motions | 6 |

Current generated cycles are G81/G82/G83/G73/G85/G89. Parser support for additional codes does not prove generation or controller support. User-defined cycles need validated post templates and explicit simulator behavior. Do not expand synchronized tapping into ordinary feed moves unless the target machine semantics actually support it.

## Sheet nesting design

**Part definition:** a block instance is a candidate part, preserving definition and instance identity. Nested blocks retain hierarchy. For loose geometry, allow explicit grouping; suggest outer-loop-plus-contained-holes grouping and ask the user to resolve ambiguous disconnected contours. Keep engraving, pockets, drill points and labels attached to the part. Open geometry cannot be an inferred perimeter without an explicit outline. A 3D block needs a selected setup plane and approved footprint.

**Constraints:** quantity, material/thickness, sheet dimensions, edge margins, part spacing, machining clearance, rotation set, grain direction, mirroring permission, fixed placements and clamp/defect exclusions. Mirroring is off by default. Distinguish finished-part outlines from cutter envelopes, tab clearance and lead-in space; one blanket spacing number does not account for all machining approaches.

**First solver:** deterministic placement order, configurable allowed rotations and bottom-left/best-fit placement using actual polygon containment and collision tests. Use bounding boxes only for broad-phase acceleration. Grow toward no-fit polygons/Minkowski clearance regions and bounded multi-start/local search. Optimize feasible sheet count first, then utilization and useful remnant shape; optionally consider machining travel after feasibility. Report best-found results without claiming a global optimum.

**Results:** sheet count, quantities placed/unplaced, utilization with a stated area definition, remaining usable area, constraint failures, search time and reproducible seed. Preview, drag/rotate/lock placements, re-optimize unlocked parts, undo and save. Invalid manual edits must be visibly rejected or flagged before CAM generation.

**CAM handoff:** create sheet setups and transformed part instances; keep operation templates attached to definitions and overrides attached to instances. Revalidate tabs, start points, grain constraints, cutter handedness and collision after transform. Cut pockets/holes before outer contours and schedule releases so parts remain supported. Reordering by tool must preserve these dependencies. Shared-line cutting and part-in-hole placement come later because they change retention and cut-order requirements.

**Nesting acceptance corpus:** concave shapes that defeat bounding-box packing, mixed quantities, holes, multiple islands, thin parts, nested/mirrored blocks, grain-constrained rectangles, clamps, multiple sheets and oversize parts. Require zero overlaps/out-of-sheet placements at the specified clearance tolerance; compare utilization and runtime to a simple baseline on fixed hardware. Set numeric performance targets after measuring these fixtures.

## Quality gates shared by every release

- Geometry: transformed units, topology, self-intersections, duplicate/degenerate curves, nested islands, near-tangent offsets, mesh seams and mixed tolerances.
- Paths: cutter envelope containment, no gouges beyond tolerance, valid entries/exits, no unverified low links, stock allowance and complete intended coverage. Check swept segments/arcs, not only endpoints.
- Stock: every rest operation references the actual predecessor state; editing, suppressing or reordering an upstream operation invalidates downstream results.
- Output: post and re-interpret supported G-code; compare motion, units, WCS, arc/helix direction and cycle behavior. Reject unsupported post capabilities. Preserve modal state through operation boundaries.
- Workflow: selection survives save/reopen, edits, instance transforms and supported reimport; cancellation leaves no partial path presented as current; undo restores dependencies.
- Delivery: test representative fixtures and relevant regression suites per implementation change; qualify machine output separately from geometric previews. Use simulation and controlled machining trials before declaring a strategy production-ready.

A release gate is an end-to-end job, not an isolated algorithm demonstration. The first implementation slice should be **stable object IDs + explicit selection roles + planar `.3dm` curves/blocks + a saved profile/pocket job**. That unlocks useful Rhino workflows and nesting while the harder meshing and adaptive work is proven.

## Sources and traceability

- [RhinoCAM-MILL2026-Reference.pdf](../resources/RhinoCAM-MILL2026-Reference.pdf): primary strategy inventory, pp. 219-252 and 267-274; associative edits and toolpath editing begin at p. 2933; simulation at p. 2990; post processing at p. 3002. The manual has 3,044 PDF pages.
- [RC2026-UI-Guide.pdf](../resources/RC2026-UI-Guide.pdf): machining browser, program/simulate tabs, tools, regions, features and preferences.
- [RC2026-2xAdvanced-Tutorial.pdf](../resources/RC2026-2xAdvanced-Tutorial.pdf): setup, stepped/open pockets, slotting, hole operations, chamfer/profile, simulation and posting; workflow coverage pp. 8-50.
- [RC2026-3xIntro-Tutorial.pdf](../resources/RC2026-3xIntro-Tutorial.pdf): roughing, parallel prefinish/finish and Z-level finish, pp. 19-37.
- [RC2026-3xAdvanced-Tutorial.pdf](../resources/RC2026-3xAdvanced-Tutorial.pdf): containment, re-roughing, clear flats, 3D offsets and pencil cleanup, pp. 19-44.
- `Cutting-Tools-Workbook-RhinoCAM2026.pdf` and `RhinoCAM2026-PPG-Decoded-Guide.pdf` are additional implementation references for later tool/post work; a detailed parameter-by-parameter audit remains part of each strategy's implementation.
- The three tutorial filenames say 2026, but their internal covers/copyright text say 2025. Use the 2026 reference manual as the inventory authority.
- `resources/MILLQuickStartGuide.pdf` is zero bytes at planning time and was not usable. The reference manual includes a Quick Start section, so this does not block planning.
- External openNURBS references are linked at the import design decision above. The architecture and sequencing in this document are proposed WarpNine design choices.
