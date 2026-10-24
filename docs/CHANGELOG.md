# Changelog

User-facing features and changes, newest first. Milestone history and open work live in [SESSION_STATUS.md](SESSION_STATUS.md) and the baselines under `docs/baselines/`. Nothing below is committed yet; the working tree holds everything since the initial commit.

## 2026-10-25

### Geometry dialog for operations
- The operation panel shows one summary line per geometry role ("all eligible objects", "stock boundary", or an object count) and a **Select geometry…** button. Objects are no longer listed in the sidebar.
- The button opens a non-modal window with the roles the operation kind uses, the selection mode, a name filter, **Check all shown** / **Clear**, a checkbox list of eligible objects (meshes and solids for part surfaces, curves or points elsewhere), per-curve Reverse and Start vertex, ↑ to machine earlier, and removal of missing references. Roles that still hold references are always shown so nothing hidden can block generation. Viewport clicks still toggle curves in the highlighted role.
- Files: `crates/warp-app/src/document_ui.rs`, `cam_ui.rs`, `app.rs`.

### Settings tab and relaxable flute-length check
- New **Settings** tab. "Allow cut depth beyond the tool's flute length" lifts the depth, drill (tip and breakthrough included) and chamfer refusals and the V-carve depth clamp. The toolpath then carries a warning naming how far the cut reaches beyond the flutes, shown in amber on the operation and counted in the status line.
- The choice is saved in the project (`Project::checks`), is part of the toolpath input key, and defaults off. Older projects load with nothing relaxed. Stickout and holder clearance are still not checked.
- Files: `crates/warp-cam/src/project.rs`, `validation.rs`, `ops2d.rs`, `toolpath.rs` (`warnings`), `lib.rs`, `document.rs`, `workflow.rs`; `crates/warp-app/src/settings_ui.rs`. Tests: `crates/warp-cam/tests/checks.rs`.

### 3D roughing speed and accuracy control
- The Z-map cutter-location map is computed row by row with a sliding-window maximum for flat and face mills, so their cost no longer depends on tool radius: 10.4 s to 0.25 s on the `test.warp` roughing job (11.1 s to 0.66 s end to end, identical moves). Level contours for roughing and waterline are extracted in parallel.
- New 3D roughing operations start at 0.25 mm sampling; finishing stays at 0.1 mm; existing operations keep their saved value. A **Default** button sits beside Surface sampling.
- The 3D panel shows a **Z-map work** line (cells × samples, labelled fast / slow / very slow) that updates with the step and tool. The move-count line and the status bar show the generation time.
- Known limit: ball, bull-nose and V tools still pay cells × samples, so the sampling step is the control for them. On the 1.2 m × 0.9 m contour fixture the roughing is now bound by the concentric offsetting of the cleared region, not the Z-map; see D02a in `docs/baselines/M5_3D.md`.
- Examples: `crates/warp-cam/examples/time_project.rs`, `kernel_bench.rs`.

### Tool library divisions and imperial starter set
- Every tool carries a **division**, metric or imperial (older libraries load as metric). The library tab, the CAM tool picker and the New tool menu group by division; imperial lengths display in inches.
- Imperial starter set T101–T114: 1/8, 1/4, 3/8 and 1/2 in flat and ball mills, 1/4 and 3/8 in 3-flute roughers, a 90° V-mill, a 1/4 in to 0.075 in tapered carving mill, a 2 in 3-flute surfacing cutter on a 1/2 in shank, and a 1/4 in shank T-slot cutter. The metric set is T1–T9.
- The library records which starter sets it has received, so an existing library gains the imperial set once at the next launch and deleted tools are not re-added. The **Starter sets** menu re-merges a set on demand, renumbering collisions instead of overwriting.
- Files: `crates/warp-tools/src/lib.rs`, `crates/warp-app/src/tools_ui.rs`, `cam_ui.rs`, `app.rs`.

### Viewport
- Removed the full-height vertical line between the Fit and Model buttons in the CAM viewport overlay (`crates/warp-app/src/app.rs`).

### Earlier the same day (M5)
- D02: exact cutter-contact queries on the mesh behind a `SurfaceQuery` interface; finishing operations sample exact contacts and refine chords to half the tolerance (`crates/warp-cam/src/surface.rs`, tests in `crates/warp-cam/tests/surface.rs`).
- D01b: STEP import on a pinned static Open CASCADE 7.9.3 build (`crates/warp-import-step`, `scripts/fetch-occt.ps1`); solids, shells and faces are meshed at the import tolerance and measured against the exact B-rep.

## 2026-10-24

- M5 D01: Rhino-exported machining meshes carry provenance; the importer measures them against their solids and CAM refuses meshes that miss the operation tolerance (`crates/warp-import-3dm/native/qualify.cpp`, `scripts/rhino/`).
- M4 N01–N04: nesting data model, footprints and feasibility checks, deterministic bottom-left solver, Nesting tab (`crates/warp-nest`, `crates/warp-app/src/nest_ui.rs`). See `docs/baselines/M4_NESTING.md`.

## Before

M0–M3: core geometry and interchange, Rhino `.3dm` import, 2.5D CAM (facing, prismatic roughing, pockets, profile, slot, drill, hole operations, thread milling, V-carve, chamfer, engraving), checked post output, simulation and machine tabs. See `docs/baselines/` and `docs/CAM_BUILD_PLAN.md`.
