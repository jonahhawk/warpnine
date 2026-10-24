# WarpNine CAM implementation handoff

Updated 2026-10-25. User-facing changes by date are in [CHANGELOG.md](CHANGELOG.md).

## Current position

M0–M2 are implemented. M3 has a usable, bounded 2.5D preview and a follow-up for checked G-code output and open-wall pockets. It is not full RhinoCAM parity. M4 nesting started on 2026-10-24: N01–N04 are done: data model and candidate discovery, footprints and exact feasibility checks, the deterministic bottom-left solver, and the Nesting tab in the app. N05/N06 remain open. M5 started the same day with D01: Rhino-exported machining meshes carry provenance claims, the importer measures them against their solids, and CAM refuses meshes that miss the operation tolerance. D01b (2026-10-25) added STEP import on a pinned static Open CASCADE build, so users without Rhino get qualified solids and open surfaces from `.step`/`.stp` files. D02 (2026-10-25) added exact cutter-contact queries on the mesh behind a `SurfaceQuery` interface: finishing operations sample exact contacts and refine chords to half the tolerance, 3D roughing keeps the Z-map, and tests measure the path's gouge against independent oracles. D03 (qualifying the 3D strategies) is next. C02/C03 continue in parallel.

The Windows release app at `target/release/warpnine.exe` was rebuilt after D01b and smoke-tested on `fixtures/rhino-m5/sculpted-v8.step` (and earlier on the `.3dm`). `warpnine [project.warp|model.3dm|model.step] [--tab cam|nesting|simulate|tools|machine]` opens a project (or imports a Rhino or STEP file into a new one) and selects a tab from the command line; open `target/m4-nesting/nest-demo.warp` with `--tab nesting` to see the nesting demo. The saved demo project is `target/m3-acceptance/open-wall-job.warp`; its standalone posted program is `target/m3-acceptance/open-wall.tap`.

## Working features

- Native `.3dm` import for supported curves, points, meshes and nested blocks, with instance-aware selections and reimport diagnostics. Imports run in the background with cancellation and a triangle memory limit.
- Qualified solid machining: `scripts/rhino/warpnine_export_meshes.py` exports measured machining meshes from Rhino 8; the importer re-measures them (source-to-mesh, mesh-to-source, outside-trim samples), represents the solid by its qualified mesh, and 3D operations refuse meshes whose measured deviation exceeds the operation tolerance. Mirrored block instances keep outward mesh normals.
- STEP import (`crates/warp-import-step`, Open CASCADE 7.9.3 pinned and built once by `scripts/fetch-occt.ps1`): solids, free shells and faces are meshed at the import tolerance, measured against the exact B-rep with the same policy, named from the file and scaled from its units. One import entry handles `.3dm`, `.step` and `.stp`.
- 2.5D facing, stock-driven prismatic roughing, closed and directed open-wall pockets, profile, slot, drill, hole pocket/profile, thread milling, V-carve finishing, chamfer and existing engraving/follow operations.
- Region and chain validation, island-aware clipping and links, plunge/ramp/helix handling, cycle and tool-parameter checks, and conservative mesh-fixture envelope checks.
- `post_checked` interprets output and compares source and posted motions, arcs, feeds, cycles, active tools and spindle state. The app uses checked output for preview, export and machine loading.
- Geometry dialog (2026-10-25): the operation panel shows one summary line per geometry role (all eligible, stock boundary, or an object count) and a "Select geometry…" button; the objects themselves are chosen in a non-modal window (`crates/warp-app/src/document_ui.rs`, `geometry_dialog`) with the roles the operation kind uses, the selection mode, a name filter, "Check all shown", per-curve reverse and start vertex, reordering and removal of missing references. Roles with stored references are always shown so nothing hidden can block generation. Viewport clicks still toggle curves in the highlighted role.
- Relaxable flute-length check (2026-10-25): the Settings tab holds `Project::checks`, saved in the project file and part of the toolpath input key. With "Allow cut depth beyond the tool's flute length" on, the depth, drill (tip and breakthrough included) and chamfer refusals and the V-carve depth clamp are lifted; the generated `Toolpath` then carries a `warnings` entry naming how far the cut reaches beyond the flutes, shown in amber on the operation and counted in the status line. Off by default; older projects load with nothing relaxed. Stickout and holder clearance are not checked. Tests: `crates/warp-cam/tests/checks.rs`.
- 3D roughing speed (2026-10-25): the Z-map drop-cutter map is computed row-wise with a sliding-window maximum for flat tools (10.4 s to 0.25 s on the `test.warp` roughing job), level contours are extracted in parallel, new roughing operations start at 0.25 mm sampling, and the 3D operation panel predicts the Z-map work for the chosen step and tool and shows the generation time. See D02a in [M5_3D.md](baselines/M5_3D.md).
- Tool library divisions (2026-10-25): every tool carries a `units` division (metric or imperial; older libraries load as metric) and the library tab, the CAM tool picker and the New tool menu group by it. Two built-in starter sets exist: metric T1-T9 and imperial T101-T114 (1/8, 1/4, 3/8 and 1/2 in flat and ball mills, 1/4 and 3/8 in 3-flute roughers, a 90 degree V-mill, a 1/4 in to 0.075 in tapered carving mill, a 2 in 3-flute surfacing cutter on a 1/2 in shank and a 1/4 in shank T-slot cutter). Dimensions stay in millimetres; imperial lengths are shown in inches in the list and picker. The library records which starter sets it has received (`seeded`), so an existing library gains the imperial set once at the next launch and tools the user deletes afterwards are not re-added; the Starter sets menu re-merges a set on demand, renumbering collisions instead of overwriting.

## Still open

- **C02:** broaden stock, tool and fixture contact checks beyond conservative axis-aligned mesh bounds; qualify transfer and entry behavior for more geometry and setups.
- **C03:** complete profile/pocket/facing qualifications and add controller cutter compensation only with valid leads and verified post support.
- **M4:** N01–N04 done (`crates/warp-nest`, `crates/warp-app/src/nest_ui.rs`). N05 CAM handoff and N06 packing remain. See [M4_NESTING.md](baselines/M4_NESTING.md).
- **M5:** D01, D01b and D02 done (`crates/warp-import-3dm/native/qualify.cpp`, `crates/warp-import-step`, `warp_core::interchange::finish_native_import`, `warp_cam::surface`, exporter scripts). D03–D05 remain: qualifying the existing 3D strategies, projection pocketing and extended verification. See [M5_3D.md](baselines/M5_3D.md).
- M6–M8 remain future work: adaptive high-speed paths, advanced 3-axis strategies and remaining specialty operations.

See [CAM_BUILD_PLAN.md](CAM_BUILD_PLAN.md) for the milestone ledger, [M3_25D_CAM.md](baselines/M3_25D_CAM.md) for the user test workflow and limits, and [M3_POST_QUALIFICATION.md](baselines/M3_POST_QUALIFICATION.md) for output-verification evidence.

## Acceptance artifacts

Run the native `.3dm` acceptance example with:

```powershell
cargo run -p warp-import-3dm --example m3_acceptance
```

It writes artifacts under `target/m3-acceptance/`, including `rhino-job.warp`, the T20 test-tool library, five posted programs, `report.txt` and the open-wall example. The last recorded native acceptance report verified 81,894 motion/dwell events for each of five posts, with zero rapid stock collisions and 36.420 mm³ removed in its 512-cell stock simulation.

The last full workspace run passed 137 tests (128 after D02, plus four `warp-tools` tests for the imperial starter set, legacy libraries without a division, one-time seeding and merge renumbering, two `zmap` tests for the windowed cutter-location map and its work estimate, and three `checks` tests for the relaxable flute-length check). Building `warp-import-step` requires the one-time `scripts/fetch-occt.ps1` build. The nesting demo (`cargo run -p warp-cam --example nest_demo`) writes `target/m4-nesting/nest-demo.warp` and `report.txt`; the debug app was smoke-tested on it with `warpnine target/m4-nesting/nest-demo.warp --tab nesting` (drag, rotate, undo and background Nest all verified by screenshot). The final release build succeeded, and the release app smoke test opened and generated the open-wall project with the G-code verification status shown.

