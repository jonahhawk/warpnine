# M3 — curve-driven 2.5D CAM preview

Implemented 2026-10-24. This milestone delivers a bounded 2.5D workflow; the full RhinoCAM inventory remains tracked through M8. It does not add nesting (M4), qualified solid machining (M5), or high-speed adaptive clearing (M6).

## Delivered behavior

- Region validation rejects self-intersections, crossing/touching loops, duplicates, degenerate segments and empty selections. Scene resolution checks planarity in setup coordinates. Nested islands and separate regions use polygon Boolean operations and segment clipping.
- Shared links require a full segment inside the cutter-centre region. A short distance alone cannot authorize crossing an island. Retracts feed out of material before rapid travel above stock. Closed-loop and open-chain ramps use the configured ramp feed. Offset pockets can use a fitted circular helix with explicit radius and pitch; a helix that cannot fit fails generation.
- Distinct **Face Top**, **2.5D Roughing**, **Open Pocket**, **Hole Pocket**, **Hole Profile**, and **Slot** choices appear alongside the existing operations. Face Top uses selected closed clearing regions; stock-facing retains its separate operation. Roughing clears rectangular stock around selected retained part outlines. Open Pocket uses a closed clearing extent inside stock, with at least one complete edge on the stock perimeter, or directed open wall chains whose endpoints meet stock edges. The left side of each wall is cleared; reverse the chain to select the other side. These stock edges allow cutter overrun. Slot follows selected curves at tool width.
- Profile sides, radial finish passes, spring passes, wall/floor allowance, tabs and lead arcs have validation. Exterior profile orientation now matches climb/conventional choice. Arc leads are checked against retained material. On-curve profiles reject radial offsets and tabs; tabs plus arc leads are rejected rather than silently ignoring a setting. Compensation is calculated in CAM; controller G41/G42 modes are not exposed.
- Circle detection supports nonuniform point spacing. Hole profiling follows the circumference; hole pocketing clears concentric radii. Helices preserve requested pitch through partial turns. Thread milling preserves handedness and requires a thread tool with matching pitch. Drill depth includes tip compensation and breakthrough in flute-length checks. The project's Settings can relax the flute-length checks (`Project::checks.allow_depth_beyond_flute`); the toolpath then carries a warning naming the excess instead of being refused. Expanded dwell cycles now emit the dwell at the hole bottom.
- Pointed-cone V finishing respects maximum depth, floor allowance and stepdown. Flat-tip tools are rejected for sharp-corner V finishing. Chamfer width/depth uses the actual conical angle and flat-tip radius; oversized/unreachable offsets fail.
- Setup fixtures can be assigned from preselected meshes. Generation checks the tool, shank and configured holder envelope against transformed fixture bounding boxes. Fixture and holder changes invalidate dependent paths.
- Export, preview and machine loading share checked posting. Unsupported work offsets, missing tools, insufficient output precision and multiple tools without a post tool-change command block output. Existing revision checks reject stale generation; document edits clear preview and simulation.

## Automated evidence

`cargo test --workspace`: 75 tests passed, including thirteen M3 regressions and six post-qualification tests in `crates/warp-cam/tests/m3.rs`. Existing V-carve analytic tests also check the new depth-limited passes. The all-operation test covers every operation menu entry. A 60-case matrix checks all six supported drilling cycles across five posts in both inch and millimetre output, including depth, dwell and feed-out semantics.

Run the reproducible native acceptance job:

```powershell
cargo run -p warp-import-3dm --example m3_acceptance
```

It imports `fixtures/rhino-m2/parts-mm-v8.3dm`, selects Part B by instance identity, creates an island pocket, external profile, hole pocket and loose-curve follow operation, includes a mesh fixture, saves/reopens the document and requires identical regenerated moves. It posts, interprets and compares every motion/dwell event for Mach3/Mach4, UCCNC, LinuxCNC, GRBL and TurboCNC output. See [the post qualification follow-up](M3_POST_QUALIFICATION.md).

Outputs are in `target/m3-acceptance/`: `rhino-job.warp`, `tools.json`, five posted programs and `report.txt`. The 512-cell stock simulation measured **zero rapid collisions** and **36.420 mm³ removed**. The tiny native fixture uses a dedicated **T20, 0.5 mm flat cutter**; import its supplied tool library before generating the job in the desktop app.

Desktop smoke test: the release executable launched, the dedicated T20 library imported without replacing existing tools, the saved Rhino job opened, all four operations generated, posted preview displayed, and the 400-cell desktop simulation reached 100% with no rapid-collision warnings. The app was left open on the result.

## Test it in the desktop app

1. Launch `target/release/warpnine.exe`.
2. In **Tools → Import**, load `target/m3-acceptance/tools.json`.
3. Open `target/m3-acceptance/rhino-job.warp` and choose **Generate all** in CAM.
4. Inspect the four operations and their geometry roles, preview the paths, then run Simulate to completion.
5. Preview/export a posted program. Change the pocket depth or T20 diameter and confirm paths require regeneration.
6. For fixture checks, preselect a mesh in the geometry browser, assign it under setup coordinates, and generate. A fixture intersecting the tool envelope must produce an error. The acceptance job already includes a non-intersecting clamp bounding box.
7. Open `target/m3-acceptance/open-wall-job.warp` for an open-wall example. Generate, inspect the clearing side, reverse the assigned wall chain, and regenerate. Alternatively select a closed clearing extent with one full edge on the rectangular stock perimeter.

## Supported limits

Fixtures use conservative axis-aligned mesh bounds in setup coordinates, including a cylindrical shank/holder envelope. This can reject geometrically clear moves near a concave fixture. It does not verify the machine's unknown initial position or machine structure. Stock simulation remains a height field; it cannot verify undercuts or thread flank geometry. Thread milling requires a pre-existing hole/exterior clearance; the operation does not create that clearance automatically.

Milling uses a clockwise spindle. Circular recognition has a 0.01 mm radial fitting tolerance. Open-pocket openings must lie on rectangular stock edges. Open walls may have multiple segments; both endpoints must meet the stock perimeter. Overlapping/ambiguous wall selections and nonrectangular stock openings are rejected. Helical entry is limited to offset pockets/open pockets/prismatic roughing, not raster Face Top or zigzag pockets. Open-chain ramps traverse back and forth before the final pass in selected direction.

V finishing traces the medial axis and outlines depth-limited wide regions; clearing the entire wide flat floor is V-carve roughing (M8). Flat-tip V finishing, general controller cutter compensation, tabs with arc leads, start-point objects, and full 3D fixture contact remain outside this preview. Generated offsets can leave corners narrower than the cutter; this is not an adaptive/rest-machining implementation. Controller presets are software-tested, not physical-machine certifications.
