# M1: geometry and workflow foundation

Implemented 2026-10-24. M1 establishes the document and selection workflow; it does not add the remaining RhinoCAM strategy catalog.

## Delivered

- Scene objects, layers, groups, reusable definitions and nested instance transforms with stable identifiers and cycle checks. Exact curve source data has a separate representation from machining polylines.
- Version 1 project files, deterministic legacy migration, explicit selection modes, unresolved-reference errors, validated temporary saves and `.warp.bak` backups on overwrite.
- Separate region, containment, avoid, drive, part, check, hole and start roles. New empty selections never mean all geometry. Legacy empty-selection intent is migrated explicitly.
- Named setups with translated/rotated coordinates, stock and G54–G59. Generation and export use the active setup; model coordinates are transformed once.
- Geometry/layer browser, name/type filtering, curve picking, additive preselection, visibility/isolation, reversal and start-vertex controls, and missing-reference repair. DXF layers survive import.
- A single worker with a bounded queue, cooperative cancellation, and result acceptance by operation identity, request epoch and resolved inputs. Parameter, setup, selected geometry and tool edits invalidate dependent paths; visibility edits do not.
- Document undo/redo, including selection, parameter, operation order and setup changes. History is capped at 32 entries and approximately 64 MiB, except for a single oversized document.
- Projected open drive curves independently clipped to closed containment and cutter-expanded avoid regions, with retracts between disconnected runs.

## Test the application

The side-by-side optimized build is `D:\dev\warpnine\target\m1-build\release\warpnine.exe`. The existing `target\release` executable was left intact because it was running. To rebuild this separate copy, close that copy and run:

```powershell
cargo build --release --target-dir target/m1-build
.\target\m1-build\release\warpnine.exe
```

1. Open `fixtures/cam-baseline/open-profile.warp`. Select its operation and inspect the Regions selection. Generate all: this fixture follows an L-shaped path, totaling 55 mm of cutting.
2. Change a machining parameter. Its ready indicator and generated output must invalidate. Generate again, then preview G-code.
3. Set the selection mode to Selected and clear the region assignment. Generation must report missing geometry. Re-select the curve through the browser or viewport; it must generate again.
4. Use Edit > Undo/Redo to restore selection or parameter changes. Generated output must be regenerated after an undo that changes machining inputs.
5. Add a setup. The operation list should show only that setup's operations. Switch back to Setup 1 to recover the original list. Expand Setup coordinates to try an origin, rotation and G55; regenerate and check the posted work-offset command.
6. Use Save As to save a copy under `target`, then reopen it. Selections, setups and operation parameters must persist. Save that copy again to produce a `.warp.bak` backup.
7. Open `projected-open-curve.warp` from the same fixture directory. Its old ambiguous selection requires explicit review. Assign the open curve as Drive, choose containment, acknowledge the review and generate. The automated test exercises a separate closed boundary and avoid island.

For repeatable automated checks:

```powershell
cargo test --workspace
cargo test -p warp-cam --test document
```

The workspace suite passed on 2026-10-24, including all 11 new document tests. They cover path-equivalent legacy migration, reorder/delete reference behavior, explicit-empty selection, independent instances and cycle rejection, setup transforms and units, projection containment/avoid clipping, invalid schemas/identities, backup persistence, input invalidation, undo/redo and worker cancellation tokens. A desktop smoke test opened the legacy profile, generated its 55 mm path, switched to a newly added setup, then used Undo to restore Setup 1 and invalidate the derived path.

## Limits and follow-on work

- `.3dm`/openNURBS import, Rhino blocks from files, B-rep faces/edges and reimport matching are M2 work. Current imports are DXF and STL. Instance representation/resolution is tested; interactive instance placement is not yet exposed.
- Check-surface collision, fixture collision and point-controlled entry are represented but block generation when assigned. They are not silently ignored. Point hole locations require drilling with CurveStarts; other hole strategies require curves.
- Curve start/reversal controls establish source ordering; comprehensive offset entry/lead behavior belongs to M3. Separate containment/avoid controls currently apply to 3D; 2D pockets use region islands.
- Rest-stock dependencies and predecessor-stock invalidation arrive with rest machining. Current operation keys track the inputs consumed by current generators.
- Cancellation is cooperative; an individual polygon or Z-map computation may finish before observing cancellation. Cancelled or obsolete output is rejected.
- Tools and post settings remain external. Missing tools or unresolved post names block generation/export respectively. Tool-library edits and post settings are outside document undo.
- M0 rapid-link simulator warnings remain open. This milestone does not qualify every existing machining strategy, add high-speed machining, or implement nesting.
