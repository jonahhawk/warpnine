# M2 Rhino / openNURBS import

Implemented 2026-10-24. The native reader is `warp-import-3dm`; the app has **Import Rhino / openNURBS…** and **Reimport Rhino source** controls.

## Delivered

- Pinned McNeel openNURBS source, built as a static MSVC library with a narrow C ABI. A serialized native call owns the model and all its objects; Rust receives one UTF-8 result allocation and releases it through the matching bridge function. Native exceptions become errors. No Rhino installation is required to read files.
- Points, lines, arcs/circles, polylines, polycurves and rational/periodic NURBS curves. Exact NURBS representations are retained alongside 3D machining polylines. Each positive-weight Bezier span is subdivided until its control hull lies within the requested chord tolerance; cumulative instance scale is included in that bound. Non-positive weights and refinement limits produce unavailable-object diagnostics.
- Model units converted once to millimeters, with original units, model tolerance and approximation tolerance retained. Unitless files require units to be set in Rhino first. UUIDs, names, layers and visibility are retained.
- Explicit triangle/quad meshes, block definitions and nested instance transforms. Instances remain separate selectable objects, including mirrors and nonuniform scale. Browser labels identify the instance chain. No destructive block explosion occurs.
- Reimport of the same source path replaces that source's objects transactionally, matching UUIDs and instance identity. Deleted or structurally changed curves leave unresolved references. Source scopes prevent UUID collisions between different files. Changes invalidate generated paths through the M1 dependency checks; import/reimport is undoable.
- Persistent unavailable objects and structured diagnostics for unsupported geometry and missing linked content. Linked files are never loaded automatically. Cyclic definitions reject the import. Explicitly choose supported geometry to machine an incomplete file; AllEligible is blocked when unavailable objects exist.
- [Solid meshing decision and measurements](../decisions/3dm-meshing.md). M2 uses explicit meshes; B-reps/extrusions/surfaces/SubD require mesh creation in Rhino. Cached render meshes are not silently used for machining.

## Build

Windows x64 with Rust MSVC, Git, Visual Studio C++ build tools and CMake. The build script finds Visual Studio's bundled CMake or uses the `CMAKE` environment variable. The SDK is pinned to `eb92af3ba1806b0a34a99aba0d3bda83e3d46083`; fresh builds verify the checkout against `vendor/opennurbs-pin.txt`.

```powershell
.\scripts\fetch-opennurbs.ps1
cargo build --release --target-dir target/m2-build
.\target\m2-build\release\warpnine.exe
```

The first native build takes several minutes. Subsequent builds reuse it. `vendor/opennurbs` is an ignored SDK checkout; the fetch script and revision pin are the reproducible source of that dependency. Native runtime libraries are linked statically. Include [the openNURBS/zlib notice](../third-party/openNURBS-LICENSE.txt) when distributing the executable. This native integration is currently qualified on Windows/MSVC only.

## Test it

1. Launch the M2 executable above. In CAM, click **Import Rhino / openNURBS…** and open `fixtures/rhino-m2/parts-mm-v8.3dm`. Leave the curve tolerance at 0.01 mm.
2. Expand the import report. Expect two definitions, named layers, loose curves, a point, an explicit mesh and three resolved copies of the block perimeter. The nested copy is mirrored/scaled. The browser distinguishes Part A, Part B and the nested chain.
3. Preselect only **Block perimeter · Part B**, then add a Profile operation. Verify its Regions selection is Selected and contains only that instance. Configure tool/depth/stock, generate and preview: the selected perimeter is at X=30–40 mm, Y=0–5 mm. Other instances must remain uncut.
4. Save a new `.warp` project and reopen it. The same instance assignment must persist. Use **Reimport Rhino source** after editing/saving that source in Rhino. Changing a selected curve invalidates its path; deleting it requires selection repair. Undo restores the previous imported document.
5. In a fresh project, import `parts-inch-v6.3dm`. It must occupy the same physical coordinates as the mm file. Do not import both into one project for this comparison: different source files deliberately create independent scopes.
6. Import `solids-v8.3dm` to inspect solid/mesh diagnostics. Its trimmed face, spherical B-rep and extrusion stay unavailable; they must not be machined through AllEligible.

```powershell
cargo test --workspace
cargo test -p warp-import-3dm --test import
cargo run -p warp-import-3dm --example generate_fixtures
python scripts/probe_3dm_meshing.py
```

Acceptance tests cover native files in versions 6/8 plus McNeel's version-7 curve/mesh/point samples, unit scaling, rational weights, mixed polycurves, periodic/nonplanar curves, nested/mirrored/scaled blocks, one-instance machining, project round-trip, reimport changes/deletion/topology, corrupt input, missing definitions, cycles, unitless files and solid diagnostics. An unnamed-object crash found in the official samples was fixed and remains covered by those tests.

## Limits

- Qualified file versions: Rhino 6–8. Earlier formats are best effort through the SDK; newer files require Save As Rhino 8. Unsupported types, including point clouds, remain diagnostic objects.
- References are whole-object and instance references. B-rep face/edge selection and topology matching await the solid workflow. Changing curve topology or sampling count conservatively requires re-selection instead of reusing a potentially wrong start vertex.
- Reimport uses the same canonical source path. Moving or renaming a source file creates a new import scope; automatic relinking is not implemented. Source files are not modified by import.
- Solid tessellation, nesting and high-speed machining are later milestones. The meshing experiment is an analytic feasibility check, not a generic solid mesher or accuracy certification.
- File import currently runs synchronously. Large native reads can pause the UI. Toolpath generation retains M1's cancellable background queue.

## Verification recorded 2026-10-24

The workspace suite passed all 56 tests (including seven importer integration tests), and the final linked-content guard passed the focused importer suite. Desktop smoke testing used the native `.3dm` file dialog to import the mm fixture, checked the named instance hierarchy, preselected Part B, created a Profile with one region reference, and generated its toolpath. The other block instances remained unselected and uncut. The optimized app is packaged separately at `target/m2-build/release/warpnine.exe`.

The desktop smoke project is also saved locally at `target/m2-smoke-part-b.warp`. Its saved Regions selection contains one reference with Part B's source instance UUID (00000015-1234-5678-9000-000000000000). It uses the local T1 tool binding; inspect tool/depth settings before posting.
