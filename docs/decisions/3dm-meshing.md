# Decision: Rhino solid meshing (I05)

Date: 2026-10-24. Status: decided for M2; implemented for M5/D01 on 2026-10-24 with the exporter contract below. Evidence: [M5 3D baseline](../baselines/M5_3D.md). Addendum 2026-10-25 (D01b): users without Rhino import STEP instead; `crates/warp-import-step` meshes the exact B-rep with a pinned static Open CASCADE build and qualifies the result with the same measurement and policy, so Rhino is no longer the only route to a qualified solid.

## Decision

M2 imports explicit mesh objects for machining. B-reps, extrusions, surfaces and SubD objects retain an unavailable-object identity and an actionable diagnostic. They are not replaced by cached render meshes. For the first solid workflow, use Rhino's explicit Mesh command with controlled parameters and save those mesh objects in the `.3dm` file. A future Rhino-side exporter can automate this through `Mesh.CreateFromBrep(brep, meshingParameters)` and record its settings and measured deviation. That exporter is not shipped in M2.

This keeps the standalone WarpNine reader independent of a running Rhino installation. It also separates file reading from the much larger problem of trimmed-surface tessellation. The official openNURBS API reads saved mesh caches; RhinoCommon provides B-rep meshing inside the Rhino host. Sources: [reading render meshes](https://developer.rhino3d.com/en/guides/opennurbs/reading-render-meshes/), [RhinoCommon meshing API](https://mcneel.github.io/rhinocommon-api-docs/api/RhinoCommon/html/M_Rhino_Geometry_Mesh_CreateFromBrep_1.htm).

## Prototypes and evidence

`native/fixture.cpp` constructs actual openNURBS objects: a circular trimmed face with an inner hole, a spherical B-rep with a seam, and a capped extrusion. They are written to `fixtures/rhino-m2/solids-v8.3dm`, read through the native bridge, and tested as unavailable for machining. The trimmed face also contains a deliberately inaccurate cached render mesh that crosses its hole. Its presence cannot enable generation. B-rep diagnostics report face, loop and cache counts.

`scripts/probe_3dm_meshing.py` compares coarse/fine analytic-only meshes of a radius-10 sphere, a planar annulus (radii 3 and 10), and its height-5 extrusion. These are feasibility fixtures, **not a general B-rep conversion algorithm**. Results are recorded in [3dm-meshing-measurements.json](3dm-meshing-measurements.json).

| Candidate | Accuracy / trim behavior | Topology | Dependency / build cost | Decision |
| --- | --- | --- | --- | --- |
| Saved render cache | No dependable machining-error bound; synthetic cache fills a required hole | Depends on source cache | Already accessible through openNURBS | Rejected as machining input |
| Analytic-only parameter grids, 16 segments | Sphere sampled radial error 0.338 mm; annulus boundary error 0.192 mm; hole intrusion 0.0576 mm | Closed sphere/extrusion manifold; planar annulus correctly open | Python standard library in experiment; no general trim support | Fails 0.01 mm target |
| Analytic-only parameter grids, 128 segments | Sphere sampled radial error 0.00535 mm; annulus boundary error 0.00301 mm; hole intrusion 0.000904 mm | Welded sphere seam and extrusion watertight; zero nonmanifold edges | About 12.2 ms sphere construction, 0.4–1.0 ms annulus/extrusion on this machine | Useful test oracle; does not qualify arbitrary B-reps |
| Explicit Rhino-generated mesh / future host exporter | Parameters can be controlled; deviation and trims still need verification | Rhino mesh plus importer validation; watertightness must be measured per part | Rhino required for mesh creation; none required to read resulting file | Selected first integration route |

Sphere error is sampled at triangle centroids, not a proven global bound. Annulus errors use exact polygon sagitta. Timing covers mesh construction only, excludes verification and file I/O, and is not a product performance guarantee. A single trimmed face is intentionally not watertight; its boundary-edge count is expected.

## Exporter contract (implemented in D01)

`scripts/rhino/warpnine_export_meshes.py` runs inside Rhino 8 and writes one mesh object per source solid with these user strings. Lengths are in the file's model units; the importer converts them to millimetres.

| Key | Meaning |
| --- | --- |
| `warpnine.source` | UUID of the B-rep, extrusion, surface or SubD the mesh approximates |
| `warpnine.source_type` | `brep`, `extrusion`, `surface` or `subd` (the importer checks it against the file) |
| `warpnine.tolerance` | requested maximum deviation; a claim, never treated as measured |
| `warpnine.exporter` | exporter name/version |
| `warpnine.revision` | SHA-1 of the source geometry JSON at export time |
| `warpnine.face_ranges` | `face:first-end;...` mesh-face ranges per B-rep face (converted to triangle ranges on import) |
| `warpnine.mesher_tolerance`, `warpnine.exporter_measured` | informational: the Rhino meshing tolerance that met the request and the exporter's own sampled deviation |

The importer measures source-to-mesh and mesh-to-source deviation, counts mesh samples that project outside the source trims, checks manifoldness and orientation, and represents the solid by the mesh only when the measurement meets the claim within the model tolerance. SubD sources are measured against a subdivided limit polyhedron and reported as approximate. Reimport replaces the meshes and invalidates dependent operations through the existing geometry keys.

## Required before M5 solid qualification (status)

The host exporter must record source UUID, units, model tolerance, meshing parameters and source revision. Test curved faces, seam welding, hole boundaries, tiny trims, adjacent-face cracks, mirrored instances and nonuniform scales. Measure source-to-mesh and mesh-to-source deviation, manifold edges and orientation. Reject invalid or incomplete meshes. Reimport must invalidate affected operations. Do not treat requested meshing tolerance as measured accuracy.

A standalone arbitrary trimmed-B-rep mesher is deferred. Implementing only surface grids would miss trim loops, singularities and seam coordination; the analytic prototype does not resolve those problems.
