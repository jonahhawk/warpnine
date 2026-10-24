# Accuracy and benchmark contract (M0 / B03)

This establishes measurement semantics, not a claim that the current implementation meets a production tolerance.

## Separate quantities

| Quantity | M0 fixture setting / measurement | Interpretation |
| --- | --- | --- |
| Source units | DXF explicitly mm; STL assumed mm; `.warp` internally mm | Import scaling must be correct regardless of display/fallback units |
| Source model tolerance | No CAD-file tolerance in synthetic inputs | Do not invent Rhino modeling tolerance for these files |
| Analytic-to-mesh approximation | Paraboloid at 2 mm XY grid | On each triangle, quadratic interpolation error is at most 0.004 mm in Z; this is not arbitrary Brep certification |
| DXF arc approximation | 0.01 mm request | Check geometry independently, not just the requested setting |
| Saved circle approximation | 64 segments, radius 2 mm | Maximum radial chord error is 2*(1-cos(pi/64)), about 0.00241 mm |
| CAM path tolerance | 0.01 mm | Generator simplification/tessellation input, not final part error |
| Z-map resolution | 0.25 mm | Sampling distance; does not equal maximum surface/cutter error. Since D02 (2026-10-25) finishing operations sample exact mesh contacts instead and measure the path's distance to the contact surface; see [M5_3D.md](M5_3D.md) |
| Inspection segment tolerance | 0.01 mm | Arc/helix expansion for baseline length/simulation measurement |
| Pocket verification sampling | At most 0.05 mm along cut segments | Sampled analytic clearance check with 0.02 mm allowance; not swept-volume proof |
| Simulation | Stock maximum dimension / 200 cells, 0.2 mm here | Height-field approximation; cannot certify undercuts/holders |
| Post | Mach3 default, four decimal mm coordinates; config snapshot in results | Rounding is separate from generation/contact error |

Future production error budget must combine source/meshing, cutter contact, path sampling/fitting and post rounding, using compatible distance definitions and independently measured bounds. Refuse unsupported geometry/accuracy combinations rather than advertising a bound from grid size alone. Simulation resolution is an independent inspection setting and may miss a smaller violation.

## Baseline measurement protocol

1. Generate fixtures with the standard-library script; review any fixture changes. Freeze tool definitions and operation parameters in the fixture files.
2. Run the release example `cam_baseline` on the fixed manifest: one warmup and three measured generations per case. Store all samples and their median. Import, post, simulation and compile times are excluded from generation timing.
3. Record OS, architecture, CPU, available logical processors, RAM, toolchain/build profile, Rayon workers and static CRT flags. Avoid comparing performance across different hardware/configurations without labeling it.
4. Record cut and rapid lengths, move count, nominal motion time, simulation volume/rapid warnings and posted interpreter motion count. Motion time uses segment feed and 3000 mm/min rapid; excludes acceleration, dwell, spindle and tool-change time.
5. Memory metrics are exact vector-capacity accounting for inline path storage and stock-grid floats only. They are **not** peak process RSS or CAM scratch memory. Add peak-allocation/RSS instrumentation before using memory results for scaling limits.
6. Store input SHA-256 hashes with the report so changed input cannot masquerade as an algorithm improvement. Store the complete post config and tools with each run.
7. The expected projected-open-curve error is a measured known limitation. Unexpected success or a different error requires review of the baseline rather than automatic acceptance.

The benchmark is informational; no elapsed-time test threshold is set in M0. Establish targets only after representative larger jobs and repeated runs. Fixture equality does not require identical JSON numeric formatting or the last binary floating-point digit; semantic round-trip comparisons use 1e-12 relative/absolute floor for numeric fields and exact array order/field presence.

## Required accuracy evidence beyond M0

Use swept cutter checks, model-distance/gouge comparisons, coverage/residual-stock analysis, holders/fixtures, arc-fitting checks and interpreted posted motion. Include narrow gaps, near-tangent intersections, vertical walls, steep/shallow transitions and undercut tools where supported. M0 does not certify any operation family or authorize machine trials.
