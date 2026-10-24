# M0 baseline record

Date: 2026-10-24. Scope: B01-B04 in [the build plan](../CAM_BUILD_PLAN.md). Production generators and the project format are unchanged.

## Original workspace

Before adding tests/development dependencies, `cargo test --workspace` passed **31 tests, 0 failed, 0 ignored**, plus doc tests with no cases. Breakdown: app 1, CAM unit 3, CAM roundtrip 4, V-carve 4, core 7, G-code 6, machine 3, simulation 1, tools 2. The initial sandbox invocation was slow to produce output; both that invocation and the subsequent run outside the sandbox ultimately completed successfully. No pre-existing suite failures were observed.

The UC100 demo test is environment-dependent and can return early when the vendor DLL/demo is unavailable. Its test-runner success is not evidence of real hardware validation. Existing sample files (`Drawing1.dxf`, `Untitled.stl`, `test.warp`) are preserved and are optional manual smoke-test inputs.

## New evidence

- Seven focused characterization tests in `crates/warp-cam/tests/baseline.rs`, covering DXF/STL geometry/units, legacy save/reopen, analytic open profile, drill centers/depth, sampled pocket/island clearance and two legacy selection limitations.
- Five deterministic legacy projects, frozen tool definitions, DXF and STL in `fixtures/cam-baseline`, generated without the CAM implementation.
- Release benchmark harness at `crates/warp-cam/examples/cam_baseline.rs`, driven by `fixtures/cam-baseline/benchmark.json`.
- [Project migration contract](../contracts/PROJECT_FORMAT.md), [accuracy contract](ACCURACY_CONTRACT.md), and [strategy ledger](STRATEGY_COVERAGE.md).

After adding the M0 harness, `cargo test --workspace` passed **38 tests, 0 failed, 0 ignored**. The seven new tests are additional to the original 31. The new numeric round-trip test initially compared JSON number representation (`2` versus `2.0`); its oracle was corrected to compare numeric meaning with a 1e-12 tolerance. This was a test-authoring issue, not a production regression.

The release example successfully measured four generated paths and the expected projected-open-curve rejection. Detailed metrics live in [m0-measurements.json](m0-measurements.json), with hardware/toolchain and input/source hashes in [environment.json](environment.json). Memory numbers cover vector backing storage only, not peak RSS; motion-time estimates omit acceleration and auxiliary actions. This baseline is not a performance guarantee.

Recorded environment: Windows x86_64, 8 available logical processors, 34,257,506,304 bytes installed RAM, Rust/Cargo 1.99.0, release profile with static CRT, 8 Rayon workers. RAM required an approved read-only hardware query after sandbox access was denied.

| Fixture | Median generation ms (3 samples) | Cutting travel mm | Rapid/material warnings |
| --- | ---: | ---: | ---: |
| Pocket with island | 1.4218 | 1548.2868 | 2 |
| Open profile | 0.0041 | 55.0000 | 0 |
| Drill pattern | 0.0035 | 16.0000 | 0 |
| Sculpted parallel | 2.1629 | 1411.1762 | 0 |
| Projected open curve | 0.0005 to expected rejection | Not generated | Not simulated |

Cutting travel includes feed entries and links, so the profile's 55 mm exceeds its two 25 mm horizontal cuts. Each simulated stock grid occupies 121,404 bytes; path vector backing storage ranges from 1,152 to 147,456 bytes. These small fixtures characterize behavior rather than representative production throughput.

## Known limitations to carry into M1

| ID | Observation | Evidence | Resolution |
| --- | --- | --- | --- |
| M0-K01 | Open projected drive curve is rejected as closed containment | `projected-open-curve.warp`; explicit expected-error test | G03/G08 separate roles; migration flags ambiguous legacy intent |
| M0-K02 | Empty indexed selection means all curves in 2D; stock boundary in 3D | `Project::select_curves`, `ops3d::setup`; legacy-selection test | G02/G03 preserve explicit per-operation intent |
| M0-K03 | Missing curve indices silently disappear | `select_curves([1,999])` characterization | G02 unresolved-reference validation |
| M0-K04 | Z-map/contact and height-field simulation are approximations | Current implementation; accuracy contract | M5 contact/verification work; M8 undercut representation |
| M0-K05 | Pocket fixture reports two rapid/material warnings in the existing simulator | Release benchmark; collision positions/depths retained in measurement JSON | Investigate entry/retract motions and simulation sampling in M3/C02; root cause not established by M0 |

Known-bug characterization tests pass when they reproduce the old limitation. Replace their expectations when the corresponding fix lands; do not retain them as desired behavior. No known failure has been hidden with `#[ignore]`.
