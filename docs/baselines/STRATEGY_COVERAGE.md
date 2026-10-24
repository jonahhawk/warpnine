# CAM strategy coverage ledger (M0 / B04)

Inventory authority: [RhinoCAM feature specification](../RHINOCAM_FEATURE_PLAN.md), derived from the supplied 2026 reference. Status records implementation coverage, not product parity. Updated 2026-10-24.

**Missing:** no corresponding generator. **Prototype:** a corresponding or partial code path exists but full family acceptance is unproven. **Verified:** required parameters, geometry controls, cutter motion, simulation and post tests pass for documented capabilities. **Released:** verified and packaged with end-to-end release evidence. No family is promoted to Verified by M0's limited baseline tests.

Every row requires tool support, role-specific selection, cut/height/feed controls, entry/link validation, simulation, post qualification and a representative project before promotion. The “remaining coverage” column highlights differences rather than enumerating every parameter. Code paths are in `warp-cam/src/ops.rs`, `ops2d.rs`, `ops3d.rs` and `toolpath.rs` unless noted.

## 2.5-axis - 18 reference families

| ID | Family | State | Existing path | Remaining parameter/behavior coverage | Delivery / restriction |
| --- | --- | --- | --- | --- | --- |
| 25-01 | Face Top | Prototype | FaceTop | Selected-region raster delivered; full family qualification remains | M3 preview |
| 25-02 | Roughing | Prototype | Roughing2D | Stock-driven prismatic clearing delivered; engagement/rest pending | M3 preview; adaptive M6 |
| 25-03 | Facing | Prototype | Facing | Island/outer-region behavior, approach and linking qualification | M3 |
| 25-04 | Pocketing | Prototype | Pocket | Open pockets, islands, finish allowances, region validation | M3; pocket-island baseline only |
| 25-05 | Profiling | Prototype | Profile | Side/chain semantics, compensation and lead validity, tab retention | M3; open-profile baseline only |
| 25-06 | V-Carve Roughing | Missing | None | Rough clearance relative to finishing V-tool envelope | M8 |
| 25-07 | V-Carve Finishing | Prototype | VCarve | Depth/corner/tool-envelope qualification and cleanup | M3 |
| 25-08 | Engraving | Prototype | Engrave | True 3D drive curves, contact and entry semantics | M3/M5 |
| 25-09 | Chamfering | Prototype | Chamfer | Contact geometry and selected-edge width/depth | M3 |
| 25-10 | Filleting | Missing | None | Edge rounding with ball/corner-radius tools | M8 |
| 25-11 | Hole Pocketing | Prototype | HelicalBore | Spiral area clear, helix engagement and final circular cleanup | M3 |
| 25-12 | Hole Profiling | Prototype | HoleProfile | Distinct pitch/spring controls delivered; full family qualification remains | M3 preview |
| 25-13 | Thread Milling | Prototype | ThreadMill | Handedness/internal/external, multipass and helix qualification | M3 |
| 25-14 | T-Slotting | Missing | SlotCutter tool shape only | Neck/head access, top/bottom/side stock, undercut stock | M8; volumetric verification required |
| 25-15 | Knife Cutting | Missing | None | Drag-tip offset, corner swivel/lift compensation | M8; suitable knife model |
| 25-16 | Deburring | Missing | Chamfer is related | Selected/automatic edge detection, sharpness and clearance | M8 |
| 25-17 | Saw Cutting | Missing | None | Blade/holder, tool orientation and post semantics | M8; orientation-capable machine for full family |
| 25-18 | Re-Machining | Missing | None | Actual remaining-stock cleanup after pocket/profile/facing | M6 |

## 3-axis - 21 reference families

| ID | Family | State | Existing path | Remaining parameter/behavior coverage | Delivery / restriction |
| --- | --- | --- | --- | --- | --- |
| 3-01 | Horizontal Roughing | Prototype | Roughing3D | Linear/stock-offset/part-offset, levels, stock/contact accuracy | M5 |
| 3-02 | Parallel Finishing | Prototype | Parallel | Surface accuracy, boundary contacts and safe links | M5; sculpted-parallel baseline only |
| 3-03 | Horizontal Finishing | Prototype | Waterline | Exact levels, vertical-wall accuracy, containment | M5 |
| 3-04 | Radial Machining | Prototype | Radial | Region-specific centers and angular limits | M5 |
| 3-05 | Spiral Machining | Prototype | Spiral | Region-specific centers, clipping and continuous linking | M5 |
| 3-06 | Adaptive Roughing | Missing | None | Stock-aware bounded engagement, smoothing, entry/recovery | M6 |
| 3-07 | Plunge Roughing | Missing | None | Z-cut coverage, tool suitability, approach/retreat | M7 |
| 3-08 | Horizontal Re-Roughing | Missing | None | Predecessor stock, rest regions and Z-level clearing | M6 |
| 3-09 | Plunge Re-Roughing | Missing | None | Residual-stock plunge pattern and retreat | M7 |
| 3-10 | Clear Flats Machining | Missing | None | Flat-level detection and stock-filtered cleanup | M6 |
| 3-11 | Projection Pocketing | Missing | None | Planar offsets projected to selected surfaces | M5 |
| 3-12 | 3D Offset Pocketing | Missing | None | True surface-distance offsets, collapse/disconnected areas | M7 |
| 3-13 | 3D Offset Profiling | Missing | None | Surface-following offsets from selected drive curves | M7 |
| 3-14 | Pencil Tracing | Missing | None | Cutter-dependent double-contact paths and branches | M7 |
| 3-15 | Valley Re-Machining | Missing | None | Previous-cutter accessibility and residual cleanup | M7 |
| 3-16 | Flats (Plateau) Machining | Missing | None | Shallow slope classification, coverage/transition overlap | M7 |
| 3-17 | Steeps (Parallel Hill) Machining | Missing | None | Steep classification and locally appropriate direction | M7 |
| 3-18 | Steeps Z (Horizontal Hill) Finishing | Missing | Waterline is related | Slope-restricted constant-Z finishing | M7 |
| 3-19 | Curve Machining | Prototype | ProjectCurves | Separate drive/containment roles, open curves and contact | M1 fix / M5 qualification; M0-K01 |
| 3-20 | Between 2 Curves Machining | Missing | None | Ordered guides, parallel/normal blend, fold detection | M7 |
| 3-21 | Reverse Post Milling | Missing | External G-code simulation only | APT/ISO import plus surface projection and verification | M8; supported input dialects explicit |

## Non-rotary hole making - four reference families

| ID | Family | State | Existing path | Remaining parameter/behavior coverage | Delivery / restriction |
| --- | --- | --- | --- | --- | --- |
| H-01 | Drilling | Prototype | Drill: Simple/Dwell/Peck/ChipBreak | Countersink semantics, sorting, user Drill 1/2, post qualification | M3 basics / M8 custom; drill-pattern baseline only |
| H-02 | Tapping | Missing | Tap shape only | Standard/peck, CW/CCW, user Tap 1-4, synchronization | M8; controller synchronization required |
| H-03 | Boring | Prototype | Drill: BoreFeedOut/BoreDwellFeedOut | Drag/no-drag/manual, user Bore 1/2; spindle orientation | M8; supported controller actions only |
| H-04 | Reverse Boring | Missing | None | Reverse/oriented clearance motions, user RBore 1/2 | M8; spindle orientation required |

## Additional requested/workflow capabilities

| ID | Capability | State | Acceptance dependency |
| --- | --- | --- | --- |
| X-01 | First-class slotting | Prototype | M3; explicit tutorial workflow, tool/slot width and entry |
| X-02 | Planar adaptive clearing | Missing | M6; independent engagement analysis |
| X-03 | Scallop controls | Missing | M7; tool/surface-aware error measurement |
| X-04 | Rhino/openNURBS import | Verified | M2; metadata, blocks and transforms; M5 solid meshing |
| X-05 | Explicit control-geometry roles | Verified | M1; existing indexed picking is a partial foundation |
| X-06 | Block/loose-geometry nesting | Prototype | M4 N01 data model, candidate discovery and persistence; footprints, solver, UI and CAM handoff pending |
| X-07 | Irregular sheets/part-in-hole/common-line | Missing | M8; retention and machining sequence constraints |

M0 inventory totals: **43 reference families** (16 Prototype, 27 Missing), plus seven additional/workflow items. Verified: 0. Released: 0. These deliberately conservative states distinguish existing implementation from qualified feature parity.

M3 update: [preview evidence and restrictions](M3_25D_CAM.md). New operation entries are tracked as prototypes at the full reference-family level; passing the bounded preview does not imply full RhinoCAM parameter parity. The inventory totals above remain the historical M0 baseline.
