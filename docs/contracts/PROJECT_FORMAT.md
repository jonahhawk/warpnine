# Project format and migration contract (M0 / B02)

Status: M1 implements the version 1 document envelope, stable whole-object/instance references, legacy migration, validation and transactional saves in `warp-cam/src/document.rs`. The legacy section below describes the pre-M1 baseline. Face/edge topology references, reimport matching, opaque extension preservation and embedded tool/post snapshots remain future extensions; tool numbers and post names remain external bindings.

## Legacy v0, as inspected

`warp-cam/src/project.rs` serializes `Project` directly to JSON: `name`, `display_units`, box `stock`, `curves`, optional `mesh`, `operations`, and `post`. There is no version discriminator. Coordinates, tool dimensions and feeds are internally millimeters and mm/min; `display_units` changes display, not the meaning of stored numbers. Curves are 2D tessellated polylines; one mesh stores triangles. Operation kinds are Serde's externally tagged enum variants.

`Common.curves` contains array indices. In 2D, an empty list selects every curve. In 3D setup, an empty list instead uses the stock XY extent expanded by tool radius. Explicit indices that are out of bounds are silently skipped. Projected-curve operations currently also treat their drive curves as closed containment, so an open-only selection fails. These behaviors are characterized in `crates/warp-cam/tests/baseline.rs`; they are not all desirable future behavior.

Missing `Common.resolution` currently defaults to 0.1 mm; missing pocket `order` defaults to LayerFirst. Other missing required fields fail deserialization. `Project::save` writes directly to the destination; it is not currently transactional. Tool definitions live outside the project, and `post` is only a string, not a complete post configuration.

## Version 1 envelope

M1 writes `{ "schema_version": 1, "document": { ... } }`. The document contains `document_id`, scene objects/definitions/instances, setups, operations, tool numbers/post name and source provenance. Legacy curve/mesh view arrays and active stock caches are omitted; setup stock and scene geometry are authoritative. Runtime toolpaths and job state are not saved. The rules below also define the intended contract for future import and cache extensions.

- No `schema_version` means legacy v0 **only when the v0 required shape parses successfully**. A malformed v1 file must not fall back to v0.
- A non-integer, zero, negative, unsupported future version or unknown operation kind yields a specific unsupported-format diagnostic. Do not coerce it, silently discard it or overwrite it.
- Migrations are explicit ordered functions, `v0 -> v1 -> ...`; validate before and after each step. Test unsupported versions and partial/corrupt documents.
- Versioned objects with unknown required semantics must fail clearly. Optional extension metadata may be preserved in a dedicated opaque map, never mistaken for supported machining data.
- Persist document intent; generation caches are optional and may be discarded. Any cached output needs geometry/tool/setup/parameter/predecessor-stock/algorithm revisions and cannot be considered verified merely because it was loaded.

## Identity and reference rules

- Introduce typed `ObjectId`, `DefinitionId`, `InstanceId`, `OperationId` and `SetupId` wrappers. IDs are independent of list order and names; copying an instance/operation creates a new ID, moving/reordering it does not. Persist IDs on save.
- For deterministic legacy conversion, use a fixed legacy namespace and typed ordinal keys such as `legacy/curve/0`, `legacy/mesh/0`, `legacy/operation/0`, `legacy/setup/0`. Allocate fresh IDs for new objects outside this namespace. References are document-scoped; importing/merging another project remaps all its IDs consistently.
- A geometry reference identifies a definition object plus its ordered instance path, and optionally a face/edge identifier tied to the source topology revision. Two instances of the same block cannot collapse into one selectable object.
- Rhino UUID, source file identity/hash, import settings, original units and source tolerance are provenance, not the sole local identity. Reimport can match stable source objects; changed/unmatched subelement topology requires repair rather than ordinal guessing.
- Store unresolved references with their prior label/provenance and invalidate dependent operations. Never replace a missing reference with AllEligible, an adjacent index, or another instance.

## Legacy migration mappings

| v0 input | v1 intent |
| --- | --- |
| Curve index i | Stable reference to legacy curve i, in original order |
| Single mesh | One scene model object, selected as the legacy part model |
| Stock / operation list | One identity-transform setup, original stock and operation order |
| Nonempty selection for 2D operations | Selected machining regions, drive paths or hole sources according to operation kind |
| Empty selection for 2D operations | Explicit AllEligible legacy curve scope; preserve the original inclusion behavior |
| Empty selection for ordinary 3D operations | Explicit StockBoundary with documented legacy tool-radius expansion |
| Selected closed curves for ordinary 3D operations | Selected containment regions; original model as part surface |
| ProjectCurves selection | Selected drive paths; preserve old containment intent for closed selections; flag open-only/ambiguous cases as requiring geometry-role review |
| Invalid legacy indices | Unresolved references, operation disabled for generation until repaired; record this intentional behavior change |
| Tool number / post string | Preserve bindings and report unresolved tools/posts; do not silently use a different starter tool or post |

Duplicate ordered drive references stay ordered until reviewed. Do not deduplicate selections indiscriminately. A previously invalid projected-open-curve job must not silently become a new unrestricted cutting job during migration; M1's user repair creates the new explicit intent.

## Units, setups and persistence

All internal model positions, stock, tool dimensions and tolerances remain in millimeters; feed remains mm/min. Source-unit conversion happens exactly once at import. Display units never rescale stored coordinates. Store source units/tolerance alongside normalized values. Setup transforms are invertible rigid model-to-setup transforms; nonuniform block transforms belong to geometry resolution, not the WCS. Validate finite coordinates and transformation matrices.

Loading or previewing an old project does not modify the source. Save an upgraded project to a sibling temporary file, flush/close, validate it, then atomically replace the destination where supported. Retain a recoverable backup on upgrade/overwrite. If replacement fails, report it and leave the original intact. Never overwrite an unsupported/corrupt source merely because defaults can be constructed. Exporting back to v0 is not required.

## Required M1 acceptance tests

1. Every M0 `.warp` fixture migrates deterministically and save/reopens with the same IDs, selection intent, geometry, parameters, order and units.
2. Valid legacy jobs generate geometrically equivalent output within the declared numerical budget; intentional error-handling changes are explicit.
3. Empty 2D vs. 3D selections preserve their different meanings; invalid indices become unresolved, never broadened selections.
4. Reorder/delete objects, duplicate an instance, rename a layer and edit a source: unaffected references survive, affected dependent output becomes dirty.
5. Unknown schema/kind, truncated JSON, missing required fields, nonfinite values and interrupted save fail safely with useful diagnostics.
6. Inch display on a legacy mm-coordinate fixture does not rescale geometry. Transformed setups apply coordinates exactly once.
7. External tool/post changes invalidate generation/output verification even if their human-readable names are unchanged.

M0 freezes the source format and intent fixtures. It deliberately does not change serialization or add a partially implemented schema to production code.


## M2 import extension

Scene geometry now supports `Unavailable { reason }` to retain unsupported Rhino objects without substituting approximation meshes. `Source.approximation_tolerance_mm` is optional for legacy compatibility and distinct from source model tolerance. Rhino IDs are scoped by canonical source path, source UUID and a conservative curve topology signature; instance-path entries use their source object IDs. Reimport replaces only that scope and validates before committing. Whole-object edits remain selected when structurally compatible; removed or changed topology remains unresolved. Older M1 executables cannot open a document containing the new geometry variant; use the M2 build.
