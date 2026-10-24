use glam::DVec3;
use std::{
    path::PathBuf,
    sync::atomic::{AtomicU64, Ordering},
};
use warp_core::{interchange::*, scene::*, selection::*};
use warp_import_3dm::{write_acceptance_fixture, OpenNurbsImporter};
static NEXT: AtomicU64 = AtomicU64::new(0);
struct Fixture(PathBuf);
impl Fixture {
    fn new(variant: i32, version: i32) -> Self {
        let p = std::env::temp_dir().join(format!(
            "warp-3dm-{}-{}.3dm",
            std::process::id(),
            NEXT.fetch_add(1, Ordering::Relaxed)
        ));
        write_acceptance_fixture(&p, variant, version).unwrap();
        Self(p)
    }
    fn read(&self) -> ImportResult {
        OpenNurbsImporter
            .read(&self.0, &ImportOptions::default())
            .unwrap()
    }
}
impl Drop for Fixture {
    fn drop(&mut self) {
        let _ = std::fs::remove_file(&self.0);
    }
}
fn curve_points(o: &ResolvedObject) -> Vec<DVec3> {
    match &o.geometry {
        Geometry::Curve { points, .. } => points
            .iter()
            .map(|p| o.transform.transform_point3(*p))
            .collect(),
        _ => panic!("Not a curve"),
    }
}
fn fixture_project(import: &ImportResult) -> warp_cam::Project {
    let root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../fixtures/cam-baseline");
    let mut p = warp_cam::Project::load(&root.join("open-profile.warp")).unwrap();
    p.scene = Some(import.scene.clone());
    p.refresh_view().unwrap();
    p
}
fn tools() -> warp_tools::ToolLibrary {
    warp_tools::ToolLibrary::load(
        &PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../fixtures/cam-baseline/tools.json"),
    )
    .unwrap()
}
fn part_b(scene: &Scene) -> ResolvedObject {
    scene
        .resolved()
        .unwrap()
        .into_iter()
        .find(|o| {
            o.name == "Block perimeter"
                && o.reference
                    .instances
                    .first()
                    .is_some_and(|i| i.0.contains("00000015-"))
        })
        .unwrap()
}
#[test]
fn units_curves_metadata_and_exact_rational_data() {
    let mm = Fixture::new(0, 8).read();
    let inch = Fixture::new(1, 6).read();
    assert_eq!(mm.file_version, 80);
    assert_eq!(inch.file_version, 60);
    assert!((inch.unit_scale_mm - 25.4).abs() < 1e-12);
    assert!(mm.diagnostics.is_empty(), "{:?}", mm.diagnostics);
    assert_eq!(mm.scene.definitions.len(), 2);
    assert_eq!(mm.scene.layers.len(), 3);
    for imported in [&mm, &inch] {
        assert!((imported.source_tolerance_mm - 0.001).abs() < 1e-12);
        let objects = imported.scene.resolved().unwrap();
        let rational = objects
            .iter()
            .find(|o| o.name == "Rational quarter circle")
            .unwrap();
        let pts = curve_points(rational);
        for p in pts {
            assert!((p.length() - 10.0).abs() < 1e-9);
        }
        let Geometry::Curve {
            analytic:
                Some(AnalyticCurve::Nurbs {
                    weights,
                    knots,
                    controls,
                    ..
                }),
            ..
        } = &rational.geometry
        else {
            panic!()
        };
        assert!((weights[1] - 0.5f64.sqrt()).abs() < 1e-12);
        assert_eq!(knots.len(), 6);
        assert!((controls[1].x - 10.0).abs() < 1e-9);
        let mixed = objects
            .iter()
            .find(|o| o.name == "Mixed polycurve")
            .unwrap();
        let pts = curve_points(mixed);
        assert!(pts[0].distance(DVec3::new(0., 30., 0.)) < 1e-9);
        assert!(pts.last().unwrap().distance(DVec3::new(15., 35., 0.)) < 1e-9);
        let periodic = objects
            .iter()
            .find(|o| o.name == "Periodic nonplanar")
            .unwrap();
        assert!(matches!(
            periodic.geometry,
            Geometry::Curve { closed: true, .. }
        ));
        assert!(imported
            .scene
            .objects
            .iter()
            .all(|o| o.source.as_ref().unwrap().uuid.is_some()));
    }
    let a = part_b(&mm.scene);
    let b = part_b(&inch.scene);
    for (a, b) in curve_points(&a).iter().zip(curve_points(&b)) {
        assert!(a.distance(b) < 1e-9);
    }
}
#[test]
fn nested_mirrored_scaled_instances_remain_distinct_and_bound_error() {
    let i = Fixture::new(0, 8).read();
    let objects = i.scene.resolved().unwrap();
    let parts: Vec<_> = objects
        .iter()
        .filter(|o| o.name == "Block perimeter")
        .collect();
    assert_eq!(parts.len(), 3);
    let nested = parts
        .iter()
        .find(|o| o.reference.instances.len() == 2)
        .unwrap();
    let pts = curve_points(nested);
    assert!(pts
        .iter()
        .any(|p| p.distance(DVec3::new(60., 45., 0.)) < 1e-9));
    assert!(nested.transform.determinant() < 0.);
    assert_ne!(parts[0].reference, parts[1].reference);
    let hole = objects
        .iter()
        .find(|o| o.name == "Block hole" && o.reference.instances.len() == 2)
        .unwrap();
    let Geometry::Curve { points, .. } = &hole.geometry else {
        panic!()
    };
    for edge in points
        .iter()
        .zip(points.iter().cycle().skip(1))
        .take(points.len())
    {
        let mid = (*edge.0 + *edge.1) * 0.5;
        let center = DVec3::new(5., 2.5, 0.);
        let analytic = center + (mid - center).normalize() * 0.8;
        assert!(
            hole.transform
                .transform_point3(mid)
                .distance(hole.transform.transform_point3(analytic))
                <= 0.0100001
        );
    }
}
#[test]
fn one_instance_machines_and_survives_save_reimport_and_deletion() {
    let f = Fixture::new(0, 8);
    let imported = f.read();
    let b = part_b(&imported.scene);
    let mut p = fixture_project(&imported);
    p.operations[0].common.geometry.as_mut().unwrap().regions = Selection {
        mode: SelectionMode::Selected,
        references: vec![b.reference.clone()],
    };
    let tp = warp_cam::generate(&p.operations[0], &p, &tools()).unwrap();
    for (_, end, rapid, _) in tp.segments(DVec3::ZERO, 0.01) {
        if !rapid {
            assert!(end.x >= 30. - 1e-8 && end.x <= 40. + 1e-8);
            assert!(end.y >= -1e-8 && end.y <= 5. + 1e-8);
        }
    }
    let restored = warp_cam::Project::from_json(&p.document_json().unwrap()).unwrap();
    assert_eq!(restored.operations[0], p.operations[0]);
    let before = warp_cam::workflow::input_key(&p.operations[0], &p, &tools()).unwrap();
    write_acceptance_fixture(&f.0, 2, 8).unwrap();
    f.read().merge_into(p.scene.as_mut().unwrap()).unwrap();
    p.refresh_view().unwrap();
    assert_eq!(
        p.operations[0]
            .common
            .geometry
            .as_ref()
            .unwrap()
            .regions
            .references[0],
        b.reference
    );
    assert_ne!(
        before,
        warp_cam::workflow::input_key(&p.operations[0], &p, &tools()).unwrap()
    );
    write_acceptance_fixture(&f.0, 3, 8).unwrap();
    f.read().merge_into(p.scene.as_mut().unwrap()).unwrap();
    p.refresh_view().unwrap();
    assert!(warp_cam::generate(&p.operations[0], &p, &tools())
        .unwrap_err()
        .to_string()
        .contains("Unresolved"));
}
#[test]
fn solid_diagnostics_and_nonplanar_curves_do_not_silently_machine() {
    let imported = Fixture::new(4, 8).read();
    assert_eq!(imported.diagnostics.len(), 3);
    assert!(
        imported
            .diagnostics
            .iter()
            .any(|d| d.message.contains("1 cached render meshes")),
        "{:?}",
        imported.diagnostics
    );
    assert_eq!(
        imported
            .scene
            .objects
            .iter()
            .filter(|o| matches!(o.geometry, Geometry::Unavailable { .. }))
            .count(),
        3
    );
    let mut p = fixture_project(&imported);
    p.operations[0].common.geometry.as_mut().unwrap().regions = Selection::all();
    assert!(warp_cam::generate(&p.operations[0], &p, &tools())
        .unwrap_err()
        .to_string()
        .contains("unavailable"));
    let nonplanar = imported
        .scene
        .resolved()
        .unwrap()
        .into_iter()
        .find(|o| o.name == "Periodic nonplanar")
        .unwrap();
    p.operations[0].common.geometry.as_mut().unwrap().regions = Selection {
        mode: SelectionMode::Selected,
        references: vec![nonplanar.reference],
    };
    assert!(warp_cam::generate(&p.operations[0], &p, &tools())
        .unwrap_err()
        .to_string()
        .contains("not planar"));
}
#[test]
fn corrupt_files_and_invalid_tolerance_fail_without_scene_mutation() {
    let f = Fixture::new(0, 8);
    let imported = f.read();
    let mut existing = Scene::default();
    imported.merge_into(&mut existing).unwrap();
    let count = existing.objects.len();
    std::fs::write(&f.0, b"not a 3dm archive").unwrap();
    assert!(OpenNurbsImporter
        .read(&f.0, &ImportOptions::default())
        .is_err());
    assert_eq!(existing.objects.len(), count);
    assert!(OpenNurbsImporter
        .read(
            &f.0,
            &ImportOptions {
                tolerance_mm: f64::NAN,
                ..Default::default()
            }
        )
        .is_err());
}

#[test]
fn missing_definitions_cycles_units_and_changed_topology_require_repair() {
    let missing = Fixture::new(5, 8).read();
    assert!(missing
        .diagnostics
        .iter()
        .any(|d| d.message.contains("Missing linked")));
    let cycle = Fixture::new(6, 8);
    let err = OpenNurbsImporter
        .read(&cycle.0, &ImportOptions::default())
        .unwrap_err()
        .to_string();
    assert!(err.contains("Cyclic"), "{err}");
    let unitless = Fixture::new(7, 8);
    assert!(OpenNurbsImporter
        .read(&unitless.0, &ImportOptions::default())
        .unwrap_err()
        .to_string()
        .contains("no units"));
    let f = Fixture::new(0, 8);
    let mut scene = f.read().scene;
    let selected = part_b(&scene).reference;
    write_acceptance_fixture(&f.0, 8, 8).unwrap();
    f.read().merge_into(&mut scene).unwrap();
    assert!(!scene
        .resolved()
        .unwrap()
        .iter()
        .any(|o| o.reference.same_object(&selected)));
}
#[test]
fn official_sdk_v7_curve_and_mesh_samples_import() {
    let root =
        PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../vendor/opennurbs/example_files/V7");
    for name in ["v7_my_curves.3dm", "v7_my_mesh.3dm", "v7_my_points.3dm"] {
        let imported = OpenNurbsImporter
            .read(&root.join(name), &ImportOptions::default())
            .unwrap();
        assert_eq!(imported.file_version, 70);
        assert!(!imported.scene.objects.is_empty());
        if name == "v7_my_points.3dm" {
            assert!(imported
                .scene
                .objects
                .iter()
                .any(|o| matches!(o.geometry, Geometry::Point(_))));
            assert_eq!(imported.diagnostics.len(), 1); // sample also contains an unsupported point cloud
            assert_eq!(imported.diagnostics[0].code, "unavailable_geometry");
        } else {
            assert!(
                imported.diagnostics.is_empty(),
                "{name}: {:?}",
                imported.diagnostics
            );
        }
    }
}

fn mesh_of<'a>(scene: &'a Scene, name: &str) -> &'a warp_core::Mesh {
    match &scene
        .objects
        .iter()
        .find(|o| o.name == name)
        .unwrap_or_else(|| panic!("no object {name}"))
        .geometry
    {
        Geometry::Mesh(m) => m,
        other => panic!("{name} is not a mesh: {other:?}"),
    }
}
fn solid_of<'a>(scene: &'a Scene, name: &str) -> (&'a str, Option<&'a ObjectId>) {
    match &scene.objects.iter().find(|o| o.name == name).unwrap().geometry {
        Geometry::Unavailable {
            reason,
            represented_by,
        } => (reason.as_str(), represented_by.as_ref()),
        other => panic!("{name} is not a solid: {other:?}"),
    }
}
fn sculpted_project(scene: &Scene) -> warp_cam::Project {
    let root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../fixtures/cam-baseline");
    let mut p = warp_cam::Project::load(&root.join("sculpted-parallel.warp")).unwrap();
    p.scene = Some(scene.clone());
    p.refresh_view().unwrap();
    p
}

#[test]
fn exported_machining_meshes_are_measured_against_their_solids() {
    let imported = Fixture::new(9, 8).read();
    let scene = &imported.scene;
    let codes: Vec<&str> = imported.diagnostics.iter().map(|d| d.code.as_str()).collect();
    assert!(
        !codes
            .iter()
            .any(|c| ["mesh_unqualified", "mesh_topology", "mesh_source_missing"].contains(c)),
        "{:?}",
        imported.diagnostics
    );
    assert_eq!(codes.iter().filter(|c| **c == "solid_represented").count(), 4);

    // Inscribed 128x64 polyhedron of a radius-10 sphere: the sagitta is measurable but
    // stays under 0.0105 mm.
    let sphere = mesh_of(scene, "Sphere mesh");
    assert_eq!(sphere.tris.len(), 16128);
    let p = sphere.provenance.as_ref().unwrap();
    let m = p.measured.as_ref().unwrap();
    assert_eq!(
        (p.source_type.as_str(), p.requested_tolerance_mm),
        ("brep", Some(0.01))
    );
    assert!(
        m.source_to_mesh_mm > 0.003 && m.source_to_mesh_mm < 0.0105,
        "{m:?}"
    );
    assert!(m.mesh_to_source_mm < 0.0105, "{m:?}");
    assert_eq!(m.outside_trim_samples, 0);
    assert!(m.source_samples > 1000 && m.mesh_samples > 1000);
    assert_eq!(p.face_ranges.len(), 1);
    assert_eq!((p.face_ranges[0].start, p.face_ranges[0].end), (0, 16128));
    assert_eq!(p.exporter.as_deref(), Some("fixture/1"));

    // Planar annulus with a hole: the 128-gon chord error at radius 10 is 0.0030 mm.
    let annulus = mesh_of(scene, "Annulus mesh").provenance.as_ref().unwrap();
    let dev = annulus.deviation_mm().unwrap();
    assert!(dev > 0.002 && dev < 0.0035, "{dev}");
    assert_eq!(annulus.measured.as_ref().unwrap().outside_trim_samples, 0);
    assert_eq!(annulus.measured.as_ref().unwrap().per_face_mm.len(), 1);

    let extrusion = mesh_of(scene, "Extrusion mesh").provenance.as_ref().unwrap();
    let dev = extrusion.deviation_mm().unwrap();
    assert!(dev > 0.002 && dev < 0.0105, "{dev}");
    assert_eq!(extrusion.source_type, "extrusion");

    let subd = mesh_of(scene, "SubD mesh").provenance.as_ref().unwrap();
    assert_eq!(subd.source_type, "subd");
    let dev = subd.deviation_mm().unwrap();
    assert!(dev > 0.0 && dev < 0.5, "{dev}");
    assert!(subd
        .note
        .as_deref()
        .unwrap_or("")
        .contains("limit polyhedron"));

    for name in [
        "Qualified sphere",
        "Qualified annulus",
        "Qualified extrusion",
        "Qualified SubD",
    ] {
        let (reason, by) = solid_of(scene, name);
        assert!(
            reason.contains("Represented by qualified machining mesh"),
            "{name}: {reason}"
        );
        let by = by.unwrap_or_else(|| panic!("{name} has no mesh"));
        assert!(matches!(
            scene.objects.iter().find(|o| &o.id == by).unwrap().geometry,
            Geometry::Mesh(_)
        ));
    }

    // Mirrored block instance: the resolved mesh keeps outward normals.
    let boxed = scene
        .resolved()
        .unwrap()
        .into_iter()
        .find(|o| o.name == "Box mesh")
        .unwrap();
    assert!(boxed.transform.determinant() < 0.0);
    let Geometry::Mesh(bm) = &boxed.geometry else {
        panic!()
    };
    let placed = bm.transformed(&boxed.transform);
    assert!(
        (placed.signed_volume() - 64.0).abs() < 1e-9,
        "{}",
        placed.signed_volume()
    );
    assert!(placed.analyze(1e-6).closed());
    assert!((placed.bbox().min.x - 116.0).abs() < 1e-9);

    // Selecting the solid machines its mesh in setup coordinates; a tight tolerance refuses it.
    let mut p = sculpted_project(scene);
    let solid = scene
        .resolved()
        .unwrap()
        .into_iter()
        .find(|o| o.name == "Qualified sphere")
        .unwrap();
    let control = p.operations[0].common.geometry.as_mut().unwrap();
    control.part_surfaces = Selection {
        mode: SelectionMode::Selected,
        references: vec![solid.reference],
    };
    p.operations[0].common.tolerance = 0.02;
    let (resolved, _) = p.resolve_operation(&p.operations[0]).unwrap();
    assert_eq!(resolved.mesh.unwrap().tris.len(), 16128);
    p.operations[0].common.tolerance = 0.002;
    let err = p
        .resolve_operation(&p.operations[0])
        .unwrap_err()
        .to_string();
    assert!(err.contains("exceeds the operation tolerance"), "{err}");
    // Nothing in this file is unrepresented, so AllEligible part surfaces are allowed,
    // but every included mesh must still meet the tolerance: the SubD mesh does not at 0.02 mm.
    p.operations[0].common.tolerance = 0.02;
    p.operations[0]
        .common
        .geometry
        .as_mut()
        .unwrap()
        .part_surfaces = Selection::all();
    let err = p
        .resolve_operation(&p.operations[0])
        .unwrap_err()
        .to_string();
    assert!(
        err.contains("SubD mesh") && err.contains("exceeds the operation tolerance"),
        "{err}"
    );
    p.operations[0].common.tolerance = 1.0;
    let (resolved, _) = p.resolve_operation(&p.operations[0]).unwrap();
    assert!(resolved.mesh.unwrap().tris.len() > 16128);
}

#[test]
fn unqualified_meshes_keep_their_solids_unavailable_with_reasons() {
    let imported = Fixture::new(10, 8).read();
    let scene = &imported.scene;
    let count = |code: &str| {
        imported
            .diagnostics
            .iter()
            .filter(|d| d.code == code)
            .count()
    };
    assert_eq!(count("mesh_unqualified"), 3, "{:?}", imported.diagnostics);
    assert_eq!(count("mesh_source_missing"), 1);
    assert_eq!(count("mesh_topology"), 1);
    assert_eq!(count("solid_represented"), 0);

    // 12x6 sphere: the sagitta of a 0.37 rad half-diagonal on radius 10 is about 0.68 mm.
    let coarse = mesh_of(scene, "Coarse sphere mesh")
        .provenance
        .as_ref()
        .unwrap();
    let dev = coarse.deviation_mm().unwrap();
    assert!(dev > 0.5 && dev < 0.8, "{dev}");
    assert_eq!(coarse.meets_claim(0.001), Some(false));

    // A disc over an annulus fills the radius-3 hole: its centre is 3 mm from the trim edge.
    let disc = mesh_of(scene, "Hole-filling disc")
        .provenance
        .as_ref()
        .unwrap();
    let m = disc.measured.as_ref().unwrap();
    assert!(
        m.mesh_to_source_mm > 2.9 && m.mesh_to_source_mm < 3.05,
        "{m:?}"
    );
    assert!(m.outside_trim_samples > 0);
    assert!(m.source_to_mesh_mm < 0.0035, "{m:?}");

    let net = mesh_of(scene, "Control net mesh")
        .provenance
        .as_ref()
        .unwrap();
    assert!(net.deviation_mm().unwrap() > 0.5);

    let orphan = mesh_of(scene, "Orphan mesh").provenance.as_ref().unwrap();
    assert!(orphan.measured.is_none());
    assert!(orphan.note.as_deref().unwrap().contains("not in the file"));

    let flipped = mesh_of(scene, "Flipped mesh").analyze(1e-6);
    assert_eq!(flipped.inconsistent_edges, 1);
    assert!(!flipped.oriented());

    for name in ["Qualified sphere", "Qualified annulus", "Qualified SubD"] {
        let (reason, by) = solid_of(scene, name);
        assert!(
            by.is_none(),
            "{name} must not be represented by a rejected mesh"
        );
        assert!(reason.contains("Re-mesh"), "{name}: {reason}");
    }
    let (reason, by) = solid_of(scene, "Qualified extrusion");
    assert!(by.is_none() && reason.contains("explicit Mesh"), "{reason}");

    // Unrepresented solids block AllEligible part surfaces.
    let mut p = sculpted_project(scene);
    p.operations[0]
        .common
        .geometry
        .as_mut()
        .unwrap()
        .part_surfaces = Selection::all();
    let err = p
        .resolve_operation(&p.operations[0])
        .unwrap_err()
        .to_string();
    assert!(err.contains("unavailable"), "{err}");
}

#[test]
fn import_cancellation_and_memory_limit_fail_cleanly() {
    use std::sync::{atomic::AtomicBool, Arc};
    let f = Fixture::new(9, 8);
    let err = OpenNurbsImporter
        .read(
            &f.0,
            &ImportOptions {
                cancel: Some(Arc::new(AtomicBool::new(true))),
                ..Default::default()
            },
        )
        .unwrap_err()
        .to_string();
    assert!(err.contains("cancelled"), "{err}");
    let err = OpenNurbsImporter
        .read(
            &f.0,
            &ImportOptions {
                max_triangles: 1000,
                ..Default::default()
            },
        )
        .unwrap_err()
        .to_string();
    assert!(err.contains("memory limit"), "{err}");
    assert!(OpenNurbsImporter
        .read(
            &f.0,
            &ImportOptions {
                max_triangles: 100_000,
                ..Default::default()
            },
        )
        .is_ok());
}

/// The M5 acceptance fixture is produced by Rhino 8 itself through
/// `scripts/rhino/warpnine_make_m5_fixture.py`; the importer must measure the exporter's
/// mesh independently of the recorded request.
#[test]
fn rhino_exported_sculpted_solid_is_qualified_independently() {
    let path = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../fixtures/rhino-m5/sculpted-v8.3dm");
    let imported = OpenNurbsImporter
        .read(&path, &ImportOptions::default())
        .unwrap();
    assert_eq!(imported.file_version, 80);
    let scene = &imported.scene;
    let mesh = scene
        .objects
        .iter()
        .find_map(|o| match &o.geometry {
            Geometry::Mesh(m) if m.provenance.is_some() => Some(m),
            _ => None,
        })
        .expect("exporter mesh with provenance");
    let p = mesh.provenance.as_ref().unwrap();
    assert_eq!(p.exporter.as_deref(), Some("warpnine-rhino-export/1"));
    assert_eq!(p.source_type, "brep");
    assert_eq!(p.requested_tolerance_mm, Some(0.01));
    assert!(p.revision.as_deref().is_some_and(|r| r.len() == 40));
    assert!(p.face_ranges.len() >= 3, "{:?}", p.face_ranges);
    assert_eq!(p.face_ranges.last().unwrap().end, mesh.tris.len());
    let m = p.measured.as_ref().unwrap();
    assert!(m.source_samples > 5000 && m.mesh_samples > 5000, "{m:?}");
    assert_eq!(m.outside_trim_samples, 0, "{m:?}");
    // Rhino's mesher honours its tolerance within the model tolerance; the hole must be open.
    assert!(
        p.meets_claim(imported.source_tolerance_mm) == Some(true),
        "{}",
        p.summary()
    );
    assert!(p.deviation_mm().unwrap() > 1e-5, "a curved solid cannot be met exactly: {}", p.summary());
    let quality = mesh.analyze(1e-4);
    println!("rhino fixture: {} | {} | {:?}", p.summary(), quality.describe(), m);
    assert!(quality.closed() && quality.oriented(), "{}", quality.describe());
    assert!(quality.signed_volume > 0.0);
    let (reason, by) = solid_of(scene, "Sculpted solid");
    assert!(by.is_some() && reason.contains("qualified"), "{reason}");
    assert!(imported.diagnostics.iter().all(|d| d.code == "solid_represented"), "{:?}", imported.diagnostics);
    // The through hole is a mesh-to-source trap: a filled hole would show up as outside-trim samples.
    let mut p3 = sculpted_project(scene);
    p3.operations[0].common.tolerance = 0.02;
    p3.operations[0].common.geometry.as_mut().unwrap().part_surfaces = Selection::all();
    let (resolved, _) = p3.resolve_operation(&p3.operations[0]).unwrap();
    assert_eq!(resolved.mesh.unwrap().tris.len(), mesh.tris.len());
}
