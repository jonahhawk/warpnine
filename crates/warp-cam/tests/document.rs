use glam::{DMat4, DVec2, DVec3};
use std::{
    path::PathBuf,
    sync::{atomic::AtomicBool, Arc},
    time::Duration,
};
use warp_cam::{
    generate,
    ops::OpKind,
    workflow::{input_key, History, Job, JobQueue},
    Project,
};
use warp_core::{scene::*, selection::*, Polyline2};
use warp_tools::ToolLibrary;

fn root() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../fixtures/cam-baseline")
}
fn load(name: &str) -> Project {
    Project::load(&root().join(format!("{name}.warp"))).unwrap()
}
fn tools() -> ToolLibrary {
    ToolLibrary::load(&root().join("tools.json")).unwrap()
}
fn key(p: &Project) -> Vec<u8> {
    input_key(&p.operations[0], p, &tools()).unwrap()
}

#[test]
fn migration_preserves_valid_paths_and_stable_identity() {
    for name in [
        "pocket-island",
        "open-profile",
        "drill-pattern",
        "sculpted-parallel",
    ] {
        let text = std::fs::read_to_string(root().join(format!("{name}.warp"))).unwrap();
        let legacy: Project = serde_json::from_str(&text).unwrap();
        let p = Project::from_json(&text).unwrap();
        let other = Project::from_json(&text).unwrap();
        assert_eq!(p.operations[0].common.id, other.operations[0].common.id);
        assert_eq!(p.curve_refs, other.curve_refs);
        let a = generate(&legacy.operations[0], &legacy, &tools()).unwrap();
        let b = generate(&p.operations[0], &p, &tools()).unwrap();
        let aa = a.segments(DVec3::ZERO, 0.01);
        let bb = b.segments(DVec3::ZERO, 0.01);
        assert_eq!(aa.len(), bb.len(), "{name}");
        for (a, b) in aa.iter().zip(&bb) {
            assert!(
                a.0.distance(b.0) < 1e-8 && a.1.distance(b.1) < 1e-8,
                "{name}"
            );
        }
        let restored = Project::from_json(&p.document_json().unwrap()).unwrap();
        assert_eq!(p.operations[0], restored.operations[0]);
        assert_eq!(p.curve_refs, restored.curve_refs);
    }
}

#[test]
fn deleting_or_reordering_geometry_never_retargets_a_selection() {
    let mut p = load("open-profile");
    let before = key(&p);
    p.scene.as_mut().unwrap().add(
        "unrelated".into(),
        Geometry::polyline(&Polyline2::rect(DVec2::splat(70.0), DVec2::splat(80.0))),
    );
    p.scene.as_mut().unwrap().objects.reverse();
    p.refresh_view().unwrap();
    assert_eq!(before, key(&p));
    let id = p.operations[0]
        .common
        .geometry
        .as_ref()
        .unwrap()
        .regions
        .references[0]
        .object
        .clone();
    p.scene.as_mut().unwrap().objects.retain(|o| o.id != id);
    assert!(generate(&p.operations[0], &p, &tools())
        .unwrap_err()
        .to_string()
        .contains("Unresolved"));
}

#[test]
fn explicit_empty_selection_is_not_all_geometry() {
    let mut p = load("pocket-island");
    p.operations[0].common.geometry.as_mut().unwrap().regions = Selection::default();
    assert!(generate(&p.operations[0], &p, &tools())
        .unwrap_err()
        .to_string()
        .contains("No Machining regions"));
    p.operations[0].common.geometry.as_mut().unwrap().regions = Selection::all();
    assert!(generate(&p.operations[0], &p, &tools()).is_ok());
}

#[test]
fn instances_are_independent_and_cycles_are_rejected() {
    let mut p = load("open-profile");
    let scene = p.scene.as_mut().unwrap();
    let leaf = scene.objects.remove(0);
    let leaf_id = leaf.id.clone();
    let def = DefinitionId("block".into());
    scene.definitions.push(Definition {
        id: def.clone(),
        name: "Part".into(),
        objects: vec![leaf],
    });
    let a = scene.add(
        "A".into(),
        Geometry::Instance {
            definition: def.clone(),
            transform: DMat4::IDENTITY,
        },
    );
    let b = scene.add(
        "B".into(),
        Geometry::Instance {
            definition: def.clone(),
            transform: DMat4::from_translation(DVec3::new(10.0, 2.0, 0.0)),
        },
    );
    scene.validate().unwrap();
    let reference = GeometryRef {
        object: leaf_id,
        instances: vec![InstanceId(b.0.clone())],
        reversed: false,
        start_vertex: 0,
    };
    p.operations[0]
        .common
        .geometry
        .as_mut()
        .unwrap()
        .regions
        .references = vec![reference];
    let (resolved, _) = p.resolve_operation(&p.operations[0]).unwrap();
    assert_eq!(resolved.curves[0].pts[0], DVec2::new(15.0, 7.0));
    p.scene.as_mut().unwrap().objects.retain(|o| o.id != a);
    assert!(generate(&p.operations[0], &p, &tools()).is_ok());
    let scene = p.scene.as_mut().unwrap();
    scene.definitions[0].objects.push(SceneObject {
        id: ObjectId("cycle".into()),
        name: "cycle".into(),
        layer: LayerId("default".into()),
        visible: true,
        source: None,
        geometry: Geometry::Instance {
            definition: def,
            transform: DMat4::IDENTITY,
        },
    });
    assert!(scene.validate().unwrap_err().to_string().contains("cyclic"));
}

#[test]
fn setup_transform_units_and_post_wcs_apply_once() {
    let mut p = load("open-profile");
    p.display_units = warp_core::Units::Inches;
    p.setups[0].origin = DVec3::new(5.0, 5.0, 0.0);
    p.setups[0].rotation_deg.z = 90.0;
    p.setups[0].wcs = 55;
    let (resolved, _) = p.resolve_operation(&p.operations[0]).unwrap();
    assert!(resolved.curves[0].pts[0].length() < 1e-8);
    assert!(resolved.curves[0].pts[1].distance(DVec2::new(0.0, -15.0)) < 1e-8);
    let path = generate(&p.operations[0], &p, &tools()).unwrap();
    assert_eq!(path.wcs, 55);
    let code = warp_cam::post::post(
        &warp_cam::post::PostConfig::mach3(),
        "setup",
        &[path],
        &tools(),
    );
    assert!(code.lines().any(|line| line == "G55"));
    let q = Project::from_json(&p.document_json().unwrap()).unwrap();
    assert_eq!(q.display_units, p.display_units);
    assert_eq!(key(&p), key(&q));
}

#[test]
fn projection_uses_open_drive_closed_containment_and_avoid_independently() {
    let mut p = load("projected-open-curve");
    let scene = p.scene.as_mut().unwrap();
    let boundary = scene.add(
        "containment".into(),
        Geometry::polyline(&Polyline2::rect(
            DVec2::new(4.0, 4.0),
            DVec2::new(21.0, 16.0),
        )),
    );
    let avoid = scene.add(
        "avoid".into(),
        Geometry::polyline(&Polyline2::rect(
            DVec2::new(10.0, 4.0),
            DVec2::new(12.0, 6.0),
        )),
    );
    let g = p.operations[0].common.geometry.as_mut().unwrap();
    assert!(g.review.is_some());
    g.review = None;
    g.containment = Selection {
        mode: SelectionMode::Selected,
        references: vec![GeometryRef::object(boundary)],
    };
    g.avoid = Selection {
        mode: SelectionMode::Selected,
        references: vec![GeometryRef::object(avoid)],
    };
    let path = generate(&p.operations[0], &p, &tools()).unwrap();
    let mut count = 0;
    for (a, b, rapid, _) in path.segments(DVec3::new(0.0, 0.0, 10.0), 0.01) {
        if rapid || a.z > 0.0 || b.z > 0.0 {
            continue;
        }
        for i in 0..=30 {
            let q = a.lerp(b, i as f64 / 30.0);
            count += 1;
            assert!(
                q.x >= 4.0 - 1e-6 && q.x <= 21.0 + 1e-6 && q.y >= 4.0 - 1e-6 && q.y <= 16.0 + 1e-6
            );
            assert!(
                !(q.x > 9.01 && q.x < 12.99 && (q.y - 5.0).abs() < 0.1),
                "crossed avoid at {q}"
            );
        }
    }
    assert!(count > 20);
}

#[test]
fn invalid_schema_and_duplicate_identity_do_not_fall_back() {
    for text in [
        "{}",
        "{\"schema_version\":2}",
        "{\"schema_version\":1}",
        "{\"schema_version\":1.0}",
        "{\"schema_version\":\"1\"}",
        "{",
    ] {
        assert!(Project::from_json(text).is_err(), "{text}");
    }
    let mut p = load("open-profile");
    p.operations.push(p.operations[0].clone());
    assert!(Project::from_json(&p.document_json().unwrap()).is_err());
}

#[test]
fn save_is_versioned_backed_up_and_rejects_corrupt_destination() {
    let p = load("open-profile");
    let path = std::env::temp_dir().join(format!("warpnine-m1-{}.warp", std::process::id()));
    let _ = std::fs::remove_file(&path);
    p.save(&path).unwrap();
    let first = std::fs::read_to_string(&path).unwrap();
    assert_eq!(
        serde_json::from_str::<serde_json::Value>(&first).unwrap()["schema_version"],
        1
    );
    let mut q = p.clone();
    q.name = "changed".into();
    q.save(&path).unwrap();
    assert_eq!(
        std::fs::read_to_string(path.with_extension("warp.bak")).unwrap(),
        first
    );
    std::fs::write(&path, "broken").unwrap();
    assert!(p.save(&path).is_err());
    assert_eq!(std::fs::read_to_string(&path).unwrap(), "broken");
    std::fs::remove_file(&path).unwrap();
    std::fs::remove_file(path.with_extension("warp.bak")).unwrap();
}

#[test]
fn edits_change_generation_key_but_visibility_does_not() {
    let mut p = load("open-profile");
    let original = key(&p);
    p.scene.as_mut().unwrap().objects[0].visible = false;
    assert_eq!(original, key(&p));
    p.operations[0].common.stepdown = 0.5;
    assert_ne!(original, key(&p));
    let mut tools = tools();
    let before = input_key(&p.operations[0], &p, &tools).unwrap();
    tools.tools[0].diameter = 4.0;
    assert_ne!(before, input_key(&p.operations[0], &p, &tools).unwrap());
}

#[test]
fn undo_redo_restores_selection_and_setup_intent() {
    let mut p = load("open-profile");
    let mut h = History::default();
    let before = p.document_json().unwrap();
    let original = key(&p);
    p.operations[0]
        .common
        .geometry
        .as_mut()
        .unwrap()
        .regions
        .references[0]
        .reversed = true;
    p.setups[0].origin.x = 5.0;
    h.record(before.clone());
    let edited = p.document_json().unwrap();
    let undone = h.undo(edited.clone()).unwrap();
    assert_eq!(key(&Project::from_json(&undone).unwrap()), original);
    assert_eq!(h.redo(undone).unwrap(), edited);
}

#[test]
fn bounded_worker_cancellation_and_edit_tokens_are_observable() {
    let mut p = load("open-profile");
    let queue = JobQueue::default();
    let captured = key(&p);
    let id = p.operations[0].common.id.clone();
    queue
        .submit(Job {
            id: id.clone(),
            epoch: 7,
            key: captured.clone(),
            op: p.operations[0].clone(),
            project: p.clone(),
            tools: tools(),
            cancelled: Arc::new(AtomicBool::new(true)),
        })
        .unwrap();
    let result = queue.rx.recv_timeout(Duration::from_secs(10)).unwrap();
    assert_eq!(result.id, id);
    assert_eq!(result.epoch, 7);
    assert_eq!(result.result.unwrap_err(), "Cancelled");
    p.operations[0].kind = OpKind::Engrave;
    assert_ne!(
        Some(captured),
        input_key(&p.operations[0], &p, &tools()).ok()
    );
}

#[test]
fn nesting_job_persists_and_is_validated_against_the_scene() {
    use warp_nest::*;
    let mut p = load("pocket-island");
    let scene = p.scene.as_mut().unwrap();
    let outline = scene.objects[0].id.clone();
    let outline_ref = GeometryRef::object(outline.clone());
    let s = candidates::loose_suggestion(scene, &[outline_ref.clone()]).unwrap();
    assert_eq!(s.outline, Some(outline_ref.clone()));
    let mut job = NestJob::default();
    let id = PartId(job.fresh_id("part"));
    let mut part = candidates::part_from_loose(id, "Plate", vec![outline_ref.clone()], &s);
    part.quantity = 4;
    job.parts.push(part);
    let sheet_id = SheetId(job.fresh_id("sheet"));
    job.sheets.push(Sheet::new(
        sheet_id,
        "Sheet",
        DVec2::new(600.0, 400.0),
        12.0,
    ));
    job.placements.push(Placement {
        instance: job.instances()[1].id.clone(),
        sheet: job.sheets[0].id.clone(),
        sheet_copy: 0,
        position: DVec2::new(10.0, 20.0),
        rotation_deg: 90.0,
        mirrored: false,
        locked: true,
    });
    p.nesting = Some(job.clone());
    let text = p.document_json().unwrap();
    let restored = Project::from_json(&text).unwrap();
    assert_eq!(restored.nesting.as_ref(), Some(&job));
    assert_eq!(
        restored.nesting.as_ref().unwrap().report().unplaced.len(),
        3
    );
    // Projects without nesting keep their JSON shape.
    assert!(!load("pocket-island")
        .document_json()
        .unwrap()
        .contains("\"nesting\""));
    // Deleting the referenced geometry makes the project unreadable until repaired.
    let mut broken = restored.clone();
    broken
        .scene
        .as_mut()
        .unwrap()
        .objects
        .retain(|o| o.id != outline);
    broken.operations.clear();
    let err = Project::from_json(&broken.document_json().unwrap()).unwrap_err();
    assert!(format!("{err:#}").contains("no longer exists"), "{err:#}");
}

/// Axis-aligned box with outward winding (independent of warp-core's private test helper).
fn boxed(min: DVec3, max: DVec3) -> warp_core::Mesh {
    let p = |x: f64, y: f64, z: f64| {
        DVec3::new(
            if x > 0.0 { max.x } else { min.x },
            if y > 0.0 { max.y } else { min.y },
            if z > 0.0 { max.z } else { min.z },
        )
    };
    let quad = |a: DVec3, b: DVec3, c: DVec3, d: DVec3| [[a, b, c], [a, c, d]];
    let mut tris = vec![];
    tris.extend(quad(p(0., 0., 0.), p(0., 1., 0.), p(1., 1., 0.), p(1., 0., 0.)));
    tris.extend(quad(p(0., 0., 1.), p(1., 0., 1.), p(1., 1., 1.), p(0., 1., 1.)));
    tris.extend(quad(p(0., 0., 0.), p(1., 0., 0.), p(1., 0., 1.), p(0., 0., 1.)));
    tris.extend(quad(p(1., 0., 0.), p(1., 1., 0.), p(1., 1., 1.), p(1., 0., 1.)));
    tris.extend(quad(p(1., 1., 0.), p(0., 1., 0.), p(0., 1., 1.), p(1., 1., 1.)));
    tris.extend(quad(p(0., 1., 0.), p(0., 0., 0.), p(0., 0., 1.), p(0., 1., 1.)));
    warp_core::Mesh {
        tris,
        provenance: None,
    }
}

#[test]
fn qualified_meshes_represent_solids_and_mirrored_instances_stay_outward() {
    use warp_core::mesh::{MeasuredDeviation, MeshProvenance};
    let mut p = load("sculpted-parallel");
    p.operations[0].common.tolerance = 0.01;
    let mut mesh = boxed(DVec3::ZERO, DVec3::splat(4.0));
    mesh.provenance = Some(MeshProvenance {
        source: "solid".into(),
        source_type: "brep".into(),
        requested_tolerance_mm: Some(0.05),
        exporter: None,
        revision: None,
        face_ranges: vec![],
        measured: Some(MeasuredDeviation {
            source_to_mesh_mm: 0.02,
            mesh_to_source_mm: 0.015,
            source_samples: 10,
            mesh_samples: 10,
            outside_trim_samples: 0,
            per_face_mm: vec![],
        }),
        note: None,
    });
    let scene = p.scene.as_mut().unwrap();
    let def = DefinitionId("solids".into());
    let object = |id: &str, geometry: Geometry| SceneObject {
        id: ObjectId(id.into()),
        name: id.into(),
        layer: LayerId("default".into()),
        visible: true,
        source: None,
        geometry,
    };
    scene.definitions.push(Definition {
        id: def.clone(),
        name: "Solids".into(),
        objects: vec![
            object("mesh", Geometry::Mesh(mesh)),
            object(
                "solid",
                Geometry::Unavailable {
                    reason: "represented".into(),
                    represented_by: Some(ObjectId("mesh".into())),
                },
            ),
        ],
    });
    // Translate, then mirror X: the box lands at x in [-14, -10].
    let inst = scene.add(
        "Mirrored".into(),
        Geometry::Instance {
            definition: def,
            transform: DMat4::from_scale(DVec3::new(-1.0, 1.0, 1.0))
                * DMat4::from_translation(DVec3::new(10.0, 10.0, -10.0)),
        },
    );
    let solid = GeometryRef {
        object: ObjectId("solid".into()),
        instances: vec![InstanceId(inst.0.clone())],
        reversed: false,
        start_vertex: 0,
    };
    p.operations[0]
        .common
        .geometry
        .as_mut()
        .unwrap()
        .part_surfaces = Selection {
        mode: SelectionMode::Selected,
        references: vec![solid],
    };
    let err = p
        .resolve_operation(&p.operations[0])
        .unwrap_err()
        .to_string();
    assert!(
        err.contains("0.0200 mm exceeds the operation tolerance 0.0100"),
        "{err}"
    );
    p.operations[0].common.tolerance = 0.05;
    let (resolved, _) = p.resolve_operation(&p.operations[0]).unwrap();
    let mesh = resolved.mesh.unwrap();
    assert_eq!(mesh.tris.len(), 12);
    assert!(
        (mesh.signed_volume() - 64.0).abs() < 1e-9,
        "mirrored instance must keep outward winding: {}",
        mesh.signed_volume()
    );
    assert!(mesh.analyze(1e-6).oriented());
    let b = mesh.bbox();
    assert!((b.min.x + 14.0).abs() < 1e-9 && (b.max.x + 10.0).abs() < 1e-9);
    // AllEligible: a represented solid is not an incomplete import; an unrepresented one is.
    p.operations[0]
        .common
        .geometry
        .as_mut()
        .unwrap()
        .part_surfaces = Selection::all();
    assert!(p.resolve_operation(&p.operations[0]).is_ok());
    p.scene.as_mut().unwrap().add(
        "Raw solid".into(),
        Geometry::Unavailable {
            reason: "no mesh".into(),
            represented_by: None,
        },
    );
    let err = p
        .resolve_operation(&p.operations[0])
        .unwrap_err()
        .to_string();
    assert!(err.contains("unavailable"), "{err}");
}
