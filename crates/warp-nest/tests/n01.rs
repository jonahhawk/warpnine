//! N01: parts, instances, sheets, constraints and placements persist and validate.
use glam::{DMat4, DVec2, DVec3};
use warp_core::scene::*;
use warp_core::selection::GeometryRef;
use warp_core::Polyline2;
use warp_nest::candidates::*;
use warp_nest::*;

fn curve(id: &str, name: &str, c: &Polyline2) -> SceneObject {
    SceneObject {
        id: ObjectId(id.into()),
        name: name.into(),
        layer: LayerId("default".into()),
        visible: true,
        source: None,
        geometry: Geometry::polyline(c),
    }
}
fn instance(id: &str, def: &str, at: DVec3) -> SceneObject {
    SceneObject {
        id: ObjectId(id.into()),
        name: id.into(),
        layer: LayerId("default".into()),
        visible: true,
        source: None,
        geometry: Geometry::Instance {
            definition: DefinitionId(def.into()),
            transform: DMat4::from_translation(at),
        },
    }
}
fn r(id: &str) -> GeometryRef {
    GeometryRef::object(ObjectId(id.into()))
}

/// Bracket block: 50x30 outline, a 6 mm hole, an engraving line, placed twice plus once
/// through a nested assembly block.
fn bracket_scene() -> Scene {
    let mut scene = Scene::default();
    scene.definitions.push(Definition {
        id: DefinitionId("def/bracket".into()),
        name: "Bracket".into(),
        objects: vec![
            curve(
                "b/outline",
                "Outline",
                &Polyline2::rect(DVec2::ZERO, DVec2::new(50.0, 30.0)),
            ),
            curve(
                "b/hole",
                "Hole",
                &Polyline2::circle(DVec2::new(10.0, 15.0), 3.0, 0.01),
            ),
            curve(
                "b/engrave",
                "Label",
                &Polyline2::new(vec![DVec2::new(20.0, 10.0), DVec2::new(40.0, 10.0)], false),
            ),
        ],
    });
    scene.definitions.push(Definition {
        id: DefinitionId("def/assembly".into()),
        name: "Assembly".into(),
        objects: vec![instance(
            "asm/bracket",
            "def/bracket",
            DVec3::new(200.0, 0.0, 0.0),
        )],
    });
    scene
        .objects
        .push(instance("i1", "def/bracket", DVec3::ZERO));
    scene
        .objects
        .push(instance("i2", "def/bracket", DVec3::new(100.0, 0.0, 0.0)));
    scene
        .objects
        .push(instance("i3", "def/assembly", DVec3::new(0.0, 100.0, 0.0)));
    scene.validate().unwrap();
    scene
}

#[test]
fn block_candidate_suggests_outline_hole_and_feature() {
    let scene = bracket_scene();
    let c = block_candidates(&scene).unwrap();
    let bracket = c.iter().find(|c| c.name == "Bracket").unwrap();
    assert_eq!(
        bracket.instance_count, 3,
        "two direct plus one nested instance"
    );
    assert_eq!(bracket.suggestion.outline, Some(r("b/outline")));
    assert_eq!(bracket.suggestion.holes, vec![r("b/hole")]);
    assert_eq!(bracket.suggestion.features, vec![r("b/engrave")]);
    assert!(bracket.suggestion.ambiguities.is_empty());
    // The assembly contains only a nested instance: no loop, explicit ambiguity.
    let asm = c.iter().find(|c| c.name == "Assembly").unwrap();
    assert_eq!(asm.instance_count, 1);
    assert!(asm.suggestion.outline.is_none());
    assert!(asm
        .suggestion
        .ambiguities
        .iter()
        .any(|a| a.contains("nested block")));

    let part = part_from_block(PartId("part/1".into()), bracket);
    assert_eq!(part.quantity, 3);
    assert!(matches!(part.source, PartSource::Block { .. }));
}

#[test]
fn loose_group_reports_disconnected_outlines_and_open_curves() {
    let mut scene = Scene::default();
    scene.objects.push(curve(
        "a",
        "A",
        &Polyline2::rect(DVec2::ZERO, DVec2::splat(20.0)),
    ));
    scene.objects.push(curve(
        "b",
        "B",
        &Polyline2::rect(DVec2::splat(30.0), DVec2::splat(50.0)),
    ));
    scene.objects.push(curve(
        "open",
        "Open",
        &Polyline2::new(vec![DVec2::ZERO, DVec2::new(5.0, 5.0)], false),
    ));
    let s = loose_suggestion(&scene, &[r("a"), r("b")]).unwrap();
    assert!(s.outline.is_none());
    assert!(s
        .ambiguities
        .iter()
        .any(|a| a.contains("2 disconnected outer loops")));
    assert_eq!(
        s.features.len(),
        2,
        "both loops are kept, neither is promoted"
    );

    let s = loose_suggestion(&scene, &[r("open")]).unwrap();
    assert!(s.outline.is_none());
    assert!(s
        .ambiguities
        .iter()
        .any(|a| a.contains("never used as an outline")));

    // Alone, A is an unambiguous outline.
    let s = loose_suggestion(&scene, &[r("a"), r("open")]).unwrap();
    assert_eq!(s.outline, Some(r("a")));
    assert_eq!(s.features, vec![r("open")]);
    assert!(s.ambiguities.is_empty());
}

#[test]
fn nested_island_inside_hole_is_a_feature_not_a_hole() {
    let mut scene = Scene::default();
    scene.objects.push(curve(
        "outer",
        "Outer",
        &Polyline2::rect(DVec2::ZERO, DVec2::splat(100.0)),
    ));
    scene.objects.push(curve(
        "hole",
        "Hole",
        &Polyline2::rect(DVec2::splat(20.0), DVec2::splat(80.0)),
    ));
    scene.objects.push(curve(
        "island",
        "Island",
        &Polyline2::rect(DVec2::splat(40.0), DVec2::splat(60.0)),
    ));
    let s = loose_suggestion(&scene, &[r("outer"), r("hole"), r("island")]).unwrap();
    assert_eq!(s.outline, Some(r("outer")));
    assert_eq!(s.holes, vec![r("hole")]);
    assert_eq!(s.features, vec![r("island")]);
    assert!(s.ambiguities.is_empty());
}

#[test]
fn crossing_loops_and_nonplanar_curves_are_ambiguous() {
    let mut scene = Scene::default();
    scene.objects.push(curve(
        "a",
        "A",
        &Polyline2::rect(DVec2::ZERO, DVec2::splat(20.0)),
    ));
    scene.objects.push(curve(
        "b",
        "B",
        &Polyline2::rect(DVec2::splat(10.0), DVec2::splat(30.0)),
    ));
    scene.objects.push(SceneObject {
        id: ObjectId("tilt".into()),
        name: "Tilted".into(),
        layer: LayerId("default".into()),
        visible: true,
        source: None,
        geometry: Geometry::Curve {
            points: vec![
                DVec3::ZERO,
                DVec3::new(10.0, 0.0, 0.0),
                DVec3::new(10.0, 10.0, 5.0),
            ],
            closed: true,
            analytic: None,
        },
    });
    let s = loose_suggestion(&scene, &[r("a"), r("b")]).unwrap();
    assert!(s.ambiguities.iter().any(|a| a.contains("cross or touch")));
    let s = loose_suggestion(&scene, &[r("tilt")]).unwrap();
    assert!(s.ambiguities.iter().any(|a| a.contains("not planar")));
    assert!(s.outline.is_none());
}

#[test]
fn loose_suggestion_uses_instance_transforms() {
    // A block instance translated by 100 mm still yields a planar outline for its curves.
    let scene = bracket_scene();
    let member = GeometryRef {
        object: ObjectId("b/outline".into()),
        instances: vec![InstanceId("i2".into())],
        reversed: false,
        start_vertex: 0,
    };
    let hole = GeometryRef {
        object: ObjectId("b/hole".into()),
        instances: vec![InstanceId("i2".into())],
        reversed: false,
        start_vertex: 0,
    };
    let s = loose_suggestion(&scene, &[member.clone(), hole.clone()]).unwrap();
    assert_eq!(s.outline, Some(member));
    assert_eq!(s.holes, vec![hole]);
    assert!(loose_suggestion(&scene, &[r("missing")]).is_err());
}

fn job() -> NestJob {
    let mut job = NestJob::default();
    let mut part = PartDefinition::new(
        PartId("part/1".into()),
        "Bracket",
        PartSource::Block {
            definition: DefinitionId("def/bracket".into()),
        },
    );
    part.outline = Some(r("b/outline"));
    part.holes = vec![r("b/hole")];
    part.features = vec![r("b/engrave")];
    part.quantity = 3;
    part.rotations = Rotations::Fixed(vec![0.0, 180.0]);
    part.grain_deg = Some(0.0);
    job.parts.push(part);
    let mut sheet = Sheet::new(
        SheetId("sheet/1".into()),
        "Ply 1220x2440",
        DVec2::new(1220.0, 2440.0),
        18.0,
    );
    sheet.edge_margin = 10.0;
    sheet.grain_deg = Some(0.0);
    sheet.exclusions.push(Exclusion {
        name: "Clamp".into(),
        min: DVec2::ZERO,
        max: DVec2::new(50.0, 50.0),
    });
    sheet.available = 2;
    job.sheets.push(sheet);
    job
}

#[test]
fn instances_are_stable_and_report_tracks_quantity_changes() {
    let mut job = job();
    let ids: Vec<_> = job.instances().iter().map(|i| i.id.0.clone()).collect();
    assert_eq!(ids, ["part/1/0", "part/1/1", "part/1/2"]);
    job.placements.push(Placement {
        instance: PartInstanceId("part/1/2".into()),
        sheet: SheetId("sheet/1".into()),
        sheet_copy: 1,
        position: DVec2::new(100.0, 100.0),
        rotation_deg: 180.0,
        mirrored: false,
        locked: true,
    });
    let report = job.report();
    assert!(report.errors.is_empty(), "{:?}", report.errors);
    assert_eq!(
        report.unplaced,
        vec![
            PartInstanceId("part/1/0".into()),
            PartInstanceId("part/1/1".into())
        ]
    );
    assert!(report.orphaned.is_empty());

    // Raising the quantity keeps copy 2's placement; lowering it orphans it, loudly.
    job.parts[0].quantity = 5;
    assert!(job.report().orphaned.is_empty());
    job.parts[0].quantity = 2;
    assert_eq!(
        job.report().orphaned,
        vec![PartInstanceId("part/1/2".into())]
    );
    let removed = job.prune_orphans();
    assert_eq!(removed.len(), 1);
    assert!(job.placements.is_empty());
    assert!(job.report().orphaned.is_empty());
}

#[test]
fn report_flags_rotation_mirror_and_grain_violations() {
    let mut job = job();
    let place = |rot: f64, mirrored: bool| Placement {
        instance: PartInstanceId("part/1/0".into()),
        sheet: SheetId("sheet/1".into()),
        sheet_copy: 0,
        position: DVec2::ZERO,
        rotation_deg: rot,
        mirrored,
        locked: false,
    };
    job.placements = vec![place(90.0, false)];
    let e = job.report().errors;
    assert!(e.iter().any(|m| m.contains("does not allow")), "{e:?}");
    job.placements = vec![place(180.0, true)];
    let e = job.report().errors;
    assert!(e.iter().any(|m| m.contains("forbids mirroring")), "{e:?}");
    // 180° keeps the grain parallel.
    job.placements = vec![place(180.0, false)];
    assert!(job.report().errors.is_empty());
    // Allow 45°, which breaks grain alignment even though rotation is permitted.
    job.parts[0].rotations = Rotations::Step { step_deg: 45.0 };
    job.placements = vec![place(45.0, false)];
    let e = job.report().errors;
    assert!(
        e.iter()
            .any(|m| m.contains("not aligned with the sheet grain")),
        "{e:?}"
    );
    // Without a sheet grain the same placement is fine.
    job.sheets[0].grain_deg = None;
    assert!(job.report().errors.is_empty());
}

#[test]
fn validate_rejects_bad_structure() {
    let mut j = job();
    j.placements.push(Placement {
        instance: PartInstanceId("part/1/0".into()),
        sheet: SheetId("sheet/9".into()),
        sheet_copy: 0,
        position: DVec2::ZERO,
        rotation_deg: 0.0,
        mirrored: false,
        locked: false,
    });
    assert!(j
        .validate()
        .unwrap_err()
        .to_string()
        .contains("missing sheet"));
    j.placements[0].sheet = SheetId("sheet/1".into());
    j.placements[0].sheet_copy = 2;
    assert!(j
        .validate()
        .unwrap_err()
        .to_string()
        .contains("only 2 available"));
    j.placements[0].sheet_copy = 1;
    j.placements.push(j.placements[0].clone());
    assert!(j
        .validate()
        .unwrap_err()
        .to_string()
        .contains("more than once"));
    j.placements.pop();
    j.validate().unwrap();

    let mut j = job();
    j.parts[0].quantity = 0;
    assert!(j.validate().is_err());
    let mut j = job();
    j.sheets[0].edge_margin = 700.0;
    assert!(j
        .validate()
        .unwrap_err()
        .to_string()
        .contains("no usable area"));
    let mut j = job();
    j.parts[0].holes.push(r("b/outline"));
    assert!(j
        .validate()
        .unwrap_err()
        .to_string()
        .contains("outline as a hole"));
    let mut j = job();
    j.parts[0].outline = Some(GeometryRef {
        object: ObjectId("b/outline".into()),
        instances: vec![InstanceId("i1".into())],
        reversed: false,
        start_vertex: 0,
    });
    assert!(j
        .validate()
        .unwrap_err()
        .to_string()
        .contains("definition-local"));
    let mut j = job();
    j.parts.push(PartDefinition::new(
        PartId("part/2".into()),
        "Loose",
        PartSource::Loose {
            members: vec![r("x")],
        },
    ));
    j.parts[1].outline = Some(r("y"));
    assert!(j
        .validate()
        .unwrap_err()
        .to_string()
        .contains("not one of its members"));
}

#[test]
fn validate_against_scene_catches_missing_geometry() {
    let scene = bracket_scene();
    let mut j = job();
    j.validate_against(&scene).unwrap();
    j.parts[0].holes.push(r("b/missing"));
    assert!(j
        .validate_against(&scene)
        .unwrap_err()
        .to_string()
        .contains("not in block"));
    let mut j = job();
    j.parts[0].source = PartSource::Block {
        definition: DefinitionId("def/gone".into()),
    };
    assert!(j
        .validate_against(&scene)
        .unwrap_err()
        .to_string()
        .contains("missing block definition"));
    let mut j = job();
    j.parts.push(PartDefinition::new(
        PartId("part/2".into()),
        "Loose",
        PartSource::Loose {
            members: vec![r("nope")],
        },
    ));
    assert!(j
        .validate_against(&scene)
        .unwrap_err()
        .to_string()
        .contains("no longer exists"));
}

#[test]
fn job_round_trips_through_json() {
    let mut j = job();
    j.placements.push(Placement {
        instance: PartInstanceId("part/1/1".into()),
        sheet: SheetId("sheet/1".into()),
        sheet_copy: 1,
        position: DVec2::new(12.5, 7.25),
        rotation_deg: 180.0,
        mirrored: false,
        locked: true,
    });
    j.constraints.part_spacing = 4.0;
    j.constraints.seed = 42;
    let text = serde_json::to_string(&j).unwrap();
    let back: NestJob = serde_json::from_str(&text).unwrap();
    assert_eq!(back, j);
    // Fields added later are defaulted when absent.
    let minimal: NestJob = serde_json::from_str(r#"{"parts":[],"sheets":[]}"#).unwrap();
    assert_eq!(minimal, NestJob::default());
    assert_eq!(minimal.constraints.part_spacing, 5.0);
}

#[test]
fn rotation_sets() {
    assert!(Rotations::Step { step_deg: 90.0 }.allows(270.0));
    assert!(Rotations::Step { step_deg: 90.0 }.allows(-90.0));
    assert!(!Rotations::Step { step_deg: 90.0 }.allows(45.0));
    assert!(Rotations::Step { step_deg: 0.0 }.allows(360.0));
    assert!(!Rotations::Step { step_deg: 0.0 }.allows(10.0));
    assert!(Rotations::Fixed(vec![30.0]).allows(390.0));
    assert!(!Rotations::Fixed(vec![30.0]).allows(0.0));
    assert!(Rotations::Fixed(vec![]).validate().is_err());
}
