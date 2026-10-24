//! N02: footprints, clearances, exclusions and exact collision after broad-phase bounds.
use glam::{DMat4, DVec2, DVec3};
use warp_core::scene::*;
use warp_core::selection::GeometryRef;
use warp_core::Polyline2;
use warp_nest::feasibility::*;
use warp_nest::footprint::*;
use warp_nest::*;

fn curve(id: &str, c: &Polyline2) -> SceneObject {
    SceneObject {
        id: ObjectId(id.into()),
        name: id.into(),
        layer: LayerId("default".into()),
        visible: true,
        source: None,
        geometry: Geometry::polyline(c),
    }
}
fn r(id: &str) -> GeometryRef {
    GeometryRef::object(ObjectId(id.into()))
}
/// L-shaped 60x60 part with a 40x40 notch at the top right, plus a hole; the block is
/// defined 100 mm away from the origin to prove the local frame is bbox-relative.
fn l_shape() -> Polyline2 {
    Polyline2::new(
        [
            (0., 0.),
            (60., 0.),
            (60., 20.),
            (20., 20.),
            (20., 60.),
            (0., 60.),
        ]
        .iter()
        .map(|(x, y)| DVec2::new(x + 100.0, y + 100.0))
        .collect(),
        true,
    )
}
fn scene() -> Scene {
    let mut s = Scene::default();
    s.definitions.push(Definition {
        id: DefinitionId("def/L".into()),
        name: "L".into(),
        objects: vec![
            curve("L/outline", &l_shape()),
            curve(
                "L/hole",
                &Polyline2::circle(DVec2::new(110.0, 110.0), 4.0, 0.01),
            ),
        ],
    });
    s.objects.push(curve(
        "plate",
        &Polyline2::rect(DVec2::new(300.0, 50.0), DVec2::new(350.0, 80.0)),
    ));
    s.objects.push(SceneObject {
        id: ObjectId("tilted".into()),
        name: "Tilted".into(),
        layer: LayerId("default".into()),
        visible: true,
        source: None,
        geometry: Geometry::Curve {
            points: vec![
                DVec3::new(0.0, 0.0, 0.0),
                DVec3::new(30.0, 0.0, 0.0),
                DVec3::new(30.0, 10.0, 10.0),
                DVec3::new(0.0, 10.0, 10.0),
            ],
            closed: true,
            analytic: None,
        },
    });
    s.objects.push(SceneObject {
        id: ObjectId("inst".into()),
        name: "inst".into(),
        layer: LayerId("default".into()),
        visible: true,
        source: None,
        geometry: Geometry::Instance {
            definition: DefinitionId("def/L".into()),
            transform: DMat4::from_translation(DVec3::new(500.0, 500.0, 0.0)),
        },
    });
    s.validate().unwrap();
    s
}
fn l_part(id: &str) -> PartDefinition {
    let mut p = PartDefinition::new(
        PartId(id.into()),
        "L",
        PartSource::Block {
            definition: DefinitionId("def/L".into()),
        },
    );
    p.outline = Some(r("L/outline"));
    p.holes = vec![r("L/hole")];
    p.rotations = Rotations::Step { step_deg: 90.0 };
    p
}
fn plate_part(id: &str) -> PartDefinition {
    let mut p = PartDefinition::new(
        PartId(id.into()),
        "Plate",
        PartSource::Loose {
            members: vec![r("plate")],
        },
    );
    p.outline = Some(r("plate"));
    p
}
fn place(inst: &str, x: f64, y: f64, rot: f64) -> Placement {
    Placement {
        instance: PartInstanceId(inst.into()),
        sheet: SheetId("sheet".into()),
        sheet_copy: 0,
        position: DVec2::new(x, y),
        rotation_deg: rot,
        mirrored: false,
        locked: false,
    }
}
fn job() -> NestJob {
    let mut j = NestJob::default();
    let mut l = l_part("L");
    l.quantity = 3;
    j.parts.push(l);
    let mut plate = plate_part("plate");
    plate.quantity = 2;
    j.parts.push(plate);
    let mut sheet = Sheet::new(
        SheetId("sheet".into()),
        "Sheet",
        DVec2::new(300.0, 200.0),
        12.0,
    );
    sheet.edge_margin = 10.0;
    sheet.available = 2;
    j.sheets.push(sheet);
    j.constraints.part_spacing = 6.0;
    j
}

#[test]
fn footprints_are_bbox_relative_and_oriented() {
    let s = scene();
    let f = footprint(&l_part("L"), &s).unwrap();
    assert_eq!(f.bbox().min, DVec2::ZERO);
    assert_eq!(f.bbox().max, DVec2::new(60.0, 60.0));
    assert!(f.outline.is_ccw());
    assert_eq!(f.source_offset, DVec2::new(-100.0, -100.0));
    assert_eq!(f.holes.len(), 1);
    assert!(!f.holes[0].is_ccw());
    let hole_center = f.holes[0].pts.iter().sum::<DVec2>() / f.holes[0].pts.len() as f64;
    assert!(hole_center.distance(DVec2::new(10.0, 10.0)) < 1e-6);
    // 60*60 - 40*40 - pi*16
    assert!((f.area() - (3600.0 - 1600.0 - std::f64::consts::PI * 16.0)).abs() < 0.5);

    // A loose part through an instance path uses the instance transform.
    let member = GeometryRef {
        object: ObjectId("L/outline".into()),
        instances: vec![InstanceId("inst".into())],
        reversed: false,
        start_vertex: 0,
    };
    let mut via = PartDefinition::new(
        PartId("via".into()),
        "via",
        PartSource::Loose {
            members: vec![member.clone()],
        },
    );
    via.outline = Some(member);
    let g = footprint(&via, &s).unwrap();
    assert_eq!(g.outline.pts, f.outline.pts);
    assert_eq!(g.source_offset, DVec2::new(-600.0, -600.0));
}

#[test]
fn nonplanar_parts_need_an_approved_projection() {
    let s = scene();
    let mut p = PartDefinition::new(
        PartId("t".into()),
        "Tilted",
        PartSource::Loose {
            members: vec![r("tilted")],
        },
    );
    p.outline = Some(r("tilted"));
    let e = footprint(&p, &s).unwrap_err().to_string();
    assert!(e.contains("not planar"), "{e}");
    p.projection = Some(ProjectionPlane::Xy);
    let f = footprint(&p, &s).unwrap();
    assert_eq!(f.bbox().size(), DVec2::new(30.0, 10.0));
    p.projection = Some(ProjectionPlane::Xz);
    let f = footprint(&p, &s).unwrap();
    assert_eq!(f.bbox().size(), DVec2::new(30.0, 10.0));
    p.projection = Some(ProjectionPlane::Yz);
    // Y and Z rise together: a degenerate line in the YZ plane is rejected.
    assert!(footprint(&p, &s).is_err());
}

#[test]
fn placement_transform_mirrors_then_rotates() {
    let f = footprint(&l_part("L"), &scene()).unwrap();
    let mut pl = place("L/0", 100.0, 50.0, 90.0);
    // Local (60,0) rotated 90° CCW -> (0,60), plus position.
    assert!(place_point(DVec2::new(60.0, 0.0), &pl).distance(DVec2::new(100.0, 110.0)) < 1e-9);
    pl.mirrored = true;
    // Mirror first: (60,0) -> (-60,0); rotate 90° -> (0,-60).
    assert!(place_point(DVec2::new(60.0, 0.0), &pl).distance(DVec2::new(100.0, -10.0)) < 1e-9);
    let (outline, holes) = placed(&f, &pl);
    assert!(outline.is_ccw() && !holes[0].is_ccw());
    // Same mapping through the source-to-sheet matrix for the original source point.
    let m = source_to_sheet(&f, &pl);
    let p = m.transform_point3(DVec3::new(160.0, 100.0, 0.0));
    assert!(p.truncate().distance(DVec2::new(100.0, -10.0)) < 1e-9 && p.z.abs() < 1e-9);
}

#[test]
fn spacing_is_enforced_exactly_and_parts_may_share_sheets_but_not_space() {
    let s = scene();
    let mut j = job();
    // Two 50x30 plates side by side with exactly 6 mm gap: feasible.
    j.placements = vec![
        place("plate/0", 10.0, 10.0, 0.0),
        place("plate/1", 66.0, 10.0, 0.0),
    ];
    let v = check(&j, &s).unwrap();
    assert!(v.is_empty(), "{v:?}");
    // 5.9 mm gap: overlap violation with the required gap reported.
    j.placements[1].position.x = 65.9;
    let v = check(&j, &s).unwrap();
    assert!(
        matches!(&v[..], [Violation::Overlap { required_gap, .. }] if (*required_gap - 6.0).abs() < 1e-9),
        "{v:?}"
    );
    // Same positions on another copy of the same sheet kind: no collision.
    j.placements[1].sheet_copy = 1;
    assert!(check(&j, &s).unwrap().is_empty());
    // Extra clearance on the plate part applies to both plates: 6 + 2 + 2 = 10 mm gap,
    // and each inflated outline must still clear the 10 mm margin.
    j.placements[1].sheet_copy = 0;
    j.parts[1].extra_clearance = 2.0;
    j.placements[0].position = DVec2::new(12.0, 12.0);
    j.placements[1].position = DVec2::new(72.0, 12.0);
    let v = check(&j, &s).unwrap();
    assert!(v.is_empty(), "{v:?}");
    j.placements[1].position.x = 71.9;
    let v = check(&j, &s).unwrap();
    assert!(
        matches!(&v[..], [Violation::Overlap { required_gap, .. }] if (*required_gap - 10.0).abs() < 1e-9),
        "{v:?}"
    );
    j.placements[1].position.x = 72.0;
    j.placements[0].position.y = 11.9;
    let v = check(&j, &s).unwrap();
    assert!(matches!(&v[..], [Violation::OutsideSheet { .. }]), "{v:?}");
}

#[test]
fn margins_and_exclusions_use_the_inflated_outline() {
    let s = scene();
    let mut j = job();
    j.placements = vec![place("plate/0", 9.0, 10.0, 0.0)];
    let v = check(&j, &s).unwrap();
    assert!(matches!(&v[..], [Violation::OutsideSheet { .. }]), "{v:?}");
    j.placements[0].position.x = 10.0;
    assert!(check(&j, &s).unwrap().is_empty());
    // Extra clearance pushes the inflated outline into the margin.
    j.parts[1].extra_clearance = 1.0;
    assert!(matches!(
        &check(&j, &s).unwrap()[..],
        [Violation::OutsideSheet { .. }]
    ));
    j.parts[1].extra_clearance = 0.0;
    // Rotated 90° about the local origin the plate extends to negative X: outside.
    j.placements[0].rotation_deg = 90.0;
    assert!(matches!(
        &check(&j, &s).unwrap()[..],
        [Violation::OutsideSheet { .. }]
    ));
    j.placements[0].position = DVec2::new(40.0, 10.0);
    assert!(check(&j, &s).unwrap().is_empty());
    // A clamp exclusion touching the plate edge is fine; overlapping it is not.
    j.placements[0] = place("plate/0", 10.0, 10.0, 0.0);
    j.sheets[0].exclusions.push(Exclusion {
        name: "Clamp".into(),
        min: DVec2::new(60.0, 0.0),
        max: DVec2::new(80.0, 30.0),
    });
    assert!(check(&j, &s).unwrap().is_empty());
    j.sheets[0].exclusions[0].min.x = 59.0;
    let v = check(&j, &s).unwrap();
    assert!(
        matches!(&v[..], [Violation::Exclusion { exclusion, .. }] if exclusion == "Clamp"),
        "{v:?}"
    );
}

#[test]
fn concave_parts_pass_when_only_bounding_boxes_overlap() {
    let s = scene();
    let mut j = job();
    // L occupies [0,60]x[0,60] minus the notch [20,60]x[20,60]. A plate (50x30) placed in
    // the notch at (26,26) is 6 mm from both arms: bounding boxes overlap, polygons do not.
    j.placements = vec![
        place("L/0", 10.0, 10.0, 0.0),
        place("plate/0", 36.0, 36.0, 0.0),
    ];
    let v = check(&j, &s).unwrap();
    assert!(v.is_empty(), "{v:?}");
    // Move the plate 1 mm closer to the vertical arm: real overlap of the clearance zones.
    j.placements[1].position.x = 35.0;
    assert!(matches!(
        &check(&j, &s).unwrap()[..],
        [Violation::Overlap { .. }]
    ));
    // Interlocking Ls. L/0 spans [10,70]^2 with arm tips at (30,70) and (70,30). A second
    // L rotated 180° about its origin and placed at (p,p) has arm tips at (p-60, p-20)
    // and (p-20, p-60). The closest approach is tip to tip: sqrt(2) * (p - 90). Round
    // clearance joins make that exact. p = 94.3 gives 6.08 mm: feasible, although the
    // bounding boxes overlap by more than 50 mm in each axis.
    j.placements = vec![
        place("L/0", 10.0, 10.0, 0.0),
        place("L/1", 94.3, 94.3, 180.0),
    ];
    let v = check(&j, &s).unwrap();
    assert!(v.is_empty(), "{v:?}");
    // p = 94.1 gives 5.80 mm: the clearance zones meet.
    j.placements[1].position = DVec2::new(94.1, 94.1);
    assert!(matches!(
        &check(&j, &s).unwrap()[..],
        [Violation::Overlap { .. }]
    ));
}

#[test]
fn missing_footprints_are_reported_not_skipped() {
    let s = scene();
    let mut j = job();
    j.parts[0].outline = None;
    j.placements = vec![place("L/0", 10.0, 10.0, 0.0), place("L/1", 10.0, 10.0, 0.0)];
    let v = check(&j, &s).unwrap();
    assert_eq!(
        v.iter()
            .filter(|v| matches!(v, Violation::NoFootprint { .. }))
            .count(),
        1,
        "{v:?}"
    );
    assert!(
        v.iter().all(|v| matches!(v, Violation::NoFootprint { .. })),
        "{v:?}"
    );
}

#[test]
fn oversize_detection_respects_rotation_and_margin() {
    let s = scene();
    let j = job();
    let f = footprint(&j.parts[1], &s).unwrap();
    let mut sheet = Sheet::new(SheetId("s".into()), "s", DVec2::new(40.0, 60.0), 12.0);
    sheet.edge_margin = 2.0;
    // 50x30 only fits the 36x56 usable area when rotated 90°.
    assert!(!fits_sheet(&j.parts[1], &f, &sheet, &[0.0]));
    assert!(fits_sheet(&j.parts[1], &f, &sheet, &[0.0, 90.0]));
    sheet.edge_margin = 6.0;
    assert!(!fits_sheet(&j.parts[1], &f, &sheet, &[0.0, 90.0]));
}
