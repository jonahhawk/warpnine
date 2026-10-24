//! N03: deterministic bottom-left solver with rotations, grain, locks, sheets and reasons.
use glam::DVec2;
use std::cell::Cell;
use warp_core::scene::*;
use warp_core::selection::GeometryRef;
use warp_core::Polyline2;
use warp_nest::feasibility::{check_placements, footprints};
use warp_nest::solver::*;
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
fn scene() -> Scene {
    let mut s = Scene::default();
    s.objects.push(curve(
        "plate",
        &Polyline2::rect(DVec2::new(300.0, 50.0), DVec2::new(350.0, 80.0)),
    ));
    s.objects.push(curve(
        "square",
        &Polyline2::rect(DVec2::ZERO, DVec2::splat(20.0)),
    ));
    s.validate().unwrap();
    s
}
fn part(id: &str, member: &str, quantity: u32) -> PartDefinition {
    let mut p = PartDefinition::new(
        PartId(id.into()),
        id,
        PartSource::Loose {
            members: vec![r(member)],
        },
    );
    p.outline = Some(r(member));
    p.quantity = quantity;
    p
}
/// 200x100 sheet, 10 mm margin, 5 mm spacing: usable 180x80 fits six 50x30 plates
/// as three columns (50+5+50+5+50 = 160) by two rows (30+5+30 = 65).
fn job(plates: u32) -> NestJob {
    let mut j = NestJob::default();
    j.parts.push(part("plate", "plate", plates));
    let mut sheet = Sheet::new(
        SheetId("sheet".into()),
        "Sheet",
        DVec2::new(200.0, 100.0),
        12.0,
    );
    sheet.edge_margin = 10.0;
    j.sheets.push(sheet);
    j.constraints.part_spacing = 5.0;
    j
}
fn solve_job(j: &NestJob) -> Layout {
    let s = scene();
    let (fps, errors) = footprints(j, &s);
    assert!(errors.is_empty(), "{errors:?}");
    let layout = solve(j, &fps, &|| false);
    let mut applied = j.clone();
    apply(&mut applied, &layout);
    let v = check_placements(&applied, &fps);
    assert!(v.is_empty(), "solver produced violations: {v:?}");
    layout
}
fn pos(layout: &Layout, inst: &str) -> DVec2 {
    layout
        .placements
        .iter()
        .find(|p| p.instance.0 == inst)
        .unwrap_or_else(|| panic!("{inst} not placed"))
        .position
}

#[test]
fn bottom_left_fills_rows_deterministically() {
    let j = job(6);
    let a = solve_job(&j);
    assert!(a.unplaced.is_empty(), "{:?}", a.unplaced);
    assert_eq!(a.score.sheets_used, 1);
    assert!((a.score.utilization - 9000.0 / 14400.0).abs() < 1e-9);
    let expected = [
        (10.0, 10.0),
        (65.0, 10.0),
        (120.0, 10.0),
        (10.0, 45.0),
        (65.0, 45.0),
        (120.0, 45.0),
    ];
    for (i, (x, y)) in expected.iter().enumerate() {
        let p = pos(&a, &format!("plate/{i}"));
        assert!(p.distance(DVec2::new(*x, *y)) < 1e-9, "plate/{i} at {p}");
    }
    // Free strips: 15 mm across the full width above the rows (parts end at y = 75,
    // usable top is 90) beats 20 mm down the right side (20 x 80).
    assert!((a.score.remnant - (15.0 * 180.0) / (180.0 * 80.0)).abs() < 1e-9);
    assert_eq!(solve_job(&j), a, "solver is not deterministic");
}

#[test]
fn overflow_uses_the_next_sheet_copy_or_reports_no_room() {
    let mut j = job(7);
    let a = solve_job(&j);
    assert_eq!(a.unplaced.len(), 1);
    assert!(a.unplaced[0].reason.contains("no room"), "{:?}", a.unplaced);
    assert_eq!(a.unplaced[0].instance.0, "plate/6");
    j.sheets[0].available = 2;
    let b = solve_job(&j);
    assert!(b.unplaced.is_empty());
    assert_eq!(b.score.sheets_used, 2);
    let seventh = b
        .placements
        .iter()
        .find(|p| p.instance.0 == "plate/6")
        .unwrap();
    assert_eq!(seventh.sheet_copy, 1);
    assert!(seventh.position.distance(DVec2::new(10.0, 10.0)) < 1e-9);
    // Utilization now counts both sheets' usable area.
    assert!((b.score.utilization - 10500.0 / 28800.0).abs() < 1e-9);
}

#[test]
fn rotation_is_used_when_needed_and_oversize_is_explained() {
    // 55 mm wide sheet with 5 mm margin: 45 mm usable, so the 50x30 plate only fits upright.
    let mut j = job(2);
    j.sheets[0].size = DVec2::new(55.0, 200.0);
    j.sheets[0].edge_margin = 5.0;
    j.parts[0].rotations = Rotations::Step { step_deg: 90.0 };
    let a = solve_job(&j);
    assert!(a.unplaced.is_empty(), "{:?}", a.unplaced);
    for p in &a.placements {
        assert!((p.rotation_deg - 90.0).abs() < 1e-9 || (p.rotation_deg - 270.0).abs() < 1e-9);
    }
    // Rotated 90° about its origin the plate spans x in [-30,0]; bbox min lands at (5,5).
    assert!(pos(&a, "plate/0").distance(DVec2::new(35.0, 5.0)) < 1e-9);
    assert!(pos(&a, "plate/1").distance(DVec2::new(35.0, 60.0)) < 1e-9);
    j.parts[0].rotations = Rotations::Fixed(vec![0.0]);
    let b = solve_job(&j);
    assert_eq!(b.unplaced.len(), 2);
    assert!(
        b.unplaced[0].reason.contains("oversize"),
        "{:?}",
        b.unplaced
    );
}

#[test]
fn grain_restricts_orientation() {
    let mut j = job(1);
    j.parts[0].grain_deg = Some(0.0);
    j.sheets[0].grain_deg = Some(90.0);
    j.parts[0].rotations = Rotations::Step { step_deg: 90.0 };
    let a = solve_job(&j);
    assert_eq!(a.placements.len(), 1);
    let rot = a.placements[0].rotation_deg;
    assert!(
        (rot - 90.0).abs() < 1e-9 || (rot - 270.0).abs() < 1e-9,
        "{rot}"
    );
    j.parts[0].rotations = Rotations::Fixed(vec![0.0, 180.0]);
    let b = solve_job(&j);
    assert!(b.placements.is_empty());
    assert!(b.unplaced[0].reason.contains("grain"), "{:?}", b.unplaced);
}

#[test]
fn locked_placements_are_kept_and_respected() {
    let mut j = job(4);
    j.placements.push(Placement {
        instance: PartInstanceId("plate/1".into()),
        sheet: SheetId("sheet".into()),
        sheet_copy: 0,
        position: DVec2::new(40.0, 20.0),
        rotation_deg: 0.0,
        mirrored: false,
        locked: true,
    });
    // An unlocked stale placement is discarded and re-solved.
    j.placements.push(Placement {
        instance: PartInstanceId("plate/2".into()),
        sheet: SheetId("sheet".into()),
        sheet_copy: 0,
        position: DVec2::new(40.0, 20.0),
        rotation_deg: 0.0,
        mirrored: false,
        locked: false,
    });
    let a = solve_job(&j);
    assert!(a.unplaced.is_empty(), "{:?}", a.unplaced);
    let locked = a
        .placements
        .iter()
        .find(|p| p.instance.0 == "plate/1")
        .unwrap();
    assert!(locked.locked && locked.position == DVec2::new(40.0, 20.0));
    // The locked plate covers x 40..90, y 20..50; the first free bottom-left spot that
    // clears it by 5 mm is to its right at (95,10).
    assert!(pos(&a, "plate/0").distance(DVec2::new(95.0, 10.0)) < 1e-9);
    assert!(pos(&a, "plate/2") != DVec2::new(40.0, 20.0));
}

#[test]
fn exclusions_shift_the_first_placement() {
    let mut j = job(1);
    j.sheets[0].exclusions.push(Exclusion {
        name: "Clamp".into(),
        min: DVec2::ZERO,
        max: DVec2::new(30.0, 30.0),
    });
    let a = solve_job(&j);
    // Right of the clamp: x = 30 + tolerance, y at the margin.
    let p = pos(&a, "plate/0");
    assert!(
        (p.x - 30.01).abs() < 1e-9 && (p.y - 10.0).abs() < 1e-9,
        "{p}"
    );
}

#[test]
fn mixed_sizes_place_largest_first_and_fill_gaps() {
    let mut j = job(2);
    j.parts.push(part("square", "square", 3));
    let a = solve_job(&j);
    assert!(a.unplaced.is_empty(), "{:?}", a.unplaced);
    // Plates first along the bottom row; squares follow bottom-left of what remains.
    assert!(pos(&a, "plate/0").distance(DVec2::new(10.0, 10.0)) < 1e-9);
    assert!(pos(&a, "plate/1").distance(DVec2::new(65.0, 10.0)) < 1e-9);
    assert!(pos(&a, "square/0").distance(DVec2::new(120.0, 10.0)) < 1e-9);
    assert!(pos(&a, "square/1").distance(DVec2::new(145.0, 10.0)) < 1e-9);
    assert!(pos(&a, "square/2").distance(DVec2::new(170.0, 10.0)) < 1e-9);
}

#[test]
fn cancellation_keeps_the_feasible_partial_layout() {
    let j = job(6);
    let s = scene();
    let (fps, _) = footprints(&j, &s);
    let calls = Cell::new(0);
    let layout = solve(&j, &fps, &|| {
        calls.set(calls.get() + 1);
        calls.get() > 2
    });
    assert!(layout.cancelled);
    assert_eq!(layout.placements.len(), 2);
    assert_eq!(layout.unplaced.len(), 4);
    assert!(layout
        .unplaced
        .iter()
        .all(|u| u.reason.contains("cancelled")));
    let mut applied = j.clone();
    apply(&mut applied, &layout);
    assert!(check_placements(&applied, &fps).is_empty());
}

#[test]
fn missing_footprints_are_reported_per_instance() {
    let mut j = job(2);
    j.parts[0].outline = None;
    let s = scene();
    let (fps, errors) = footprints(&j, &s);
    assert_eq!(errors.len(), 1);
    let layout = solve(&j, &fps, &|| false);
    assert_eq!(layout.unplaced.len(), 2);
    assert!(layout.unplaced[0].reason.contains("no footprint"));
}

#[test]
fn score_ordering() {
    let a = Score {
        sheets_used: 1,
        utilization: 0.5,
        remnant: 0.1,
    };
    let b = Score {
        sheets_used: 2,
        utilization: 0.9,
        remnant: 0.5,
    };
    assert!(a.better_than(&b));
    let c = Score {
        sheets_used: 1,
        utilization: 0.5,
        remnant: 0.2,
    };
    assert!(c.better_than(&a) && !a.better_than(&c));
    assert_eq!(
        Rotations::Step { step_deg: 90.0 }.candidates(),
        vec![0.0, 90.0, 180.0, 270.0]
    );
    assert_eq!(
        Rotations::Fixed(vec![270.0, -90.0, 30.0]).candidates(),
        vec![30.0, 270.0]
    );
    assert_eq!(Rotations::Step { step_deg: 0.0 }.candidates(), vec![0.0]);
}

#[test]
fn background_queue_returns_keyed_results_and_honours_cancellation() {
    use std::sync::atomic::AtomicBool;
    use std::sync::Arc;
    use warp_nest::job::*;
    let j = job(6);
    let s = scene();
    let (fps, _) = footprints(&j, &s);
    let queue = NestQueue::default();
    let key = input_key(&j, &fps);
    // Unlocked placements do not change the key; locked ones and parameters do.
    let mut k2 = j.clone();
    k2.placements.push(Placement {
        instance: PartInstanceId("plate/0".into()),
        sheet: SheetId("sheet".into()),
        sheet_copy: 0,
        position: DVec2::ZERO,
        rotation_deg: 0.0,
        mirrored: false,
        locked: false,
    });
    assert_eq!(input_key(&k2, &fps), key);
    k2.placements[0].locked = true;
    assert_ne!(input_key(&k2, &fps), key);
    let mut k3 = j.clone();
    k3.constraints.part_spacing += 1.0;
    assert_ne!(input_key(&k3, &fps), key);

    let token = Arc::new(AtomicBool::new(false));
    queue
        .submit(NestTask {
            epoch: 7,
            key: key.clone(),
            job: j.clone(),
            footprints: fps.clone(),
            cancelled: token.clone(),
        })
        .unwrap();
    let out = queue
        .rx
        .recv_timeout(std::time::Duration::from_secs(10))
        .unwrap();
    assert_eq!(out.epoch, 7);
    assert_eq!(out.key, key);
    assert_eq!(out.layout.placements.len(), 6);
    assert!(!out.layout.cancelled);

    let token = Arc::new(AtomicBool::new(true));
    queue
        .submit(NestTask {
            epoch: 8,
            key,
            job: j,
            footprints: fps,
            cancelled: token,
        })
        .unwrap();
    let out = queue
        .rx
        .recv_timeout(std::time::Duration::from_secs(10))
        .unwrap();
    assert!(out.layout.cancelled);
    assert!(out.layout.placements.is_empty());
    assert_eq!(out.layout.unplaced.len(), 6);
}
