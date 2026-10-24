use glam::{DVec2, DVec3};
use warp_cam::{
    ops::{CutOrder, HelixEntry, PocketPattern},
    post::{post_checked, PostConfig},
    toolpath::DrillCycle,
    *,
};
use warp_core::Polyline2;
use warp_tools::ToolLibrary;
fn job(kind: OpKind) -> (Project, Operation, ToolLibrary) {
    let mut tools = ToolLibrary::starter();
    if let OpKind::ThreadMill { pitch, .. } = kind {
        tools
            .tools
            .iter_mut()
            .find(|t| t.number == 2)
            .unwrap()
            .shape = warp_tools::ToolShape::ThreadMill { pitch };
    }
    let mut p = Project::default();
    p.stock = Stock {
        min: DVec3::new(0., 0., -6.),
        max: DVec3::new(60., 40., 0.),
    };
    p.curves = vec![Polyline2::rect(DVec2::new(5., 5.), DVec2::new(55., 35.))];
    let mut common = Common::for_tool(tools.tool(2).unwrap());
    common.entry = Entry::Plunge;
    common.curves = vec![0];
    common.heights.bottom = -2.3;
    (
        p,
        Operation {
            name: "M3".into(),
            enabled: true,
            common,
            kind,
        },
        tools,
    )
}
fn pocket() -> OpKind {
    OpKind::Pocket {
        pattern: PocketPattern::Offset,
        stepover: 1.,
        finish_pass: true,
        inside_out: false,
        order: CutOrder::LayerFirst,
    }
}
#[test]
fn invalid_regions_are_rejected() {
    let a = Polyline2::rect(DVec2::ZERO, DVec2::splat(10.));
    let bow = Polyline2::new(
        vec![
            DVec2::ZERO,
            DVec2::splat(10.),
            DVec2::new(0., 10.),
            DVec2::new(10., 0.),
        ],
        true,
    );
    assert!(region::validated(&[&a, &a]).is_err());
    assert!(region::validated(&[&bow]).is_err());
    let mut deg = a.clone();
    deg.pts.insert(1, deg.pts[0]);
    assert!(region::validated(&[&deg]).is_err());
    let other = Polyline2::rect(DVec2::splat(20.), DVec2::splat(30.));
    assert_eq!(region::validated(&[&a, &other]).unwrap().0.len(), 2);
}

#[test]
fn open_wall_direction_selects_clearing_side_and_survives_document_resolution() {
    use geo::{Area, Contains, Point};
    let min = DVec2::ZERO;
    let max = DVec2::new(60., 40.);
    let wall = Polyline2::new(
        vec![
            DVec2::new(0., 10.),
            DVec2::new(20., 10.),
            DVec2::new(20., 30.),
            DVec2::new(0., 30.),
        ],
        false,
    );
    let left = region::open_pocket(&[&wall], min, max, 0.01).unwrap();
    let right = region::open_pocket(&[&wall.reversed()], min, max, 0.01).unwrap();
    assert!((left.unsigned_area() - 400.).abs() < 1e-8);
    assert!((right.unsigned_area() - 2000.).abs() < 1e-8);
    assert!(left.contains(&Point::new(10., 20.)));
    assert!(!right.contains(&Point::new(10., 20.)));
    let (mut p, op, t) = job(OpKind::OpenPocket { stepover: 1. });
    p.curves = vec![wall];
    p.operations = vec![op];
    p.ensure_document().unwrap();
    let tp = generate(&p.operations[0], &p, &t).unwrap();
    assert!(tp.cut_length() > 0.);
    let restored = Project::from_json(&p.document_json().unwrap()).unwrap();
    assert_eq!(
        generate(&restored.operations[0], &restored, &t)
            .unwrap()
            .moves,
        tp.moves
    );
}

#[test]
fn ambiguous_open_walls_and_interoperation_fixture_clearance_are_rejected() {
    let bad = Polyline2::new(vec![DVec2::new(5., 10.), DVec2::new(20., 20.)], false);
    assert!(region::open_pocket(&[&bad], DVec2::ZERO, DVec2::splat(40.), 0.01).is_err());
    let (mut p, op, t) = job(pocket());
    p.execution_fixtures = vec![Stock {
        min: DVec3::new(100., 100., 0.),
        max: DVec3::new(105., 105., 20.),
    }];
    // It is clear of this operation, but crosses an unverified transfer-height envelope.
    assert!(generate(&op, &p, &t)
        .unwrap_err()
        .to_string()
        .contains("above every fixture"));
}
#[test]
fn links_cannot_cross_an_island() {
    let outer = Polyline2::rect(DVec2::ZERO, DVec2::splat(10.));
    let hole = Polyline2::rect(DVec2::splat(4.), DVec2::splat(6.));
    let policy = planner::LinkPolicy {
        centre_region: Some(region::validated(&[&outer, &hole]).unwrap()),
    };
    assert!(!policy.permits(DVec3::new(3.9, 5., -1.), DVec3::new(6.1, 5., -1.)));
    assert!(policy.permits(DVec3::new(1., 1., -1.), DVec3::new(9., 1., -1.)));
    assert!(!policy.permits(DVec3::new(1., 1., -1.), DVec3::new(9., 1., -2.)));
}
#[test]
fn circle_fit_handles_nonuniform_sampling() {
    let c = DVec2::new(21., -7.);
    let angles = [0., 0.01, 0.03, 0.2, 0.5, 1., 2., 3., 4., 5.];
    let pl = Polyline2::new(
        angles
            .into_iter()
            .map(|a| c + DVec2::from_angle(a) * 8.)
            .collect(),
        true,
    );
    let (found, d) = as_circle(&pl).unwrap();
    assert!(found.distance(c) < 1e-9);
    assert!((d - 16.).abs() < 1e-9);
}
#[test]
fn thread_pitch_and_handedness_survive_partial_turns() {
    for internal in [true, false] {
        for right_hand in [true, false] {
            let (mut p, op, t) = job(OpKind::ThreadMill {
                pitch: 1.,
                internal,
                major_diameter: 16.,
                right_hand,
            });
            p.curves = vec![Polyline2::circle(DVec2::splat(20.), 8., 0.01)];
            let path = generate(&op, &p, &t).unwrap();
            let mut previous = None;
            let mut count = 0;
            for m in path.moves {
                match m {
                    Move::Rapid(to) | Move::Linear { to, .. } => previous = Some(to),
                    Move::Arc { to, center, cw, .. } => {
                        let from = previous.unwrap();
                        let a = from.truncate() - center;
                        let b = to.truncate() - center;
                        let mut angle = a.perp_dot(b).atan2(a.dot(b));
                        if cw && angle >= 0. {
                            angle -= std::f64::consts::TAU;
                        }
                        if !cw && angle <= 0. {
                            angle += std::f64::consts::TAU;
                        }
                        let pitch = (to.z - from.z) / angle * std::f64::consts::TAU;
                        assert!((pitch.abs() - 1.).abs() < 1e-8);
                        assert_eq!(pitch > 0., right_hand);
                        previous = Some(to);
                        count += 1;
                    }
                    _ => {}
                }
            }
            assert_eq!(count, 5);
        }
    }
}
#[test]
fn fixture_and_depth_errors_block_generation() {
    let (mut p, mut op, t) = job(pocket());
    p.execution_fixtures = vec![Stock {
        min: DVec3::new(5., 5., -3.),
        max: DVec3::new(15., 15., 4.),
    }];
    assert!(generate(&op, &p, &t)
        .unwrap_err()
        .to_string()
        .contains("fixture"));
    p.execution_fixtures.clear();
    op.common.heights.bottom = -100.;
    assert!(generate(&op, &p, &t)
        .unwrap_err()
        .to_string()
        .contains("flute"));
}
#[test]
fn open_pocket_crosses_stock_edge_without_crossing_closed_walls() {
    let (mut p, op, t) = job(OpKind::OpenPocket { stepover: 1. });
    assert!(generate(&op, &p, &t).is_err());
    p.curves[0] = Polyline2::rect(DVec2::new(0., 10.), DVec2::new(20., 30.));
    let tp = generate(&op, &p, &t).unwrap();
    let cuts: Vec<_> = tp
        .segments(DVec3::new(0., 0., 15.), 0.01)
        .into_iter()
        .filter(|s| !s.2 && s.1.z < 0.)
        .collect();
    assert!(cuts.iter().any(|s| s.1.x < 0.));
    for (_, b, _, _) in cuts {
        if b.x >= 0. {
            assert!(b.x <= 18.501 && b.y >= 11.499 && b.y <= 28.501, "{b}");
        }
    }
}
#[test]
fn helix_entry_is_bounded_and_uses_requested_feed() {
    let (p, mut op, t) = job(pocket());
    op.common.helix_entry = Some(HelixEntry {
        radius: 1.,
        pitch: 0.7,
    });
    let path = generate(&op, &p, &t).unwrap();
    assert!(path
        .moves
        .iter()
        .any(|m| matches!(m,Move::Arc{feed,..} if (*feed-op.common.feeds.ramp).abs()<1e-9)));
    op.common.helix_entry = Some(HelixEntry {
        radius: 100.,
        pitch: 1.,
    });
    assert!(generate(&op, &p, &t).is_err());
}
#[test]
fn posts_preserve_hole_endpoints_units_and_dwell() {
    let (mut p, op, t) = job(OpKind::HoleProfile {
        pitch: 0.7,
        spring_pass: true,
    });
    p.curves = vec![Polyline2::circle(DVec2::splat(20.), 8., 0.001)];
    let path = generate(&op, &p, &t).unwrap();
    for mut cfg in PostConfig::presets() {
        for inches in [false, true] {
            cfg.inches = inches;
            let code = post_checked(&cfg, "m3", &[path.clone()], &t).unwrap();
            let program = warp_gcode::Interpreter::default().run_str(&code).unwrap();
            let min = program
                .motions
                .iter()
                .map(|m| m.to.z)
                .fold(f64::INFINITY, f64::min);
            assert!((min + 2.3).abs() < 0.003, "{}: {min}", cfg.name);
            assert!(program
                .motions
                .iter()
                .any(|m| matches!(m.kind, warp_gcode::MotionKind::Arc { .. })));
        }
    }
    let (p, op, t) = job(OpKind::Drill {
        cycle: DrillCycle::Dwell,
        peck: 0.,
        dwell: 1.25,
        source: ops::DrillSource::CurveStarts,
        tip_compensation: false,
        breakthrough: 0.,
    });
    let path = generate(&op, &p, &t).unwrap();
    let code = post_checked(&PostConfig::grbl(), "dwell", &[path], &t).unwrap();
    let dwell = code.find("G4 P1.25").unwrap();
    assert!(code[..dwell].contains("Z-2.3"));
    assert!(code[dwell..].contains("G0"));
}
#[test]
fn unsupported_post_tool_changes_are_blocked() {
    let (p, op, t) = job(pocket());
    let path = generate(&op, &p, &t).unwrap();
    let mut second = path.clone();
    second.tool = 1;
    assert!(post_checked(&PostConfig::grbl(), "multi", &[path, second], &t).is_err());
}
#[test]
fn flat_tip_chamfer_uses_cone_contact() {
    let (p, mut op, mut t) = job(OpKind::Chamfer {
        width: 1.,
        extra_depth: 0.,
        side: Side::Outside,
    });
    op.common.tool = 5;
    t.tools.iter_mut().find(|t| t.number == 5).unwrap().shape =
        warp_tools::ToolShape::ChamferMill {
            angle: 90.,
            tip_diameter: 2.,
        };
    let path = generate(&op, &p, &t).unwrap();
    let cuts: Vec<_> = path
        .segments(DVec3::new(0., 0., 15.), 0.01)
        .into_iter()
        .filter(|s| !s.2 && s.1.z < -0.999)
        .collect();
    assert!(!cuts.is_empty());
    assert!(cuts.iter().all(|s| (s.1.z + 1.).abs() < 1e-8));
    // One-mm flat tip radius puts the tool axis one mm outside the edge at a one-mm depth.
    assert!(cuts.iter().any(|s| (s.1.x - 4.).abs() < 0.01));
}

#[test]
fn all_drill_cycles_roundtrip_every_post_in_both_units() {
    use warp_gcode::{Interpreter, MotionKind};
    for cycle in [
        DrillCycle::Simple,
        DrillCycle::Dwell,
        DrillCycle::Peck,
        DrillCycle::ChipBreak,
        DrillCycle::BoreFeedOut,
        DrillCycle::BoreDwellFeedOut,
    ] {
        let (p, op, t) = job(OpKind::Drill {
            cycle,
            peck: 0.7,
            dwell: 1.25,
            source: ops::DrillSource::CurveStarts,
            tip_compensation: false,
            breakthrough: 0.2,
        });
        let path = generate(&op, &p, &t).unwrap();
        for mut cfg in PostConfig::presets() {
            for inches in [false, true] {
                cfg.inches = inches;
                let program = Interpreter::default()
                    .run_str(&post_checked(&cfg, "cycles", &[path.clone()], &t).unwrap())
                    .unwrap();
                let lowest = program
                    .motions
                    .iter()
                    .map(|m| m.to.z)
                    .fold(f64::INFINITY, f64::min);
                assert!(
                    (lowest + 2.5).abs() < 0.003,
                    "{} {cycle:?}: {lowest}",
                    cfg.name
                );
                let dwells: Vec<_> = program
                    .motions
                    .iter()
                    .filter(|m| matches!(m.kind, MotionKind::Dwell { .. }))
                    .collect();
                let expected = matches!(cycle, DrillCycle::Dwell | DrillCycle::BoreDwellFeedOut);
                assert_eq!(!dwells.is_empty(), expected, "{} {cycle:?}", cfg.name);
                for dwell in dwells {
                    assert!(
                        matches!(dwell.kind,MotionKind::Dwell{seconds} if (seconds-1.25).abs()<1e-9)
                    );
                    assert!((dwell.to.z + 2.5).abs() < 0.003);
                }
                if matches!(
                    cycle,
                    DrillCycle::BoreFeedOut | DrillCycle::BoreDwellFeedOut
                ) {
                    assert!(program
                        .motions
                        .iter()
                        .any(|m| m.kind == MotionKind::Feed && m.to.z > m.from.z));
                }
                assert!(program.motions.iter().any(|m| (m.to.z - 15.).abs() < 0.003));
            }
        }
    }
}
