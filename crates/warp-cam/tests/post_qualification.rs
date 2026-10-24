use glam::{DVec2, DVec3};
use warp_cam::{
    post::{post, post_checked, PostConfig},
    toolpath::DrillCycle,
    verification::verify_post,
    Move, Toolpath,
};
use warp_gcode::{Interpreter, MotionKind};
use warp_tools::{Coolant, ToolLibrary};

fn path(moves: Vec<Move>) -> Toolpath {
    Toolpath {
        wcs: 54,
        name: "Audit".into(),
        tool: 1,
        rpm: 12000.,
        clockwise: true,
        coolant: Coolant::Off,
        moves,
        warnings: vec![],
    }
}
fn arc_job() -> Toolpath {
    path(vec![
        Move::Rapid(DVec3::new(15., 20., 10.)),
        Move::Linear {
            to: DVec3::new(15., 20., -1.),
            feed: 120.,
        },
        Move::Arc {
            to: DVec3::new(10., 25., -1.75),
            center: DVec2::new(10., 20.),
            cw: false,
            feed: 600.,
        },
        Move::Arc {
            to: DVec3::new(15., 20., -2.),
            center: DVec2::new(10., 20.),
            cw: true,
            feed: 600.,
        },
        Move::Arc {
            to: DVec3::new(15., 20., -2.),
            center: DVec2::new(10., 20.),
            cw: true,
            feed: 600.,
        },
        Move::Linear {
            to: DVec3::new(15., 20., 1.),
            feed: 120.,
        },
        Move::Rapid(DVec3::new(15., 20., 10.)),
    ])
}

#[test]
fn all_posts_preserve_arcs_helices_and_raise_before_xy() {
    let tools = ToolLibrary::starter();
    let p = arc_job();
    for mut cfg in PostConfig::presets() {
        for inches in [false, true] {
            for arcs in [false, true] {
                cfg.inches = inches;
                cfg.arcs = arcs;
                for absolute in [false, true] {
                    cfg.arc_ij_absolute = absolute;
                    if absolute && (cfg.name == "GRBL" || cfg.name == "TurboCNC") {
                        assert!(post_checked(&cfg, "unsupported", &[p.clone()], &tools).is_err());
                        continue;
                    }
                    let code = post_checked(&cfg, "audit", &[p.clone()], &tools).unwrap();
                    let prog = Interpreter::default().run_str(&code).unwrap();
                    let moves: Vec<_> = prog.motions.iter().filter(|m| m.is_move()).collect();
                    assert!(moves[0].to.distance(DVec3::new(0., 0., 10.)) < 0.003);
                    assert!(moves[1].to.distance(DVec3::new(15., 20., 10.)) < 0.003);
                    if arcs {
                        let circles: Vec<_> = moves
                            .iter()
                            .filter(|m| matches!(m.kind, MotionKind::Arc { .. }))
                            .collect();
                        assert_eq!(circles.len(), 3);
                        for (m, (z, cw)) in
                            circles
                                .iter()
                                .zip([(-1.75, false), (-2., true), (-2., true)])
                        {
                            assert!((m.to.z - z).abs() < 0.003);
                            assert!(
                                matches!(m.kind,MotionKind::Arc{center,cw:direction,..} if center.truncate().distance(DVec2::new(10.,20.))<0.003&&direction==cw)
                            );
                        }
                    } else {
                        // Independent geometric check: every descending point is on the R5 cylinder.
                        for m in &moves {
                            if m.to.z <= -1. && m.from.z <= -1. && m.to.z <= m.from.z {
                                assert!(
                                    (m.to.truncate().distance(DVec2::new(10., 20.)) - 5.).abs()
                                        < 0.003
                                );
                            }
                        }
                    }
                }
            }
        }
    }
}

#[test]
fn cycle_traces_match_hand_calculated_depth_sequences() {
    let tools = ToolLibrary::starter();
    for (cycle, cut) in [
        (DrillCycle::Simple, vec![(false, -1.3), (true, 5.)]),
        (DrillCycle::Dwell, vec![(false, -1.3), (true, 5.)]),
        (
            DrillCycle::Peck,
            vec![
                (false, 0.),
                (true, 1.),
                (true, 0.5),
                (false, -1.),
                (true, 1.),
                (true, -0.5),
                (false, -1.3),
                (true, 5.),
            ],
        ),
        (
            DrillCycle::ChipBreak,
            vec![
                (false, 0.),
                (true, 0.5),
                (false, -1.),
                (true, -0.5),
                (false, -1.3),
                (true, 5.),
            ],
        ),
        (DrillCycle::BoreFeedOut, vec![(false, -1.3), (false, 5.)]),
        (
            DrillCycle::BoreDwellFeedOut,
            vec![(false, -1.3), (false, 5.)],
        ),
    ] {
        let p = path(vec![
            Move::Rapid(DVec3::new(2., 3., 5.)),
            Move::Drill {
                at: DVec2::new(2., 3.),
                r_plane: 1.,
                bottom: -1.3,
                clearance: 5.,
                peck: 1.,
                dwell: 0.75,
                cycle,
                feed: 120.,
            },
        ]);
        for mut cfg in PostConfig::presets() {
            for inches in [false, true] {
                cfg.inches = inches;
                let prog = Interpreter::default()
                    .run_str(&post_checked(&cfg, "cycles", &[p.clone()], &tools).unwrap())
                    .unwrap();
                let actual: Vec<_> = prog
                    .motions
                    .iter()
                    .filter(|m| m.is_move() && m.from.distance(m.to) > 1e-8)
                    .collect();
                let expected: Vec<_> = [(true, 5.), (true, 5.), (true, 1.)]
                    .into_iter()
                    .chain(cut.iter().copied())
                    .collect();
                assert_eq!(actual.len(), expected.len(), "{} {cycle:?}", cfg.name);
                for (m, (rapid, z)) in actual.iter().zip(expected) {
                    assert_eq!(m.kind == MotionKind::Rapid, rapid);
                    assert!((m.to.z - z).abs() < 0.004);
                    if !rapid {
                        assert!((m.feed - 120.).abs() < 1.3);
                    }
                }
                for m in prog
                    .motions
                    .iter()
                    .filter(|m| matches!(m.kind, MotionKind::Dwell { .. }))
                {
                    assert!((m.to.z + 1.3).abs() < 0.003);
                }
            }
        }
    }
}

#[test]
fn verification_rejects_corrupted_output_and_post_macros() {
    let tools = ToolLibrary::starter();
    let cfg = PostConfig::mach3();
    let p = arc_job();
    let code = post(&cfg, "audit", &[p.clone()], &tools);
    for bad in [
        code.replace("G3", "G2"),
        code.replace("F600.0", "F60.0"),
        code.replace("Z-1.75", "Z-7.75"),
        code.replace("M30", "G0 X999\nM30"),
        code.replace("G91.1", "G90.1"),
        code.replace("T1 M6", "T2 M6"),
        code.replace("S12000 M3", "S12000 M4"),
        code.replace("M5", "M3"),
        code.replace("G94", "G95"),
        code.replace("M30", "T2 M6\nM30"),
    ] {
        assert!(verify_post(&cfg, &[p.clone()], &bad).is_err());
    }
    let mut cfg = cfg;
    cfg.postamble = vec!["G41 D1".into()];
    assert!(post_checked(&cfg, "comp", &[p.clone()], &tools).is_err());
    cfg.postamble = vec!["G0 X999".into()];
    assert!(post_checked(&cfg, "extra", &[p.clone()], &tools).is_err());
    cfg.postamble.clear();
    cfg.preamble.push("G92 X50".into());
    assert!(post_checked(&cfg, "offset", &[p], &tools).is_err());
}

#[test]
fn custom_output_modes_and_tool_change_height_are_verified() {
    let tools = ToolLibrary::starter();
    let a = arc_job();
    let mut b = a.clone();
    b.tool = 2;
    b.wcs = 55;
    for modal in [true, false] {
        let mut cfg = PostConfig::linuxcnc();
        cfg.modal_axes = modal;
        cfg.modal_motion = modal;
        cfg.tool_change_z_g53 = Some(25.);
        cfg.spindle_delay = 0.5;
        post_checked(&cfg, "changes", &[a.clone(), b.clone()], &tools).unwrap();
    }
}

#[test]
fn invalid_numeric_paths_fail_before_interpretation() {
    let tools = ToolLibrary::starter();
    let cfg = PostConfig::mach3();
    let mut p = arc_job();
    p.moves[1] = Move::Linear {
        to: DVec3::splat(f64::NAN),
        feed: 20.,
    };
    assert!(post_checked(&cfg, "nan", &[p], &tools).is_err());
}

#[test]
fn rounding_a_short_arc_into_a_full_circle_is_rejected() {
    let tools = ToolLibrary::starter();
    let mut cfg = PostConfig::mach3();
    cfg.decimals = 3;
    let p = path(vec![
        Move::Rapid(DVec3::new(5., 0., 10.)),
        Move::Arc {
            to: DVec3::new(5. * 0.00002f64.cos(), 5. * 0.00002f64.sin(), 10.),
            center: DVec2::ZERO,
            cw: false,
            feed: 600.,
        },
    ]);
    assert!(post_checked(&cfg, "short arc", &[p.clone()], &tools).is_err());
    cfg.decimals = 6;
    post_checked(&cfg, "short arc", &[p], &tools).unwrap();
}
