//! Generate every operation type, post it, and re-interpret the G-code to check it is valid
//! and stays within the requested depth.

use glam::{DVec2, DVec3};
use warp_cam::ops::{Common, OpKind, Operation};
use warp_cam::post::{post, PostConfig};
use warp_cam::{generate, Project};
use warp_core::{Mesh, Polyline2};
use warp_gcode::{Interpreter, MotionKind};
use warp_tools::ToolLibrary;

fn dome() -> Mesh {
    // Low-poly dome over 60x60, peak at z=0, edges at z=-10.
    let n = 30;
    let f = |i: usize, j: usize| {
        let x = i as f64 * 2.0;
        let y = j as f64 * 2.0;
        let d = ((x - 30.0).powi(2) + (y - 30.0).powi(2)).sqrt();
        DVec3::new(x, y, -10.0 + 10.0 * (1.0 - (d / 30.0).min(1.0).powi(2)))
    };
    let mut tris = Vec::new();
    for j in 0..n {
        for i in 0..n {
            tris.push([f(i, j), f(i + 1, j), f(i + 1, j + 1)]);
            tris.push([f(i, j), f(i + 1, j + 1), f(i, j + 1)]);
        }
    }
    Mesh {
        tris,
        provenance: None,
    }
}

fn project() -> Project {
    let mut p = Project::default();
    p.curves.push(Polyline2::rect(
        DVec2::new(10.0, 10.0),
        DVec2::new(50.0, 40.0),
    ));
    p.curves.push(Polyline2::rect(
        DVec2::new(25.0, 20.0),
        DVec2::new(32.0, 28.0),
    ));
    p.curves
        .push(Polyline2::circle(DVec2::new(70.0, 20.0), 6.0, 0.01));
    p.curves
        .push(Polyline2::circle(DVec2::new(70.0, 50.0), 12.0, 0.01));
    p.curves
        .push(Polyline2::rect(DVec2::new(0., 45.), DVec2::new(25., 65.)));
    p.mesh = Some(dome());
    p.stock.min = DVec3::new(0.0, 0.0, -12.0);
    p.stock.max = DVec3::new(90.0, 70.0, 0.0);
    p
}

fn tool_for(kind: &OpKind) -> u32 {
    match kind {
        OpKind::VCarve { .. } | OpKind::Chamfer { .. } => 5,
        OpKind::Drill { .. } => 7,
        OpKind::Facing { .. } => 9,
        OpKind::ThreadMill { .. } => 10,
        OpKind::Roughing3D { .. } => 1,
        OpKind::Parallel { .. }
        | OpKind::Spiral { .. }
        | OpKind::Radial { .. }
        | OpKind::Waterline { .. }
        | OpKind::ProjectCurves { .. } => 3,
        _ => 2,
    }
}

#[test]
fn every_operation_posts_valid_gcode() {
    let mut tools = ToolLibrary::starter();
    tools.tools.push(warp_tools::Tool::new(
        10,
        "Thread",
        warp_tools::ToolShape::ThreadMill { pitch: 1.5 },
        3.,
    ));
    let proj = project();
    let cfg = PostConfig::mach3();
    for kind in OpKind::all_defaults() {
        let tool = tools.tool(tool_for(&kind)).unwrap();
        let mut c = Common::for_tool(tool);
        c.heights.bottom = -3.0;
        c.resolution = 0.25;
        c.curves = match &kind {
            OpKind::Drill { .. }
            | OpKind::HelicalBore { .. }
            | OpKind::ThreadMill { .. }
            | OpKind::HolePocket { .. }
            | OpKind::HoleProfile { .. } => vec![2, 3],
            OpKind::OpenPocket { .. } => vec![4],
            OpKind::Facing { .. } => vec![],
            k if k.is_3d() && !matches!(k, OpKind::ProjectCurves { .. }) => vec![],
            _ => vec![0, 1],
        };
        if matches!(kind, OpKind::Roughing3D { .. } | OpKind::Waterline { .. }) {
            c.heights.bottom = -10.0;
            c.stepdown = 3.0;
        }
        if let OpKind::ThreadMill { .. } = kind {
            c.heights.bottom = -8.0;
        }
        let op = Operation {
            name: kind.label().into(),
            enabled: true,
            common: c.clone(),
            kind: kind.clone(),
        };
        let tp = generate(&op, &proj, &tools).unwrap_or_else(|e| panic!("{}: {e:#}", kind.label()));
        assert!(tp.cut_length() > 0.0, "{}: empty toolpath", kind.label());

        let code = warp_cam::post::post_checked(&cfg, "test", &[tp], &tools)
            .unwrap_or_else(|e| panic!("{}: {e:#}", kind.label()));
        let prog = Interpreter::default()
            .run_str(&code)
            .unwrap_or_else(|e| panic!("{}: {e}\n{code}", kind.label()));
        let min_z = prog
            .motions
            .iter()
            .filter(|m| matches!(m.kind, MotionKind::Feed | MotionKind::Arc { .. }))
            .map(|m| m.to.z)
            .fold(f64::INFINITY, f64::min);
        let limit = c.heights.bottom - tool.profile_height(tool.radius()).unwrap_or(0.0) - 1e-6;
        assert!(
            min_z >= limit,
            "{}: cut to {min_z}, below {limit}",
            kind.label()
        );
        assert!(
            prog.motions.iter().any(|m| m.kind == MotionKind::End),
            "{}: no program end",
            kind.label()
        );
    }
}

#[test]
fn pocket_stays_inside_boundary() {
    let tools = ToolLibrary::starter();
    let proj = project();
    let tool = tools.tool(2).unwrap();
    let mut c = Common::for_tool(tool);
    c.curves = vec![0, 1];
    let op = Operation {
        name: "p".into(),
        enabled: true,
        common: c,
        kind: OpKind::all_defaults()[2].clone(),
    };
    let tp = generate(&op, &proj, &tools).unwrap();
    for (_, to, rapid, _) in tp.segments(DVec3::new(0.0, 0.0, 15.0), 0.01) {
        if rapid || to.z > -0.01 {
            continue;
        }
        let p = to.truncate();
        // inside outer rectangle shrunk by the tool radius, outside the island grown by it
        assert!(
            p.x >= 11.5 - 1e-3 && p.x <= 48.5 + 1e-3 && p.y >= 11.5 - 1e-3 && p.y <= 38.5 + 1e-3,
            "{p}"
        );
        let in_island =
            p.x > 23.5 + 1e-2 && p.x < 33.5 - 1e-2 && p.y > 18.5 + 1e-2 && p.y < 29.5 - 1e-2;
        let corner = (p.x < 25.0 || p.x > 32.0) && (p.y < 20.0 || p.y > 28.0); // rounded corners of the grown island
        assert!(!in_island || corner, "gouged island at {p}");
    }
}

#[test]
fn pocket_cut_order() {
    use warp_cam::ops::{CutOrder, PocketPattern};
    let tools = ToolLibrary::starter();
    let mut proj = Project::default();
    proj.curves.push(Polyline2::rect(
        DVec2::new(0.0, 0.0),
        DVec2::new(20.0, 20.0),
    ));
    proj.curves.push(Polyline2::rect(
        DVec2::new(50.0, 0.0),
        DVec2::new(70.0, 20.0),
    ));
    let tool = tools.tool(2).unwrap();
    let mut c = Common::for_tool(tool);
    c.heights.bottom = -3.0;
    c.stepdown = 1.0;
    for order in [CutOrder::DepthFirst, CutOrder::LayerFirst] {
        let kind = OpKind::Pocket {
            pattern: PocketPattern::Offset,
            stepover: 1.2,
            finish_pass: false,
            inside_out: true,
            order,
        };
        let op = Operation {
            name: "p".into(),
            enabled: true,
            common: c.clone(),
            kind,
        };
        let tp = generate(&op, &proj, &tools).unwrap();
        // Deepest cut in the first pocket (x < 35) before the second pocket is first touched.
        let cuts: Vec<DVec3> = tp
            .segments(DVec3::new(0.0, 0.0, 15.0), 0.01)
            .into_iter()
            .filter(|s| !s.2 && s.1.z < -0.01)
            .map(|s| s.1)
            .collect();
        let first_in_b = cuts.iter().position(|p| p.x > 35.0).unwrap();
        let a_done_before_b = cuts[..first_in_b].iter().any(|p| (p.z + 3.0).abs() < 1e-6);
        match order {
            CutOrder::DepthFirst => {
                assert!(a_done_before_b, "depth-first should finish pocket A first")
            }
            CutOrder::LayerFirst => assert!(
                !a_done_before_b,
                "layer-first should visit pocket B before A reaches full depth"
            ),
        }
    }
}

#[test]
fn inch_post_matches_mm_post() {
    let tools = ToolLibrary::starter();
    let proj = project();
    let tool = tools.tool(2).unwrap();
    let mut c = Common::for_tool(tool);
    c.curves = vec![0, 1];
    let op = Operation {
        name: "p".into(),
        enabled: true,
        common: c,
        kind: OpKind::all_defaults()[1].clone(),
    };
    let tp = generate(&op, &proj, &tools).unwrap();
    let mm = PostConfig::mach3();
    let inch = PostConfig {
        inches: true,
        ..PostConfig::mach3()
    };
    let code_in = post(&inch, "t", std::slice::from_ref(&tp), &tools);
    assert!(code_in.contains("G20") && !code_in.contains("G21"));
    let a = Interpreter::default()
        .run_str(&post(&mm, "t", std::slice::from_ref(&tp), &tools))
        .unwrap();
    let b = Interpreter::default().run_str(&code_in).unwrap();
    let ends = |p: &warp_gcode::Program| {
        p.motions
            .iter()
            .filter(|m| m.is_move())
            .map(|m| m.to)
            .collect::<Vec<_>>()
    };
    let (ea, eb) = (ends(&a), ends(&b));
    assert_eq!(ea.len(), eb.len());
    for (x, y) in ea.iter().zip(&eb) {
        assert!(x.distance(*y) < 0.003, "{x} vs {y}");
    }
}
