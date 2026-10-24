//! Relaxed safety checks: a project may allow cut depths beyond the tool's flute length. The
//! refusal becomes a generated path that carries a warning naming the excess, the choice is
//! saved with the project, and it is part of the toolpath input key.
use glam::{DVec2, DVec3};
use std::path::PathBuf;
use warp_cam::{generate, workflow::input_key, Checks, Common, Move, OpKind, Operation, Project};
use warp_core::Polyline2;
use warp_tools::ToolLibrary;

fn root() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../fixtures/cam-baseline")
}

fn project() -> Project {
    let mut p = Project::default();
    p.curves.push(Polyline2::rect(DVec2::new(10.0, 10.0), DVec2::new(50.0, 40.0)));
    p.curves.push(Polyline2::circle(DVec2::new(70.0, 20.0), 6.0, 0.01));
    p.stock.min = DVec3::new(0.0, 0.0, -60.0);
    p.stock.max = DVec3::new(90.0, 70.0, 0.0);
    p
}

fn kind(pick: impl Fn(&OpKind) -> bool) -> OpKind {
    OpKind::all_defaults().into_iter().find(pick).unwrap()
}

fn deepest(moves: &[Move]) -> f64 {
    moves
        .iter()
        .filter_map(|m| match m {
            Move::Linear { to, .. } | Move::Arc { to, .. } => Some(to.z),
            Move::Drill { bottom, .. } => Some(*bottom),
            _ => None,
        })
        .fold(f64::INFINITY, f64::min)
}

#[test]
fn depth_beyond_flute_is_refused_unless_the_project_allows_it() {
    let tools = ToolLibrary::starter();
    let excess = 4.0;
    let drill = match kind(|k| matches!(k, OpKind::Drill { .. })) {
        OpKind::Drill { cycle, peck, dwell, source, .. } => OpKind::Drill {
            cycle,
            peck,
            dwell,
            source,
            tip_compensation: false,
            breakthrough: 0.0,
        },
        _ => unreachable!(),
    };
    let cases = [
        (kind(|k| matches!(k, OpKind::Pocket { .. })), 2u32, vec![0usize]),
        (drill, 7, vec![1]),
    ];
    for (kind, tool_number, curves) in cases {
        let tool = tools.tool(tool_number).unwrap();
        let mut c = Common::for_tool(tool);
        c.heights.top = 0.0;
        c.heights.bottom = -(tool.flute_length + excess);
        c.stock_z = 0.0;
        c.curves = curves;
        let op = Operation {
            name: kind.label().into(),
            enabled: true,
            common: c,
            kind: kind.clone(),
        };
        let mut p = project();

        let refused = generate(&op, &p, &tools).err().map(|e| format!("{e:#}"));
        assert!(
            refused.as_deref().is_some_and(|e| e.contains("flute length")),
            "{}: expected a flute-length refusal, got {refused:?}",
            kind.label()
        );

        p.checks.allow_depth_beyond_flute = true;
        let tp = generate(&op, &p, &tools).unwrap_or_else(|e| panic!("{}: {e:#}", kind.label()));
        let bottom = deepest(&tp.moves);
        assert!(
            (bottom - op.common.heights.bottom).abs() < 1e-6,
            "{}: cut to {bottom}, requested {}",
            kind.label(),
            op.common.heights.bottom
        );
        assert_eq!(tp.warnings.len(), 1, "{}: {:?}", kind.label(), tp.warnings);
        let w = &tp.warnings[0];
        assert!(
            w.contains(&format!("{excess:.2} mm beyond"))
                && w.contains(&format!("{:.2} mm flute length", tool.flute_length))
                && w.contains(&format!("T{tool_number}")),
            "{}: {w}",
            kind.label()
        );

        // A cut within the flutes carries no warning even with the check relaxed.
        let mut shallow = op.clone();
        shallow.common.heights.bottom = -(tool.flute_length * 0.5);
        let tp = generate(&shallow, &p, &tools).unwrap();
        assert!(tp.warnings.is_empty(), "{}: {:?}", kind.label(), tp.warnings);
    }
}

#[test]
fn checks_are_saved_with_the_project_and_default_off() {
    let path = root().join("open-profile.warp");
    let text = std::fs::read_to_string(&path).unwrap();
    assert!(!text.contains("\"checks\""), "fixture predates checks");
    let mut p = Project::from_json(&text).unwrap();
    assert_eq!(p.checks, Checks::default());
    assert!(!p.checks.allow_depth_beyond_flute);

    p.checks.allow_depth_beyond_flute = true;
    let saved = p.document_json().unwrap();
    let reloaded = Project::from_json(&saved).unwrap();
    assert!(reloaded.checks.allow_depth_beyond_flute);
}

#[test]
fn relaxing_a_check_changes_the_toolpath_input_key() {
    let tools = ToolLibrary::load(&root().join("tools.json")).unwrap();
    let mut p = Project::load(&root().join("open-profile.warp")).unwrap();
    let before = input_key(&p.operations[0], &p, &tools).unwrap();
    p.checks.allow_depth_beyond_flute = true;
    let after = input_key(&p.operations[0], &p, &tools).unwrap();
    assert_ne!(before, after);
    let (resolved, _) = p.resolve_operation(&p.operations[0]).unwrap();
    assert!(resolved.checks.allow_depth_beyond_flute, "resolved project drops the checks");
}
