//! Compare interpreted output with the exact toolpaths being posted.
//! Cycle expectations are expanded here independently of Toolpath::segments and the interpreter.
use crate::{
    post::PostConfig,
    toolpath::{arc_points, DrillCycle},
    Move, Toolpath,
};
use anyhow::{ensure, Context, Result};
use glam::{DVec2, DVec3};
use warp_gcode::{Interpreter, MotionKind, Plane, SpindleState};

#[derive(Clone, Debug)]
enum Kind {
    Rapid,
    Feed(f64),
    Arc(DVec2, bool, f64),
    Dwell(f64),
}
fn sweep(from: DVec2, to: DVec2, center: DVec2, cw: bool) -> f64 {
    let a = from - center;
    let b = to - center;
    let angle = a.perp_dot(b).atan2(a.dot(b));
    if cw {
        if angle >= -1e-9 {
            angle - std::f64::consts::TAU
        } else {
            angle
        }
    } else if angle <= 1e-9 {
        angle + std::f64::consts::TAU
    } else {
        angle
    }
}
#[derive(Debug)]
struct Event {
    from: DVec3,
    to: DVec3,
    kind: Kind,
    tool: u32,
    rpm: f64,
    clockwise: bool,
}
struct Trace {
    pos: DVec3,
    events: Vec<Event>,
    quantum: f64,
    tool: u32,
    rpm: f64,
    clockwise: bool,
}
impl Trace {
    fn push(&mut self, to: DVec3, kind: Kind) {
        let rounded = |p: DVec3| (p / self.quantum).round();
        if matches!(kind, Kind::Arc(..) | Kind::Dwell(..)) || rounded(self.pos) != rounded(to) {
            self.events.push(Event {
                from: self.pos,
                to,
                kind,
                tool: self.tool,
                rpm: self.rpm,
                clockwise: self.clockwise,
            });
        }
        self.pos = to;
    }
}

/// Rechecks every motion, feed, arc centre/direction, dwell and program termination.
/// Coordinates are compared in setup space; external controller WCS/TLO values are not known.
pub fn verify_post(cfg: &PostConfig, paths: &[Toolpath], code: &str) -> Result<usize> {
    ensure!(
        (3..=8).contains(&cfg.decimals) && cfg.feed_decimals <= 8,
        "Invalid verification output precision"
    );
    let scale = if cfg.inches { 25.4 } else { 1. };
    let quantum = scale * 10f64.powi(-(cfg.decimals as i32));
    let tolerance = quantum * 2. + 1e-8;
    let feed_tolerance = scale * 10f64.powi(-(cfg.feed_decimals as i32)) * 0.51 + 1e-8;
    let mut trace = Trace {
        pos: DVec3::ZERO,
        events: vec![],
        quantum,
        tool: 0,
        rpm: 0.,
        clockwise: true,
    };
    let mut tool = None;
    let mut wcs = None;
    let mut spindle = None;
    let mut split = false;
    for path in paths {
        if wcs != Some(path.wcs) {
            split = true;
            wcs = Some(path.wcs);
        }
        if tool != Some(path.tool) {
            if !cfg.tool_change.is_empty() {
                if let Some(z) = cfg.tool_change_z_g53 {
                    trace.push(DVec3::new(trace.pos.x, trace.pos.y, z), Kind::Rapid);
                    split = true;
                }
            }
            tool = Some(path.tool);
            spindle = None;
        }
        trace.tool = if cfg.tool_change.is_empty() {
            0
        } else {
            path.tool
        };
        trace.rpm = path.rpm;
        trace.clockwise = path.clockwise;
        if spindle != Some((path.rpm, path.clockwise)) && cfg.spindle_delay > 0. {
            trace.push(trace.pos, Kind::Dwell(cfg.spindle_delay));
        }
        spindle = Some((path.rpm, path.clockwise));
        for m in &path.moves {
            match *m {
                Move::Comment(_) => {}
                Move::Rapid(to) => {
                    if split {
                        trace.push(DVec3::new(trace.pos.x, trace.pos.y, to.z), Kind::Rapid);
                        split = false;
                    }
                    trace.push(to, Kind::Rapid);
                }
                Move::Linear { to, feed } => trace.push(to, Kind::Feed(feed)),
                Move::Arc {
                    to,
                    center,
                    cw,
                    feed,
                } => {
                    if cfg.arcs {
                        trace.push(to, Kind::Arc(center, cw, feed));
                    } else {
                        for p in arc_points(trace.pos, to, center, cw, cfg.arc_tolerance) {
                            trace.push(p, Kind::Feed(feed));
                        }
                    }
                }
                Move::Drill {
                    at,
                    r_plane,
                    bottom,
                    clearance,
                    peck,
                    dwell,
                    cycle,
                    feed,
                } => {
                    // G98 starts at the commanded cycle clearance, then positions above R.
                    if (trace.pos.z - clearance).abs() > 1e-9 {
                        trace.push(at.extend(clearance), Kind::Rapid);
                    }
                    trace.push(at.extend(clearance), Kind::Rapid);
                    trace.push(at.extend(r_plane), Kind::Rapid);
                    if peck > 0. && matches!(cycle, DrillCycle::Peck | DrillCycle::ChipBreak) {
                        let count = ((r_plane - bottom) / peck).ceil() as usize;
                        ensure!(count <= 100_000, "Excessive peck count during verification");
                        for i in 1..=count {
                            let z = (r_plane - i as f64 * peck).max(bottom);
                            trace.push(at.extend(z), Kind::Feed(feed));
                            if z > bottom + 1e-9 {
                                if cycle == DrillCycle::Peck {
                                    trace.push(at.extend(r_plane), Kind::Rapid);
                                }
                                trace.push(at.extend(z + peck.min(0.5)), Kind::Rapid);
                            }
                        }
                    } else {
                        trace.push(at.extend(bottom), Kind::Feed(feed));
                    }
                    if dwell > 0.
                        && matches!(cycle, DrillCycle::Dwell | DrillCycle::BoreDwellFeedOut)
                    {
                        trace.push(trace.pos, Kind::Dwell(dwell));
                    }
                    let out = if matches!(
                        cycle,
                        DrillCycle::BoreFeedOut | DrillCycle::BoreDwellFeedOut
                    ) {
                        Kind::Feed(feed)
                    } else {
                        Kind::Rapid
                    };
                    trace.push(at.extend(clearance), out);
                }
            }
        }
    }
    let program = Interpreter::default()
        .run_str(code)
        .context("Posted program cannot be interpreted")?;
    let mut expected_tools = Vec::new();
    if !cfg.tool_change.is_empty() {
        for path in paths {
            if expected_tools.last() != Some(&path.tool) {
                expected_tools.push(path.tool);
            }
        }
    }
    let actual_tools: Vec<_> = program
        .motions
        .iter()
        .filter_map(|m| {
            if let MotionKind::ToolChange { tool } = m.kind {
                Some(tool)
            } else {
                None
            }
        })
        .collect();
    ensure!(
        actual_tools == expected_tools,
        "Posted tool-change sequence differs from the source operations"
    );
    ensure!(
        program.warnings.is_empty(),
        "Posted program contains unverified commands or invalid arcs: {}",
        program.warnings.join("; ")
    );
    ensure!(
        program
            .motions
            .iter()
            .filter(|m| m.kind == MotionKind::End)
            .count()
            == 1,
        "Posted program must end exactly once"
    );
    ensure!(
        !program
            .motions
            .iter()
            .any(|m| matches!(m.kind, MotionKind::Pause { .. })),
        "Unverified pause in custom post text"
    );
    let mut spindle_state = SpindleState::Off;
    let mut spindle_rpm = 0.;
    let mut actual = Vec::new();
    for m in &program.motions {
        match m.kind {
            MotionKind::Spindle { state, rpm } => {
                spindle_state = state;
                spindle_rpm = rpm;
            }
            MotionKind::Rapid | MotionKind::Feed if m.from.distance(m.to) > quantum * 0.1 => {
                actual.push((m, spindle_state, spindle_rpm))
            }
            MotionKind::Arc { .. } | MotionKind::Dwell { .. } => {
                actual.push((m, spindle_state, spindle_rpm))
            }
            _ => {}
        }
    }
    ensure!(
        spindle_state == SpindleState::Off,
        "Posted program does not stop the spindle"
    );
    ensure!(actual.len()==trace.events.len(),"Posted motion count changed (expected {}, got {}); check custom post text and output precision",trace.events.len(),actual.len());
    for (index, (expected, (actual, state, rpm))) in trace.events.iter().zip(actual).enumerate() {
        if matches!(expected.kind, Kind::Feed(..) | Kind::Arc(..)) {
            ensure!(
                expected.tool == actual.tool,
                "Posted cut uses the wrong tool at line {}",
                actual.line
            );
            let direction = if expected.clockwise {
                SpindleState::Cw
            } else {
                SpindleState::Ccw
            };
            ensure!(
                state == direction && (rpm - expected.rpm).abs() <= 0.5,
                "Posted cut changes spindle direction/speed at line {}",
                actual.line
            );
        }
        ensure!(expected.to.distance(actual.to)<=tolerance&&expected.from.distance(actual.from)<=tolerance,"Posted motion {} changes position (G-code line {}); increase precision or correct the post",index+1,actual.line);
        let valid = match (&expected.kind, actual.kind) {
            (Kind::Rapid, MotionKind::Rapid) => true,
            (Kind::Feed(f), MotionKind::Feed) => (*f - actual.feed).abs() <= feed_tolerance,
            (
                Kind::Arc(c, cw, f),
                MotionKind::Arc {
                    center,
                    cw: out_cw,
                    plane,
                },
            ) => {
                let radius = expected.from.truncate().distance(*c);
                let expected_sweep =
                    sweep(expected.from.truncate(), expected.to.truncate(), *c, *cw);
                let actual_sweep = sweep(
                    actual.from.truncate(),
                    actual.to.truncate(),
                    center.truncate(),
                    out_cw,
                );
                *cw == out_cw
                    && plane == Plane::XY
                    && c.distance(center.truncate()) <= tolerance
                    && (*f - actual.feed).abs() <= feed_tolerance
                    && (expected_sweep - actual_sweep).abs() * radius <= tolerance * 4.
            }
            (Kind::Dwell(s), MotionKind::Dwell { seconds }) => (*s - seconds).abs() < 1e-9,
            _ => false,
        };
        ensure!(
            valid,
            "Posted motion {} changes feed, motion type, arc or dwell (G-code line {})",
            index + 1,
            actual.line
        );
    }
    Ok(trace.events.len())
}
