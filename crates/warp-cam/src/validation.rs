//! Input and conservative stock/fixture envelope checks shared by all generators.
use crate::{Operation, Project, Stock, Toolpath};
use anyhow::{ensure, Result};
use glam::DVec3;
use warp_tools::{Tool, ToolLibrary};
/// Lowest Z any cutting move of `path` reaches: feed and arc end points and drill bottoms.
pub fn deepest_cut(path: &Toolpath) -> Option<f64> {
    path.moves
        .iter()
        .filter_map(|m| match m {
            crate::Move::Linear { to, .. } | crate::Move::Arc { to, .. } => Some(to.z),
            crate::Move::Drill { bottom, .. } => Some(*bottom),
            _ => None,
        })
        .fold(None, |acc: Option<f64>, z| Some(acc.map_or(z, |a| a.min(z))))
}

/// The warning a generated path carries when its cut reaches further below the operation top
/// than the tool's flute length. Reachable only when `Project::checks` allowed the depth.
pub fn flute_reach_warning(path: &Toolpath, top: f64, t: &Tool) -> Option<String> {
    let reach = top - deepest_cut(path)?;
    let excess = reach - t.flute_length;
    (excess > 1e-9).then(|| {
        format!(
            "Cut reaches {reach:.2} mm below the top, {excess:.2} mm beyond the {:.2} mm flute length of T{}: the shank or holder may rub the wall (flute-length check relaxed in Settings)",
            t.flute_length, t.number
        )
    })
}

pub fn parameters(op: &Operation, p: &Project, t: &Tool) -> Result<()> {
    let c = &op.common;
    let h = c.heights;
    let f = c.feeds;
    for fixture in &p.execution_fixtures {
        ensure!(
            h.clearance >= fixture.max.z + c.tolerance,
            "Operation clearance must be above every fixture for transfers between operations"
        );
    }
    ensure!(
        p.stock.min.is_finite() && p.stock.max.is_finite() && p.stock.max.cmpgt(p.stock.min).all(),
        "Stock bounds must be finite and have positive volume"
    );
    ensure!(
        f.clockwise,
        "These milling strategies currently require a clockwise spindle"
    );
    ensure!(
        [
            h.top,
            h.bottom,
            h.clearance,
            h.retract,
            h.feed_height,
            c.stepdown,
            c.stock_xy,
            c.stock_z,
            c.tolerance,
            f.feed,
            f.plunge,
            f.ramp,
            f.spindle_rpm,
            t.diameter,
            t.flute_length,
            t.stickout,
            t.shank_diameter
        ]
        .iter()
        .all(|v| v.is_finite()),
        "Parameters and tool dimensions must be finite"
    );
    ensure!(
        t.diameter > 0. && t.flute_length > 0. && t.stickout > 0. && t.shank_diameter > 0.,
        "Tool dimensions must be positive"
    );
    ensure!(
        f.feed > 0. && f.plunge > 0. && f.ramp > 0. && f.spindle_rpm > 0.,
        "Feeds and spindle speed must be positive"
    );
    ensure!(
        h.feed_height >= 0. && c.stock_xy >= 0. && c.stock_z >= 0.,
        "Feed height and stock allowances cannot be negative"
    );
    ensure!(
        c.stock_z <= h.top - h.bottom + 1e-9,
        "Floor allowance exceeds cut depth"
    );
    ensure!(
        h.clearance >= p.stock.max.z + c.tolerance && h.clearance >= h.top + h.feed_height,
        "Clearance must be above stock and feed height"
    );
    if !p.checks.allow_depth_beyond_flute {
        ensure!(
            h.top - h.bottom <= t.flute_length + 1e-9,
            "Requested depth exceeds tool flute length (Settings can allow this with a warning)"
        );
    }
    ensure!(
        (h.top - h.bottom) / c.stepdown < 100000.,
        "Too many depth levels; increase stepdown"
    );
    if let crate::Entry::Ramp { angle } = c.entry {
        ensure!(
            angle.is_finite() && (0.1..=45.).contains(&angle),
            "Ramp angle must be between 0.1 and 45 degrees"
        );
    }
    if let Some(entry) = c.helix_entry {
        ensure!(
            entry.radius.is_finite()
                && entry.radius > 0.
                && entry.pitch.is_finite()
                && entry.pitch > 0.01,
            "Helix radius/pitch must be positive (pitch > 0.01 mm)"
        );
        ensure!(
            matches!(
                op.kind,
                crate::OpKind::Pocket { .. }
                    | crate::OpKind::OpenPocket { .. }
                    | crate::OpKind::Roughing2D { .. }
            ),
            "Helical entry is supported for offset pockets and 2.5D roughing"
        );
    }
    if c.helix_entry.is_some() {
        ensure!(
            !matches!(
                op.kind,
                crate::OpKind::Pocket {
                    pattern: crate::ops::PocketPattern::ZigZag { .. },
                    ..
                }
            ),
            "Choose offset pocket pattern for helical entry"
        );
    }
    use crate::OpKind::*;
    let positive = |v: f64| v.is_finite() && v > 0.;
    match &op.kind {
        Facing {
            stepover,
            angle,
            overlap,
            ..
        } => ensure!(
            positive(*stepover) && angle.is_finite() && overlap.is_finite() && *overlap >= 0.,
            "Invalid facing parameters"
        ),
        FaceTop { stepover, angle } => ensure!(
            positive(*stepover) && angle.is_finite(),
            "Invalid Face Top parameters"
        ),
        Pocket { stepover, .. } | OpenPocket { stepover } | Roughing2D { stepover, .. } => {
            ensure!(positive(*stepover), "Stepover must be positive")
        }
        HolePocket { stepover, pitch }
        | HelicalBore {
            stepover, pitch, ..
        } => ensure!(
            positive(*stepover) && positive(*pitch) && *pitch >= 0.01,
            "Stepover and pitch must be positive"
        ),
        HoleProfile { pitch, .. } | ThreadMill { pitch, .. } => {
            ensure!(positive(*pitch), "Pitch must be positive")
        }
        Profile {
            finish_passes,
            finish_stepover,
            lead_radius,
            tabs,
            ..
        } => {
            ensure!(
                *finish_passes <= 1000
                    && finish_stepover.is_finite()
                    && *finish_stepover >= 0.
                    && lead_radius.is_finite()
                    && *lead_radius >= 0.,
                "Invalid finish/lead parameters"
            );
            ensure!(
                tabs.is_none() || *lead_radius == 0.,
                "Tabs and arc leads cannot be combined yet; set lead radius to zero"
            );
            if let Some(t) = tabs {
                ensure!(
                    t.count > 0
                        && t.count <= 1000
                        && positive(t.width)
                        && positive(t.height)
                        && t.height <= h.top - h.bottom,
                    "Invalid tab dimensions"
                );
            }
        }
        Drill {
            peck,
            dwell,
            breakthrough,
            ..
        } => ensure!(
            [*peck, *dwell, *breakthrough]
                .iter()
                .all(|v| v.is_finite() && *v >= 0.),
            "Invalid drilling cycle parameters"
        ),
        VCarve { max_depth, step } => ensure!(
            positive(*max_depth)
                && positive(*step)
                && t.profile_height(t.radius()).unwrap_or(0.) > 0.,
            "V-carving needs a tapered/pointed tool and positive depth/resolution"
        ),
        Chamfer {
            width, extra_depth, ..
        } => ensure!(
            positive(*width) && extra_depth.is_finite() && *extra_depth >= 0.,
            "Invalid chamfer width/depth"
        ),
        _ => {}
    }
    if let Pocket { stepover, .. }
    | OpenPocket { stepover }
    | Roughing2D { stepover, .. }
    | FaceTop { stepover, .. }
    | Facing { stepover, .. }
    | HolePocket { stepover, .. }
    | HelicalBore { stepover, .. } = &op.kind
    {
        ensure!(
            *stepover <= t.diameter,
            "Stepover exceeds cutter diameter and would leave uncut strips"
        );
    }
    if let ThreadMill {
        pitch,
        major_diameter,
        ..
    } = op.kind
    {
        ensure!(
            major_diameter.is_finite() && major_diameter >= 0.,
            "Thread major diameter must be finite and nonnegative"
        );
        let warp_tools::ToolShape::ThreadMill { pitch: tool_pitch } = t.shape else {
            anyhow::bail!("Select a thread-milling tool with the requested pitch");
        };
        ensure!(
            (pitch - tool_pitch).abs() < 1e-9,
            "Tool thread pitch differs from operation pitch"
        );
    }
    if let VCarve { .. } = op.kind {
        use warp_tools::ToolShape::*;
        let (angle, tip) = match t.shape {
            VBit {
                included_angle,
                tip_diameter,
            }
            | Engraver {
                included_angle,
                tip_diameter,
            } => (included_angle, tip_diameter),
            ChamferMill {
                angle,
                tip_diameter,
            } => (angle, tip_diameter),
            _ => anyhow::bail!("V-carve finishing requires a conical V/engraving/chamfer cutter"),
        };
        ensure!(
            angle > 0. && angle < 180. && tip == 0.,
            "Flat-tip V tools cannot reach sharp corners; use a pointed cutter"
        );
        ensure!(
            c.stock_xy == 0.,
            "V-carve radial allowance is not supported"
        );
    }
    Ok(())
}
// Slab intersection against a swept conservative cylindrical tool envelope.
fn segment_box(a: DVec3, b: DVec3, min: DVec3, max: DVec3) -> bool {
    let mut lo: f64 = 0.;
    let mut hi: f64 = 1.;
    for i in 0..3 {
        let d = b[i] - a[i];
        if d.abs() < 1e-12 {
            if a[i] < min[i] || a[i] > max[i] {
                return false;
            }
        } else {
            let x = (min[i] - a[i]) / d;
            let y = (max[i] - a[i]) / d;
            lo = lo.max(x.min(y));
            hi = hi.min(x.max(y));
            if lo > hi {
                return false;
            }
        }
    }
    true
}
pub fn fixtures(
    path: &Toolpath,
    boxes: &[Stock],
    tool: &Tool,
    tools: &ToolLibrary,
    tol: f64,
) -> Result<()> {
    if boxes.is_empty() {
        return Ok(());
    }
    let mut envelopes = vec![(
        tool.radius().max(tool.shank_diameter * 0.5),
        0.,
        tool.stickout,
    )];
    if let Some(id) = &tool.holder {
        let holder = tools
            .holder(id)
            .ok_or_else(|| anyhow::anyhow!("Unresolved holder {id}"))?;
        envelopes.push((
            holder.max_diameter() * 0.5,
            tool.stickout,
            tool.stickout + holder.length(),
        ));
    }
    let start = path
        .moves
        .iter()
        .find_map(|m| {
            if let crate::Move::Rapid(p) = m {
                Some(*p)
            } else {
                None
            }
        })
        .unwrap_or(DVec3::ZERO);
    for (a, b, _, _) in path.segments(start, tol.min(0.01)) {
        for f in boxes {
            for &(r, z0, z1) in &envelopes {
                let min = f.min - DVec3::new(r + tol, r + tol, z1 + tol);
                let max = f.max + DVec3::new(r + tol, r + tol, -z0 + tol);
                ensure!(!segment_box(a,b,min,max),"Tool/shank/holder envelope intersects a fixture bounding box; change clearance, selection or fixture placement");
            }
        }
    }
    Ok(())
}
