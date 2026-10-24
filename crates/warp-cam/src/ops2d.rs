//! 2D / 2.5D operations.

use crate::medial;
use crate::ops::{depth_levels, CutOrder, DrillSource, OpKind, PocketPattern, Side, Tabs};
use crate::region::{self, Region};
use crate::toolpath::{Move, PathBuilder};
use crate::Gen;
use anyhow::{bail, Result};
use glam::{DVec2, DVec3};
use warp_core::Polyline2;

pub fn generate(g: &Gen) -> Result<Vec<Move>> {
    match &g.op.kind {
        OpKind::Facing {
            stepover,
            angle,
            overlap,
            zigzag,
        } => facing(g, *stepover, *angle, *overlap, *zigzag),
        OpKind::Profile {
            side,
            finish_passes,
            finish_stepover,
            spring_pass,
            lead_radius,
            tabs,
        } => profile(
            g,
            *side,
            *finish_passes,
            *finish_stepover,
            *spring_pass,
            *lead_radius,
            *tabs,
        ),
        OpKind::Pocket {
            pattern,
            stepover,
            finish_pass,
            inside_out,
            order,
        } => pocket(g, *pattern, *stepover, *finish_pass, *inside_out, *order),
        OpKind::Drill {
            cycle,
            peck,
            dwell,
            source,
            tip_compensation,
            breakthrough,
        } => drill(
            g,
            *cycle,
            *peck,
            *dwell,
            *source,
            *tip_compensation,
            *breakthrough,
        ),
        OpKind::Engrave | OpKind::Slot => engrave(g),
        OpKind::OpenPocket { stepover } => {
            let area = region::open_pocket(
                &g.project.select_curves(&g.c.curves),
                g.project.stock.min.truncate(),
                g.project.stock.max.truncate(),
                g.c.tolerance,
            )?;
            let stock = region::rect(
                g.project.stock.min.truncate(),
                g.project.stock.max.truncate(),
            );
            anyhow::ensure!(
                region::is_empty(&region::difference(&area, &stock)),
                "Open-pocket extent must be inside stock"
            );
            let touches = region::loops(&area, true).iter().any(|lp| {
                lp.iter().zip(lp.iter().cycle().skip(1)).any(|(a, b)| {
                    (0..2).any(|i| {
                        [g.project.stock.min[i], g.project.stock.max[i]]
                            .iter()
                            .any(|edge| (a[i] - edge).abs() < 1e-8 && (b[i] - edge).abs() < 1e-8)
                    })
                })
            });
            anyhow::ensure!(
                touches,
                "Open pocket requires an edge on the stock perimeter"
            );
            let grown = region::offset(&area, g.tool.diameter + g.c.stock_xy * 2.);
            let outside = region::difference(&grown, &stock);
            anyhow::ensure!(
                !region::is_empty(&outside),
                "Open pocket must meet a stock edge"
            );
            // Extend only through stock edges; retained stock still constrains every wall.
            use geo::BooleanOps;
            let extended = area.union(&outside);
            let mut project = g.project.clone();
            project.curves = region::loops(&extended, true)
                .into_iter()
                .map(|pts| Polyline2::new(pts, true))
                .collect();
            let mut op = g.op.clone();
            op.common.curves = (0..project.curves.len()).collect();
            let adapted = Gen {
                op: &op,
                c: &op.common,
                tool: g.tool,
                project: &project,
            };
            pocket(
                &adapted,
                PocketPattern::Offset,
                *stepover,
                true,
                false,
                CutOrder::LayerFirst,
            )
        }
        OpKind::FaceTop { stepover, angle } => pocket(
            g,
            PocketPattern::ZigZag { angle: *angle },
            *stepover,
            true,
            false,
            CutOrder::LayerFirst,
        ),
        OpKind::Roughing2D {
            stepover,
            inside_out,
        } => {
            let parts = selected_region(g)?;
            let stock = region::rect(
                g.project.stock.min.truncate() - DVec2::splat(g.tool.diameter + g.c.stock_xy),
                g.project.stock.max.truncate() + DVec2::splat(g.tool.diameter + g.c.stock_xy),
            );
            let clear = region::difference(&stock, &parts);
            let mut project = g.project.clone();
            project.curves = region::loops(&clear, true)
                .into_iter()
                .map(|pts| Polyline2::new(pts, true))
                .collect();
            let mut op = g.op.clone();
            op.common.curves = (0..project.curves.len()).collect();
            anyhow::ensure!(
                !project.curves.is_empty(),
                "Retained part covers all stock; nothing to rough"
            );
            let adapted = Gen {
                op: &op,
                c: &op.common,
                tool: g.tool,
                project: &project,
            };
            pocket(
                &adapted,
                PocketPattern::Offset,
                *stepover,
                true,
                *inside_out,
                CutOrder::LayerFirst,
            )
        }
        OpKind::HolePocket { stepover, pitch } => helical_bore(g, 0., f64::MAX, *pitch, *stepover),
        OpKind::HoleProfile { pitch, spring_pass } => hole_profile(g, *pitch, *spring_pass),
        OpKind::VCarve { max_depth, step } => vcarve(g, *max_depth, *step),
        OpKind::HelicalBore {
            min_diameter,
            max_diameter,
            pitch,
            stepover,
        } => helical_bore(g, *min_diameter, *max_diameter, *pitch, *stepover),
        OpKind::ThreadMill {
            pitch,
            internal,
            major_diameter,
            right_hand,
        } => thread_mill(g, *pitch, *internal, *major_diameter, *right_hand),
        OpKind::Chamfer {
            width,
            extra_depth,
            side,
        } => chamfer(g, *width, *extra_depth, *side),
        _ => bail!("not a 2D operation"),
    }
}

fn selected_region(g: &Gen) -> Result<Region> {
    let curves = g.project.select_curves(&g.c.curves);
    region::validated(&curves)
}

fn final_z(g: &Gen) -> f64 {
    g.c.heights.bottom + g.c.stock_z
}

/// Parallel lines at `angle` covering `bounds`, spaced `step` apart.
fn raster_lines(min: DVec2, max: DVec2, step: f64, angle_deg: f64) -> Vec<(DVec2, DVec2)> {
    let dir = DVec2::from_angle(angle_deg.to_radians());
    let nrm = dir.perp();
    let c = (min + max) * 0.5;
    let half = (max - min).length() * 0.5 + step;
    let n = (2.0 * half / step.max(1e-3)).ceil() as i64;
    (0..=n)
        .map(|i| {
            let o = c + nrm * (-half + i as f64 * step);
            (o - dir * half, o + dir * half)
        })
        .collect()
}

/// Order clipped raster segments into a zigzag, grouped by line. Returns polylines.
fn zigzag(segs: Vec<(DVec2, DVec2)>, dir: DVec2, zig: bool) -> Vec<(DVec2, DVec2)> {
    // Sort by perpendicular offset, then along the line.
    let nrm = dir.perp();
    let mut segs: Vec<_> = segs
        .into_iter()
        .map(|(a, b)| {
            if (b - a).dot(dir) < 0.0 {
                (b, a)
            } else {
                (a, b)
            }
        })
        .collect();
    segs.sort_by(|x, y| {
        let ox = (x.0.dot(nrm) * 1e4).round();
        let oy = (y.0.dot(nrm) * 1e4).round();
        ox.total_cmp(&oy)
            .then(x.0.dot(dir).total_cmp(&y.0.dot(dir)))
    });
    let mut out = Vec::with_capacity(segs.len());
    let mut flip = false;
    let mut last_off = f64::NAN;
    let mut row: Vec<(DVec2, DVec2)> = Vec::new();
    let flush = |row: &mut Vec<(DVec2, DVec2)>, flip: bool, out: &mut Vec<(DVec2, DVec2)>| {
        if flip {
            out.extend(row.drain(..).rev().map(|(a, b)| (b, a)));
        } else {
            out.append(row);
        }
    };
    for s in segs {
        let off = (s.0.dot(nrm) * 1e4).round();
        if off != last_off && !row.is_empty() {
            flush(&mut row, flip, &mut out);
            if zig {
                flip = !flip;
            }
        }
        last_off = off;
        row.push(s);
    }
    flush(&mut row, flip, &mut out);
    out
}

fn segment_inside(r: &Region, a: DVec2, b: DVec2) -> bool {
    region::covers_segment(r, a, b)
}

fn facing(g: &Gen, stepover: f64, angle: f64, overlap: f64, zig: bool) -> Result<Vec<Move>> {
    let s = &g.project.stock;
    let min = s.min.truncate() - DVec2::splat(overlap);
    let max = s.max.truncate() + DVec2::splat(overlap);
    let area = region::rect(min, max);
    let dir = DVec2::from_angle(angle.to_radians());
    let segs = zigzag(
        region::clip_lines(&area, &raster_lines(min, max, stepover, angle)),
        dir,
        zig,
    );
    let mut b = g.builder();
    for z in depth_levels(g.c.heights.top, final_z(g), g.c.stepdown) {
        crate::workflow::check_cancel()?;
        b.retract();
        for (i, (p0, p1)) in segs.iter().enumerate() {
            b.go_to(p0.extend(z), zig && i > 0);
            b.feed(p1.extend(z));
        }
    }
    b.finish()
}

/// Insert vertices at arc-lengths in `cuts` and raise points inside tab intervals.
fn apply_tabs(lp: &[DVec2], z: f64, tabs: &Tabs, tool_r: f64, tab_top: f64) -> Vec<DVec3> {
    let n = lp.len();
    let perim: f64 = (0..n).map(|i| lp[i].distance(lp[(i + 1) % n])).sum();
    if z >= tab_top || tabs.count == 0 || perim < 1e-6 {
        let mut v: Vec<DVec3> = lp.iter().map(|p| p.extend(z)).collect();
        v.push(lp[0].extend(z));
        return v;
    }
    let half = tabs.width * 0.5 + tool_r;
    let spacing = perim / tabs.count as f64;
    let intervals: Vec<(f64, f64)> = (0..tabs.count)
        .map(|k| {
            let c = (k as f64 + 0.5) * spacing;
            (c - half, c + half)
        })
        .collect();
    let in_tab = |s: f64| intervals.iter().any(|&(a, b)| s > a + 1e-9 && s < b - 1e-9);
    let mut cuts: Vec<f64> = intervals
        .iter()
        .flat_map(|&(a, b)| [a, b])
        .filter(|s| *s > 0.0 && *s < perim)
        .collect();
    cuts.sort_by(f64::total_cmp);

    let mut out = vec![lp[0].extend(if in_tab(0.0) { tab_top } else { z })];
    let mut s0 = 0.0;
    let mut ci = 0;
    for i in 0..n {
        let (a, b) = (lp[i], lp[(i + 1) % n]);
        let l = a.distance(b);
        while ci < cuts.len() && cuts[ci] < s0 + l {
            let t = (cuts[ci] - s0) / l.max(1e-12);
            let p = a.lerp(b, t);
            // Step vertically at the tab boundary.
            let before = if in_tab(cuts[ci] - 1e-6) { tab_top } else { z };
            let after = if in_tab(cuts[ci] + 1e-6) { tab_top } else { z };
            out.push(p.extend(before));
            out.push(p.extend(after));
            ci += 1;
        }
        s0 += l;
        let sb = if i + 1 == n { 0.0 } else { s0 };
        out.push(b.extend(if in_tab(sb) { tab_top } else { z }));
    }
    out
}

fn lead_arc(
    b: &mut PathBuilder,
    lp: &[DVec2],
    z: f64,
    radius: f64,
    climb: bool,
    entering: bool,
    allowed: &Region,
) -> Result<()> {
    // Waste side is left of travel for climb-oriented loops, right for conventional.
    let (p, t) = if entering {
        (lp[0], (lp[1] - lp[0]).normalize_or_zero())
    } else {
        (lp[0], (lp[0] - lp[lp.len() - 1]).normalize_or_zero())
    };
    let n = if climb { t.perp() } else { -t.perp() };
    let c = p + n * radius;
    let cw = !climb;
    let (a, e) = if entering {
        (c - t * radius, p)
    } else {
        (p, c + t * radius)
    };
    let mut previous = a;
    for next in crate::toolpath::arc_points(a.extend(z), e.extend(z), c, cw, 0.001) {
        anyhow::ensure!(
            region::covers_segment(allowed, previous, next.truncate()),
            "Profile lead enters retained material; reduce lead radius or change start"
        );
        previous = next.truncate();
    }
    if entering {
        let a = c - t * radius;
        b.go_to(a.extend(z), false);
        b.arc(p.extend(z), c, cw);
    } else {
        let e = c + t * radius;
        b.arc(e.extend(z), c, cw);
    }
    Ok(())
}

fn profile(
    g: &Gen,
    side: Side,
    finish_passes: u32,
    finish_step: f64,
    spring: bool,
    lead: f64,
    tabs: Option<Tabs>,
) -> Result<Vec<Move>> {
    let r = g.tool.radius();
    let curves = g.project.select_curves(&g.c.curves);
    region::validate_chains(&curves, side != Side::On)?;
    anyhow::ensure!(
        side != Side::On || (finish_passes == 0 && g.c.stock_xy == 0.0 && tabs.is_none()),
        "On-curve profiles do not support radial allowance, offset finish passes or tabs"
    );
    // Offset distances from roughest to final.
    let mut dists: Vec<f64> = (0..=finish_passes)
        .rev()
        .map(|k| g.c.stock_xy + k as f64 * finish_step)
        .collect();
    if spring {
        dists.push(*dists.last().unwrap());
    }

    // Each entry: a list of loops (closed) for each offset distance, plus open polylines.
    let mut closed_sets: Vec<Vec<Vec<DVec2>>> = Vec::new();
    let mut open: Vec<Vec<DVec2>> = Vec::new();
    match side {
        Side::On => {
            let mut set = Vec::new();
            for c in &curves {
                if c.closed {
                    let pl = if c.is_ccw() == g.c.climb {
                        (*c).clone()
                    } else {
                        c.reversed()
                    };
                    set.push(pl.pts);
                } else {
                    open.push(c.pts.clone());
                }
            }
            closed_sets.push(set.clone());
            if spring {
                closed_sets.push(set);
            }
        }
        Side::Outside | Side::Inside => {
            let reg = region::validated(&curves)?;
            if region::is_empty(&reg) {
                bail!("profile {side:?} needs closed curves");
            }
            for d in &dists {
                let off = if side == Side::Outside {
                    r + d
                } else {
                    -(r + d)
                };
                let loops = region::loops(
                    &region::offset(&reg, off),
                    if side == Side::Outside {
                        !g.c.climb
                    } else {
                        g.c.climb
                    },
                );
                anyhow::ensure!(!loops.is_empty(), "Tool and allowance do not fit profile");
                closed_sets.push(loops);
            }
        }
    }

    let reg = if side == Side::On {
        Region::empty()
    } else {
        region::validated(&curves)?
    };
    let lead_allowed = if side == Side::Inside {
        region::offset(&reg, -(r + g.c.stock_xy - 0.002).max(0.))
    } else {
        let bounds = region::offset(&reg, r + lead * 4. + 1000.);
        region::difference(
            &bounds,
            &region::offset(&reg, (r + g.c.stock_xy - 0.002).max(0.)),
        )
    };
    let zf = final_z(g);
    let tab_top = zf + tabs.map(|t| t.height).unwrap_or(0.0);
    let levels = depth_levels(g.c.heights.top, zf, g.c.stepdown);
    let mut b = g.builder();
    let ramp = g.ramp_angle();
    let mut z_prev = g.c.heights.top;
    for &z in &levels {
        for set in &closed_sets {
            for lp in set {
                let lp = lp.clone();
                if let Some(t) = tabs.filter(|_| z < tab_top) {
                    let pts = apply_tabs(&lp, z, &t, r, tab_top);
                    b.polyline(&pts, false);
                } else if lead > 0.0 && side != Side::On {
                    lead_arc(&mut b, &lp, z, lead, g.c.climb, true, &lead_allowed)?;
                    for p in lp[1..].iter().chain(std::iter::once(&lp[0])) {
                        b.feed(p.extend(z));
                    }
                    lead_arc(&mut b, &lp, z, lead, g.c.climb, false, &lead_allowed)?;
                    b.retract();
                } else {
                    b.ramp_loop(&lp, z_prev, z, ramp, false);
                }
            }
        }
        for pl in &open {
            b.ramp_open(pl, z_prev, z, ramp);
            if spring {
                b.ramp_open(pl, z, z, 0.);
            }
        }
        z_prev = z;
    }
    b.finish()
}

/// One separate pocket: its tool-centre region and precomputed pattern.
struct PocketArea {
    centre: Region,
    /// Offset pattern: rings (outermost first, or reversed for inside-out), each a list of loops.
    rings: Vec<Vec<Vec<DVec2>>>,
    /// Zigzag pattern: ordered raster segments plus the boundary cleanup loops.
    segs: Vec<(DVec2, DVec2)>,
    boundary: Vec<Vec<DVec2>>,
}

fn pocket(
    g: &Gen,
    pattern: PocketPattern,
    stepover: f64,
    finish: bool,
    inside_out: bool,
    order: CutOrder,
) -> Result<Vec<Move>> {
    let r = g.tool.radius();
    let reg = selected_region(g)?;
    let stepover = stepover.clamp(0.01, g.tool.diameter);

    // Each outer boundary (with its islands) is a separate pocket.
    let mut areas = Vec::new();
    for poly in &reg.0 {
        let centre = region::offset(
            &geo::MultiPolygon::new(vec![poly.clone()]),
            -(r + g.c.stock_xy),
        );
        if region::is_empty(&centre) {
            bail!("Tool cannot reach one of the selected pocket regions; use a smaller tool or repair the selection");
        }
        let mut area = PocketArea {
            centre,
            rings: vec![],
            segs: vec![],
            boundary: vec![],
        };
        match pattern {
            PocketPattern::Offset => {
                area.rings = region::concentric(&area.centre, stepover, 10_000)
                    .iter()
                    .map(|r| region::loops(r, g.c.climb))
                    .collect();
                if inside_out {
                    area.rings.reverse();
                }
            }
            PocketPattern::ZigZag { angle } => {
                let bb = bounds_of(&area.centre);
                let dir = DVec2::from_angle(angle.to_radians());
                let inner = region::offset(&area.centre, -1e-3);
                area.segs = zigzag(
                    region::clip_lines(&inner, &raster_lines(bb.0, bb.1, stepover, angle)),
                    dir,
                    true,
                );
                area.boundary = region::loops(&area.centre, g.c.climb);
            }
        }
        areas.push(area);
    }
    if areas.is_empty() {
        bail!("tool is too large for the pocket");
    }
    // Visit pockets nearest-first.
    let mut ordered = Vec::with_capacity(areas.len());
    let mut here = DVec2::ZERO;
    while !areas.is_empty() {
        let dist = |a: &PocketArea| {
            let (lo, hi) = bounds_of(&a.centre);
            ((lo + hi) * 0.5).distance_squared(here)
        };
        let i = (0..areas.len())
            .min_by(|&a, &b| dist(&areas[a]).total_cmp(&dist(&areas[b])))
            .unwrap();
        let a = areas.swap_remove(i);
        let (lo, hi) = bounds_of(&a.centre);
        here = (lo + hi) * 0.5;
        ordered.push(a);
    }

    let ramp = g.ramp_angle();
    let top = g.c.heights.top;
    let zf = final_z(g);
    let levels = depth_levels(top, zf, g.c.stepdown);
    let mut b = g.builder();
    let finish_area = |b: &mut PathBuilder, a: &PocketArea| {
        b.retract();
        for lp in region::loops(&a.centre, g.c.climb) {
            b.ramp_loop(&lp, zf, zf, 0.0, false);
        }
    };
    match order {
        CutOrder::LayerFirst => {
            let mut z_prev = top;
            for &z in &levels {
                for a in &ordered {
                    pocket_level(&mut b, a, z, z_prev, stepover, ramp);
                }
                z_prev = z;
            }
            if finish {
                for a in &ordered {
                    finish_area(&mut b, a);
                }
            }
        }
        CutOrder::DepthFirst => {
            for a in &ordered {
                let mut z_prev = top;
                for &z in &levels {
                    pocket_level(&mut b, a, z, z_prev, stepover, ramp);
                    z_prev = z;
                }
                if finish {
                    finish_area(&mut b, a);
                }
            }
        }
    }
    b.finish()
}

/// Clear one pocket at depth `z`, entering from `z_prev` (the previous level's floor).
fn pocket_level(
    b: &mut PathBuilder,
    a: &PocketArea,
    z: f64,
    z_prev: f64,
    stepover: f64,
    ramp: f64,
) {
    b.retract();
    b.policy.centre_region = Some(a.centre.clone());
    for ring in &a.rings {
        for lp in ring {
            let mut pl = Polyline2 {
                pts: lp.clone(),
                closed: true,
            };
            if let Some(p) = b.pos {
                pl.start_nearest(p.truncate());
            }
            let close = b.pos.is_some_and(|p| {
                (p.z - z).abs() < 1e-9
                    && p.truncate().distance(pl.pts[0]) <= stepover * 1.5
                    && segment_inside(&a.centre, p.truncate(), pl.pts[0])
            });
            if close {
                b.ramp_loop(&pl.pts, z, z, 0.0, true);
            } else {
                b.ramp_loop(&pl.pts, z_prev, z, ramp, false);
            }
        }
    }
    for (i, (p0, p1)) in a.segs.iter().enumerate() {
        let linked = i > 0
            && b.pos.is_some_and(|p| {
                p.truncate().distance(*p0) <= stepover * 1.5
                    && segment_inside(&a.centre, p.truncate(), *p0)
            });
        if linked {
            b.feed(p0.extend(z));
        } else {
            // Ramp along the segment itself when entering fresh material.
            b.ramp_loop(&[*p0, *p1], z_prev, z, ramp, false);
            b.feed(p0.extend(z));
        }
        b.feed(p1.extend(z));
    }
    for lp in &a.boundary {
        b.ramp_loop(lp, z, z, 0.0, false);
    }
}

fn bounds_of(r: &Region) -> (DVec2, DVec2) {
    let mut lo = DVec2::splat(f64::INFINITY);
    let mut hi = DVec2::splat(f64::NEG_INFINITY);
    for p in &r.0 {
        for c in &p.exterior().0 {
            lo = lo.min(DVec2::new(c.x, c.y));
            hi = hi.max(DVec2::new(c.x, c.y));
        }
    }
    (lo, hi)
}

/// Detect a closed polyline that approximates a circle: (centre, diameter).
pub fn as_circle(c: &Polyline2) -> Option<(DVec2, f64)> {
    if !c.closed || c.pts.len() < 6 {
        return None;
    }
    let a = c.pts[0];
    let b = *c
        .pts
        .iter()
        .max_by(|x, y| x.distance_squared(a).total_cmp(&y.distance_squared(a)))?;
    let d = *c.pts.iter().max_by(|x, y| {
        (b - a)
            .perp_dot(**x - a)
            .abs()
            .total_cmp(&(b - a).perp_dot(**y - a).abs())
    })?;
    let u = b - a;
    let v = d - a;
    let det = 2.0 * u.perp_dot(v);
    if det.abs() < 1e-12 {
        return None;
    }
    let centre = a + DVec2::new(
        v.y * u.length_squared() - u.y * v.length_squared(),
        u.x * v.length_squared() - v.x * u.length_squared(),
    ) / det;
    let radius = centre.distance(a);
    c.pts
        .iter()
        .all(|p| (p.distance(centre) - radius).abs() <= 0.01)
        .then_some((centre, radius * 2.0))
}

fn nearest_order(mut pts: Vec<DVec2>) -> Vec<DVec2> {
    let mut out = Vec::with_capacity(pts.len());
    let mut cur = DVec2::ZERO;
    while !pts.is_empty() {
        let i = pts
            .iter()
            .enumerate()
            .min_by(|a, b| {
                a.1.distance_squared(cur)
                    .total_cmp(&b.1.distance_squared(cur))
            })
            .unwrap()
            .0;
        cur = pts.swap_remove(i);
        out.push(cur);
    }
    out
}

fn circles_in_range(g: &Gen, min_d: f64, max_d: f64) -> Vec<(DVec2, f64)> {
    g.project
        .select_curves(&g.c.curves)
        .into_iter()
        .filter_map(as_circle)
        .filter(|(_, d)| *d >= min_d && *d <= max_d)
        .collect()
}

fn drill(
    g: &Gen,
    cycle: crate::toolpath::DrillCycle,
    peck: f64,
    dwell: f64,
    source: DrillSource,
    tip_comp: bool,
    breakthrough: f64,
) -> Result<Vec<Move>> {
    let pts: Vec<DVec2> = match source {
        DrillSource::CircleCenters {
            min_diameter,
            max_diameter,
        } => circles_in_range(g, min_diameter, max_diameter)
            .into_iter()
            .map(|c| c.0)
            .collect(),
        DrillSource::CurveStarts => g
            .project
            .select_curves(&g.c.curves)
            .iter()
            .filter_map(|c| c.pts.first().copied())
            .collect(),
    };
    if pts.is_empty() {
        bail!("no drill points found in the selection");
    }
    let tip = if tip_comp {
        g.tool.profile_height(g.tool.radius()).unwrap_or(0.0)
    } else {
        0.0
    };
    let bottom = final_z(g) - tip - breakthrough;
    if !g.project.checks.allow_depth_beyond_flute {
        anyhow::ensure!(
            g.c.heights.top - bottom <= g.tool.flute_length,
            "Drill depth including tip and breakthrough exceeds flute length (Settings can allow this with a warning)"
        );
    }
    let mut b = g.builder();
    for p in nearest_order(pts) {
        b.rapid(p.extend(g.links().safe_z));
        b.moves.push(Move::Drill {
            at: p,
            r_plane: g.links().feed_z,
            bottom,
            clearance: g.links().safe_z,
            peck,
            dwell,
            cycle,
            feed: g.c.feeds.plunge,
        });
        b.pos = Some(p.extend(g.links().safe_z));
    }
    b.finish()
}

fn engrave(g: &Gen) -> Result<Vec<Move>> {
    let curves = g.project.select_curves(&g.c.curves);
    if curves.is_empty() {
        bail!("no curves selected");
    }
    region::validate_chains(&curves, false)?;
    let mut b = g.builder();
    let ramp = g.ramp_angle();
    let mut z_prev = g.c.heights.top;
    for z in depth_levels(g.c.heights.top, final_z(g), g.c.stepdown) {
        crate::workflow::check_cancel()?;
        for c in &curves {
            if c.closed {
                b.ramp_loop(&c.pts, z_prev, z, ramp, false);
            } else {
                b.ramp_open(&c.pts, z_prev, z, ramp);
            }
        }
        z_prev = z;
    }
    b.finish()
}

/// True V-carve: the tool centre follows the region's medial axis, and at every point its depth
/// is set so the bit just touches the nearest boundary. In narrow gaps the bit rises, all the
/// way to the surface at sharp corners. Where the region is wider than the bit can reach at
/// `max_depth`, the cut bottoms out and the flat area's outline is cut at full depth.
/// `step` is the boundary sampling resolution.
fn vcarve(g: &Gen, max_depth: f64, step: f64) -> Result<Vec<Move>> {
    let reg = selected_region(g)?;
    let mut max_depth = max_depth.min(g.c.heights.top - final_z(g));
    if !g.project.checks.allow_depth_beyond_flute {
        max_depth = max_depth.min(g.tool.flute_length);
    }
    anyhow::ensure!(max_depth > 0., "V-carve depth must be positive");
    let top = g.c.heights.top;
    let tool = &g.tool;
    // Radius at which the bit reaches max depth (or its full radius, if it's too short).
    let rad = tool.radius();
    let d_cap = if tool.profile_height(rad).unwrap_or(0.0) <= max_depth {
        rad
    } else {
        let (mut lo, mut hi) = (0.0, rad);
        for _ in 0..60 {
            let mid = 0.5 * (lo + hi);
            if tool.profile_height(mid).unwrap_or(f64::INFINITY) < max_depth {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        lo
    };
    if d_cap <= 1e-6 {
        bail!("tool T{} has no V/taper to carve with", tool.number);
    }
    let z_at = |r: f64| {
        top - tool
            .profile_height(r.clamp(0.0, d_cap))
            .unwrap_or(max_depth)
            .min(max_depth)
    };
    let ma = medial::medial_axis(&reg, step.max(0.01));

    // Medial-axis runs where the bit is still tangent to the walls (r <= d_cap); beyond that
    // the flat-area outline below takes over. Split runs at the exact crossing.
    let mut runs: Vec<Vec<DVec3>> = Vec::new();
    for ch in &ma.chains {
        let rs: Vec<f64> = ch
            .iter()
            .map(|p| ma.boundary.dist(*p).unwrap_or(0.0))
            .collect();
        let mut cur: Vec<DVec3> = Vec::new();
        for i in 0..ch.len() {
            let inside = rs[i] <= d_cap + 1e-9;
            if i > 0 && inside != (rs[i - 1] <= d_cap + 1e-9) {
                let t = (d_cap - rs[i - 1]) / (rs[i] - rs[i - 1]);
                cur.push(ch[i - 1].lerp(ch[i], t).extend(z_at(d_cap)));
                if !inside {
                    runs.push(std::mem::take(&mut cur));
                }
            }
            if inside {
                cur.push(ch[i].extend(z_at(rs[i])));
            }
        }
        runs.push(cur);
    }
    let tol = (g.c.tolerance * 0.5).clamp(1e-4, 0.01);
    let mut runs: Vec<Vec<DVec3>> = runs
        .into_iter()
        .filter(|r| r.len() >= 2)
        .map(|r| crate::toolpath::simplify(&r, tol))
        .collect();

    let mut b = g.builder();
    // Outline of the flat-bottomed area, at full depth.
    let floor = region::offset(&reg, -d_cap);
    for lp in region::loops(&floor, g.c.climb) {
        let mut pl = Polyline2 {
            pts: lp,
            closed: true,
        };
        if let Some(p) = b.pos {
            pl.start_nearest(p.truncate());
        }
        let z = z_at(d_cap);
        b.go_to(pl.pts[0].extend(z), false);
        for p in pl.pts[1..].iter().chain(std::iter::once(&pl.pts[0])) {
            b.feed(p.extend(z));
        }
    }
    // Medial-axis runs, nearest end first; runs meeting at a junction continue without lifting.
    while !runs.is_empty() {
        let here = b.pos.unwrap_or(DVec3::new(0.0, 0.0, top));
        let (i, rev) = (0..runs.len())
            .flat_map(|i| [(i, false), (i, true)])
            .min_by(|x, y| {
                let e = |(i, r): (usize, bool)| {
                    if r {
                        *runs[i].last().unwrap()
                    } else {
                        runs[i][0]
                    }
                };
                e(*x).distance(here).total_cmp(&e(*y).distance(here))
            })
            .unwrap();
        let mut run = runs.swap_remove(i);
        if rev {
            run.reverse();
        }
        b.polyline(&run, true);
    }
    let full = b.finish()?;
    let mut moves = Vec::new();
    for level in depth_levels(top, top - max_depth, g.c.stepdown) {
        crate::workflow::check_cancel()?;
        for m in &full {
            let mut m = m.clone();
            match &mut m {
                Move::Linear { to, .. } | Move::Arc { to, .. } => to.z = to.z.max(level),
                _ => {}
            }
            moves.push(m);
        }
    }
    Ok(moves)
}

fn helical_bore(g: &Gen, min_d: f64, max_d: f64, pitch: f64, stepover: f64) -> Result<Vec<Move>> {
    let holes = circles_in_range(g, min_d, max_d);
    if holes.is_empty() {
        bail!("no circles in the diameter range");
    }
    let r = g.tool.radius();
    let zf = final_z(g);
    let pitch = pitch.max(0.01);
    let ccw = g.c.climb; // inside a hole, climb with a CW spindle is CCW
    let mut b = g.builder();
    for (c, d) in nearest_order(holes.iter().map(|h| h.0).collect())
        .into_iter()
        .filter_map(|p| holes.iter().find(|h| h.0 == p).copied())
    {
        let rp = d * 0.5 - r - g.c.stock_xy;
        if rp <= 1e-3 {
            bail!("hole Ø{d:.3} is too small for tool Ø{:.3}", g.tool.diameter);
        }
        let mut radii = Vec::new();
        let mut ri = rp.min(stepover.max(0.01));
        while ri < rp - 1e-9 {
            radii.push(ri);
            ri += stepover.max(0.01);
        }
        radii.push(rp);
        for ri in radii {
            b.retract();
            b.rapid(c.extend(g.links().safe_z));
            b.rapid(c.extend(g.links().feed_z));
            let start = c + DVec2::new(ri, 0.0);
            b.feed_at(start.extend(g.links().feed_z), g.c.feeds.plunge);
            // Helix down in half turns.
            let mut z = g.links().feed_z;
            let mut angle = 0.0;
            while z > zf + 1e-9 {
                crate::workflow::check_cancel()?;
                let dz = (z - zf).min(pitch * 0.5);
                z -= dz;
                angle += (if ccw { 1.0 } else { -1.0 }) * std::f64::consts::TAU * dz / pitch;
                b.arc((c + DVec2::from_angle(angle) * ri).extend(z), c, !ccw);
            }
            // Full flat circle at the bottom.
            let here = b.pos.unwrap().truncate();
            let opposite = c * 2.0 - here;
            b.arc(opposite.extend(zf), c, !ccw);
            b.arc(here.extend(zf), c, !ccw);
            b.feed(c.extend(zf));
            b.retract();
        }
    }
    b.finish()
}

fn thread_mill(
    g: &Gen,
    pitch: f64,
    internal: bool,
    major: f64,
    right_hand: bool,
) -> Result<Vec<Move>> {
    let holes = circles_in_range(g, 0.0, f64::INFINITY);
    if holes.is_empty() {
        bail!("select circles for thread milling");
    }
    if pitch <= 0.0 {
        bail!("pitch must be positive");
    }
    let r = g.tool.radius();
    let h = g.c.heights;
    let zf = final_z(g);
    let mut b = g.builder();
    for (c, d) in holes {
        let dia = if major > 0.0 { major } else { d };
        let rp = if internal {
            dia * 0.5 - r
        } else {
            dia * 0.5 + r
        };
        if rp <= 0.0 {
            bail!(
                "thread Ø{dia:.3} is too small for tool Ø{:.3}",
                g.tool.diameter
            );
        }
        // CCW while rising (or CW while falling) makes a right-hand helix.
        let (ccw, upward) = match (internal, right_hand) {
            (true, true) => (true, true),
            (true, false) => (true, false),
            (false, true) => (false, false),
            (false, false) => (false, true),
        };
        let (z0, z1) = if upward { (zf, h.top) } else { (h.top, zf) };
        let start = c + DVec2::new(rp, 0.0);
        let approach = if internal {
            c
        } else {
            c + DVec2::new(rp + r + 1.0, 0.0)
        };
        b.retract();
        b.rapid(approach.extend(g.links().safe_z));
        b.rapid(approach.extend(g.links().feed_z));
        b.feed_at(approach.extend(z0), g.c.feeds.plunge);
        b.feed(start.extend(z0));
        let mut z = z0;
        let mut angle = 0.0;
        while (z1 - z).abs() > 1e-9 {
            crate::workflow::check_cancel()?;
            let dz = (z1 - z).abs().min(pitch * 0.5);
            z += if upward { dz } else { -dz };
            angle += (if ccw { 1.0 } else { -1.0 }) * std::f64::consts::TAU * dz / pitch;
            b.arc((c + DVec2::from_angle(angle) * rp).extend(z), c, !ccw);
        }
        let exit = if internal {
            c
        } else {
            c + (b.pos.unwrap().truncate() - c).normalize() * (rp + r + 1.0)
        };
        b.feed(exit.extend(z));
        b.retract();
    }
    b.finish()
}

/// Radius at which the tool profile reaches height `h` above the tip.
fn radius_at_height(tool: &warp_tools::Tool, h: f64) -> Option<f64> {
    let rmax = tool.radius();
    if tool.profile_height(rmax)? < h {
        return None;
    }
    let (mut lo, mut hi) = (0.0, rmax);
    for _ in 0..60 {
        let mid = (lo + hi) * 0.5;
        if tool.profile_height(mid).unwrap_or(f64::INFINITY) < h {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    Some(hi)
}

fn chamfer(g: &Gen, width: f64, extra: f64, side: Side) -> Result<Vec<Move>> {
    use warp_tools::ToolShape;
    let (angle, tip) = match g.tool.shape {
        ToolShape::VBit {
            included_angle,
            tip_diameter,
        }
        | ToolShape::Engraver {
            included_angle,
            tip_diameter,
        } => (included_angle, tip_diameter * 0.5),
        ToolShape::ChamferMill {
            angle,
            tip_diameter,
        } => (angle, tip_diameter * 0.5),
        ToolShape::SpotDrill { point_angle } | ToolShape::Drill { point_angle } => {
            (point_angle, 0.)
        }
        _ => bail!("Chamfer requires a conical cutter"),
    };
    anyhow::ensure!(angle > 0. && angle < 180., "Invalid cutter included angle");
    let depth = width / (angle.to_radians() * 0.5).tan() + extra;
    if !g.project.checks.allow_depth_beyond_flute {
        anyhow::ensure!(depth <= g.tool.flute_length, "Chamfer exceeds flute length (Settings can allow this with a warning)");
    }
    let rd = radius_at_height(g.tool, depth)
        .ok_or_else(|| anyhow::anyhow!("chamfer is too large for this tool"))?;
    let off = rd - width;
    anyhow::ensure!(side != Side::On || tip<=1e-9 && extra<=1e-9, "On-curve chamfer requires a pointed tip and zero extra depth; choose Inside/Outside for a flat tip");
    let z = g.c.heights.top - depth;
    let curves = g.project.select_curves(&g.c.curves);
    region::validate_chains(&curves, true)?;
    let loops = match side {
        Side::On => curves
            .iter()
            .filter(|c| c.closed)
            .map(|c| c.pts.clone())
            .collect::<Vec<_>>(),
        Side::Outside | Side::Inside => {
            let reg = region::validated(&curves)?;
            let d = if side == Side::Outside { off } else { -off };
            region::loops(&region::offset(&reg, d), g.c.climb)
        }
    };
    anyhow::ensure!(!loops.is_empty(), "Chamfer offset is unreachable");
    let mut b = g.builder();
    for lp in loops {
        b.ramp_loop(&lp, z, z, 0.0, false);
    }
    b.finish()
}

fn hole_profile(g: &Gen, pitch: f64, spring: bool) -> Result<Vec<Move>> {
    let holes = circles_in_range(g, 0., f64::MAX);
    anyhow::ensure!(!holes.is_empty(), "Select circular hole boundaries");
    let mut b = g.builder();
    let zf = final_z(g);
    for (center, diameter) in holes {
        let r = diameter * 0.5 - g.tool.radius() - g.c.stock_xy;
        anyhow::ensure!(r > g.c.tolerance, "Tool does not fit hole");
        let start = center + DVec2::X * r;
        b.go_to(start.extend(g.c.heights.top), false);
        let mut z = g.c.heights.top;
        let mut angle: f64 = 0.;
        while z > zf + 1e-9 {
            crate::workflow::check_cancel()?;
            let dz = (z - zf).min(pitch * 0.5);
            z -= dz;
            angle += (if g.c.climb { 1. } else { -1. }) * std::f64::consts::TAU * dz / pitch;
            b.arc(
                (center + DVec2::from_angle(angle) * r).extend(z),
                center,
                !g.c.climb,
            );
        }
        for _ in 0..if spring { 2 } else { 1 } {
            let at = b.pos.unwrap();
            b.arc((center * 2. - at.truncate()).extend(zf), center, !g.c.climb);
            b.arc(at, center, !g.c.climb);
        }
        b.retract();
    }
    b.finish()
}
