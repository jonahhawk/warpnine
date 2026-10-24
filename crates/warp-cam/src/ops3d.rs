//! 3D surface operations driven by cutter-contact queries (exact mesh contact or the Z-map).

use crate::ops::{depth_levels, OpKind, SurfaceMode};
use crate::region::{self, Region};
use crate::surface::{self, Cutter, MeshSurface, Sampling, SurfaceQuery, ZMapSurface};
use crate::toolpath::{simplify, Move, PathBuilder};
use crate::zmap::{Kernel, ZMap};
use crate::Gen;
use anyhow::{bail, Result};
use geo::BooleanOps;
use glam::{DVec2, DVec3};
use rayon::prelude::*;
use warp_core::Polyline2;

enum Surface {
    Exact(MeshSurface),
    Grid { zmap: ZMap, kernel: Kernel },
}

struct Setup {
    surface: Surface,
    cutter: Cutter,
    sampling: Sampling,
    /// Grid cell for level maps and the Z-map.
    cell: f64,
    /// Area the tool centre may cover.
    area: Region,
    min: DVec2,
    max: DVec2,
    /// Padded extent of the contact maps.
    map_min: DVec2,
    map_max: DVec2,
}

impl Setup {
    fn query(&self) -> Box<dyn SurfaceQuery + '_> {
        match &self.surface {
            Surface::Exact(m) => Box::new(m),
            Surface::Grid { zmap, kernel } => Box::new(ZMapSurface { map: zmap, kernel }),
        }
    }

    /// Tool-tip positions along a 2D polyline, every point on the contact surface.
    fn drop_path(&self, pts: &[DVec2]) -> Vec<DVec3> {
        let q = self.query();
        match &self.surface {
            Surface::Exact(_) => surface::drop_polyline(q.as_ref(), &self.cutter, pts, &self.sampling).pts,
            Surface::Grid { zmap, kernel } => zmap
                .drop_path(kernel, pts)
                .into_iter()
                .map(|p| DVec3::new(p.x, p.y, p.z.max(self.sampling.floor)))
                .collect(),
        }
    }

    /// Cutter-location map over the padded extent at the operation's cell size.
    fn cl_map(&self) -> ZMap {
        match &self.surface {
            Surface::Exact(m) => surface::cl_map(m, &self.cutter, self.map_min, self.map_max, self.cell, self.sampling.floor),
            Surface::Grid { zmap, kernel } => zmap.cl_map(kernel),
        }
    }
}


fn setup(g: &Gen) -> Result<Setup> {
    let Some(mesh) = &g.project.mesh else {
        bail!("3D operations need a model (import an STL)")
    };
    let r = g.tool.radius();
    let cell = g.c.resolution.clamp(0.01, 5.0);
    let boundary: Vec<&Polyline2> = if g.project.execution_resolved {
        g.project
            .execution_boundary
            .as_ref()
            .map(|v| v.iter().collect())
            .unwrap_or_default()
    } else {
        g.project.select_curves(&g.c.curves)
    };
    let stock_boundary = if g.project.execution_resolved {
        g.project.execution_boundary.is_none()
    } else {
        g.c.curves.is_empty()
    };
    let mut area = if stock_boundary {
        // Whole stock; let the tool centre run off the edges by a radius to clean them.
        let s = &g.project.stock;
        region::rect(
            s.min.truncate() - DVec2::splat(r),
            s.max.truncate() + DVec2::splat(r),
        )
    } else {
        let reg = region::from_curves(&boundary);
        if region::is_empty(&reg) {
            bail!("boundary curves must be closed");
        }
        reg
    };
    if !g.project.execution_avoid.is_empty() {
        let avoid = region::from_curves(&g.project.execution_avoid.iter().collect::<Vec<_>>());
        area = area.difference(&region::offset(&avoid, r + g.c.stock_xy.max(0.0)));
    }
    if region::is_empty(&area) {
        bail!("containment and avoid regions leave no machining area");
    }
    let (mut min, mut max) = (DVec2::splat(f64::INFINITY), DVec2::splat(f64::NEG_INFINITY));
    for p in &area.0 {
        for c in &p.exterior().0 {
            min = min.min(DVec2::new(c.x, c.y));
            max = max.max(DVec2::new(c.x, c.y));
        }
    }
    let pad = DVec2::splat(r + g.c.stock_xy + 2.0 * cell);
    let cutter = Cutter::new(g.tool, g.c.stock_xy, g.c.stock_z);
    let sampling = Sampling::new(&cutter, cell, g.c.tolerance, g.c.heights.bottom);
    let surface = match g.c.surface.resolve(&g.op.kind) {
        SurfaceMode::Exact => Surface::Exact(MeshSurface::new(mesh)),
        _ => Surface::Grid {
            zmap: ZMap::from_mesh(mesh, min - pad, max + pad, cell, g.c.heights.bottom),
            kernel: Kernel::new(g.tool, cell, g.c.stock_xy, g.c.stock_z),
        },
    };
    crate::workflow::check_cancel()?;
    Ok(Setup {
        surface,
        cutter,
        sampling,
        cell,
        area,
        min,
        max,
        map_min: min - pad,
        map_max: max + pad,
    })
}

pub fn generate(g: &Gen) -> Result<Vec<Move>> {
    let s = setup(g)?;
    match &g.op.kind {
        OpKind::Parallel {
            stepover,
            angle,
            zigzag,
        } => parallel(g, &s, *stepover, *angle, *zigzag),
        OpKind::Spiral { stepover } => spiral(g, &s, *stepover),
        OpKind::Radial { angle_step } => radial(g, &s, *angle_step),
        OpKind::ProjectCurves { depth } => project(g, &s, *depth),
        OpKind::Waterline { stepdown } => waterline(g, &s, *stepdown),
        OpKind::Roughing3D {
            stepover,
            finish_contour,
        } => roughing(g, &s, *stepover, *finish_contour),
        _ => bail!("not a 3D operation"),
    }
}

/// Split a 2D polyline into runs that lie inside the area.
fn clip_polyline(area: &Region, pts: &[DVec2]) -> Vec<Vec<DVec2>> {
    let mut out = Vec::new();
    let mut cur: Vec<DVec2> = Vec::new();
    for pair in pts.windows(2) {
        let (a, b) = (pair[0], pair[1]);
        let mut pieces = region::clip_lines(area, &[(a, b)]);
        for (p, q) in &mut pieces {
            if p.distance_squared(a) > q.distance_squared(a) {
                std::mem::swap(p, q);
            }
        }
        pieces.sort_by(|x, y| x.0.distance_squared(a).total_cmp(&y.0.distance_squared(a)));
        if pieces.is_empty() && cur.len() > 1 {
            out.push(std::mem::take(&mut cur));
        }
        for (p, q) in pieces {
            if cur.last().is_some_and(|last| last.distance(p) > 1e-8) {
                out.push(std::mem::take(&mut cur));
            }
            if cur.is_empty() {
                cur.push(p);
            }
            cur.push(q);
        }
    }
    if cur.len() > 1 {
        out.push(cur);
    }
    out.retain(|r| r.len() > 1);
    out
}

/// Emit 3D cut runs, linking consecutive runs along the surface when they are close.
fn emit_runs(g: &Gen, s: &Setup, runs: Vec<Vec<DVec2>>, link_dist: f64) -> Result<Vec<Move>> {
    let mut b: PathBuilder = g.builder();
    let tol = g.c.tolerance.max(1e-4);
    let drop = |pts: &[DVec2]| -> Vec<DVec3> { s.drop_path(pts) };
    // Sample every run first (in parallel); linking moves are sampled as they are emitted.
    let sampled: Vec<Vec<DVec3>> = runs
        .par_iter()
        .map(|run| simplify(&drop(run), tol * 0.5))
        .collect();
    crate::workflow::check_cancel()?;
    for (run, pts) in runs.iter().zip(sampled) {
        if pts.is_empty() {
            continue;
        }
        let linked = b.pos.is_some_and(|p| {
            let length = p.truncate().distance(run[0]);
            let inside_length: f64 = region::clip_lines(&s.area, &[(p.truncate(), run[0])])
                .iter()
                .map(|(a, b)| a.distance(*b))
                .sum();
            length <= link_dist
                && p.z < g.c.heights.retract
                && (length - inside_length).abs() < 1e-8
        });
        if linked {
            let from = b.pos.unwrap().truncate();
            for p in simplify(&drop(&[from, run[0]]), tol * 0.5).into_iter().skip(1) {
                b.feed(p);
            }
            for &p in &pts[1..] {
                b.feed(p);
            }
        } else {
            b.polyline(&pts, false);
        }
    }
    b.finish()
}

fn parallel(g: &Gen, s: &Setup, stepover: f64, angle: f64, zig: bool) -> Result<Vec<Move>> {
    let stepover = stepover.max(0.01);
    let dir = DVec2::from_angle(angle.to_radians());
    let nrm = dir.perp();
    let c = (s.min + s.max) * 0.5;
    let half = (s.max - s.min).length() * 0.5 + stepover;
    let n = (2.0 * half / stepover).ceil() as i64;
    let lines: Vec<(DVec2, DVec2)> = (0..=n)
        .map(|i| {
            let o = c + nrm * (-half + i as f64 * stepover);
            (o - dir * half, o + dir * half)
        })
        .collect();
    let mut runs = Vec::new();
    for (k, (a, b)) in lines.into_iter().enumerate() {
        let mut segs = region::clip_lines(&s.area, &[(a, b)]);
        segs.sort_by(|x, y| x.0.dot(dir).total_cmp(&y.0.dot(dir)));
        let forward = !zig || k % 2 == 0;
        let segs: Vec<_> = if forward {
            segs
        } else {
            segs.into_iter().rev().map(|(a, b)| (b, a)).collect()
        };
        for (p, q) in segs {
            let (p, q) = if ((q - p).dot(dir) >= 0.0) == forward {
                (p, q)
            } else {
                (q, p)
            };
            runs.push(vec![p, q]);
        }
    }
    emit_runs(g, s, runs, if zig { stepover * 2.0 } else { 0.0 })
}

fn spiral(g: &Gen, s: &Setup, stepover: f64) -> Result<Vec<Move>> {
    let stepover = stepover.max(0.01);
    let c = (s.min + s.max) * 0.5;
    let rmax = (s.max - s.min).length() * 0.5;
    let mut pts = vec![c];
    let mut theta: f64 = 0.0;
    loop {
        let r = stepover * theta / std::f64::consts::TAU;
        if r > rmax {
            break;
        }
        // Step so that chord length is about one sampling cell.
        theta += (s.cell / r.max(s.cell)).min(0.5);
        pts.push(c + DVec2::from_angle(theta) * (stepover * theta / std::f64::consts::TAU));
    }
    emit_runs(g, s, clip_polyline(&s.area, &pts), stepover * 2.0)
}

fn radial(g: &Gen, s: &Setup, angle_step: f64) -> Result<Vec<Move>> {
    let step = angle_step.clamp(0.05, 90.0);
    let c = (s.min + s.max) * 0.5;
    let rmax = (s.max - s.min).length() * 0.5;
    let n = (360.0 / step).round() as usize;
    let mut runs = Vec::new();
    for k in 0..n {
        let d = DVec2::from_angle((k as f64 * step).to_radians());
        let (a, b) = (c + d * g.tool.radius() * 0.5, c + d * rmax);
        for (p, q) in region::clip_lines(&s.area, &[(a, b)]) {
            // Alternate inward/outward to keep links short.
            let outward = (q - p).dot(d) > 0.0;
            runs.push(if outward == (k % 2 == 0) {
                vec![p, q]
            } else {
                vec![q, p]
            });
        }
    }
    emit_runs(g, s, runs, g.tool.diameter)
}

fn project(g: &Gen, s: &Setup, depth: f64) -> Result<Vec<Move>> {
    if g.c.curves.is_empty() {
        bail!("select the curves to project");
    }
    let mut b = g.builder();
    for c in g.project.select_curves(&g.c.curves) {
        let mut pts2 = c.pts.clone();
        if c.closed {
            pts2.push(c.pts[0]);
        }
        for run in clip_polyline(&s.area, &pts2) {
            let pts: Vec<DVec3> = s.drop_path(&run).into_iter().map(|p| p - DVec3::Z * depth).collect();
            b.retract();
            b.polyline(&simplify(&pts, g.c.tolerance.max(1e-4) * 0.5), false);
        }
    }
    b.finish()
}

fn waterline(g: &Gen, s: &Setup, stepdown: f64) -> Result<Vec<Move>> {
    let cl = s.cl_map();
    let mut b = g.builder();
    let lo =
        cl.z.iter()
            .copied()
            .fold(f64::INFINITY, f64::min)
            .max(g.c.heights.bottom);
    let hi =
        cl.z.iter()
            .copied()
            .fold(f64::NEG_INFINITY, f64::max)
            .min(g.c.heights.top);
    let levels = depth_levels(hi, lo, stepdown.max(0.01));
    crate::workflow::check_cancel()?;
    let contours: Vec<Vec<Polyline2>> = levels.par_iter().map(|&z| cl.contours(z)).collect();
    for (z, mut loops) in levels.into_iter().zip(contours) {
        crate::workflow::check_cancel()?;
        // Nearest-first ordering.
        while !loops.is_empty() {
            let here = b.pos.map(|p| p.truncate()).unwrap_or(s.min);
            let i = loops
                .iter()
                .enumerate()
                .min_by(|a, b| {
                    a.1.pts[0]
                        .distance_squared(here)
                        .total_cmp(&b.1.pts[0].distance_squared(here))
                })
                .unwrap()
                .0;
            let mut c = loops.swap_remove(i);
            if c.closed {
                c.start_nearest(here);
                c.pts.push(c.pts[0]);
            }
            for run in clip_polyline(&s.area, &c.pts) {
                let pts: Vec<DVec3> = run.iter().map(|p| p.extend(z)).collect();
                b.polyline(&simplify(&pts, g.c.tolerance.max(1e-4)), false);
            }
        }
    }
    b.finish()
}

fn roughing(g: &Gen, s: &Setup, stepover: f64, finish_contour: bool) -> Result<Vec<Move>> {
    let cl = s.cl_map();
    let stepover = stepover.clamp(0.01, g.tool.diameter);
    let ramp = g.ramp_angle();
    let mut b = g.builder();
    let mut z_prev = g.c.heights.top;
    let tol = (s.cell * 0.5).max(g.c.tolerance);
    let levels = depth_levels(g.c.heights.top, g.c.heights.bottom, g.c.stepdown);
    crate::workflow::check_cancel()?;
    // The blocked region of every level comes from the same map; extract them in parallel.
    let blocked_per_level: Vec<Vec<Polyline2>> = levels
        .par_iter()
        .map(|&z| {
            cl.contours(z)
                .into_iter()
                .filter(|c| c.closed)
                .map(|c| {
                    let pts: Vec<DVec3> = c.pts.iter().map(|p| p.extend(0.0)).collect();
                    Polyline2::new(simplify(&pts, tol).iter().map(|p| p.truncate()).collect(), true)
                })
                .filter(|c| c.pts.len() >= 3)
                .collect()
        })
        .collect();
    for (z, blocked) in levels.into_iter().zip(blocked_per_level) {
        crate::workflow::check_cancel()?;
        let forbidden = region::from_curves(&blocked.iter().collect::<Vec<_>>());
        let allowed = region::difference(&s.area, &forbidden);
        let rings = region::concentric(&allowed, stepover, 10_000);
        for (k, ring) in rings.iter().enumerate() {
            for lp in region::loops(ring, g.c.climb) {
                let mut pl = Polyline2 {
                    pts: lp,
                    closed: true,
                };
                if let Some(p) = b.pos {
                    pl.start_nearest(p.truncate());
                }
                let close = k > 0
                    && b.pos.is_some_and(|p| {
                        (p.z - z).abs() < 1e-9 && p.truncate().distance(pl.pts[0]) <= stepover * 1.5
                    });
                if close {
                    b.ramp_loop(&pl.pts, z, z, 0.0, true);
                } else {
                    b.ramp_loop(&pl.pts, z_prev, z, ramp, false);
                }
            }
        }
        if finish_contour {
            if let Some(first) = rings.first() {
                b.retract();
                for lp in region::loops(first, g.c.climb) {
                    b.ramp_loop(&lp, z, z, 0.0, false);
                }
            }
        }
        z_prev = z;
    }
    b.finish()
}
