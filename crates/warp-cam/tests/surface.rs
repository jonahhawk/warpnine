//! D02: exact cutter-contact queries and bounded path sampling.
//!
//! Expectations are independent of the query code: closed forms, a brute-force contact oracle that
//! samples triangle points densely, and a penetration oracle that measures how far sampled mesh
//! points sit inside the tool body along the generated path.

use glam::{DVec2, DVec3};
use std::time::Instant;
use warp_cam::ops::{Common, OpKind, Operation, SurfaceMode};
use warp_cam::surface::{self, Cutter, MeshSurface, SurfaceQuery};
use warp_cam::{generate, Project, Toolpath};
use warp_core::Mesh;
use warp_tools::{Tool, ToolLibrary, ToolShape};

fn quad(tris: &mut Vec<[DVec3; 3]>, a: DVec3, b: DVec3, c: DVec3, d: DVec3) {
    tris.push([a, b, c]);
    tris.push([a, c, d]);
}

/// Two plateaus joined by a vertical wall at x = 6.1 (top z = 0, floor z = -3), plus a fin
/// 0.05 mm wide and 1.5 mm tall at x = 9.0..9.05. Open shell: only machinable faces.
fn stepped_block() -> Mesh {
    let p = DVec3::new;
    let mut t = Vec::new();
    let (y0, y1) = (0.0, 4.0);
    quad(&mut t, p(0., y0, 0.), p(6.1, y0, 0.), p(6.1, y1, 0.), p(0., y1, 0.));
    quad(&mut t, p(6.1, y0, -3.), p(6.1, y0, 0.), p(6.1, y1, 0.), p(6.1, y1, -3.));
    quad(&mut t, p(6.1, y0, -3.), p(9.0, y0, -3.), p(9.0, y1, -3.), p(6.1, y1, -3.));
    quad(&mut t, p(9.0, y0, -3.), p(9.0, y0, -1.5), p(9.0, y1, -1.5), p(9.0, y1, -3.));
    quad(&mut t, p(9.0, y0, -1.5), p(9.05, y0, -1.5), p(9.05, y1, -1.5), p(9.0, y1, -1.5));
    quad(&mut t, p(9.05, y0, -3.), p(9.05, y0, -1.5), p(9.05, y1, -1.5), p(9.05, y1, -3.));
    quad(&mut t, p(9.05, y0, -3.), p(12., y0, -3.), p(12., y1, -3.), p(9.05, y1, -3.));
    Mesh { tris: t, provenance: None }
}

/// Low-poly dome over 60 x 60 (peak z = 0, rim z = -10), 2 mm facets.
fn dome() -> Mesh {
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
    Mesh { tris, provenance: None }
}

fn tools() -> Vec<Tool> {
    vec![
        Tool::new(1, "flat 6", ToolShape::FlatEndMill, 6.0),
        Tool::new(2, "ball 6", ToolShape::BallEndMill, 6.0),
        Tool::new(3, "bull 6/1", ToolShape::BullNose { corner_radius: 1.0 }, 6.0),
        Tool::new(4, "v 90", ToolShape::VBit { included_angle: 90.0, tip_diameter: 0.4 }, 6.0),
        Tool::new(5, "taper ball", ToolShape::TaperedBall { taper_angle: 10.0, tip_radius: 1.0 }, 6.0),
    ]
}

/// Dense samples of every triangle (barycentric grid at most `spacing` apart), sorted by x.
struct Samples(Vec<DVec3>);

impl Samples {
    fn new(mesh: &Mesh, spacing: f64) -> Self {
        let mut pts = Vec::new();
        for t in &mesh.tris {
            let longest = t[0].distance(t[1]).max(t[1].distance(t[2])).max(t[2].distance(t[0]));
            let n = ((longest / spacing).ceil() as usize).max(1);
            for i in 0..=n {
                for j in 0..=(n - i) {
                    let (u, v) = (i as f64 / n as f64, j as f64 / n as f64);
                    pts.push(t[0] + (t[1] - t[0]) * u + (t[2] - t[0]) * v);
                }
            }
        }
        pts.sort_by(|a, b| a.x.total_cmp(&b.x));
        Samples(pts)
    }

    fn window(&self, x0: f64, x1: f64) -> &[DVec3] {
        let a = self.0.partition_point(|p| p.x < x0);
        let b = self.0.partition_point(|p| p.x <= x1);
        &self.0[a..b]
    }

    /// Brute-force contact height: the highest `z - h(r)` over the sampled points.
    fn drop(&self, tool: &Tool, p: DVec2) -> f64 {
        let r = tool.radius();
        let mut best = f64::NEG_INFINITY;
        for q in self.window(p.x - r, p.x + r) {
            let d = q.truncate().distance(p);
            if let Some(h) = tool.profile_height(d) {
                best = best.max(q.z - h);
            }
        }
        best
    }

    /// Deepest penetration of a sampled point into the tool body at `at` (tip position).
    fn penetration(&self, tool: &Tool, at: DVec3) -> f64 {
        let r = tool.radius();
        let mut worst: f64 = 0.0;
        for q in self.window(at.x - r, at.x + r) {
            let d = q.truncate().distance(at.truncate());
            let zeta = q.z - at.z;
            if d > r || zeta <= 0.0 {
                continue;
            }
            let h = tool.profile_height(d).unwrap();
            if zeta <= h + 1e-12 {
                continue;
            }
            worst = worst.max(profile_distance(tool, d, zeta));
        }
        worst
    }
}

/// Distance from the point (d, zeta) in the tool's meridian plane to the tool boundary
/// (profile curve plus the cylindrical shank above it).
fn profile_distance(tool: &Tool, d: f64, zeta: f64) -> f64 {
    let r = tool.radius();
    let n = 512;
    let pt = |k: usize| {
        let rho = r * k as f64 / n as f64;
        DVec2::new(rho, tool.profile_height(rho).unwrap_or(0.0))
    };
    let q = DVec2::new(d, zeta);
    let mut best = f64::INFINITY;
    let mut prev = pt(0);
    for k in 1..=n {
        let cur = pt(k);
        let ab = cur - prev;
        let l2 = ab.length_squared();
        let t = if l2 > 0.0 { ((q - prev).dot(ab) / l2).clamp(0.0, 1.0) } else { 0.0 };
        best = best.min(q.distance(prev + ab * t));
        prev = cur;
    }
    let top = tool.profile_height(r).unwrap_or(0.0);
    let shank = if zeta >= top { r - d } else { q.distance(DVec2::new(r, top)) };
    best.min(shank)
}

#[test]
fn exact_contact_never_gouges_dense_samples_and_stays_close() {
    let cases = [(stepped_block(), 0.02), (dome(), 0.08)];
    for (mesh, spacing) in &cases {
        let surf = MeshSurface::new(mesh);
        let samples = Samples::new(mesh, *spacing);
        let bb = mesh.bbox();
        let mut seed = 0x9e37_79b9_7f4a_7c15u64;
        let mut rnd = || {
            seed ^= seed << 13;
            seed ^= seed >> 7;
            seed ^= seed << 17;
            (seed >> 11) as f64 / (1u64 << 53) as f64
        };
        for tool in tools() {
            let c = Cutter::new(&tool, 0.0, 0.0);
            let mut gap_max: f64 = 0.0;
            for _ in 0..40 {
                let p = DVec2::new(
                    bb.min.x - 1.0 + rnd() * (bb.size().x + 2.0),
                    bb.min.y + rnd() * bb.size().y,
                );
                let exact = surf.drop(&c, p);
                let brute = samples.drop(&tool, p);
                assert!(
                    exact >= brute - 1e-9,
                    "{} at {p}: exact {exact} below a sampled contact {brute}",
                    tool.name
                );
                if brute.is_finite() {
                    gap_max = gap_max.max(exact - brute);
                }
            }
            // The brute force misses contacts between samples; the gap is bounded by the sampling.
            assert!(gap_max < 0.3, "{}: exact exceeds brute force by {gap_max}", tool.name);
        }
    }
}

fn cut_segments(tp: &Toolpath) -> Vec<(DVec3, DVec3)> {
    tp.segments(DVec3::new(0.0, 0.0, 15.0), 0.001)
        .into_iter()
        .filter(|s| !s.2)
        .map(|s| (s.0, s.1))
        .collect()
}

/// Points along the cut segments: at most `spacing` apart and at least four per segment, so
/// the interior of every short chord is probed.
fn cut_points(segs: &[(DVec3, DVec3)], spacing: f64) -> Vec<DVec3> {
    let mut out = Vec::new();
    for &(from, to) in segs {
        let n = ((from.distance(to) / spacing).ceil() as usize).max(4);
        for k in 0..=n {
            out.push(from.lerp(to, k as f64 / n as f64));
        }
    }
    out
}

fn parallel_op(tool: &Tool, mode: SurfaceMode, stepover: f64, bottom: f64) -> Operation {
    let mut c = Common::for_tool(tool);
    c.heights.bottom = bottom;
    c.tolerance = 0.01;
    c.resolution = 0.25;
    c.surface = mode;
    c.entry = warp_cam::Entry::Plunge;
    Operation {
        name: format!("parallel {}", mode.label()),
        enabled: true,
        common: c,
        kind: OpKind::Parallel { stepover, angle: 0.0, zigzag: true },
    }
}

#[test]
fn exact_parallel_finish_respects_walls_and_fins_where_the_zmap_cannot() {
    let mesh = stepped_block();
    let samples = Samples::new(&mesh, 0.02);
    let tool = Tool::new(1, "flat 2", ToolShape::FlatEndMill, 2.0);
    let mut tools = ToolLibrary::default();
    tools.tools.push(tool.clone());
    let mut proj = Project::default();
    proj.stock.min = DVec3::new(0.0, 0.0, -4.0);
    proj.stock.max = DVec3::new(12.0, 4.0, 0.0);
    proj.mesh = Some(mesh);

    let mut results = Vec::new();
    for mode in [SurfaceMode::Exact, SurfaceMode::ZMap] {
        let op = parallel_op(&tool, mode, 0.5, -3.0);
        let t0 = Instant::now();
        let tp = generate(&op, &proj, &tools).unwrap();
        let elapsed = t0.elapsed();
        let pts = cut_points(&cut_segments(&tp), 0.05);
        let mut worst = (0.0f64, DVec3::ZERO);
        for &p in &pts {
            let pen = samples.penetration(&tool, p);
            if pen > worst.0 {
                worst = (pen, p);
            }
        }
        if mode == SurfaceMode::Exact {
            // Both plateaus are machined at their heights: centres more than a radius past the
            // wall (x > 7.1) and the fin (x > 10.05) reach the floor, centres before the wall
            // step stay on top. The Z-map ramps a cell early, which its penetration shows.
            let on_top = pts.iter().filter(|p| p.x > 0.5 && p.x < 7.09 && p.y > 0.5 && p.y < 3.5).all(|p| p.z > -1e-6);
            let on_floor = pts
                .iter()
                .filter(|p| (p.x > 7.12 && p.x < 7.99 || p.x > 10.07 && p.x < 11.0) && p.y > 0.5 && p.y < 3.5)
                .all(|p| (p.z + 3.0).abs() < 1e-6);
            assert!(on_top && on_floor, "plateaus not machined at their heights");
        }
        eprintln!(
            "{}: {} moves in {:.0} ms, worst penetration {:.4} mm at {:.3?}",
            mode.label(),
            tp.moves.len(),
            elapsed.as_secs_f64() * 1e3,
            worst.0,
            worst.1
        );
        results.push((mode, worst));
    }
    let exact = results[0].1;
    let zmap = results[1].1;
    assert!(exact.0 <= 0.01 + 1e-6, "exact mode penetrates {} mm at {}", exact.0, exact.1);
    // The Z-map snaps the wall (x = 6.1) and the fin edge (x = 9.05) to the 0.25 mm grid.
    assert!(zmap.0 > 0.04, "Z-map mode should over-cut the wall by about 0.05-0.1 mm, got {}", zmap.0);
}

#[test]
fn exact_ball_finish_on_a_dome_meets_the_tolerance_both_ways() {
    let mesh = dome();
    let samples = Samples::new(&mesh, 0.05);
    let tool = Tool::new(2, "ball 6", ToolShape::BallEndMill, 6.0);
    let mut tools = ToolLibrary::default();
    tools.tools.push(tool.clone());
    let mut proj = Project::default();
    proj.stock.min = DVec3::new(0.0, 0.0, -12.0);
    proj.stock.max = DVec3::new(60.0, 60.0, 0.0);
    // Machine a 10 x 10 window on the flank of the dome (slopes up to about 33 degrees).
    proj.curves.push(warp_core::Polyline2::rect(DVec2::new(5.0, 25.0), DVec2::new(15.0, 35.0)));
    proj.mesh = Some(mesh.clone());

    let mut op = parallel_op(&tool, SurfaceMode::Exact, 2.0, -12.0);
    op.common.curves = vec![0];
    let t0 = Instant::now();
    let tp = generate(&op, &proj, &tools).unwrap();
    let elapsed = t0.elapsed();
    let window = |p: &DVec3| p.x > 5.5 && p.x < 14.5 && p.y > 25.5 && p.y < 34.5;
    let segs: Vec<(DVec3, DVec3)> = cut_segments(&tp).into_iter().filter(|(a, b)| window(a) && window(b)).collect();
    let inside = cut_points(&segs, 0.05);
    assert!(inside.len() > 500, "{}", inside.len());
    let mut pen: f64 = 0.0;
    for &p in &inside {
        pen = pen.max(samples.penetration(&tool, p));
    }
    // Vertical deviation from the exact contact surface, measured by the query itself.
    let surf = MeshSurface::new(&mesh);
    let c = Cutter::new(&tool, 0.0, 0.0);
    let mut dev = surface::PathDeviation::default();
    for (a, b) in &segs {
        surface::path_deviation(&surf, &c, &[*a, *b], 0.05, -12.0, &mut dev);
    }
    eprintln!(
        "dome: {} moves in {:.0} ms, penetration {:.4} mm, gouge bound {:.4} mm, above {:.4} mm over {} samples",
        tp.moves.len(),
        elapsed.as_secs_f64() * 1e3,
        pen,
        dev.gouge,
        dev.above,
        dev.samples
    );
    assert!(pen <= 0.01 + 1e-6, "penetration {pen}");
    assert!(dev.gouge <= 0.01 + 1e-6, "tool inside the contact surface by {} at {}", dev.gouge, dev.gouge_at);
    // Slopes reach 33 degrees, so a 0.01 mm 3D band is at most 0.012 mm vertically.
    assert!(dev.above <= 0.012, "material left: {} at {}", dev.above, dev.above_at);
}

#[test]
fn legacy_operations_default_to_automatic_surface_queries() {
    let tool = Tool::new(1, "flat 6", ToolShape::FlatEndMill, 6.0);
    let mut v = serde_json::to_value(Common::for_tool(&tool)).unwrap();
    assert_eq!(v["surface"], serde_json::json!("Auto"));
    v.as_object_mut().unwrap().remove("surface");
    let c: Common = serde_json::from_value(v).unwrap();
    assert_eq!(c.surface, SurfaceMode::Auto);
    assert_eq!(c.surface.resolve(&OpKind::Roughing3D { stepover: 1.0, finish_contour: false }), SurfaceMode::ZMap);
    assert_eq!(c.surface.resolve(&OpKind::Waterline { stepdown: 1.0 }), SurfaceMode::Exact);
    assert_eq!(SurfaceMode::ZMap.resolve(&OpKind::Waterline { stepdown: 1.0 }), SurfaceMode::ZMap);
}

#[test]
fn sampling_is_bounded_by_the_footprint_and_refines_sharp_features() {
    // A 0.05 mm fin under a 2 mm flat tool raises the contact surface over a 2.05 mm band; the
    // sampler at a 0.25 mm step must find it and refine the vertical transitions.
    let mesh = stepped_block();
    let surf = MeshSurface::new(&mesh);
    let tool = Tool::new(1, "flat 2", ToolShape::FlatEndMill, 2.0);
    let c = Cutter::new(&tool, 0.0, 0.0);
    let s = surface::Sampling::new(&c, 0.25, 0.01, -3.0);
    assert!(s.max_step <= 0.25 && s.min_step <= 0.005 && s.tol <= 0.005);
    let path = surface::drop_polyline(&surf, &c, &[DVec2::new(7.5, 2.0), DVec2::new(11.5, 2.0)], &s);
    assert_eq!(path.residual, 0.0, "every chord met the tolerance");
    let raised: Vec<&DVec3> = path.pts.iter().filter(|p| (p.z + 1.5).abs() < 1e-9).collect();
    let lo = raised.iter().map(|p| p.x).fold(f64::INFINITY, f64::min);
    let hi = raised.iter().map(|p| p.x).fold(f64::NEG_INFINITY, f64::max);
    assert!((lo - 8.0).abs() <= 0.005 && (hi - 10.05).abs() <= 0.005, "fin band {lo}..{hi}");
    // Sampling with a wide step is capped at the footprint radius.
    let wide = surface::Sampling::new(&c, 5.0, 0.01, -3.0);
    assert!((wide.max_step - 1.0).abs() < 1e-12);
}
