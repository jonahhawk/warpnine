//! Surface query services: exact cutter-contact queries against a mesh behind a spatial index,
//! and path sampling with measured interpolation error.
//!
//! [`MeshSurface`] answers "how low can the tool tip go at (x, y) without gouging" exactly for the
//! triangle mesh: the tool is a solid of revolution described by [`warp_tools::Tool::profile_height`],
//! and the lowest non-gouging tip height is the maximum over facet, edge and vertex contacts. Every
//! supported profile is convex and non-decreasing in the radius, so each contact is the maximum of a
//! concave one-dimensional function and is found by golden-section search to a fixed precision
//! instead of by rasterising the mesh. The Z-map in [`crate::zmap`] stays as the fast approximation
//! for roughing; [`SurfaceQuery`] hides which one an operation uses.
//!
//! Error budget of a sampled path (see [`drop_polyline`]): every emitted point is an exact contact
//! height; between points the tool moves on a straight chord, and the chord is accepted only after
//! the exact contact at its midpoint lies within the requested distance of it, down to a minimum
//! step. The remaining error is measured and returned, never assumed from the step size.

use glam::{DVec2, DVec3};
use rayon::prelude::*;
use warp_core::{geom::Aabb3, Mesh};
use warp_tools::{Tool, ToolShape};

/// A tool profile grown by the stock to leave: `radial` on walls, `axial` on floors.
///
/// The grown profile is `profile_height(max(r - radial, 0)) - axial`, the convention shared with
/// the Z-map kernel: the tool is swept horizontally by `radial` and lowered by `axial`.
#[derive(Debug, Clone)]
pub struct Cutter {
    tool: Tool,
    radial: f64,
    axial: f64,
    reach: f64,
}

impl Cutter {
    pub fn new(tool: &Tool, radial_leave: f64, axial_leave: f64) -> Self {
        let radial = radial_leave.max(0.0);
        Cutter {
            tool: tool.clone(),
            radial,
            axial: axial_leave.max(0.0),
            reach: tool.radius() + radial,
        }
    }

    /// Radius of the grown footprint.
    pub fn reach(&self) -> f64 {
        self.reach
    }

    pub fn tool(&self) -> &Tool {
        &self.tool
    }

    /// Height of the grown cutting profile above the tip at radial distance `r`, or `None` beyond
    /// the footprint.
    #[inline]
    pub fn height(&self, r: f64) -> Option<f64> {
        if r > self.reach + 1e-9 {
            return None;
        }
        let r = (r - self.radial).max(0.0).min(self.tool.radius());
        Some(self.tool.profile_height(r).unwrap_or(0.0) - self.axial)
    }

    /// Radius at which the grown profile touches a plane of slope `s` (dz per unit XY): the
    /// maximiser of `r * s - height(r)`, in closed form for every tool shape. Profiles are convex,
    /// so this is where the profile slope reaches `s`, or the rim of the segment that cannot.
    pub fn contact_radius(&self, s: f64) -> f64 {
        if s <= 0.0 {
            return 0.0;
        }
        let rad = self.tool.radius();
        let sin = s / (1.0 + s * s).sqrt();
        let cone = |half_angle_deg: f64, tip_r: f64| {
            let cot = 1.0 / half_angle_deg.to_radians().tan().max(1e-6);
            if s <= cot { tip_r.min(rad) } else { rad }
        };
        let r0 = match self.tool.shape {
            ToolShape::FlatEndMill
            | ToolShape::FaceMill
            | ToolShape::ThreadMill { .. }
            | ToolShape::SlotCutter { .. }
            | ToolShape::Tap { .. } => rad,
            ToolShape::BallEndMill => rad * sin,
            ToolShape::BullNose { corner_radius } => {
                let cr = corner_radius.min(rad);
                (rad - cr) + cr * sin
            }
            ToolShape::VBit { included_angle, tip_diameter }
            | ToolShape::ChamferMill { angle: included_angle, tip_diameter }
            | ToolShape::Engraver { included_angle, tip_diameter } => cone(included_angle * 0.5, tip_diameter * 0.5),
            ToolShape::Drill { point_angle } | ToolShape::SpotDrill { point_angle } => cone(point_angle * 0.5, 0.0),
            ToolShape::TaperedBall { taper_angle, tip_radius } => {
                let tr = tip_radius.min(rad);
                let a = taper_angle.to_radians();
                if s <= 1.0 / a.tan().max(1e-6) { tr * sin } else { rad }
            }
        };
        (self.radial + r0).min(self.reach)
    }
}

/// Cutter-contact queries on a part surface.
pub trait SurfaceQuery: Sync {
    /// Lowest tool-tip Z at `p` that does not gouge the surface, or `NEG_INFINITY` when nothing
    /// lies under the footprint.
    fn drop(&self, c: &Cutter, p: DVec2) -> f64;

    /// Vertical distance by which a tool tip at `at` sits below the contact height, or 0.
    /// Exact on flat and shallow surfaces; on steep walls it overstates the penetration, which
    /// [`gouge_distance`] bounds instead.
    fn penetration(&self, c: &Cutter, at: DVec3) -> f64 {
        (self.drop(c, at.truncate()) - at.z).max(0.0)
    }
}

/// Upper bound of the 3D distance from a tool tip at `at` to the contact surface when the tip
/// lies below it, or 0. The bound starts from the vertical gap and is improved by probing the
/// contact height along 32 directions out to the current bound and bisecting to where the
/// surface falls below the tip; within 0.5% of the true distance for a planar wall, and never
/// below it. When the bound is already at or under `ceiling` the search stops, so a caller that
/// only needs the largest gouge along a path can pass its running maximum.
pub fn gouge_distance(q: &dyn SurfaceQuery, c: &Cutter, at: DVec3, ceiling: f64) -> f64 {
    const DIRS: usize = 32;
    let p = at.truncate();
    let gap = q.drop(c, p) - at.z;
    if gap <= 0.0 {
        return 0.0;
    }
    let mut best = gap;
    let f = |r: f64, u: DVec2| q.drop(c, p + u * r) - at.z;
    let dir = |k: usize| DVec2::from_angle(k as f64 * std::f64::consts::TAU / DIRS as f64);
    // Local-plane estimate: the gradient from four probes gives the foot of the perpendicular on
    // a planar surface; probing there yields a valid bound that is tight on smooth surfaces.
    let rho = (gap * 0.5).min(ceiling.max(1e-3)).max(1e-4);
    let gx = (f(rho, DVec2::X) - f(rho, -DVec2::X)) / (2.0 * rho);
    let gy = (f(rho, DVec2::Y) - f(rho, -DVec2::Y)) / (2.0 * rho);
    let grad = DVec2::new(gx, gy);
    let slope2 = grad.length_squared();
    if slope2 > 0.0 && slope2.is_finite() {
        let foot = gap * slope2.sqrt() / (1.0 + slope2);
        let g = f(foot, -grad / slope2.sqrt());
        best = best.min(if g <= 0.0 { foot } else { (foot * foot + g * g).sqrt() });
        if best <= ceiling {
            return best;
        }
    }
    // Cheap test for steps: if the surface is already below the tip at `ceiling` in some
    // direction, the distance cannot exceed the ceiling.
    if ceiling > 0.0 && ceiling < best {
        for k in 0..DIRS {
            if f(ceiling, dir(k)) <= 0.0 {
                return ceiling;
            }
        }
    }
    // Full search: scan every direction out to the current bound, then bisect the crossing in
    // the direction where the surface first falls below the tip.
    let mut crossing: Option<(usize, f64, f64)> = None;
    for k in 0..DIRS {
        let u = dir(k);
        let mut prev = 0.0;
        let limit = best;
        for i in 1..=8 {
            let r = limit * i as f64 / 8.0;
            if r >= best {
                break;
            }
            let g = f(r, u);
            if g <= 0.0 {
                if crossing.is_none_or(|c| r < c.2) {
                    crossing = Some((k, prev, r));
                }
                best = best.min(r);
                break;
            }
            best = best.min((r * r + g * g).sqrt());
            prev = r;
        }
    }
    if let Some((k, lo, hi)) = crossing {
        let u = dir(k);
        let (mut lo, mut hi) = (lo, hi);
        for _ in 0..14 {
            let mid = 0.5 * (lo + hi);
            if f(mid, u) <= 0.0 {
                hi = mid;
            } else {
                lo = mid;
            }
        }
        best = best.min(hi);
    }
    best
}

#[derive(Debug, Clone)]
struct Tri {
    p: [DVec3; 3],
    min: DVec2,
    max: DVec2,
    zmax: f64,
    /// Unit uphill direction in XY and the plane slope (dz per unit XY) along it; `None` for
    /// vertical facets, which only make edge contacts.
    slope: Option<(DVec2, f64)>,
    /// Plane as `z = z0 + gx*x + gy*y` (only valid with a slope).
    plane: (f64, f64, f64),
}

impl Tri {
    fn new(p: [DVec3; 3]) -> Self {
        let (a, b, c) = (p[0], p[1], p[2]);
        let mut n = (b - a).cross(c - a);
        let len = n.length();
        let min = a.truncate().min(b.truncate()).min(c.truncate());
        let max = a.truncate().max(b.truncate()).max(c.truncate());
        let zmax = a.z.max(b.z).max(c.z);
        if len < 1e-18 {
            return Tri { p, min, max, zmax, slope: None, plane: (0.0, 0.0, 0.0) };
        }
        n /= len;
        if n.z < 0.0 {
            n = -n;
        }
        // Treat facets within ~1e-7 rad of vertical as vertical: their contact is on their edges.
        if n.z < 1e-7 {
            return Tri { p, min, max, zmax, slope: None, plane: (0.0, 0.0, 0.0) };
        }
        let gx = -n.x / n.z;
        let gy = -n.y / n.z;
        let z0 = a.z - gx * a.x - gy * a.y;
        let g = DVec2::new(gx, gy);
        let s = g.length();
        let dir = if s > 0.0 { g / s } else { DVec2::ZERO };
        Tri { p, min, max, zmax, slope: Some((dir, s)), plane: (z0, gx, gy) }
    }

    #[inline]
    fn plane_z(&self, q: DVec2) -> f64 {
        self.plane.0 + self.plane.1 * q.x + self.plane.2 * q.y
    }

    /// Point-in-triangle in XY with a tiny tolerance in barycentric space.
    fn contains_xy(&self, q: DVec2) -> bool {
        let (a, b, c) = (self.p[0].truncate(), self.p[1].truncate(), self.p[2].truncate());
        let d = (b - a).perp_dot(c - a);
        if d.abs() < 1e-18 {
            return false;
        }
        let w1 = (q - a).perp_dot(c - a) / d;
        let w2 = (b - a).perp_dot(q - a) / d;
        let w0 = 1.0 - w1 - w2;
        let e = -1e-9;
        w0 >= e && w1 >= e && w2 >= e
    }

    /// XY distance from `q` to the triangle (0 inside).
    fn xy_distance(&self, q: DVec2) -> f64 {
        if self.contains_xy(q) {
            return 0.0;
        }
        let mut best = f64::INFINITY;
        for k in 0..3 {
            let (a, b) = (self.p[k].truncate(), self.p[(k + 1) % 3].truncate());
            best = best.min(segment_distance(q, a, b));
        }
        best
    }
}

fn segment_distance(q: DVec2, a: DVec2, b: DVec2) -> f64 {
    let ab = b - a;
    let l2 = ab.length_squared();
    let t = if l2 > 0.0 { ((q - a).dot(ab) / l2).clamp(0.0, 1.0) } else { 0.0 };
    q.distance(a + ab * t)
}

/// Golden-section maximisation of a concave function on `[lo, hi]` to an interval width of
/// `width_tol`. The endpoints are evaluated too, so kinks at the ends are exact.
fn maximise(lo: f64, hi: f64, width_tol: f64, f: impl Fn(f64) -> f64) -> (f64, f64) {
    const INV_PHI: f64 = 0.618_033_988_749_895;
    let mut best = {
        let (fl, fh) = (f(lo), f(hi));
        if fl >= fh { (lo, fl) } else { (hi, fh) }
    };
    let (mut lo, mut hi) = (lo, hi);
    let mut x1 = hi - INV_PHI * (hi - lo);
    let mut x2 = lo + INV_PHI * (hi - lo);
    let (mut f1, mut f2) = (f(x1), f(x2));
    for _ in 0..80 {
        if hi - lo <= width_tol {
            break;
        }
        if f1 < f2 {
            lo = x1;
            x1 = x2;
            f1 = f2;
            x2 = lo + INV_PHI * (hi - lo);
            f2 = f(x2);
        } else {
            hi = x2;
            x2 = x1;
            f2 = f1;
            x1 = hi - INV_PHI * (hi - lo);
            f1 = f(x1);
        }
    }
    let (x, fx) = if f1 >= f2 { (x1, f1) } else { (x2, f2) };
    if fx > best.1 {
        best = (x, fx);
    }
    best
}

/// Triangle mesh with a uniform XY bin index for cutter-contact queries.
#[derive(Debug, Clone)]
pub struct MeshSurface {
    tris: Vec<Tri>,
    origin: DVec2,
    cell: f64,
    nx: usize,
    ny: usize,
    /// CSR bins: triangle ids of bin `k` are `items[starts[k]..starts[k + 1]]`.
    starts: Vec<u32>,
    items: Vec<u32>,
    bbox: Aabb3,
}

/// Width tolerance of the edge-contact search, in millimetres along the edge. The contact height
/// is stationary at an interior maximum and evaluated exactly at the ends, so the height error is
/// second order in this width.
const SEARCH_TOL: f64 = 1e-6;

impl MeshSurface {
    /// Index a mesh. Bins are sized so that a typical triangle spans about one bin; the bin count
    /// is capped at about four million.
    pub fn new(mesh: &Mesh) -> Self {
        let tris: Vec<Tri> = mesh.tris.iter().map(|t| Tri::new(*t)).collect();
        let bbox = mesh.bbox();
        let size = bbox.size();
        let n = tris.len().max(1) as f64;
        let area = (size.x * size.y).max(1e-12);
        let mut cell = (area / n).sqrt().max(1e-3);
        let min_cell = (area / 4_000_000.0).sqrt();
        if cell < min_cell {
            cell = min_cell;
        }
        let nx = ((size.x / cell).ceil() as usize + 1).max(1);
        let ny = ((size.y / cell).ceil() as usize + 1).max(1);
        let origin = bbox.min.truncate();
        let bin = |q: DVec2| -> (usize, usize) {
            let i = ((q.x - origin.x) / cell).floor().clamp(0.0, (nx - 1) as f64) as usize;
            let j = ((q.y - origin.y) / cell).floor().clamp(0.0, (ny - 1) as f64) as usize;
            (i, j)
        };
        let mut counts = vec![0u32; nx * ny + 1];
        for t in &tris {
            let (i0, j0) = bin(t.min);
            let (i1, j1) = bin(t.max);
            for j in j0..=j1 {
                for i in i0..=i1 {
                    counts[j * nx + i + 1] += 1;
                }
            }
        }
        for k in 1..counts.len() {
            counts[k] += counts[k - 1];
        }
        let starts = counts.clone();
        let mut fill = counts;
        let mut items = vec![0u32; *starts.last().unwrap() as usize];
        for (id, t) in tris.iter().enumerate() {
            let (i0, j0) = bin(t.min);
            let (i1, j1) = bin(t.max);
            for j in j0..=j1 {
                for i in i0..=i1 {
                    let k = j * nx + i;
                    items[fill[k] as usize] = id as u32;
                    fill[k] += 1;
                }
            }
        }
        MeshSurface { tris, origin, cell, nx, ny, starts, items, bbox }
    }

    pub fn bbox(&self) -> Aabb3 {
        self.bbox
    }

    pub fn triangle_count(&self) -> usize {
        self.tris.len()
    }

    fn bin_of(&self, q: DVec2) -> (isize, isize) {
        (
            ((q.x - self.origin.x) / self.cell).floor() as isize,
            ((q.y - self.origin.y) / self.cell).floor() as isize,
        )
    }

    /// Visit every triangle whose XY box meets the box `[min, max]` exactly once.
    fn visit(&self, min: DVec2, max: DVec2, mut f: impl FnMut(u32)) {
        let (i0, j0) = self.bin_of(min);
        let (i1, j1) = self.bin_of(max);
        let (i0, j0) = (i0.max(0), j0.max(0));
        let (i1, j1) = (i1.min(self.nx as isize - 1), j1.min(self.ny as isize - 1));
        if i0 > i1 || j0 > j1 {
            return;
        }
        for j in j0..=j1 {
            for i in i0..=i1 {
                let k = j as usize * self.nx + i as usize;
                for &id in &self.items[self.starts[k] as usize..self.starts[k + 1] as usize] {
                    let t = &self.tris[id as usize];
                    if t.min.x > max.x || t.max.x < min.x || t.min.y > max.y || t.max.y < min.y {
                        continue;
                    }
                    // A triangle spanning several bins is handled in the first bin of the overlap
                    // between its own range and the query range.
                    let (ti, tj) = self.bin_of(t.min);
                    if (ti.max(i0), tj.max(j0)) != (i, j) {
                        continue;
                    }
                    f(id);
                }
            }
        }
    }

    /// Contact of the cutter with one triangle: the highest tip Z forced by its facet or edges.
    fn contact(&self, t: &Tri, c: &Cutter, p: DVec2, best: &mut f64) {
        let reach = c.reach();
        if let Some((dir, s)) = t.slope {
            // Facet: the profile touches the plane at the closed-form contact radius; the contact
            // point must lie on the facet.
            let r = c.contact_radius(s).min(reach);
            let q = p + dir * r;
            if t.contains_xy(q) {
                let z = t.plane_z(q) - c.height(r).unwrap_or(0.0);
                if z > *best {
                    *best = z;
                }
            }
        }
        for k in 0..3 {
            let (a, b) = (t.p[k], t.p[(k + 1) % 3]);
            edge_contact(a, b, c, p, best);
        }
    }
}

/// Highest tip Z forced by the segment `a-b`, folded into `best`.
fn edge_contact(a: DVec3, b: DVec3, c: &Cutter, p: DVec2, best: &mut f64) {
    let reach = c.reach();
    let d = (b - a).truncate();
    let e = a.truncate() - p;
    let dd = d.length_squared();
    let dz = b.z - a.z;
    if dd < 1e-20 {
        // Vertical edge: one radial distance, contact at its top.
        let r = e.length();
        if r <= reach {
            let z = a.z.max(b.z) - c.height(r).unwrap_or(0.0);
            if z > *best {
                *best = z;
            }
        }
        return;
    }
    // Parameter range where the edge lies inside the footprint: |e + t d|^2 <= reach^2.
    let bq = e.dot(d);
    let cq = e.length_squared() - reach * reach;
    let disc = bq * bq - dd * cq;
    if disc < 0.0 {
        return;
    }
    let sq = disc.sqrt();
    let t0 = ((-bq - sq) / dd).max(0.0);
    let t1 = ((-bq + sq) / dd).min(1.0);
    if t0 > t1 {
        return;
    }
    // Quick bound: the highest point of the edge over the nearest footprint distance.
    let tc = (-bq / dd).clamp(t0, t1);
    let rmin = (e + d * tc).length();
    if a.z.max(b.z) - c.height(rmin).unwrap_or(0.0) <= *best {
        return;
    }
    let f = |t: f64| a.z + t * dz - c.height((e + d * t).length()).unwrap_or(f64::INFINITY);
    let width = SEARCH_TOL / dd.sqrt();
    let (_, z) = maximise(t0, t1, width, f);
    if z > *best {
        *best = z;
    }
}

impl<T: SurfaceQuery + ?Sized> SurfaceQuery for &T {
    fn drop(&self, c: &Cutter, p: DVec2) -> f64 {
        (**self).drop(c, p)
    }
}

impl SurfaceQuery for MeshSurface {
    fn drop(&self, c: &Cutter, p: DVec2) -> f64 {
        let reach = c.reach();
        let r = DVec2::splat(reach);
        // Candidates with an upper bound on their contact height (top of the triangle over the
        // nearest footprint distance), evaluated best-first so most are pruned by the bound.
        let mut cands: Vec<(f64, u32)> = Vec::new();
        self.visit(p - r, p + r, |id| {
            let t = &self.tris[id as usize];
            let dmin = t.xy_distance(p);
            if dmin <= reach {
                cands.push((t.zmax - c.height(dmin).unwrap_or(0.0), id));
            }
        });
        cands.sort_unstable_by(|a, b| b.0.total_cmp(&a.0));
        let mut best = f64::NEG_INFINITY;
        for (bound, id) in cands {
            if bound <= best {
                break;
            }
            self.contact(&self.tris[id as usize], c, p, &mut best);
        }
        best
    }
}

/// A Z-map with its precomputed kernel, exposed through the query interface.
pub struct ZMapSurface<'a> {
    pub map: &'a crate::zmap::ZMap,
    pub kernel: &'a crate::zmap::Kernel,
}

impl SurfaceQuery for ZMapSurface<'_> {
    fn drop(&self, _c: &Cutter, p: DVec2) -> f64 {
        self.map.drop_tool(self.kernel, p)
    }
}

/// Result of sampling a 2D path on the surface.
#[derive(Debug, Clone, Default)]
pub struct SampledPath {
    /// Tool-tip positions; every point is an exact contact height (clamped to the floor).
    pub pts: Vec<DVec3>,
    /// Largest 3D distance between a chord and the exact midpoint contact that could not be
    /// reduced further because the step reached `min_step`. Zero when every chord met `tol`.
    pub residual: f64,
    /// Contact queries made.
    pub samples: usize,
}

/// Sampling limits for [`drop_polyline`].
#[derive(Debug, Clone, Copy)]
pub struct Sampling {
    /// Longest chord before refinement. Must not exceed the cutter reach, or a feature narrower
    /// than the footprint could fall between samples.
    pub max_step: f64,
    /// Chords at or below this length are not refined further. At a vertical step the chord's XY
    /// length bounds the lateral overshoot, so this is half the tolerance.
    pub min_step: f64,
    /// Accepted 3D distance from a chord to the exact contact at its midpoint.
    pub tol: f64,
    /// Heights never go below this (the operation bottom).
    pub floor: f64,
}

impl Sampling {
    /// Limits for an operation tolerance: chords are accepted within half of it so that, with the
    /// path simplified within the other half, the emitted path stays within `tolerance` of the
    /// sampled contacts; `min_step` equals that half, bounding the lateral overshoot at a step.
    pub fn new(c: &Cutter, resolution: f64, tolerance: f64, floor: f64) -> Self {
        let tol = (tolerance.max(1e-5)) * 0.5;
        let min_step = tol.max(1e-4);
        let max_step = resolution.max(min_step).min(c.reach().max(min_step));
        Sampling { max_step, min_step, tol, floor }
    }
}

/// Drop the cutter along a 2D polyline, refining each chord until the exact contact at its
/// midpoint lies within `tol` of it (or the chord is shorter than `min_step`).
pub fn drop_polyline(q: &dyn SurfaceQuery, c: &Cutter, pts: &[DVec2], s: &Sampling) -> SampledPath {
    let mut out = SampledPath::default();
    if pts.is_empty() {
        return out;
    }
    let mut drop = |p: DVec2| -> DVec3 {
        out.samples += 1;
        p.extend(q.drop(c, p).max(s.floor))
    };
    let first = drop(pts[0]);
    out.pts.push(first);
    let mut stack: Vec<(DVec3, DVec3)> = Vec::new();
    for w in pts.windows(2) {
        let (a2, b2) = (w[0], w[1]);
        let len = a2.distance(b2);
        if len < 1e-12 {
            continue;
        }
        let n = ((len / s.max_step).ceil() as usize).max(1);
        let mut prev = *out.pts.last().unwrap();
        let mut knots = Vec::with_capacity(n);
        for k in 1..=n {
            let p = a2.lerp(b2, k as f64 / n as f64);
            knots.push(drop(p));
        }
        for b in knots {
            stack.push((prev, b));
            while let Some((a, b)) = stack.pop() {
                let l = a.truncate().distance(b.truncate());
                let m2 = (a.truncate() + b.truncate()) * 0.5;
                let m = drop(m2);
                let err = point_segment_distance(m, a, b);
                if err > s.tol && l > s.min_step {
                    // Process the first half next: it is pushed last.
                    stack.push((m, b));
                    stack.push((a, m));
                } else {
                    if err > s.tol && err > out.residual {
                        out.residual = err;
                    }
                    out.pts.push(m);
                    out.pts.push(b);
                }
            }
            prev = b;
        }
    }
    out
}

fn point_segment_distance(p: DVec3, a: DVec3, b: DVec3) -> f64 {
    let ab = b - a;
    let l2 = ab.length_squared();
    let t = if l2 > 0.0 { ((p - a).dot(ab) / l2).clamp(0.0, 1.0) } else { 0.0 };
    p.distance(a + ab * t)
}

/// Fill a Z-map grid with exact contact heights (the cutter-location map used by level-based
/// operations). Linear interpolation between nodes is the only approximation left.
pub fn cl_map(q: &dyn SurfaceQuery, c: &Cutter, min: DVec2, max: DVec2, cell: f64, floor: f64) -> crate::zmap::ZMap {
    let mut out = crate::zmap::ZMap::new(min, max, cell, floor);
    let (nx, origin) = (out.nx, out.origin);
    out.z.par_chunks_mut(nx).enumerate().for_each(|(j, row)| {
        for (i, v) in row.iter_mut().enumerate() {
            let p = origin + DVec2::new(i as f64, j as f64) * cell;
            *v = q.drop(c, p).max(floor);
        }
    });
    out
}

/// Measured deviation of a 3D polyline from the contact surface, sampled at most `spacing`
/// apart along every chord (and at least four points per chord).
#[derive(Debug, Clone, Copy, Default)]
pub struct PathDeviation {
    /// Largest 3D distance bound by which the tip sits inside the contact surface (a gouge).
    pub gouge: f64,
    pub gouge_at: DVec3,
    /// Largest vertical distance the tip sits above the contact height (material left on
    /// shallow surfaces; on walls this reads the step height).
    pub above: f64,
    pub above_at: DVec3,
    pub samples: usize,
}

/// Accumulate the deviation of a 3D polyline into `d`. The running maximum gouge serves as the
/// ceiling for [`gouge_distance`], so samples that cannot raise it are cheap to dismiss.
pub fn path_deviation(q: &dyn SurfaceQuery, c: &Cutter, pts: &[DVec3], spacing: f64, floor: f64, d: &mut PathDeviation) {
    let mut probe = |p: DVec3| {
        d.samples += 1;
        let cl = q.drop(c, p.truncate()).max(floor);
        if cl - p.z > d.gouge {
            let g = gouge_distance(q, c, p, d.gouge);
            if g > d.gouge {
                d.gouge = g;
                d.gouge_at = p;
            }
        }
        if p.z - cl > d.above {
            d.above = p.z - cl;
            d.above_at = p;
        }
    };
    if let Some(&p) = pts.first() {
        probe(p);
    }
    for w in pts.windows(2) {
        let n = ((w[0].distance(w[1]) / spacing.max(1e-6)).ceil() as usize).max(4);
        for k in 1..=n {
            probe(w[0].lerp(w[1], k as f64 / n as f64));
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use warp_tools::ToolShape;

    fn plane_mesh(slope_deg: f64) -> Mesh {
        // z = tan(slope) * x over x,y in [-50, 50].
        let s = slope_deg.to_radians().tan();
        let p = |x: f64, y: f64| DVec3::new(x, y, s * x);
        Mesh {
            tris: vec![
                [p(-50., -50.), p(50., -50.), p(50., 50.)],
                [p(-50., -50.), p(50., 50.), p(-50., 50.)],
            ],
            provenance: None,
        }
    }

    #[test]
    fn contacts_on_a_tilted_plane_match_closed_forms() {
        let theta: f64 = 30f64.to_radians();
        let s = theta.tan();
        let surf = MeshSurface::new(&plane_mesh(30.0));
        let at = DVec2::new(3.0, -2.0);
        let base = s * at.x;
        let cases: Vec<(Tool, f64)> = vec![
            (Tool::new(1, "flat", ToolShape::FlatEndMill, 6.0), base + 3.0 * s),
            (Tool::new(2, "ball", ToolShape::BallEndMill, 6.0), base + 3.0 * (1.0 / theta.cos() - 1.0)),
            (
                Tool::new(3, "bull", ToolShape::BullNose { corner_radius: 1.0 }, 6.0),
                base + 2.0 * s + 1.0 * (1.0 / theta.cos() - 1.0),
            ),
            // 90 degree V: cone slope 1 > plane slope, so the tip (radius 0.1 flat) rim touches.
            (Tool::new(4, "v90", ToolShape::VBit { included_angle: 90.0, tip_diameter: 0.2 }, 6.0), base + 0.1 * s),
            // 150 degree V: cone slope tan(15 deg) < plane slope, so the outer rim touches.
            (
                Tool::new(5, "v150", ToolShape::VBit { included_angle: 150.0, tip_diameter: 0.0 }, 6.0),
                base + 3.0 * s - 3.0 / 75f64.to_radians().tan(),
            ),
        ];
        for (tool, expected) in cases {
            let c = Cutter::new(&tool, 0.0, 0.0);
            let z = surf.drop(&c, at);
            assert!((z - expected).abs() < 1e-6, "{}: {z} vs {expected}", tool.name);
        }
        // Stock to leave: the grown flat tool reaches 0.5 further up the slope and sits 0.2 higher.
        let c = Cutter::new(&Tool::new(1, "flat", ToolShape::FlatEndMill, 6.0), 0.5, 0.2);
        let z = surf.drop(&c, at);
        assert!((z - (base + 3.5 * s + 0.2)).abs() < 1e-6, "{z}");
        // A grown ball: the 0.5 mm horizontal sweep moves the contact 0.5 further up the slope.
        let c = Cutter::new(&Tool::new(2, "ball", ToolShape::BallEndMill, 6.0), 0.5, 0.2);
        let z = surf.drop(&c, at);
        assert!((z - (base + 0.5 * s + 3.0 * (1.0 / theta.cos() - 1.0) + 0.2)).abs() < 1e-6, "{z}");
        // Tapered ball: the 1 mm tip ball touches a 30 degree plane (below the 10 degree taper's
        // 80 degree slope), like a 1 mm ball.
        let c = Cutter::new(&Tool::new(5, "taper", ToolShape::TaperedBall { taper_angle: 10.0, tip_radius: 1.0 }, 6.0), 0.0, 0.0);
        let z = surf.drop(&c, at);
        assert!((z - (base + 1.0 * (1.0 / theta.cos() - 1.0))).abs() < 1e-6, "{z}");
    }

    #[test]
    fn nothing_under_the_footprint_is_unbounded() {
        let surf = MeshSurface::new(&plane_mesh(0.0));
        let c = Cutter::new(&Tool::new(1, "flat", ToolShape::FlatEndMill, 6.0), 0.0, 0.0);
        assert_eq!(surf.drop(&c, DVec2::new(60.0, 0.0)), f64::NEG_INFINITY);
        assert_eq!(surf.drop(&c, DVec2::new(52.9, 0.0)), 0.0);
    }

    #[test]
    fn refinement_measures_its_residual() {
        let surf = MeshSurface::new(&plane_mesh(20.0));
        let c = Cutter::new(&Tool::new(2, "ball", ToolShape::BallEndMill, 6.0), 0.0, 0.0);
        let s = Sampling::new(&c, 0.5, 0.01, -100.0);
        assert!((s.tol - 0.005).abs() < 1e-12 && (s.min_step - 0.005).abs() < 1e-12);
        let path = drop_polyline(&surf, &c, &[DVec2::new(-40.0, 0.0), DVec2::new(40.0, 0.0)], &s);
        assert_eq!(path.residual, 0.0);
        assert!(path.pts.len() >= 161, "{}", path.pts.len());
        for w in path.pts.windows(2) {
            assert!(w[0].truncate().distance(w[1].truncate()) <= 0.5 + 1e-9);
        }
    }
}
