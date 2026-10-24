//! Z-map (height grid) of a mesh and drop-cutter queries for any tool shape.
//!
//! The mesh is rasterised to a grid of top-surface heights. Dropping a tool at (x, y) takes the
//! maximum over grid cells under the tool of `surface_z - profile_height(distance)`, which works
//! for every [`warp_tools::ToolShape`] because it only needs the tool's profile function.
//! Accuracy is limited by the grid cell size, which the operation's tolerance controls.

use glam::{DVec2, DVec3};
use rayon::prelude::*;
use warp_core::Mesh;
use warp_tools::Tool;

#[derive(Debug, Clone)]
pub struct ZMap {
    pub origin: DVec2,
    pub cell: f64,
    pub nx: usize,
    pub ny: usize,
    pub z: Vec<f64>,
}

impl ZMap {
    pub fn new(min: DVec2, max: DVec2, cell: f64, fill: f64) -> Self {
        let nx = (((max.x - min.x) / cell).ceil() as usize + 1).max(2);
        let ny = (((max.y - min.y) / cell).ceil() as usize + 1).max(2);
        ZMap { origin: min, cell, nx, ny, z: vec![fill; nx * ny] }
    }

    #[inline]
    pub fn at(&self, i: usize, j: usize) -> f64 {
        self.z[j * self.nx + i]
    }

    pub fn pos(&self, i: usize, j: usize) -> DVec2 {
        self.origin + DVec2::new(i as f64, j as f64) * self.cell
    }

    /// Rasterise the top surface of a mesh. Cells with no surface keep `floor`.
    pub fn from_mesh(mesh: &Mesh, min: DVec2, max: DVec2, cell: f64, floor: f64) -> Self {
        let mut zm = ZMap::new(min, max, cell, floor);
        let (nx, ny) = (zm.nx, zm.ny);
        let origin = zm.origin;
        // Rasterise in parallel by row bands to avoid write contention.
        let rows: Vec<(usize, Vec<f64>)> = (0..ny)
            .into_par_iter()
            .map(|j| {
                let y = origin.y + j as f64 * cell;
                let mut row = vec![floor; nx];
                for t in &mesh.tris {
                    let (ymin, ymax) = (t[0].y.min(t[1].y).min(t[2].y), t[0].y.max(t[1].y).max(t[2].y));
                    if y < ymin - cell * 0.5 || y > ymax + cell * 0.5 {
                        continue;
                    }
                    let xmin = t[0].x.min(t[1].x).min(t[2].x);
                    let xmax = t[0].x.max(t[1].x).max(t[2].x);
                    let i0 = (((xmin - origin.x) / cell).floor().max(0.0)) as usize;
                    let i1 = ((((xmax - origin.x) / cell).ceil()) as usize).min(nx - 1);
                    for (i, cellz) in row.iter_mut().enumerate().take(i1 + 1).skip(i0) {
                        let p = DVec2::new(origin.x + i as f64 * cell, y);
                        if let Some(z) = tri_z(t, p) {
                            if z > *cellz {
                                *cellz = z;
                            }
                        }
                    }
                    // Thin or vertical features: stamp vertices and edge samples into their cells.
                    for k in 0..3 {
                        let (a, b) = (t[k], t[(k + 1) % 3]);
                        let n = ((a.truncate().distance(b.truncate()) / cell).ceil() as usize).max(1);
                        for s in 0..=n {
                            let v = a.lerp(b, s as f64 / n as f64);
                            if ((v.y - origin.y) / cell).round() as isize != j as isize {
                                continue;
                            }
                            let i = ((v.x - origin.x) / cell).round();
                            if i >= 0.0 && (i as usize) < nx && v.z > row[i as usize] {
                                row[i as usize] = v.z;
                            }
                        }
                    }
                }
                (j, row)
            })
            .collect();
        for (j, row) in rows {
            zm.z[j * nx..(j + 1) * nx].copy_from_slice(&row);
        }
        zm
    }

    /// Lowest tool-tip Z at `p` that does not gouge the surface.
    pub fn drop_tool(&self, k: &Kernel, p: DVec2) -> f64 {
        let fi = (p.x - self.origin.x) / self.cell;
        let fj = (p.y - self.origin.y) / self.cell;
        let ci = fi.round() as isize;
        let cj = fj.round() as isize;
        let mut best = f64::NEG_INFINITY;
        for row in &k.rows {
            let j = cj + row.dj as isize;
            if j < 0 || j as usize >= self.ny {
                continue;
            }
            let src = &self.z[j as usize * self.nx..(j as usize + 1) * self.nx];
            let a = ci + row.di0 as isize;
            let lo = a.max(0);
            let hi = (a + row.h.len() as isize).min(self.nx as isize);
            for i in lo..hi {
                let v = src[i as usize] - row.h[(i - a) as usize];
                if v > best {
                    best = v;
                }
            }
        }
        best
    }

    /// Cutter-location map: the drop-cutter height at every grid node.
    ///
    /// The kernel is applied one footprint row at a time against contiguous map rows. A row whose
    /// profile height is constant (every row of a flat or face mill, the central rows of a bull
    /// nose) is a sliding-window maximum, computed in constant time per node with block prefix
    /// and suffix maxima; other rows take the direct loop over the chord.
    pub fn cl_map(&self, k: &Kernel) -> ZMap {
        let mut out = self.clone();
        let nx = self.nx;
        out.z.par_chunks_mut(nx).enumerate().for_each_init(
            || (Vec::new(), Vec::new(), Vec::new()),
            |(padded, prefix, suffix), (j, row)| {
                row.fill(f64::NEG_INFINITY);
                for kr in &k.rows {
                    let sj = j as isize + kr.dj as isize;
                    if sj < 0 || sj as usize >= self.ny {
                        continue;
                    }
                    let src = &self.z[sj as usize * nx..(sj as usize + 1) * nx];
                    match kr.flat {
                        Some(h0) => window_max(src, kr.di0, kr.h.len(), h0, row, padded, prefix, suffix),
                        None => {
                            for (i, v) in row.iter_mut().enumerate() {
                                let a = i as isize + kr.di0 as isize;
                                let lo = a.max(0);
                                let hi = (a + kr.h.len() as isize).min(nx as isize);
                                for x in lo..hi {
                                    let c = src[x as usize] - kr.h[(x - a) as usize];
                                    if c > *v {
                                        *v = c;
                                    }
                                }
                            }
                        }
                    }
                }
            },
        );
        out
    }

    /// Sample a 2D polyline at grid resolution and drop the tool at each point.
    pub fn drop_path(&self, k: &Kernel, pts: &[DVec2]) -> Vec<DVec3> {
        let mut out = Vec::new();
        for w in pts.windows(2) {
            let n = ((w[0].distance(w[1]) / self.cell).ceil() as usize).max(1);
            for s in 0..n {
                let p = w[0].lerp(w[1], s as f64 / n as f64);
                out.push(p.extend(self.drop_tool(k, p)));
            }
        }
        if let Some(&l) = pts.last() {
            out.push(l.extend(self.drop_tool(k, l)));
        }
        out
    }

    /// Iso-contours of the map at `level` (marching squares), as closed or open polylines.
    pub fn contours(&self, level: f64) -> Vec<warp_core::Polyline2> {
        let mut segs: Vec<(DVec2, DVec2)> = Vec::new();
        let edge = |i0: usize, j0: usize, i1: usize, j1: usize| {
            let (a, b) = (self.at(i0, j0), self.at(i1, j1));
            let t = ((level - a) / (b - a)).clamp(0.0, 1.0);
            self.pos(i0, j0).lerp(self.pos(i1, j1), t)
        };
        for j in 0..self.ny - 1 {
            for i in 0..self.nx - 1 {
                let c = [self.at(i, j), self.at(i + 1, j), self.at(i + 1, j + 1), self.at(i, j + 1)];
                let idx = c.iter().enumerate().fold(0, |acc, (k, v)| acc | (((*v > level) as usize) << k));
                if idx == 0 || idx == 15 {
                    continue;
                }
                let e = |k: usize| match k {
                    0 => edge(i, j, i + 1, j),
                    1 => edge(i + 1, j, i + 1, j + 1),
                    2 => edge(i, j + 1, i + 1, j + 1),
                    _ => edge(i, j, i, j + 1),
                };
                let mut add = |a: usize, b: usize| segs.push((e(a), e(b)));
                match idx {
                    1 | 14 => add(0, 3),
                    2 | 13 => add(0, 1),
                    3 | 12 => add(1, 3),
                    4 | 11 => add(1, 2),
                    6 | 9 => add(0, 2),
                    7 | 8 => add(2, 3),
                    5 => {
                        add(0, 1);
                        add(2, 3);
                    }
                    10 => {
                        add(0, 3);
                        add(1, 2);
                    }
                    _ => {}
                }
            }
        }
        let polys = segs.into_iter().map(|(a, b)| warp_core::Polyline2 { pts: vec![a, b], closed: false }).collect();
        warp_core::geom::chain(polys, self.cell * 1e-3)
    }
}

/// `out[i] = max(out[i], max(src[i + di0 .. i + di0 + w]) - h0)` for every `i`, with cells outside
/// `src` ignored. Van Herk / Gil-Werman: the window of width `w` spans at most two blocks of
/// width `w`, so its maximum is the suffix maximum of the first block from the window start
/// combined with the prefix maximum of the second block up to the window end.
#[allow(clippy::too_many_arguments)]
fn window_max(src: &[f64], di0: i32, w: usize, h0: f64, out: &mut [f64], padded: &mut Vec<f64>, prefix: &mut Vec<f64>, suffix: &mut Vec<f64>) {
    let n = src.len();
    if w == 0 || n == 0 {
        return;
    }
    // Pad by one window on each side so every window lies inside the padded array.
    let len = n + 2 * w;
    padded.clear();
    padded.resize(len, f64::NEG_INFINITY);
    padded[w..w + n].copy_from_slice(src);
    prefix.clear();
    prefix.resize(len, f64::NEG_INFINITY);
    suffix.clear();
    suffix.resize(len, f64::NEG_INFINITY);
    for start in (0..len).step_by(w) {
        let end = (start + w).min(len);
        let mut m = f64::NEG_INFINITY;
        for x in start..end {
            m = m.max(padded[x]);
            prefix[x] = m;
        }
        let mut m = f64::NEG_INFINITY;
        for x in (start..end).rev() {
            m = m.max(padded[x]);
            suffix[x] = m;
        }
    }
    for (i, v) in out.iter_mut().enumerate() {
        // Window start in padded coordinates: i + di0 + w >= 0 because |di0| < w.
        let a = (i as isize + di0 as isize + w as isize) as usize;
        let b = a + w - 1;
        if b >= len {
            // Window runs past the padding (cannot happen for |di0| < w, kept for safety).
            let m = padded[a..len].iter().copied().fold(f64::NEG_INFINITY, f64::max);
            *v = v.max(m - h0);
            continue;
        }
        let m = suffix[a].max(prefix[b]);
        if m > f64::NEG_INFINITY {
            *v = v.max(m - h0);
        }
    }
}

/// Predicted cost of a Z-map cutter-location map, so the user can see what the surface sampling
/// step buys before generating. Cells grow with the square of `1/cell` and the footprint samples
/// with it again, so the work of a non-flat tool grows with the fourth power of `1/cell`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ZMapWork {
    /// Grid nodes of the padded map.
    pub cells: usize,
    /// Footprint samples one drop evaluates.
    pub samples: usize,
    /// Footprint rows (the cost unit of a flat tool, whose rows are sliding-window maxima).
    pub rows: usize,
    /// Whether every footprint row is flat (flat end mill, face mill, slot or thread mill).
    pub flat: bool,
    /// Predicted sample visits: `cells * samples`, or about `3 * cells * rows` for flat tools.
    pub visits: f64,
}

impl ZMapWork {
    /// `extent` is the XY size of the area the tool centre covers (before padding by the tool
    /// radius, the radial stock and two cells, as the operations pad it).
    pub fn estimate(tool: &Tool, cell: f64, radial_leave: f64, extent: DVec2) -> Self {
        let cell = cell.clamp(0.01, 5.0);
        let rr = tool.radius() + radial_leave.max(0.0);
        let pad = rr + 2.0 * cell;
        let nx = (((extent.x + 2.0 * pad) / cell).ceil() as usize + 1).max(2);
        let ny = (((extent.y + 2.0 * pad) / cell).ceil() as usize + 1).max(2);
        let cells = nx * ny;
        let n = (rr / cell).ceil() as i64;
        let limit = (rr / cell) * (rr / cell) + 1e-9;
        let (mut samples, mut rows) = (0usize, 0usize);
        for dj in -n..=n {
            let rem = limit - (dj * dj) as f64;
            if rem < 0.0 {
                continue;
            }
            // Same criterion as `Kernel::new`: offsets with `di^2 + dj^2 <= (rr / cell)^2`.
            let half = rem.sqrt().floor() as usize;
            samples += 2 * half + 1;
            rows += 1;
        }
        let flat = (0..=16).all(|k| {
            let r = tool.radius() * k as f64 / 16.0;
            tool.profile_height(r).unwrap_or(0.0).abs() < 1e-12
        });
        let visits = if flat { 3.0 * cells as f64 * rows as f64 } else { cells as f64 * samples as f64 };
        ZMapWork { cells, samples, rows, flat, visits }
    }
}

/// One row of the tool footprint: the profile heights along the chord at row offset `dj`,
/// starting at column offset `di0`.
#[derive(Debug, Clone)]
pub struct KernelRow {
    pub dj: i32,
    pub di0: i32,
    pub h: Vec<f64>,
    /// The common height when every sample of the row is equal (a flat chord).
    pub flat: Option<f64>,
}

/// Precomputed tool footprint: the profile height at every grid offset, stored by row.
#[derive(Debug, Clone)]
pub struct Kernel {
    pub rows: Vec<KernelRow>,
}

impl Kernel {
    /// `radial_leave` / `axial_leave` grow the tool to leave stock on walls / floors.
    pub fn new(tool: &Tool, cell: f64, radial_leave: f64, axial_leave: f64) -> Self {
        let rr = tool.radius() + radial_leave;
        let n = (rr / cell).ceil() as i32;
        let mut rows = Vec::new();
        for dj in -n..=n {
            let mut h = Vec::new();
            let mut di0 = None;
            for di in -n..=n {
                let d = ((di * di + dj * dj) as f64).sqrt() * cell;
                if d > rr + 1e-9 {
                    continue;
                }
                di0.get_or_insert(di);
                h.push(tool.profile_height((d - radial_leave).max(0.0)).unwrap_or(0.0) - axial_leave);
            }
            let Some(di0) = di0 else { continue };
            let flat = h.iter().all(|&v| (v - h[0]).abs() <= 1e-12).then_some(h[0]);
            rows.push(KernelRow { dj, di0, h, flat });
        }
        Kernel { rows }
    }

    /// Number of footprint samples one drop evaluates.
    pub fn samples(&self) -> usize {
        self.rows.iter().map(|r| r.h.len()).sum()
    }

    /// Iterate the footprint as `(di, dj, height)` offsets.
    pub fn offsets(&self) -> impl Iterator<Item = (i32, i32, f64)> + '_ {
        self.rows.iter().flat_map(|r| r.h.iter().enumerate().map(move |(k, &h)| (r.di0 + k as i32, r.dj, h)))
    }
}

fn tri_z(t: &[DVec3; 3], p: DVec2) -> Option<f64> {
    let (a, b, c) = (t[0].truncate(), t[1].truncate(), t[2].truncate());
    let d = (b - a).perp_dot(c - a);
    if d.abs() < 1e-14 {
        return None;
    }
    let w1 = (p - a).perp_dot(c - a) / d;
    let w2 = (b - a).perp_dot(p - a) / d;
    let w0 = 1.0 - w1 - w2;
    let e = -1e-9;
    (w0 >= e && w1 >= e && w2 >= e).then(|| w0 * t[0].z + w1 * t[1].z + w2 * t[2].z)
}

#[cfg(test)]
mod tests {
    use super::*;
    use warp_tools::{Tool, ToolShape};

    fn flat_square(z: f64) -> Mesh {
        let p = |x, y| DVec3::new(x, y, z);
        Mesh { tris: vec![[p(0., 0.), p(10., 0.), p(10., 10.)], [p(0., 0.), p(10., 10.), p(0., 10.)]], provenance: None }
    }

    #[test]
    fn ball_drops_onto_plane() {
        let zm = ZMap::from_mesh(&flat_square(-2.0), DVec2::splat(-5.0), DVec2::splat(15.0), 0.1, -10.0);
        let k = Kernel::new(&Tool::new(1, "b", ToolShape::BallEndMill, 6.0), 0.1, 0.0, 0.0);
        assert!((zm.drop_tool(&k, DVec2::new(5.0, 5.0)) + 2.0).abs() < 1e-9);
        // Off the edge by 2mm: ball touches the edge, tip sits above the floor.
        let z = zm.drop_tool(&k, DVec2::new(12.0, 5.0));
        let expected = -2.0 - (3.0 - (9.0f64 - 4.0).sqrt());
        assert!((z - expected).abs() < 0.05, "{z} vs {expected}");
    }

    /// Reference drop: the plain maximum over every footprint offset.
    fn drop_reference(zm: &ZMap, k: &Kernel, i: usize, j: usize) -> f64 {
        let mut best = f64::NEG_INFINITY;
        for (di, dj, h) in k.offsets() {
            let (x, y) = (i as i64 + di as i64, j as i64 + dj as i64);
            if x < 0 || y < 0 || x as usize >= zm.nx || y as usize >= zm.ny {
                continue;
            }
            best = best.max(zm.at(x as usize, y as usize) - h);
        }
        best
    }

    #[test]
    fn cl_map_matches_the_plain_maximum_for_every_tool_shape() {
        // A rough random surface with a tall spike and a hole, so windows at the borders, around
        // the spike and across the floor are all exercised.
        let (nx, ny) = (37usize, 29usize);
        let mut zm = ZMap::new(DVec2::ZERO, DVec2::new((nx - 1) as f64 * 0.2, (ny - 1) as f64 * 0.2), 0.2, -5.0);
        let mut seed = 12345u64;
        for v in zm.z.iter_mut() {
            seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
            *v = -3.0 + (seed >> 40) as f64 / (1u64 << 24) as f64 * 4.0;
        }
        zm.z[14 * nx + 20] = 7.5;
        zm.z[5 * nx + 3] = -20.0;
        let tools = [
            Tool::new(1, "flat", ToolShape::FlatEndMill, 3.0),
            Tool::new(2, "ball", ToolShape::BallEndMill, 2.4),
            Tool::new(3, "bull", ToolShape::BullNose { corner_radius: 0.5 }, 3.0),
            Tool::new(4, "v", ToolShape::VBit { included_angle: 90.0, tip_diameter: 0.0 }, 4.0),
            Tool::new(5, "face", ToolShape::FaceMill, 6.2),
        ];
        for t in &tools {
            for (leave_r, leave_z) in [(0.0, 0.0), (0.3, 0.1)] {
                let k = Kernel::new(t, zm.cell, leave_r, leave_z);
                assert_eq!(k.samples(), k.offsets().count());
                let cl = zm.cl_map(&k);
                for j in 0..ny {
                    for i in 0..nx {
                        let want = drop_reference(&zm, &k, i, j);
                        let got = cl.at(i, j);
                        assert!((got - want).abs() < 1e-12, "{} leave {leave_r}: ({i},{j}) {got} vs {want}", t.name);
                        let single = zm.drop_tool(&k, zm.pos(i, j));
                        assert!((single - want).abs() < 1e-12, "{} ({i},{j}) drop_tool {single} vs {want}", t.name);
                    }
                }
            }
        }
        // The flat tools really take the fast path.
        let k = Kernel::new(&tools[0], zm.cell, 0.0, 0.0);
        assert!(k.rows.iter().all(|r| r.flat.is_some()));
        let k = Kernel::new(&tools[1], zm.cell, 0.0, 0.0);
        assert!(k.rows.iter().any(|r| r.flat.is_none()));
    }

    #[test]
    fn work_estimate_counts_the_real_kernel() {
        let flat = Tool::new(1, "flat", ToolShape::FlatEndMill, 12.7);
        let ball = Tool::new(2, "ball", ToolShape::BallEndMill, 6.0);
        for (t, cell, leave) in [(&flat, 0.1, 0.0), (&ball, 0.1, 0.0), (&ball, 0.25, 0.5), (&flat, 0.33, 0.2)] {
            let k = Kernel::new(t, cell, leave, 0.0);
            let w = ZMapWork::estimate(t, cell, leave, DVec2::new(40.0, 30.0));
            assert_eq!(w.samples, k.samples(), "{} at {cell}", t.name);
            assert_eq!(w.rows, k.rows.len());
            assert_eq!(w.flat, k.rows.iter().all(|r| r.flat.is_some()));
            // The padded map the operations build has the same node count.
            let pad = DVec2::splat(t.radius() + leave + 2.0 * cell);
            let zm = ZMap::new(-pad, DVec2::new(40.0, 30.0) + pad, cell, 0.0);
            // Within a row and a column: the operations round their padded extents slightly
            // differently, and a grid edge sitting on a cell boundary can go either way.
            assert!((w.cells as i64 - (zm.nx * zm.ny) as i64).abs() <= (zm.nx + zm.ny + 1) as i64, "{} vs {}", w.cells, zm.nx * zm.ny);
        }
        // Halving the step quadruples the cells and, for a ball, the samples too.
        let a = ZMapWork::estimate(&ball, 0.2, 0.0, DVec2::splat(50.0));
        let b = ZMapWork::estimate(&ball, 0.1, 0.0, DVec2::splat(50.0));
        assert!((b.visits / a.visits - 16.0).abs() < 1.0, "{} vs {}", a.visits, b.visits);
        let a = ZMapWork::estimate(&flat, 0.2, 0.0, DVec2::splat(50.0));
        let b = ZMapWork::estimate(&flat, 0.1, 0.0, DVec2::splat(50.0));
        assert!((b.visits / a.visits - 8.0).abs() < 0.6, "{} vs {}", a.visits, b.visits);
    }

    #[test]
    fn contour_of_plateau_is_closed() {
        let zm = ZMap::from_mesh(&flat_square(0.0), DVec2::splat(-5.0), DVec2::splat(15.0), 0.25, -10.0);
        let cs = zm.contours(-5.0);
        assert_eq!(cs.len(), 1);
        assert!(cs[0].closed);
    }
}
