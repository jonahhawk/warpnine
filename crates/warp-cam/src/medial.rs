//! Approximate medial axis of a region, for V-carving.
//!
//! The boundary is sampled densely and Delaunay-triangulated. The circumcentres of the triangles
//! are Voronoi vertices; the Voronoi edges between samples that are far apart along the boundary
//! form the medial axis (edges between neighbouring samples are "hairs" that run straight to the
//! boundary and are dropped). Every point on the axis is then given its exact distance to the
//! boundary polygon, which is the radius of the largest circle centred there that fits in the
//! region: the cut depth that keeps a V-bit tangent to both sides.

use crate::region::{ring_pts, Region};
use glam::DVec2;
use spade::{DelaunayTriangulation, HasPosition, Point2, Triangulation};
use warp_core::Polyline2;

/// Samples this close along the same ring (in sample steps) only produce hairs.
const HAIR: usize = 2;

#[derive(Clone, Copy)]
struct Sample {
    p: Point2<f64>,
    ring: u32,
    idx: u32,
}

impl HasPosition for Sample {
    type Scalar = f64;
    fn position(&self) -> Point2<f64> {
        self.p
    }
}

struct Seg {
    a: DVec2,
    b: DVec2,
    /// Inward pseudo-normals at `a` and `b` (sum of the two adjacent edges' inward normals).
    na: DVec2,
    nb: DVec2,
}

/// Exact distance to a region's boundary, with inside/outside, via a uniform grid of edges.
pub struct Boundary {
    segs: Vec<Seg>,
    cells: Vec<Vec<u32>>,
    min: DVec2,
    h: f64,
    nx: usize,
    ny: usize,
}

impl Boundary {
    /// `rings` must have the region's interior on their left (exteriors CCW, holes CW).
    fn new(rings: &[Vec<DVec2>]) -> Boundary {
        let mut segs = Vec::new();
        for r in rings {
            let n = r.len();
            let inward = |i: usize| (r[(i + 1) % n] - r[i]).perp().normalize_or_zero();
            for i in 0..n {
                let (prev, cur, next) = (inward((i + n - 1) % n), inward(i), inward((i + 1) % n));
                segs.push(Seg { a: r[i], b: r[(i + 1) % n], na: prev + cur, nb: cur + next });
            }
        }
        let (mut min, mut max) = (DVec2::splat(f64::INFINITY), DVec2::splat(f64::NEG_INFINITY));
        for s in &segs {
            min = min.min(s.a);
            max = max.max(s.a);
        }
        let size = (max - min).max(DVec2::splat(1e-6));
        // Roughly a few edges per cell.
        let h = (size.x * size.y / segs.len().max(1) as f64 * 4.0).sqrt().max(size.max_element() / 2000.0).max(1e-3);
        let nx = (size.x / h).ceil() as usize + 1;
        let ny = (size.y / h).ceil() as usize + 1;
        let mut cells = vec![Vec::new(); nx * ny];
        for (k, s) in segs.iter().enumerate() {
            let (lo, hi) = (s.a.min(s.b), s.a.max(s.b));
            let (x0, y0) = (((lo.x - min.x) / h) as usize, ((lo.y - min.y) / h) as usize);
            let (x1, y1) = ((((hi.x - min.x) / h) as usize).min(nx - 1), (((hi.y - min.y) / h) as usize).min(ny - 1));
            for y in y0..=y1 {
                for x in x0..=x1 {
                    cells[y * nx + x].push(k as u32);
                }
            }
        }
        Boundary { segs, cells, min, h, nx, ny }
    }

    /// Distance from `p` to the boundary if `p` is inside the region, else None.
    pub fn dist(&self, p: DVec2) -> Option<f64> {
        let q = (p - self.min) / self.h;
        if q.x < 0.0 || q.y < 0.0 || q.x >= self.nx as f64 || q.y >= self.ny as f64 {
            return None;
        }
        let (cx, cy) = (q.x as i64, q.y as i64);
        let mut best = (f64::INFINITY, 0usize, 0.0f64);
        let max_ring = self.nx.max(self.ny) as i64;
        for k in 0..=max_ring {
            for y in cy - k..=cy + k {
                for x in cx - k..=cx + k {
                    let on_ring = (y - cy).abs() == k || (x - cx).abs() == k;
                    if !on_ring || x < 0 || y < 0 || x >= self.nx as i64 || y >= self.ny as i64 {
                        continue;
                    }
                    for &s in &self.cells[y as usize * self.nx + x as usize] {
                        let sg = &self.segs[s as usize];
                        let ab = sg.b - sg.a;
                        let t = ((p - sg.a).dot(ab) / ab.length_squared().max(1e-300)).clamp(0.0, 1.0);
                        let d = p.distance(sg.a + ab * t);
                        if d < best.0 {
                            best = (d, s as usize, t);
                        }
                    }
                }
            }
            // Anything not yet visited is at least k cells away.
            if best.0 <= k as f64 * self.h {
                break;
            }
        }
        if !best.0.is_finite() {
            return None;
        }
        let sg = &self.segs[best.1];
        let n = if best.2 <= 0.0 {
            sg.na
        } else if best.2 >= 1.0 {
            sg.nb
        } else {
            (sg.b - sg.a).perp()
        };
        let foot = sg.a + (sg.b - sg.a) * best.2;
        ((p - foot).dot(n) >= 0.0).then_some(best.0)
    }
}

/// The medial axis as polylines (in XY), plus the boundary for distance queries.
pub struct MedialAxis {
    pub chains: Vec<Vec<DVec2>>,
    pub boundary: Boundary,
}

pub fn medial_axis(reg: &Region, spacing: f64) -> MedialAxis {
    // Rings with the interior on the left.
    let mut rings: Vec<Vec<DVec2>> = Vec::new();
    for poly in &reg.0 {
        let mut e = Polyline2::new(ring_pts(poly.exterior()), true);
        if !e.is_ccw() {
            e = e.reversed();
        }
        rings.push(e.pts);
        for h in poly.interiors() {
            let mut h = Polyline2::new(ring_pts(h), true);
            if h.is_ccw() {
                h = h.reversed();
            }
            rings.push(h.pts);
        }
    }
    rings.retain(|r| r.len() >= 3);
    let boundary = Boundary::new(&rings);

    // Keep the sample count sane on very large jobs.
    let perimeter: f64 = rings.iter().map(|r| r.iter().zip(r.iter().cycle().skip(1)).map(|(a, b)| a.distance(*b)).sum::<f64>()).sum();
    let s = spacing.max(0.01).max(perimeter / 400_000.0);

    let mut samples: Vec<Vec<DVec2>> = Vec::new();
    let mut verts = Vec::new();
    for (ri, r) in rings.iter().enumerate() {
        let mut pts = Vec::new();
        for (i, &a) in r.iter().enumerate() {
            let b = r[(i + 1) % r.len()];
            let n = (a.distance(b) / s).ceil().max(1.0) as usize;
            for k in 0..n {
                pts.push(a.lerp(b, k as f64 / n as f64));
            }
        }
        pts.dedup_by(|x, y| x.distance(*y) < 1e-9);
        for (i, p) in pts.iter().enumerate() {
            verts.push(Sample { p: Point2::new(p.x, p.y), ring: ri as u32, idx: i as u32 });
        }
        samples.push(pts);
    }
    let Ok(tri) = DelaunayTriangulation::<Sample>::bulk_load(verts) else {
        return MedialAxis { chains: vec![], boundary };
    };

    // Nodes: circumcentres that lie inside the region.
    let nf = tri.num_all_faces();
    let mut node: Vec<Option<DVec2>> = vec![None; nf];
    let mut face_samples: Vec<[Sample; 3]> = vec![[Sample { p: Point2::new(0.0, 0.0), ring: 0, idx: 0 }; 3]; nf];
    for f in tri.inner_faces() {
        face_samples[f.fix().index()] = f.vertices().map(|v| *v.data());
        let c = f.circumcenter();
        let c = DVec2::new(c.x, c.y);
        if c.is_finite() && boundary.dist(c).is_some() {
            node[f.fix().index()] = Some(c);
        }
    }
    let far_apart = |a: &Sample, b: &Sample| {
        if a.ring != b.ring {
            return true;
        }
        let n = samples[a.ring as usize].len();
        let d = (a.idx as i64 - b.idx as i64).unsigned_abs() as usize;
        d.min(n - d) > HAIR
    };
    let mut adj: Vec<Vec<usize>> = vec![Vec::new(); nf];
    for e in tri.undirected_edges() {
        let d = e.as_directed();
        let [a, b] = d.vertices();
        if !far_apart(a.data(), b.data()) {
            continue;
        }
        let (Some(f1), Some(f2)) = (d.face().as_inner(), d.rev().face().as_inner()) else { continue };
        let (i, j) = (f1.fix().index(), f2.fix().index());
        if node[i].is_some() && node[j].is_some() {
            adj[i].push(j);
            adj[j].push(i);
        }
    }

    // Walk the graph into chains between junctions/leaves, then any pure cycles.
    let mut used = std::collections::HashSet::new();
    let key = |a: usize, b: usize| (a.min(b), a.max(b));
    let mut chains: Vec<Vec<usize>> = Vec::new();
    let walk = |start: usize, next: usize, used: &mut std::collections::HashSet<(usize, usize)>| {
        let mut chain = vec![start];
        let (mut prev, mut cur) = (start, next);
        used.insert(key(prev, cur));
        loop {
            chain.push(cur);
            if adj[cur].len() != 2 {
                break;
            }
            let nxt = if adj[cur][0] == prev { adj[cur][1] } else { adj[cur][0] };
            if !used.insert(key(cur, nxt)) {
                break;
            }
            prev = cur;
            cur = nxt;
        }
        chain
    };
    for i in 0..nf {
        if !adj[i].is_empty() && adj[i].len() != 2 {
            for k in 0..adj[i].len() {
                let j = adj[i][k];
                if !used.contains(&key(i, j)) {
                    chains.push(walk(i, j, &mut used));
                }
            }
        }
    }
    for i in 0..nf {
        if adj[i].len() == 2 && !used.contains(&key(i, adj[i][0])) {
            chains.push(walk(i, adj[i][0], &mut used));
        }
    }

    // Leaves next to a sharp corner stop a couple of samples short of it (the hair filter):
    // extend them straight to the corner, where the bit comes up to the surface.
    let corner = |f: usize| -> Option<DVec2> {
        let c = node[f]?;
        let vs = face_samples[f];
        if vs.iter().any(|v| v.ring != vs[0].ring) {
            return None;
        }
        let ring = &samples[vs[0].ring as usize];
        let n = ring.len();
        let mut idx: Vec<usize> = vs.iter().map(|v| v.idx as usize).collect();
        idx.sort_unstable();
        // Smallest circular arc covering the three samples: skip the largest gap.
        let gaps = [idx[1] - idx[0], idx[2] - idx[1], n - idx[2] + idx[0]];
        let (gi, &gmax) = gaps.iter().enumerate().max_by_key(|g| *g.1)?;
        let (from, len) = (idx[(gi + 1) % 3], n - gmax);
        if len > 2 * HAIR + 2 {
            return None;
        }
        let far = (0..=len).map(|k| ring[(from + k) % n]).max_by(|a, b| a.distance(c).total_cmp(&b.distance(c)))?;
        let r = boundary.dist(c)?;
        (far.distance(c) > r * 1.05 + 1e-9).then_some(far)
    };
    let to_pts = |ch: &[usize]| -> Vec<DVec2> {
        let mut pts: Vec<DVec2> = ch.iter().filter_map(|&f| node[f]).collect();
        pts.dedup_by(|a, b| a.distance(*b) < 1e-9);
        pts
    };
    let mut out = Vec::new();
    for ch in &chains {
        let mut pts = to_pts(ch);
        if pts.is_empty() {
            continue;
        }
        let extend = |pts: &mut Vec<DVec2>, f: usize| {
            if adj[f].len() == 1 {
                if let Some(tip) = corner(f) {
                    let from = *pts.last().unwrap();
                    let n = (from.distance(tip) / s).ceil().max(1.0) as usize;
                    pts.extend((1..=n).map(|k| from.lerp(tip, k as f64 / n as f64)));
                }
            }
        };
        extend(&mut pts, *ch.last().unwrap());
        pts.reverse();
        extend(&mut pts, ch[0]);
        if pts.len() >= 2 {
            out.push(pts);
        }
    }
    MedialAxis { chains: out, boundary }
}

