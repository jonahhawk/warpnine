use glam::{DVec2, DVec3};
use serde::{Deserialize, Serialize};
use std::f64::consts::TAU;

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Aabb2 {
    pub min: DVec2,
    pub max: DVec2,
}

impl Aabb2 {
    pub const EMPTY: Aabb2 = Aabb2 {
        min: DVec2::splat(f64::INFINITY),
        max: DVec2::splat(f64::NEG_INFINITY),
    };

    pub fn add(&mut self, p: DVec2) {
        self.min = self.min.min(p);
        self.max = self.max.max(p);
    }

    pub fn union(&self, o: &Aabb2) -> Aabb2 {
        Aabb2 { min: self.min.min(o.min), max: self.max.max(o.max) }
    }

    pub fn is_empty(&self) -> bool {
        self.min.x > self.max.x
    }

    pub fn size(&self) -> DVec2 {
        self.max - self.min
    }

    pub fn center(&self) -> DVec2 {
        (self.min + self.max) * 0.5
    }

    pub fn contains(&self, p: DVec2) -> bool {
        p.x >= self.min.x && p.x <= self.max.x && p.y >= self.min.y && p.y <= self.max.y
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Aabb3 {
    pub min: DVec3,
    pub max: DVec3,
}

impl Aabb3 {
    pub const EMPTY: Aabb3 = Aabb3 {
        min: DVec3::splat(f64::INFINITY),
        max: DVec3::splat(f64::NEG_INFINITY),
    };

    pub fn add(&mut self, p: DVec3) {
        self.min = self.min.min(p);
        self.max = self.max.max(p);
    }

    pub fn is_empty(&self) -> bool {
        self.min.x > self.max.x
    }

    pub fn size(&self) -> DVec3 {
        self.max - self.min
    }

    pub fn xy(&self) -> Aabb2 {
        Aabb2 { min: self.min.truncate(), max: self.max.truncate() }
    }
}

/// A 2D polyline. Arcs are tessellated on import; closed polylines do not repeat the first point.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, Default)]
pub struct Polyline2 {
    pub pts: Vec<DVec2>,
    pub closed: bool,
}

impl Polyline2 {
    pub fn new(pts: Vec<DVec2>, closed: bool) -> Self {
        let mut p = Polyline2 { pts, closed };
        p.dedup(1e-9);
        if p.closed && p.pts.len() > 1 && p.pts[0].distance(*p.pts.last().unwrap()) < 1e-9 {
            p.pts.pop();
        }
        p
    }

    pub fn circle(center: DVec2, radius: f64, tol: f64) -> Self {
        let n = arc_segments(radius, TAU, tol).max(8);
        let pts = (0..n)
            .map(|i| center + DVec2::from_angle(TAU * i as f64 / n as f64) * radius)
            .collect();
        Polyline2 { pts, closed: true }
    }

    pub fn rect(min: DVec2, max: DVec2) -> Self {
        Polyline2 {
            pts: vec![min, DVec2::new(max.x, min.y), max, DVec2::new(min.x, max.y)],
            closed: true,
        }
    }

    pub fn dedup(&mut self, tol: f64) {
        self.pts.dedup_by(|a, b| a.distance(*b) < tol);
    }

    /// Iterate segments, including the closing segment for closed polylines.
    pub fn segments(&self) -> impl Iterator<Item = (DVec2, DVec2)> + '_ {
        let n = self.pts.len();
        let count = if self.closed && n > 1 { n } else { n.saturating_sub(1) };
        (0..count).map(move |i| (self.pts[i], self.pts[(i + 1) % n]))
    }

    /// Signed area (positive = counter-clockwise). Zero for open polylines.
    pub fn signed_area(&self) -> f64 {
        if !self.closed {
            return 0.0;
        }
        self.segments().map(|(a, b)| a.perp_dot(b)).sum::<f64>() * 0.5
    }

    pub fn is_ccw(&self) -> bool {
        self.signed_area() > 0.0
    }

    pub fn reversed(&self) -> Self {
        let mut pts = self.pts.clone();
        pts.reverse();
        Polyline2 { pts, closed: self.closed }
    }

    pub fn length(&self) -> f64 {
        self.segments().map(|(a, b)| a.distance(b)).sum()
    }

    pub fn bbox(&self) -> Aabb2 {
        let mut b = Aabb2::EMPTY;
        for p in &self.pts {
            b.add(*p);
        }
        b
    }

    /// Even-odd point-in-polygon test.
    pub fn contains(&self, p: DVec2) -> bool {
        let mut inside = false;
        for (a, b) in self.segments() {
            if (a.y > p.y) != (b.y > p.y) {
                let x = a.x + (p.y - a.y) / (b.y - a.y) * (b.x - a.x);
                if p.x < x {
                    inside = !inside;
                }
            }
        }
        inside
    }

    /// Point at arc-length `s` along the polyline, plus the index of the segment it lies on.
    pub fn point_at(&self, mut s: f64) -> (DVec2, usize) {
        for (i, (a, b)) in self.segments().enumerate() {
            let l = a.distance(b);
            if s <= l {
                return (a.lerp(b, if l > 0.0 { s / l } else { 0.0 }), i);
            }
            s -= l;
        }
        let last = if self.closed { self.pts[0] } else { *self.pts.last().unwrap_or(&DVec2::ZERO) };
        (last, self.pts.len().saturating_sub(1))
    }

    /// Rotate a closed polyline so it starts at the vertex nearest `p`.
    pub fn start_nearest(&mut self, p: DVec2) {
        if !self.closed || self.pts.is_empty() {
            return;
        }
        let i = self
            .pts
            .iter()
            .enumerate()
            .min_by(|a, b| a.1.distance_squared(p).total_cmp(&b.1.distance_squared(p)))
            .map(|(i, _)| i)
            .unwrap();
        self.pts.rotate_left(i);
    }

    pub fn translate(&mut self, d: DVec2) {
        for p in &mut self.pts {
            *p += d;
        }
    }
}

/// Number of chords needed to approximate an arc of `radius` spanning `sweep` radians within `tol`.
pub fn arc_segments(radius: f64, sweep: f64, tol: f64) -> usize {
    let r = radius.abs().max(1e-9);
    let tol = tol.clamp(1e-6, r);
    let max_angle = 2.0 * (1.0 - tol / r).clamp(-1.0, 1.0).acos();
    ((sweep.abs() / max_angle.max(1e-3)).ceil() as usize).clamp(1, 10_000)
}

/// Tessellate an arc. `start` and `end` are angles in radians, `ccw` gives the direction.
pub fn tessellate_arc(center: DVec2, radius: f64, start: f64, end: f64, ccw: bool, tol: f64) -> Vec<DVec2> {
    let mut sweep = end - start;
    if ccw {
        while sweep <= 0.0 {
            sweep += TAU;
        }
    } else {
        while sweep >= 0.0 {
            sweep -= TAU;
        }
    }
    let n = arc_segments(radius, sweep, tol);
    (0..=n)
        .map(|i| center + DVec2::from_angle(start + sweep * i as f64 / n as f64) * radius)
        .collect()
}

/// Tessellate a DXF-style bulge segment from `a` to `b` (excluding `a`, including `b`).
pub fn bulge_points(a: DVec2, b: DVec2, bulge: f64, tol: f64) -> Vec<DVec2> {
    if bulge.abs() < 1e-9 {
        return vec![b];
    }
    let theta = 4.0 * bulge.atan();
    let chord = b - a;
    let d = chord.length();
    if d < 1e-12 {
        return vec![b];
    }
    let r = d / (2.0 * (theta / 2.0).sin());
    let mid = (a + b) * 0.5;
    let h = r * (theta / 2.0).cos();
    let center = mid + chord.perp().normalize() * h;
    let sa = (a - center).to_angle();
    let ea = (b - center).to_angle();
    let mut pts = tessellate_arc(center, r.abs(), sa, ea, bulge > 0.0, tol);
    pts.remove(0);
    if let Some(l) = pts.last_mut() {
        *l = b;
    }
    pts
}

/// Join loose open polylines whose endpoints coincide (within `tol`) into longer chains,
/// closing chains whose ends meet.
pub fn chain(polys: Vec<Polyline2>, tol: f64) -> Vec<Polyline2> {
    let (mut done, mut open): (Vec<_>, Vec<_>) = polys.into_iter().partition(|p| p.closed);
    while let Some(mut cur) = open.pop() {
        loop {
            let head = cur.pts[0];
            let tail = *cur.pts.last().unwrap();
            let found = open.iter().position(|o| {
                let (s, e) = (o.pts[0], *o.pts.last().unwrap());
                s.distance(tail) < tol || e.distance(tail) < tol || s.distance(head) < tol || e.distance(head) < tol
            });
            let Some(i) = found else { break };
            let o = open.swap_remove(i);
            let (s, e) = (o.pts[0], *o.pts.last().unwrap());
            if s.distance(tail) < tol {
                cur.pts.extend_from_slice(&o.pts[1..]);
            } else if e.distance(tail) < tol {
                cur.pts.extend(o.pts.iter().rev().skip(1));
            } else if e.distance(head) < tol {
                let mut p = o.pts.clone();
                p.extend_from_slice(&cur.pts[1..]);
                cur.pts = p;
            } else {
                let mut p: Vec<_> = o.pts.iter().rev().copied().collect();
                p.extend_from_slice(&cur.pts[1..]);
                cur.pts = p;
            }
        }
        if cur.pts.len() > 2 && cur.pts[0].distance(*cur.pts.last().unwrap()) < tol {
            cur.pts.pop();
            cur.closed = true;
        }
        done.push(cur);
    }
    done
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rect_area_and_contains() {
        let r = Polyline2::rect(DVec2::ZERO, DVec2::new(10.0, 5.0));
        assert!((r.signed_area() - 50.0).abs() < 1e-9);
        assert!(r.contains(DVec2::new(5.0, 2.0)));
        assert!(!r.contains(DVec2::new(11.0, 2.0)));
    }

    #[test]
    fn chain_lines_into_closed_loop() {
        let l = |a: (f64, f64), b: (f64, f64)| Polyline2::new(vec![DVec2::new(a.0, a.1), DVec2::new(b.0, b.1)], false);
        let out = chain(vec![l((0., 0.), (1., 0.)), l((1., 1.), (1., 0.)), l((1., 1.), (0., 1.)), l((0., 1.), (0., 0.))], 1e-6);
        assert_eq!(out.len(), 1);
        assert!(out[0].closed);
        assert_eq!(out[0].pts.len(), 4);
    }

    #[test]
    fn semicircle_bulge() {
        let pts = bulge_points(DVec2::new(0., 0.), DVec2::new(2., 0.), 1.0, 0.001);
        for p in &pts {
            assert!((p.distance(DVec2::new(1., 0.)) - 1.0).abs() < 1e-6);
        }
        // positive bulge = CCW from a to b, which passes below the chord
        assert!(pts[pts.len() / 2].y < 0.0);
    }
}
