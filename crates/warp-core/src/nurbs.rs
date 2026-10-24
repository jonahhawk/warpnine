//! NURBS / B-spline evaluation and adaptive tessellation (for DXF SPLINE entities).

use glam::{DVec2, DVec3};

/// A planar NURBS curve. Knots must have `ctrl.len() + degree + 1` entries.
#[derive(Debug, Clone)]
pub struct Nurbs {
    pub degree: usize,
    pub knots: Vec<f64>,
    pub ctrl: Vec<DVec2>,
    pub weights: Vec<f64>,
}

impl Nurbs {
    /// Build from raw DXF data, repairing what can be repaired: missing/invalid knots become a
    /// clamped uniform vector, missing/mismatched weights become 1.
    pub fn new(degree: usize, knots: Vec<f64>, ctrl: Vec<DVec2>, weights: Vec<f64>) -> Option<Self> {
        let n = ctrl.len();
        if n < 2 {
            return None;
        }
        let degree = degree.clamp(1, n - 1);
        let valid = knots.len() == n + degree + 1 && knots.windows(2).all(|w| w[1] >= w[0]) && knots[n] > knots[degree];
        let knots = if valid { knots } else { clamped_uniform(n, degree) };
        let weights = if weights.len() == n && weights.iter().all(|w| *w > 0.0) { weights } else { vec![1.0; n] };
        Some(Nurbs { degree, knots, ctrl, weights })
    }

    pub fn domain(&self) -> (f64, f64) {
        (self.knots[self.degree], self.knots[self.ctrl.len()])
    }

    /// Point at parameter `u` (de Boor's algorithm in homogeneous coordinates).
    pub fn eval(&self, u: f64) -> DVec2 {
        let p = self.degree;
        let n = self.ctrl.len();
        let (u0, u1) = self.domain();
        let u = u.clamp(u0, u1);
        // Knot span k with knots[k] <= u < knots[k+1], within [p, n-1].
        let mut k = p;
        while k < n - 1 && u >= self.knots[k + 1] {
            k += 1;
        }
        let mut d: Vec<DVec3> = (0..=p)
            .map(|j| {
                let i = k - p + j;
                let w = self.weights[i];
                (self.ctrl[i] * w).extend(w)
            })
            .collect();
        for r in 1..=p {
            for j in (r..=p).rev() {
                let i = k - p + j;
                let denom = self.knots[i + p + 1 - r] - self.knots[i];
                let a = if denom.abs() < 1e-14 { 0.0 } else { (u - self.knots[i]) / denom };
                d[j] = d[j - 1] * (1.0 - a) + d[j] * a;
            }
        }
        let h = d[p];
        h.truncate() / h.z
    }

    /// Polyline within `tol` of the curve: each knot span is sampled, then refined recursively
    /// wherever the chord midpoint deviates from the curve by more than `tol`.
    pub fn tessellate(&self, tol: f64) -> Vec<DVec2> {
        let tol = tol.max(1e-6);
        let mut out = vec![self.eval(self.domain().0)];
        let spans: Vec<(f64, f64)> = self.knots[self.degree..=self.ctrl.len()].windows(2).filter(|w| w[1] > w[0]).map(|w| (w[0], w[1])).collect();
        for (a, b) in spans {
            let steps = 4 * self.degree.max(1);
            for s in 0..steps {
                let t0 = a + (b - a) * s as f64 / steps as f64;
                let t1 = a + (b - a) * (s + 1) as f64 / steps as f64;
                self.refine(t0, self.eval(t0), t1, self.eval(t1), tol, 0, &mut out);
            }
        }
        out.dedup_by(|x, y| x.distance(*y) < 1e-9);
        out
    }

    fn refine(&self, t0: f64, p0: DVec2, t1: f64, p1: DVec2, tol: f64, depth: u32, out: &mut Vec<DVec2>) {
        let tm = 0.5 * (t0 + t1);
        let pm = self.eval(tm);
        // Also probe the quarter points so S-shaped pieces with a straight midpoint still split.
        let dev = |t: f64, q: DVec2| {
            let c = p0.lerp(p1, (t - t0) / (t1 - t0));
            q.distance(c)
        };
        let tq = t0 + 0.25 * (t1 - t0);
        let tr = t0 + 0.75 * (t1 - t0);
        let worst = dev(tm, pm).max(dev(tq, self.eval(tq))).max(dev(tr, self.eval(tr)));
        if depth < 16 && worst > tol {
            self.refine(t0, p0, tm, pm, tol, depth + 1, out);
            self.refine(tm, pm, t1, p1, tol, depth + 1, out);
        } else {
            out.push(p1);
        }
    }
}

fn clamped_uniform(n: usize, p: usize) -> Vec<f64> {
    let inner = n - p; // number of spans
    let mut k = vec![0.0; p + 1];
    for i in 1..inner {
        k.push(i as f64 / inner as f64);
    }
    k.extend(std::iter::repeat_n(1.0, p + 1));
    k
}

/// Smooth curve through fit points (centripetal Catmull–Rom), used when a DXF spline only stores
/// fit points. AutoCAD's own fit-point splines are cubic interpolants, so this matches closely.
pub fn through_points(pts: &[DVec2], closed: bool, tol: f64) -> Vec<DVec2> {
    let n = pts.len();
    if n < 3 {
        return pts.to_vec();
    }
    let get = |i: isize| -> DVec2 {
        if closed {
            pts[i.rem_euclid(n as isize) as usize]
        } else if i < 0 {
            pts[0] * 2.0 - pts[1]
        } else if i as usize >= n {
            pts[n - 1] * 2.0 - pts[n - 2]
        } else {
            pts[i as usize]
        }
    };
    let segs = if closed { n } else { n - 1 };
    let mut out = vec![pts[0]];
    for i in 0..segs as isize {
        let (p0, p1, p2, p3) = (get(i - 1), get(i), get(i + 1), get(i + 2));
        let t = |a: DVec2, b: DVec2| a.distance(b).sqrt().max(1e-9);
        let (t1, t2, t3) = (t(p0, p1), t(p1, p2), t(p2, p3));
        // Tangents of the centripetal parameterisation, scaled to the [p1,p2] segment.
        let m1 = ((p1 - p0) / t1 - (p2 - p0) / (t1 + t2) + (p2 - p1) / t2) * t2;
        let m2 = ((p2 - p1) / t2 - (p3 - p1) / (t2 + t3) + (p3 - p2) / t3) * t2;
        let len = p1.distance(p2);
        let steps = ((len / (8.0 * tol).sqrt().max(1e-3)).ceil() as usize).clamp(4, 512);
        for s in 1..=steps {
            let u = s as f64 / steps as f64;
            let (u2, u3) = (u * u, u * u * u);
            let h00 = 2.0 * u3 - 3.0 * u2 + 1.0;
            let h10 = u3 - 2.0 * u2 + u;
            let h01 = -2.0 * u3 + 3.0 * u2;
            let h11 = u3 - u2;
            out.push(p1 * h00 + m1 * h10 + p2 * h01 + m2 * h11);
        }
    }
    if closed {
        out.pop();
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rational_quarter_circle_is_exact() {
        let w = std::f64::consts::FRAC_1_SQRT_2;
        let c = Nurbs::new(2, vec![0., 0., 0., 1., 1., 1.], vec![DVec2::new(1., 0.), DVec2::new(1., 1.), DVec2::new(0., 1.)], vec![1., w, 1.]).unwrap();
        for p in c.tessellate(0.001) {
            assert!((p.length() - 1.0).abs() < 1e-9, "{p}");
        }
        assert!(c.eval(0.0).distance(DVec2::X) < 1e-12);
        assert!(c.eval(1.0).distance(DVec2::Y) < 1e-12);
    }

    #[test]
    fn tessellation_meets_tolerance() {
        let c = Nurbs::new(3, vec![], vec![DVec2::new(0., 0.), DVec2::new(10., 30.), DVec2::new(20., -30.), DVec2::new(30., 0.)], vec![]).unwrap();
        let pts = c.tessellate(0.01);
        assert!(pts[0].distance(DVec2::ZERO) < 1e-12 && pts.last().unwrap().distance(DVec2::new(30., 0.)) < 1e-12);
        // Check chord midpoints against a dense sampling of the true curve.
        let dense: Vec<DVec2> = (0..=20000).map(|i| c.eval(i as f64 / 20000.0)).collect();
        for w in pts.windows(2) {
            let m = (w[0] + w[1]) * 0.5;
            let d = dense.iter().map(|q| q.distance(m)).fold(f64::INFINITY, f64::min);
            assert!(d < 0.012, "chord error {d}");
        }
    }

    #[test]
    fn fit_points_pass_through() {
        let pts = [DVec2::new(0., 0.), DVec2::new(5., 5.), DVec2::new(10., 0.), DVec2::new(15., 5.)];
        let out = through_points(&pts, false, 0.01);
        for p in pts {
            assert!(out.iter().any(|q| q.distance(p) < 1e-9));
        }
    }
}
