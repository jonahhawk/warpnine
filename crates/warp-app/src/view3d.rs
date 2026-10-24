//! Software-projected 3D viewport drawn with egui's painter (orthographic, Z up).

use eframe::egui::{self, Color32, Pos2, Rect, Sense, Shape, Stroke};
use glam::DVec3;

#[derive(Debug, Clone, Copy)]
pub struct Camera {
    pub target: DVec3,
    pub yaw: f64,
    pub pitch: f64,
    /// World units visible vertically.
    pub span: f64,
}

impl Default for Camera {
    fn default() -> Self {
        Camera { target: DVec3::new(50.0, 50.0, 0.0), yaw: (-60f64).to_radians(), pitch: 35f64.to_radians(), span: 200.0 }
    }
}

/// Precomputed projection for one frame.
#[derive(Clone, Copy)]
pub struct Proj {
    pub rect: Rect,
    right: DVec3,
    up: DVec3,
    eye_dir: DVec3,
    target: DVec3,
    scale: f64,
}

impl Proj {
    #[inline]
    pub fn to_screen(&self, p: DVec3) -> Pos2 {
        let d = p - self.target;
        let c = self.rect.center();
        Pos2::new(c.x + (d.dot(self.right) * self.scale) as f32, c.y - (d.dot(self.up) * self.scale) as f32)
    }

    /// Larger = closer to the viewer.
    #[inline]
    pub fn depth(&self, p: DVec3) -> f64 {
        (p - self.target).dot(self.eye_dir)
    }

    pub fn light(&self, n: DVec3) -> f32 {
        // Headlight slightly above-left of the view direction.
        let l = (self.eye_dir + self.up * 0.6 - self.right * 0.3).normalize();
        (0.25 + 0.75 * n.normalize_or_zero().dot(l).abs()) as f32
    }
}

impl Camera {
    pub fn proj(&self, rect: Rect) -> Proj {
        let pitch = self.pitch.clamp(-89.9f64.to_radians(), 89.9f64.to_radians());
        let eye_dir = DVec3::new(pitch.cos() * self.yaw.cos(), pitch.cos() * self.yaw.sin(), pitch.sin());
        let fwd = -eye_dir;
        let right = fwd.cross(DVec3::Z).normalize();
        let up = right.cross(fwd);
        Proj { rect, right, up, eye_dir, target: self.target, scale: rect.height().max(1.0) as f64 / self.span }
    }

    pub fn top(&mut self) {
        self.yaw = (-90f64).to_radians();
        self.pitch = 89.9f64.to_radians();
    }

    pub fn front(&mut self) {
        self.yaw = (-90f64).to_radians();
        self.pitch = 0.0;
    }

    pub fn iso(&mut self) {
        self.yaw = (-60f64).to_radians();
        self.pitch = 35f64.to_radians();
    }

    pub fn fit(&mut self, min: DVec3, max: DVec3) {
        if min.x > max.x {
            return;
        }
        self.target = (min + max) * 0.5;
        self.span = (max - min).length().max(1.0) * 1.15;
    }

    /// Orbit (left drag), pan (right/middle drag), zoom (wheel). Returns the response.
    pub fn interact(&mut self, ui: &mut egui::Ui, rect: Rect) -> egui::Response {
        let resp = ui.allocate_rect(rect, Sense::click_and_drag());
        let p = self.proj(rect);
        if resp.dragged_by(egui::PointerButton::Primary) && !ui.input(|i| i.modifiers.shift) {
            let d = resp.drag_delta();
            self.yaw -= d.x as f64 * 0.01;
            self.pitch = (self.pitch + d.y as f64 * 0.01).clamp(-89.9f64.to_radians(), 89.9f64.to_radians());
        }
        if resp.dragged_by(egui::PointerButton::Secondary)
            || resp.dragged_by(egui::PointerButton::Middle)
            || (resp.dragged_by(egui::PointerButton::Primary) && ui.input(|i| i.modifiers.shift))
        {
            let d = resp.drag_delta();
            self.target -= p.right * (d.x as f64 / p.scale) - p.up * (d.y as f64 / p.scale);
        }
        if resp.hovered() {
            let scroll = ui.input(|i| i.smooth_scroll_delta.y);
            if scroll != 0.0 {
                // Zoom about the cursor.
                let factor = (-scroll as f64 * 0.002).exp();
                if let Some(mp) = resp.hover_pos() {
                    let c = rect.center();
                    let world_off = p.right * ((mp.x - c.x) as f64 / p.scale) - p.up * ((mp.y - c.y) as f64 / p.scale);
                    self.target += world_off * (1.0 - factor);
                }
                self.span = (self.span * factor).clamp(0.5, 1e5);
            }
        }
        resp
    }
}

/// Triangle with a base colour, shaded by its normal.
pub struct Tri {
    pub v: [DVec3; 3],
    pub color: Color32,
}

/// Depth-sorted flat-shaded triangles (painter's algorithm). With `edges`, each triangle's
/// outline is drawn right after its fill, so nearer surfaces hide the edges behind them.
pub fn paint_tris(painter: &egui::Painter, p: &Proj, tris: impl Iterator<Item = Tri>, edges: Option<Stroke>) {
    let mut items: Vec<(f64, [Pos2; 3], Color32)> = tris
        .map(|t| {
            let n = (t.v[1] - t.v[0]).cross(t.v[2] - t.v[0]);
            let k = p.light(n);
            let c = Color32::from_rgb((t.color.r() as f32 * k) as u8, (t.color.g() as f32 * k) as u8, (t.color.b() as f32 * k) as u8);
            let depth = p.depth((t.v[0] + t.v[1] + t.v[2]) / 3.0);
            (depth, [p.to_screen(t.v[0]), p.to_screen(t.v[1]), p.to_screen(t.v[2])], c)
        })
        .collect();
    items.sort_by(|a, b| a.0.total_cmp(&b.0));
    let mut mesh = egui::Mesh::default();
    for (_, s, c) in items {
        let i = mesh.vertices.len() as u32;
        for v in s {
            mesh.colored_vertex(v, c);
        }
        mesh.add_triangle(i, i + 1, i + 2);
        if let Some(st) = edges {
            for k in 0..3 {
                let (a, b) = (s[k], s[(k + 1) % 3]);
                let d = b - a;
                let len = d.length();
                if len < 0.5 {
                    continue;
                }
                let n = egui::vec2(-d.y, d.x) * (st.width * 0.5 / len);
                let j = mesh.vertices.len() as u32;
                for v in [a + n, a - n, b - n, b + n] {
                    mesh.colored_vertex(v, st.color);
                }
                mesh.add_triangle(j, j + 1, j + 2);
                mesh.add_triangle(j, j + 2, j + 3);
            }
        }
        if mesh.vertices.len() > 60_000 {
            painter.add(Shape::mesh(std::mem::take(&mut mesh)));
        }
    }
    if !mesh.vertices.is_empty() {
        painter.add(Shape::mesh(mesh));
    }
}

/// Reduce a triangle soup to roughly `target` triangles for display by vertex clustering:
/// vertices are snapped to the average of their grid cell and collapsed triangles dropped.
pub fn decimate(tris: &[[DVec3; 3]], target: usize) -> Vec<[DVec3; 3]> {
    if tris.len() <= target {
        return tris.to_vec();
    }
    let area: f64 = tris.iter().map(|t| (t[1] - t[0]).cross(t[2] - t[0]).length() * 0.5).sum();
    // A uniform grid of cell size c leaves about area / c² · 2 triangles on a surface.
    let mut cell = (2.0 * area / target as f64).sqrt().max(1e-9);
    loop {
        let key = |v: DVec3| ((v.x / cell).floor() as i64, (v.y / cell).floor() as i64, (v.z / cell).floor() as i64);
        let mut sums: std::collections::HashMap<(i64, i64, i64), (DVec3, f64)> = std::collections::HashMap::new();
        for t in tris {
            for &v in t {
                let e = sums.entry(key(v)).or_insert((DVec3::ZERO, 0.0));
                e.0 += v;
                e.1 += 1.0;
            }
        }
        let mut seen = std::collections::HashSet::new();
        let mut out = Vec::new();
        for t in tris {
            let k = [key(t[0]), key(t[1]), key(t[2])];
            if k[0] == k[1] || k[1] == k[2] || k[0] == k[2] {
                continue;
            }
            let mut sorted = k;
            sorted.sort_unstable();
            if seen.insert(sorted) {
                out.push(k.map(|c| sums[&c].0 / sums[&c].1));
            }
        }
        if out.len() <= target * 3 / 2 {
            return out;
        }
        cell *= 1.4;
    }
}

pub fn polyline(painter: &egui::Painter, p: &Proj, pts: &[DVec3], stroke: Stroke) {
    if pts.len() >= 2 {
        painter.add(Shape::line(pts.iter().map(|q| p.to_screen(*q)).collect(), stroke));
    }
}

pub fn box_wire(painter: &egui::Painter, p: &Proj, min: DVec3, max: DVec3, stroke: Stroke) {
    let c = |x: bool, y: bool, z: bool| DVec3::new(if x { max.x } else { min.x }, if y { max.y } else { min.y }, if z { max.z } else { min.z });
    for z in [false, true] {
        polyline(painter, p, &[c(false, false, z), c(true, false, z), c(true, true, z), c(false, true, z), c(false, false, z)], stroke);
    }
    for (x, y) in [(false, false), (true, false), (true, true), (false, true)] {
        polyline(painter, p, &[c(x, y, false), c(x, y, true)], stroke);
    }
}

pub fn axes(painter: &egui::Painter, p: &Proj, len: f64) {
    let o = DVec3::ZERO;
    for (d, col, name) in [(DVec3::X, Color32::RED, "X"), (DVec3::Y, Color32::GREEN, "Y"), (DVec3::Z, Color32::from_rgb(80, 140, 255), "Z")] {
        let e = p.to_screen(d * len);
        painter.line_segment([p.to_screen(o), e], Stroke::new(2.0, col));
        painter.text(e, egui::Align2::LEFT_BOTTOM, name, egui::FontId::proportional(12.0), col);
    }
}

/// Side profile of a tool (+ holder) as (radius, height above tip) points from the tip upward,
/// each tagged with the part it belongs to.
#[derive(Clone, Copy, PartialEq)]
pub enum ToolPart {
    Cutter,
    Shank,
    Holder,
}

pub fn tool_profile(t: &warp_tools::Tool, holder: Option<&warp_tools::Holder>) -> Vec<(f64, f64, ToolPart)> {
    let r = t.radius().max(1e-3);
    let mut pts = Vec::new();
    // Cutting end: sample r = R·sin(θ) so curved tips (ball, bull nose) get dense points near the edge.
    let n = 32;
    for i in 0..=n {
        let rr = r * (std::f64::consts::FRAC_PI_2 * i as f64 / n as f64).sin();
        let h = t.profile_height(rr).unwrap_or(0.0);
        pts.push((rr, h, ToolPart::Cutter));
    }
    let edge_top = t.profile_height(r).unwrap_or(0.0);
    let flute_top = t.flute_length.max(edge_top);
    if flute_top > edge_top + 1e-6 {
        pts.push((r, flute_top, ToolPart::Cutter));
    }
    let shank_r = (t.shank_diameter * 0.5).max(1e-3);
    let stickout = t.stickout.max(flute_top + 1.0);
    pts.push((shank_r, flute_top, ToolPart::Shank));
    pts.push((shank_r, stickout, ToolPart::Shank));
    if let Some(h) = holder {
        let mut z = stickout;
        for seg in &h.segments {
            pts.push((seg.lower_diameter * 0.5, z, ToolPart::Holder));
            z += seg.length;
            pts.push((seg.upper_diameter * 0.5, z, ToolPart::Holder));
        }
        pts.push((0.0, z, ToolPart::Holder));
    } else {
        pts.push((0.0, stickout, ToolPart::Shank));
    }
    pts
}

/// Shaded 3D tool (cutter, shank and optional holder) with its tip at `tip`.
pub fn tool_solid(painter: &egui::Painter, p: &Proj, t: &warp_tools::Tool, holder: Option<&warp_tools::Holder>, tip: DVec3) {
    let prof = tool_profile(t, holder);
    let m = 36;
    let mut tris = Vec::new();
    for w in prof.windows(2) {
        let ((r0, z0, _), (r1, z1, part)) = (w[0], w[1]);
        if (r0 - r1).abs() < 1e-9 && (z0 - z1).abs() < 1e-9 {
            continue;
        }
        let color = match part {
            ToolPart::Cutter => Color32::from_rgb(225, 190, 90),
            ToolPart::Shank => Color32::from_rgb(185, 190, 200),
            ToolPart::Holder => Color32::from_rgb(120, 125, 135),
        };
        for k in 0..m {
            let (a0, a1) = (std::f64::consts::TAU * k as f64 / m as f64, std::f64::consts::TAU * (k + 1) as f64 / m as f64);
            let v = |r: f64, z: f64, a: f64| tip + DVec3::new(a.cos() * r, a.sin() * r, z);
            let (p00, p01, p10, p11) = (v(r0, z0, a0), v(r0, z0, a1), v(r1, z1, a0), v(r1, z1, a1));
            if r0 > 1e-9 {
                tris.push(Tri { v: [p00, p01, p11], color });
            }
            if r1 > 1e-9 {
                tris.push(Tri { v: [p00, p11, p10], color });
            }
        }
    }
    paint_tris(painter, p, tris.into_iter(), None);
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn decimate_reduces_a_dense_surface() {
        // 200x200 grid of a gentle bump = 80k triangles.
        let n = 200;
        let f = |i: usize, j: usize| DVec3::new(i as f64, j as f64, ((i as f64 * 0.05).sin() + (j as f64 * 0.05).cos()) * 5.0);
        let mut tris = Vec::new();
        for j in 0..n {
            for i in 0..n {
                tris.push([f(i, j), f(i + 1, j), f(i + 1, j + 1)]);
                tris.push([f(i, j), f(i + 1, j + 1), f(i, j + 1)]);
            }
        }
        let d = decimate(&tris, 10_000);
        assert!(d.len() <= 15_000 && d.len() > 3_000, "{}", d.len());
        // Still covers the whole surface.
        let max_x = d.iter().flat_map(|t| t.iter()).map(|v| v.x).fold(0.0, f64::max);
        assert!(max_x > 190.0, "{max_x}");
        assert_eq!(decimate(&tris[..100], 10_000).len(), 100);
    }
}
