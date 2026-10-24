//! Height-field material-removal simulation.
//!
//! The stock is a grid of top heights (a "Z-buffer" / dexel model, as used by most hobby CAM
//! previews). Each cutting segment stamps the tool's profile along its length. This models
//! 3-axis machining exactly up to grid resolution, but cannot show undercuts.

use glam::{DVec2, DVec3};
use warp_tools::Tool;

#[derive(Debug, Clone)]
pub struct Stock {
    pub origin: DVec2,
    pub cell: f64,
    pub nx: usize,
    pub ny: usize,
    pub top: f32,
    pub bottom: f32,
    pub z: Vec<f32>,
    /// Incremented whenever `z` changes, so viewers know when to re-upload.
    pub revision: u64,
}

/// A material-removal event that cut into stock with a rapid move (likely a crash).
#[derive(Debug, Clone, Copy)]
pub struct RapidCollision {
    pub at: DVec3,
    pub depth: f64,
}

/// Tool footprint cached for a grid size.
pub struct Footprint {
    offsets: Vec<(i32, i32, f32)>,
    reach: i32,
}

impl Footprint {
    pub fn new(tool: &Tool, cell: f64) -> Self {
        let r = tool.radius();
        let n = (r / cell).ceil() as i32;
        let mut offsets = Vec::new();
        for dj in -n..=n {
            for di in -n..=n {
                let d = ((di * di + dj * dj) as f64).sqrt() * cell;
                if let Some(h) = tool.profile_height(d) {
                    offsets.push((di, dj, h as f32));
                }
            }
        }
        Footprint { offsets, reach: n }
    }
}

impl Stock {
    /// Create a block of stock. Cell size is chosen so the larger side has about `max_cells` cells.
    /// Fails (instead of aborting) if the grid cannot be allocated.
    pub fn new(min: DVec3, max: DVec3, max_cells: usize) -> Result<Self, String> {
        let (cell, nx, ny) = Self::grid_dims(min, max, max_cells);
        let mut z = Vec::new();
        z.try_reserve_exact(nx * ny).map_err(|_| {
            format!("not enough memory for a {nx}×{ny} stock grid ({:.1} GB); lower the resolution", Self::bytes_for(nx, ny) as f64 / 1e9)
        })?;
        z.resize(nx * ny, max.z as f32);
        Ok(Stock { origin: min.truncate(), cell, nx, ny, top: max.z as f32, bottom: min.z as f32, z, revision: 0 })
    }

    /// Cell size and grid dimensions `Stock::new` would use.
    pub fn grid_dims(min: DVec3, max: DVec3, max_cells: usize) -> (f64, usize, usize) {
        let size = (max - min).truncate();
        let cell = (size.x.max(size.y) / max_cells.max(16) as f64).max(1e-4);
        ((cell), (size.x / cell).ceil() as usize + 1, (size.y / cell).ceil() as usize + 1)
    }

    pub fn bytes_for(nx: usize, ny: usize) -> usize {
        nx * ny * std::mem::size_of::<f32>()
    }

    pub fn reset(&mut self) {
        self.z.fill(self.top);
        self.revision += 1;
    }

    pub fn height(&self, i: usize, j: usize) -> f32 {
        self.z[j * self.nx + i]
    }

    pub fn pos(&self, i: usize, j: usize) -> DVec2 {
        self.origin + DVec2::new(i as f64, j as f64) * self.cell
    }

    /// Lower the stock under the tool with its tip at `p`. Returns the deepest material removed.
    pub fn stamp(&mut self, fp: &Footprint, p: DVec3) -> f32 {
        let ci = ((p.x - self.origin.x) / self.cell).round() as i32;
        let cj = ((p.y - self.origin.y) / self.cell).round() as i32;
        if ci + fp.reach < 0 || cj + fp.reach < 0 || ci - fp.reach >= self.nx as i32 || cj - fp.reach >= self.ny as i32 {
            return 0.0;
        }
        let tip = p.z as f32;
        let mut removed = 0.0f32;
        for &(di, dj, h) in &fp.offsets {
            let (i, j) = (ci + di, cj + dj);
            if i < 0 || j < 0 || i as usize >= self.nx || j as usize >= self.ny {
                continue;
            }
            let k = j as usize * self.nx + i as usize;
            let surf = (tip + h).max(self.bottom);
            if surf < self.z[k] {
                removed = removed.max(self.z[k] - surf);
                self.z[k] = surf;
            }
        }
        removed
    }

    /// Sweep the tool along a straight segment.
    pub fn cut_segment(&mut self, fp: &Footprint, a: DVec3, b: DVec3) -> f32 {
        let n = ((a.distance(b) / (self.cell * 0.5)).ceil() as usize).max(1);
        let mut removed = 0.0f32;
        for s in 0..=n {
            removed = removed.max(self.stamp(fp, a.lerp(b, s as f64 / n as f64)));
        }
        if removed > 0.0 {
            self.revision += 1;
        }
        removed
    }

    /// Volume removed so far (mm³).
    pub fn removed_volume(&self) -> f64 {
        self.z.iter().map(|z| (self.top - z) as f64).sum::<f64>() * self.cell * self.cell
    }
}

/// Plays a list of motions into a stock, a few at a time, for animated previews.
pub struct Player {
    pub segments: Vec<SimSegment>,
    pub index: usize,
    pub collisions: Vec<RapidCollision>,
    footprints: std::collections::HashMap<u32, Footprint>,
}

#[derive(Debug, Clone, Copy)]
pub struct SimSegment {
    pub from: DVec3,
    pub to: DVec3,
    pub rapid: bool,
    /// mm/min (ignored for rapids).
    pub feed: f64,
    pub tool: u32,
    /// Source line (G-code) or move index (toolpath), for highlighting.
    pub source: usize,
}

impl Player {
    pub fn new(segments: Vec<SimSegment>) -> Self {
        Player { segments, index: 0, collisions: vec![], footprints: Default::default() }
    }

    pub fn done(&self) -> bool {
        self.index >= self.segments.len()
    }

    pub fn current(&self) -> Option<&SimSegment> {
        self.segments.get(self.index.min(self.segments.len().saturating_sub(1)))
    }

    /// Advance by up to `budget` mm of tool travel. Returns the tool tip position.
    pub fn step(&mut self, stock: &mut Stock, tools: &dyn Fn(u32) -> Option<Tool>, mut budget: f64) -> Option<DVec3> {
        let mut last = None;
        while budget > 0.0 && self.index < self.segments.len() {
            let s = self.segments[self.index];
            if !self.footprints.contains_key(&s.tool) {
                let fp = tools(s.tool).map(|t| Footprint::new(&t, stock.cell)).unwrap_or(Footprint { offsets: vec![(0, 0, 0.0)], reach: 0 });
                self.footprints.insert(s.tool, fp);
            }
            let fp = &self.footprints[&s.tool];
            let removed = stock.cut_segment(fp, s.from, s.to);
            if s.rapid && removed > 0.05 {
                self.collisions.push(RapidCollision { at: s.to, depth: removed as f64 });
            }
            budget -= s.from.distance(s.to).max(stock.cell);
            last = Some(s.to);
            self.index += 1;
        }
        last
    }

    pub fn run_to_end(&mut self, stock: &mut Stock, tools: &dyn Fn(u32) -> Option<Tool>) {
        self.step(stock, tools, f64::INFINITY);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use warp_tools::ToolShape;

    #[test]
    fn slot_depth_and_width() {
        let mut s = Stock::new(DVec3::new(0., 0., -10.), DVec3::new(50., 50., 0.), 500).unwrap();
        let t = Tool::new(1, "f", ToolShape::FlatEndMill, 6.0);
        let fp = Footprint::new(&t, s.cell);
        s.cut_segment(&fp, DVec3::new(10., 25., -2.), DVec3::new(40., 25., -2.));
        let cell = |x: f64, y: f64| s.height(((x) / s.cell).round() as usize, ((y) / s.cell).round() as usize);
        assert!((cell(25.0, 25.0) + 2.0).abs() < 1e-4);
        assert!((cell(25.0, 27.8) + 2.0).abs() < 1e-4);
        assert_eq!(cell(25.0, 28.5), 0.0);
        assert!(s.removed_volume() > 0.0);
    }
}
