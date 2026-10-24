use crate::ops::Operation;
use glam::DVec3;
use serde::{Deserialize, Serialize};
use std::path::Path;
use warp_core::{Mesh, Polyline2, Units};

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Stock {
    pub min: DVec3,
    pub max: DVec3,
}

impl Default for Stock {
    fn default() -> Self {
        Stock {
            min: DVec3::new(0.0, 0.0, -20.0),
            max: DVec3::new(100.0, 100.0, 0.0),
        }
    }
}

/// Safety checks the user has chosen to relax for this project. Each relaxed check turns a
/// refusal into a generated toolpath with a warning, so the decision is recorded with the job
/// rather than hidden in the application. Off by default.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct Checks {
    /// Generate operations whose cut depth exceeds the tool's flute length (reduced-shank
    /// tools, wide pockets). The toolpath then carries a warning naming the excess.
    #[serde(default)]
    pub allow_depth_beyond_flute: bool,
}

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct Project {
    pub name: String,
    pub display_units: Units,
    #[serde(default)]
    pub stock: Stock,
    #[serde(default)]
    pub curves: Vec<Polyline2>,
    #[serde(default)]
    pub mesh: Option<Mesh>,
    pub operations: Vec<Operation>,
    pub post: String,
    #[serde(default)]
    pub document_id: String,
    #[serde(default)]
    pub scene: Option<warp_core::scene::Scene>,
    #[serde(default)]
    pub setups: Vec<crate::document::Setup>,
    #[serde(default)]
    pub active_setup: warp_core::scene::SetupId,
    #[serde(default)]
    pub next_id: u64,
    /// Sheet nesting intent (M4). Absent in projects that never nested.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub nesting: Option<warp_nest::NestJob>,
    /// Relaxed safety checks (see [`Checks`]). Absent in older projects: nothing relaxed.
    #[serde(default)]
    pub checks: Checks,
    #[serde(skip)]
    pub curve_refs: Vec<warp_core::selection::GeometryRef>,
    #[serde(skip)]
    pub view_points: Vec<Vec<DVec3>>,
    #[serde(skip)]
    pub view_visible: Vec<bool>,
    #[serde(skip)]
    pub execution_boundary: Option<Vec<Polyline2>>,
    #[serde(skip)]
    pub execution_avoid: Vec<Polyline2>,
    #[serde(skip)]
    pub execution_resolved: bool,
    #[serde(skip)]
    pub execution_fixtures: Vec<Stock>,
}

impl Project {
    pub fn load(path: &Path) -> anyhow::Result<Self> {
        Self::from_json(&std::fs::read_to_string(path)?)
    }

    pub fn save(&self, path: &Path) -> anyhow::Result<()> {
        self.save_versioned(path)
    }

    /// Size the stock around all geometry with `margin` on each side, top at Z=0.
    pub fn fit_stock(&mut self, margin: f64) {
        let mut b = warp_core::Aabb3::EMPTY;
        for c in &self.curves {
            for p in &c.pts {
                b.add(p.extend(0.0));
            }
        }
        if let Some(m) = &self.mesh {
            let mb = m.bbox();
            if !mb.is_empty() {
                b.add(mb.min);
                b.add(mb.max);
            }
        }
        if b.is_empty() {
            return;
        }
        let depth = (b.max.z - b.min.z).max(self.stock.max.z - self.stock.min.z);
        self.stock.min = DVec3::new(b.min.x - margin, b.min.y - margin, b.max.z - depth);
        self.stock.max = DVec3::new(b.max.x + margin, b.max.y + margin, b.max.z);
    }

    /// The curves an operation applies to (all if `indices` is empty).
    pub fn select_curves(&self, indices: &[usize]) -> Vec<&Polyline2> {
        if indices.is_empty() {
            self.curves.iter().collect()
        } else {
            indices.iter().filter_map(|&i| self.curves.get(i)).collect()
        }
    }
}
