//! Core geometry types shared by every WarpNine crate.

pub mod geom;
pub mod import;
pub mod interchange;
pub mod mesh;
pub mod nurbs;
pub mod scene;
pub mod selection;

pub use geom::{Aabb2, Aabb3, Polyline2};
pub use glam::{DVec2, DVec3};
pub use mesh::Mesh;

use serde::{Deserialize, Serialize};

/// Linear units of a project or program. Internally everything is millimetres.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Default)]
pub enum Units {
    #[default]
    Millimeters,
    Inches,
}

impl Units {
    /// Multiply a value in these units by this factor to get millimetres.
    pub fn to_mm(self) -> f64 {
        match self {
            Units::Millimeters => 1.0,
            Units::Inches => 25.4,
        }
    }

    pub fn suffix(self) -> &'static str {
        match self {
            Units::Millimeters => "mm",
            Units::Inches => "in",
        }
    }
}

/// Default tolerance used for chaining and arc tessellation (mm).
pub const EPS: f64 = 1e-6;
