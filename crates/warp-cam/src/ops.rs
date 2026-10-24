//! Operation definitions. Every parameter here is user-editable in the UI.

use crate::toolpath::DrillCycle;
use serde::{Deserialize, Serialize};
use warp_tools::{Coolant, Tool};

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Heights {
    /// Absolute Z for rapids between operations and at start/end.
    pub clearance: f64,
    /// Absolute Z for rapids between cuts inside an operation.
    pub retract: f64,
    /// Distance above the top of the cut where rapid-down stops and the plunge feed starts.
    pub feed_height: f64,
    /// Absolute Z of the top of the cut.
    pub top: f64,
    /// Absolute Z of the final depth.
    pub bottom: f64,
}

impl Default for Heights {
    fn default() -> Self {
        Heights {
            clearance: 15.0,
            retract: 5.0,
            feed_height: 1.0,
            top: 0.0,
            bottom: -3.0,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Feeds {
    pub spindle_rpm: f64,
    pub clockwise: bool,
    pub feed: f64,
    pub plunge: f64,
    pub ramp: f64,
    pub coolant: Coolant,
}

impl Default for Feeds {
    fn default() -> Self {
        Self::from_tool_data(&warp_tools::CuttingData::default())
    }
}

impl Feeds {
    pub fn from_tool_data(c: &warp_tools::CuttingData) -> Self {
        Feeds {
            spindle_rpm: c.spindle_rpm,
            clockwise: c.clockwise,
            feed: c.feed_rate,
            plunge: c.plunge_rate,
            ramp: c.ramp_rate,
            coolant: c.coolant,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub enum Entry {
    Plunge,
    /// Ramp along the cut at the given angle (degrees from horizontal).
    Ramp {
        angle: f64,
    },
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct HelixEntry {
    pub radius: f64,
    pub pitch: f64,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Common {
    #[serde(default)]
    pub id: warp_core::scene::OperationId,
    #[serde(default)]
    pub setup: warp_core::scene::SetupId,
    #[serde(default)]
    pub geometry: Option<warp_core::selection::ControlGeometry>,
    pub tool: u32,
    pub heights: Heights,
    pub feeds: Feeds,
    /// Axial depth per pass.
    pub stepdown: f64,
    /// Radial stock to leave on walls.
    pub stock_xy: f64,
    /// Axial stock to leave on floors.
    pub stock_z: f64,
    /// Climb milling (true) or conventional (false), assuming a CW spindle.
    pub climb: bool,
    pub entry: Entry,
    #[serde(default)]
    pub helix_entry: Option<HelixEntry>,
    /// Chordal tolerance for tessellation and path simplification.
    pub tolerance: f64,
    /// Indices into the project's 2D curves. Empty = all curves.
    /// For 3D operations, selected closed curves act as the machining boundary.
    pub curves: Vec<usize>,
    /// Surface sampling step for 3D operations (mm): the Z-map grid size, or the longest chord
    /// between exact contact samples. Smaller is more accurate and slower.
    #[serde(default = "default_resolution")]
    pub resolution: f64,
    /// Which surface query answers cutter-contact questions for 3D operations.
    #[serde(default)]
    pub surface: SurfaceMode,
}

fn default_resolution() -> f64 {
    0.1
}

/// Surface sampling step a new operation of `kind` starts with. 3D roughing leaves stock for a
/// finishing pass and its Z-map cost grows with the fourth power of the inverse step, so it
/// starts coarser than the finishing operations.
pub fn default_resolution_for(kind: &OpKind) -> f64 {
    match kind {
        OpKind::Roughing3D { .. } => 0.25,
        _ => default_resolution(),
    }
}

/// How a 3D operation queries the part surface.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Default)]
pub enum SurfaceMode {
    /// Exact mesh contact for finishing operations, Z-map for 3D roughing.
    #[default]
    Auto,
    /// Exact cutter contact against the mesh triangles, refined until chords meet the tolerance.
    Exact,
    /// Rasterised height grid at `resolution`; fast, but walls and detail are only resolved to the
    /// grid.
    ZMap,
}

impl SurfaceMode {
    pub fn label(&self) -> &'static str {
        match self {
            SurfaceMode::Auto => "Auto",
            SurfaceMode::Exact => "Exact mesh",
            SurfaceMode::ZMap => "Z-map",
        }
    }

    /// The mode actually used for an operation kind.
    pub fn resolve(self, kind: &OpKind) -> SurfaceMode {
        match self {
            SurfaceMode::Auto => {
                if matches!(kind, OpKind::Roughing3D { .. }) {
                    SurfaceMode::ZMap
                } else {
                    SurfaceMode::Exact
                }
            }
            m => m,
        }
    }
}

impl Common {
    pub fn for_tool(tool: &Tool) -> Self {
        Common {
            id: Default::default(),
            setup: Default::default(),
            geometry: None,
            tool: tool.number,
            heights: Heights::default(),
            feeds: Feeds::from_tool_data(&tool.cutting),
            stepdown: tool.cutting.stepdown,
            stock_xy: 0.0,
            stock_z: 0.0,
            climb: true,
            entry: Entry::Ramp { angle: 3.0 },
            helix_entry: None,
            tolerance: 0.01,
            curves: vec![],
            resolution: default_resolution(),
            surface: SurfaceMode::default(),
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum Side {
    Outside,
    Inside,
    On,
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Tabs {
    pub count: u32,
    pub width: f64,
    /// Height of the tab above the final depth.
    pub height: f64,
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub enum PocketPattern {
    /// Contour-parallel offsets.
    Offset,
    /// Back-and-forth raster at `angle` degrees.
    ZigZag { angle: f64 },
}

/// Order of cutting when an operation has several separate areas.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Default)]
pub enum CutOrder {
    /// Finish each area to full depth before moving to the next.
    DepthFirst,
    /// Cut every area one stepdown at a time.
    #[default]
    LayerFirst,
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub enum DrillSource {
    /// Centres of closed circular curves whose diameter is within the range.
    CircleCenters {
        min_diameter: f64,
        max_diameter: f64,
    },
    /// Start point of every selected curve.
    CurveStarts,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum OpKind {
    // ---- 2D / 2.5D ----
    Facing {
        stepover: f64,
        angle: f64,
        overlap: f64,
        zigzag: bool,
    },
    Profile {
        side: Side,
        finish_passes: u32,
        finish_stepover: f64,
        spring_pass: bool,
        lead_radius: f64,
        tabs: Option<Tabs>,
    },
    Pocket {
        pattern: PocketPattern,
        stepover: f64,
        finish_pass: bool,
        inside_out: bool,
        #[serde(default)]
        order: CutOrder,
    },
    Drill {
        cycle: DrillCycle,
        peck: f64,
        dwell: f64,
        source: DrillSource,
        tip_compensation: bool,
        breakthrough: f64,
    },
    Engrave,
    VCarve {
        max_depth: f64,
        step: f64,
    },
    HelicalBore {
        min_diameter: f64,
        max_diameter: f64,
        pitch: f64,
        stepover: f64,
    },
    ThreadMill {
        pitch: f64,
        internal: bool,
        major_diameter: f64,
        right_hand: bool,
    },
    Chamfer {
        width: f64,
        extra_depth: f64,
        side: Side,
    },
    /// Face selected bounded top regions rather than the stock rectangle.
    FaceTop {
        stepover: f64,
        angle: f64,
    },
    /// Closed clearing extent or directed open walls ending on rectangular stock edges.
    OpenPocket {
        stepover: f64,
    },
    /// Clear stock around selected retained part outlines and islands.
    Roughing2D {
        stepover: f64,
        inside_out: bool,
    },
    /// Clear circular features from their centres outward.
    HolePocket {
        stepover: f64,
        pitch: f64,
    },
    /// Finish only the wall of circular holes using a helical entry.
    HoleProfile {
        pitch: f64,
        spring_pass: bool,
    },
    /// Tool-centre follows an open or closed chain, producing a tool-width slot.
    Slot,
    // ---- 3D ----
    Roughing3D {
        stepover: f64,
        finish_contour: bool,
    },
    Parallel {
        stepover: f64,
        angle: f64,
        zigzag: bool,
    },
    Waterline {
        stepdown: f64,
    },
    Spiral {
        stepover: f64,
    },
    Radial {
        angle_step: f64,
    },
    ProjectCurves {
        depth: f64,
    },
}

impl OpKind {
    pub fn label(&self) -> &'static str {
        match self {
            OpKind::Facing { .. } => "Facing (stock)",
            OpKind::OpenPocket { .. } => "Open Pocket (stock edge)",
            OpKind::FaceTop { .. } => "Face Top (selected regions)",
            OpKind::Roughing2D { .. } => "2.5D Roughing (stock around parts)",
            OpKind::HolePocket { .. } => "Hole Pocketing",
            OpKind::HoleProfile { .. } => "Hole Profiling",
            OpKind::Slot => "Slot (tool width)",
            OpKind::Profile { .. } => "Profile",
            OpKind::Pocket { .. } => "Pocket",
            OpKind::Drill { .. } => "Drill",
            OpKind::Engrave => "Engrave / Follow",
            OpKind::VCarve { .. } => "V-Carve",
            OpKind::HelicalBore { .. } => "Helical bore",
            OpKind::ThreadMill { .. } => "Thread mill",
            OpKind::Chamfer { .. } => "Chamfer",
            OpKind::Roughing3D { .. } => "3D Roughing",
            OpKind::Parallel { .. } => "Parallel finish",
            OpKind::Waterline { .. } => "Waterline",
            OpKind::Spiral { .. } => "Spiral finish",
            OpKind::Radial { .. } => "Radial finish",
            OpKind::ProjectCurves { .. } => "Project curves",
        }
    }

    pub fn is_3d(&self) -> bool {
        matches!(
            self,
            OpKind::Roughing3D { .. }
                | OpKind::Parallel { .. }
                | OpKind::Waterline { .. }
                | OpKind::Spiral { .. }
                | OpKind::Radial { .. }
                | OpKind::ProjectCurves { .. }
        )
    }

    /// One default instance of every operation type, for "add operation" menus.
    pub fn all_defaults() -> Vec<OpKind> {
        vec![
            OpKind::Facing {
                stepover: 4.0,
                angle: 0.0,
                overlap: 2.0,
                zigzag: true,
            },
            OpKind::Profile {
                side: Side::Outside,
                finish_passes: 0,
                finish_stepover: 0.2,
                spring_pass: false,
                lead_radius: 0.0,
                tabs: None,
            },
            OpKind::Pocket {
                pattern: PocketPattern::Offset,
                stepover: 2.4,
                finish_pass: true,
                inside_out: true,
                order: CutOrder::LayerFirst,
            },
            OpKind::Drill {
                cycle: DrillCycle::Peck,
                peck: 1.0,
                dwell: 0.0,
                source: DrillSource::CircleCenters {
                    min_diameter: 0.0,
                    max_diameter: 1000.0,
                },
                tip_compensation: false,
                breakthrough: 0.0,
            },
            OpKind::Engrave,
            OpKind::VCarve {
                max_depth: 5.0,
                step: 0.1,
            },
            OpKind::HelicalBore {
                min_diameter: 0.0,
                max_diameter: 1000.0,
                pitch: 1.0,
                stepover: 2.0,
            },
            OpKind::ThreadMill {
                pitch: 1.5,
                internal: true,
                major_diameter: 0.0,
                right_hand: true,
            },
            OpKind::Chamfer {
                width: 0.5,
                extra_depth: 0.2,
                side: Side::Outside,
            },
            OpKind::Roughing3D {
                stepover: 2.4,
                finish_contour: true,
            },
            OpKind::Parallel {
                stepover: 0.5,
                angle: 0.0,
                zigzag: true,
            },
            OpKind::Waterline { stepdown: 0.5 },
            OpKind::Spiral { stepover: 0.5 },
            OpKind::Radial { angle_step: 2.0 },
            OpKind::ProjectCurves { depth: 0.2 },
            OpKind::FaceTop {
                stepover: 2.4,
                angle: 0.0,
            },
            OpKind::Roughing2D {
                stepover: 2.4,
                inside_out: false,
            },
            OpKind::HolePocket {
                stepover: 1.0,
                pitch: 1.0,
            },
            OpKind::HoleProfile {
                pitch: 1.0,
                spring_pass: true,
            },
            OpKind::Slot,
            OpKind::OpenPocket { stepover: 2.4 },
        ]
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Operation {
    pub name: String,
    pub enabled: bool,
    pub common: Common,
    pub kind: OpKind,
}

/// Depth levels from just below `top` down to `bottom` in equal steps no larger than `stepdown`.
pub fn depth_levels(top: f64, bottom: f64, stepdown: f64) -> Vec<f64> {
    let total = top - bottom;
    if total <= 1e-9 {
        return vec![bottom];
    }
    let n = (total / stepdown.max(1e-3)).ceil().max(1.0) as usize;
    (1..=n).map(|i| top - total * i as f64 / n as f64).collect()
}
