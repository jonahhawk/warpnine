//! Toolpath generation and post-processing.
//!
//! Coordinates are millimetres, Z up, top of stock typically at Z=0. Tool-centre paths are
//! computed here (no controller-side cutter compensation is used).

pub mod document;
mod medial;
pub mod ops;
mod ops2d;
mod ops3d;
pub mod planner;
pub mod post;
pub mod project;
pub mod region;
pub mod surface;
pub mod toolpath;
pub mod validation;
pub mod verification;
pub mod workflow;
pub mod zmap;

pub use ops::{Common, Entry, Feeds, Heights, OpKind, Operation, Side, SurfaceMode};
pub use ops2d::as_circle;
pub use project::{Checks, Project, Stock};
pub use toolpath::{Move, Toolpath};

use anyhow::{anyhow, Context};
use toolpath::{Links, PathBuilder};
use warp_tools::{Tool, ToolLibrary};

/// Everything an operation generator needs.
pub(crate) struct Gen<'a> {
    pub op: &'a Operation,
    pub c: &'a Common,
    pub tool: &'a Tool,
    pub project: &'a Project,
}

impl Gen<'_> {
    pub fn links(&self) -> Links {
        let h = &self.c.heights;
        Links {
            safe_z: h
                .retract
                .max(h.top + h.feed_height)
                .max(self.project.stock.max.z + self.c.tolerance),
            feed_z: (h.top + h.feed_height).max(self.project.stock.max.z + self.c.tolerance),
            feed: self.c.feeds.feed,
            plunge: self.c.feeds.plunge,
            link_distance: self.tool.diameter,
        }
    }

    pub fn builder(&self) -> PathBuilder {
        let mut b = PathBuilder::new(self.links());
        b.ramp_feed = self.c.feeds.ramp;
        b.helix_entry = self.c.helix_entry;
        b.comment(format!(
            "{} - T{} {}",
            self.op.name, self.tool.number, self.tool.name
        ));
        b
    }

    pub fn ramp_angle(&self) -> f64 {
        match self.c.entry {
            ops::Entry::Plunge => 0.0,
            ops::Entry::Ramp { angle } => angle,
        }
    }
}

/// Generate the toolpath for one operation.
pub fn generate(
    op: &Operation,
    project: &Project,
    tools: &ToolLibrary,
) -> anyhow::Result<Toolpath> {
    workflow::check_cancel()?;
    if project.scene.is_some() {
        let (resolved, adapted) = project.resolve_operation(op)?;
        let mut path = generate(&adapted, &resolved, tools)?;
        path.wcs = project
            .setups
            .iter()
            .find(|s| s.id == op.common.setup)
            .map(|s| s.wcs)
            .unwrap_or(54);
        return Ok(path);
    }
    let tool = tools
        .tool(op.common.tool)
        .ok_or_else(|| anyhow!("tool T{} is not in the library", op.common.tool))?;
    anyhow::ensure!(
        op.common.tolerance.is_finite()
            && op.common.tolerance > 0.0
            && op.common.stepdown.is_finite()
            && op.common.stepdown > 0.0,
        "tolerance and stepdown must be finite and positive"
    );
    let h = &op.common.heights;
    if h.retract < h.top || h.clearance < h.retract {
        return Err(anyhow!("heights must satisfy clearance >= retract >= top"));
    }
    if h.bottom > h.top {
        return Err(anyhow!("bottom must be below top"));
    }
    validation::parameters(op, project, tool)?;
    let g = Gen {
        op,
        c: &op.common,
        tool,
        project,
    };
    let moves = if op.kind.is_3d() {
        ops3d::generate(&g)
    } else {
        ops2d::generate(&g)
    }
    .with_context(|| format!("generating '{}'", op.name))?;
    let mut moves = moves;
    workflow::check_cancel()?;
    // Start and end every operation at clearance height.
    if let Some(i) = moves.iter().position(|m| !matches!(m, Move::Comment(_))) {
        if let Move::Rapid(p) = moves[i] {
            moves.insert(i, Move::Rapid(glam::DVec3::new(p.x, p.y, h.clearance)));
        }
    }
    if let Some(p) = moves.iter().rev().find_map(|m| match m {
        Move::Rapid(p) => Some(*p),
        Move::Linear { to, .. } | Move::Arc { to, .. } => Some(*to),
        _ => None,
    }) {
        moves.push(Move::Rapid(glam::DVec3::new(p.x, p.y, h.clearance)));
    }
    let mut path = Toolpath {
        wcs: 54,
        name: op.name.clone(),
        tool: tool.number,
        rpm: op.common.feeds.spindle_rpm,
        clockwise: op.common.feeds.clockwise,
        coolant: op.common.feeds.coolant,
        moves,
        warnings: vec![],
    };
    path.warnings
        .extend(validation::flute_reach_warning(&path, h.top, tool));
    validation::fixtures(
        &path,
        &project.execution_fixtures,
        tool,
        tools,
        op.common.tolerance,
    )?;
    Ok(path)
}

/// Generate all enabled operations in order.
pub fn generate_all(project: &Project, tools: &ToolLibrary) -> Vec<anyhow::Result<Toolpath>> {
    project
        .operations
        .iter()
        .filter(|o| o.enabled)
        .map(|o| generate(o, project, tools))
        .collect()
}
