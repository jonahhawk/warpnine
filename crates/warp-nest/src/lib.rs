//! Sheet nesting data model (M4 N01).
//!
//! A nesting job stores *intent*: which scene geometry forms a part, how many copies are
//! required, which sheets are available and where each copy was placed. It never copies
//! source curves; parts reference scene objects, and placements are transforms. Footprint
//! geometry, feasibility checks and solvers build on this model in later tasks.
//!
//! Units are millimetres. Sheets lie in their own local XY plane with the origin at the
//! lower-left corner; a placement positions a part's local origin on that plane.

pub mod candidates;
pub mod feasibility;
pub mod footprint;
pub mod job;
pub mod solver;

use anyhow::{bail, ensure, Result};
use glam::DVec2;
use serde::{Deserialize, Serialize};
use std::collections::HashSet;
use warp_core::scene::{DefinitionId, Scene};
use warp_core::selection::GeometryRef;

macro_rules! id {
    ($name:ident) => {
        #[derive(
            Debug, Clone, Default, PartialEq, Eq, Hash, PartialOrd, Ord, Serialize, Deserialize,
        )]
        #[serde(transparent)]
        pub struct $name(pub String);
    };
}
id!(PartId);
id!(SheetId);
id!(PartInstanceId);

/// Where a part's geometry comes from. References are kept, never resolved copies.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum PartSource {
    /// Every object of a block definition, in definition-local coordinates. Part-local
    /// feature references name objects inside the definition (empty instance path).
    Block { definition: DefinitionId },
    /// Explicitly grouped loose geometry. Part-local coordinates equal scene coordinates
    /// with the outline's bounding-box minimum moved to the origin at footprint time.
    Loose { members: Vec<GeometryRef> },
}

/// Which rotations the solver may apply. Angles are degrees, counter-clockwise.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum Rotations {
    /// Only the explicitly listed angles (0 is not implied).
    Fixed(Vec<f64>),
    /// Multiples of `step_deg` in `[0, 360)`.
    Step { step_deg: f64 },
}
impl Default for Rotations {
    fn default() -> Self {
        Rotations::Step { step_deg: 90.0 }
    }
}
impl Rotations {
    pub fn allows(&self, angle_deg: f64) -> bool {
        let a = normalize_deg(angle_deg);
        match self {
            Rotations::Fixed(list) => list.iter().any(|r| {
                (normalize_deg(*r) - a).abs() < 1e-9 || (normalize_deg(*r) - a).abs() > 360.0 - 1e-9
            }),
            Rotations::Step { step_deg } => {
                if *step_deg <= 0.0 {
                    return a.abs() < 1e-9;
                }
                let k = (a / step_deg).round();
                (k * step_deg - a).abs() < 1e-9
            }
        }
    }
    /// Distinct candidate angles in `[0, 360)`, ascending. A zero step yields `[0]`;
    /// steps are limited to 360 candidates so a tiny step cannot stall the solver.
    pub fn candidates(&self) -> Vec<f64> {
        let mut out: Vec<f64> = match self {
            Rotations::Fixed(list) => list.iter().map(|a| normalize_deg(*a)).collect(),
            Rotations::Step { step_deg } => {
                if *step_deg <= 0.0 {
                    vec![0.0]
                } else {
                    let step = step_deg.max(1.0);
                    let n = (360.0 / step).floor() as usize;
                    (0..n.max(1))
                        .map(|i| normalize_deg(i as f64 * step))
                        .collect()
                }
            }
        };
        out.sort_by(|a, b| a.total_cmp(b));
        out.dedup_by(|a, b| (*a - *b).abs() < 1e-9);
        out
    }
    pub fn validate(&self) -> Result<()> {
        match self {
            Rotations::Fixed(list) => {
                ensure!(!list.is_empty(), "rotation list is empty");
                ensure!(
                    list.iter().all(|a| a.is_finite()),
                    "rotation angle is not finite"
                );
            }
            Rotations::Step { step_deg } => ensure!(
                step_deg.is_finite() && *step_deg >= 0.0 && *step_deg <= 360.0,
                "rotation step must be in [0, 360]"
            ),
        }
        Ok(())
    }
}

/// Whether a part grain at `part_deg` (local), mirrored and rotated as given, runs
/// parallel or anti-parallel to the sheet grain.
pub fn grain_aligned(part_deg: f64, mirrored: bool, rotation_deg: f64, sheet_deg: f64) -> bool {
    let eff = if mirrored { -part_deg } else { part_deg } + rotation_deg;
    let diff = normalize_deg(eff - sheet_deg);
    diff.min(360.0 - diff) <= 1e-6 || (diff - 180.0).abs() <= 1e-6
}

pub fn normalize_deg(a: f64) -> f64 {
    let r = a.rem_euclid(360.0);
    if (r - 360.0).abs() < 1e-9 {
        0.0
    } else {
        r
    }
}

/// A nestable part: a block or an explicit loose group, plus part-local feature roles.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PartDefinition {
    pub id: PartId,
    pub name: String,
    pub source: PartSource,
    /// The closed outer boundary. `None` means the part is not yet placeable; the
    /// candidate finder can suggest one, but an open curve is never inferred as an outline.
    pub outline: Option<GeometryRef>,
    /// Closed loops directly inside the outline; cut out before the outline is released.
    #[serde(default)]
    pub holes: Vec<GeometryRef>,
    /// Other geometry carried with the part (engraving, pocket boundaries, drill points).
    #[serde(default)]
    pub features: Vec<GeometryRef>,
    pub quantity: u32,
    #[serde(default)]
    pub rotations: Rotations,
    #[serde(default)]
    pub allow_mirror: bool,
    /// Part-local grain direction in degrees. `Some` constrains rotation to align with the
    /// sheet grain, if the sheet has one.
    #[serde(default)]
    pub grain_deg: Option<f64>,
    /// Extra clearance around this part beyond the job spacing (e.g. tabs, lead-ins).
    #[serde(default)]
    pub extra_clearance: f64,
    /// Approved projection for a part whose geometry is not planar. Without it, a
    /// non-planar part has no footprint and cannot be placed.
    #[serde(default)]
    pub projection: Option<ProjectionPlane>,
}

/// The plane a non-planar part is flattened onto, in part-local coordinates.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum ProjectionPlane {
    Xy,
    Xz,
    Yz,
}
impl ProjectionPlane {
    pub fn project(self, p: glam::DVec3) -> DVec2 {
        match self {
            ProjectionPlane::Xy => DVec2::new(p.x, p.y),
            ProjectionPlane::Xz => DVec2::new(p.x, p.z),
            ProjectionPlane::Yz => DVec2::new(p.y, p.z),
        }
    }
}
impl PartDefinition {
    pub fn new(id: PartId, name: impl Into<String>, source: PartSource) -> Self {
        Self {
            id,
            name: name.into(),
            source,
            outline: None,
            holes: vec![],
            features: vec![],
            quantity: 1,
            rotations: Rotations::default(),
            allow_mirror: false,
            grain_deg: None,
            extra_clearance: 0.0,
            projection: None,
        }
    }
    /// Choose the outline, removing it from the hole and feature roles.
    pub fn set_outline(&mut self, r: GeometryRef) {
        self.holes.retain(|h| !h.same_object(&r));
        self.features.retain(|f| !f.same_object(&r));
        self.outline = Some(r);
    }
    /// All part-local references in role order: outline, holes, features.
    pub fn local_refs(&self) -> impl Iterator<Item = &GeometryRef> {
        self.outline
            .iter()
            .chain(self.holes.iter())
            .chain(self.features.iter())
    }
    /// One instance per required copy, with stable identity `part/index`.
    pub fn instances(&self) -> Vec<PartInstance> {
        (0..self.quantity)
            .map(|index| PartInstance {
                id: PartInstanceId(format!("{}/{index}", self.id.0)),
                part: self.id.clone(),
                index,
            })
            .collect()
    }
}

/// One required copy of a part. Derived from quantity; identity is stable across edits
/// that keep the part ID, so a placement for copy 3 survives raising the quantity to 5.
#[derive(Debug, Clone, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct PartInstance {
    pub id: PartInstanceId,
    pub part: PartId,
    pub index: u32,
}

/// A rectangular region of a sheet that parts must not enter (clamps, damage, labels).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Exclusion {
    pub name: String,
    pub min: DVec2,
    pub max: DVec2,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Sheet {
    pub id: SheetId,
    pub name: String,
    /// Width (X) and height (Y) of the usable sheet.
    pub size: DVec2,
    pub thickness: f64,
    /// Distance from the sheet edge that parts must stay clear of.
    pub edge_margin: f64,
    /// Grain direction on the sheet in degrees; `None` = no grain.
    #[serde(default)]
    pub grain_deg: Option<f64>,
    #[serde(default)]
    pub exclusions: Vec<Exclusion>,
    /// How many identical sheets of this kind may be used. Each used copy gets its own
    /// placements through `Placement::sheet_copy`.
    pub available: u32,
}
impl Sheet {
    pub fn new(id: SheetId, name: impl Into<String>, size: DVec2, thickness: f64) -> Self {
        Self {
            id,
            name: name.into(),
            size,
            thickness,
            edge_margin: 0.0,
            grain_deg: None,
            exclusions: vec![],
            available: 1,
        }
    }
}

/// Job-wide rules the solver and manual edits both obey.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct NestConstraints {
    /// Minimum gap between any two parts' footprints.
    pub part_spacing: f64,
    /// Tolerance for treating touching/overlap as a violation.
    pub tolerance: f64,
    /// Deterministic seed for any randomized search.
    pub seed: u64,
    /// Whether a part may be placed inside another part's hole (M8 feature; off here).
    #[serde(default)]
    pub allow_part_in_hole: bool,
}
impl Default for NestConstraints {
    fn default() -> Self {
        Self {
            part_spacing: 5.0,
            tolerance: 0.01,
            seed: 0,
            allow_part_in_hole: false,
        }
    }
}

/// Where one part instance sits on one sheet copy.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Placement {
    pub instance: PartInstanceId,
    pub sheet: SheetId,
    /// Which copy of the sheet kind (0-based) this placement is on.
    #[serde(default)]
    pub sheet_copy: u32,
    /// Sheet-local position of the part's local origin.
    pub position: DVec2,
    pub rotation_deg: f64,
    #[serde(default)]
    pub mirrored: bool,
    /// Locked placements are kept by the solver and only validated.
    #[serde(default)]
    pub locked: bool,
}

/// Result of checking a job's placements against its own rules (not yet geometry).
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct JobReport {
    /// Instances with no placement, in part order.
    pub unplaced: Vec<PartInstanceId>,
    /// Placements referring to instances that no longer exist (quantity lowered or part removed).
    pub orphaned: Vec<PartInstanceId>,
    /// Hard errors that make the job unusable until repaired.
    pub errors: Vec<String>,
}
impl JobReport {
    pub fn is_complete(&self) -> bool {
        self.unplaced.is_empty() && self.orphaned.is_empty() && self.errors.is_empty()
    }
}

#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct NestJob {
    pub parts: Vec<PartDefinition>,
    pub sheets: Vec<Sheet>,
    #[serde(default)]
    pub constraints: NestConstraints,
    #[serde(default)]
    pub placements: Vec<Placement>,
    #[serde(default)]
    pub next_id: u64,
}

impl NestJob {
    pub fn fresh_id(&mut self, kind: &str) -> String {
        loop {
            self.next_id += 1;
            let id = format!("{kind}/{}", self.next_id);
            if !self.parts.iter().any(|p| p.id.0 == id) && !self.sheets.iter().any(|s| s.id.0 == id)
            {
                return id;
            }
        }
    }
    pub fn part(&self, id: &PartId) -> Option<&PartDefinition> {
        self.parts.iter().find(|p| &p.id == id)
    }
    pub fn sheet(&self, id: &SheetId) -> Option<&Sheet> {
        self.sheets.iter().find(|s| &s.id == id)
    }
    /// Every required instance, in part order then copy order.
    pub fn instances(&self) -> Vec<PartInstance> {
        self.parts
            .iter()
            .flat_map(PartDefinition::instances)
            .collect()
    }
    pub fn placement(&self, instance: &PartInstanceId) -> Option<&Placement> {
        self.placements.iter().find(|p| &p.instance == instance)
    }

    /// Structural validation independent of a scene: IDs, numbers and rotation rules.
    pub fn validate(&self) -> Result<()> {
        let mut ids = HashSet::new();
        for p in &self.parts {
            ensure!(
                !p.id.0.is_empty() && ids.insert(&p.id.0),
                "duplicate or empty part ID: {}",
                p.id.0
            );
            ensure!(
                p.quantity >= 1,
                "part '{}' quantity must be at least 1",
                p.name
            );
            ensure!(
                p.extra_clearance.is_finite() && p.extra_clearance >= 0.0,
                "part '{}' clearance must be finite and non-negative",
                p.name
            );
            ensure!(
                p.grain_deg.is_none_or(f64::is_finite),
                "part '{}' grain angle is not finite",
                p.name
            );
            p.rotations
                .validate()
                .map_err(|e| anyhow::anyhow!("part '{}': {e}", p.name))?;
            if let PartSource::Loose { members } = &p.source {
                ensure!(
                    !members.is_empty(),
                    "part '{}' has no member geometry",
                    p.name
                );
            }
            // Part-local references must point at the part's own geometry.
            for r in p.local_refs() {
                match &p.source {
                    PartSource::Block { .. } => ensure!(
                        r.instances.is_empty(),
                        "part '{}' references {} through an instance path; block parts use definition-local references",
                        p.name,
                        r.object.0
                    ),
                    PartSource::Loose { members } => ensure!(
                        members.iter().any(|m| m.same_object(r)),
                        "part '{}' feature {} is not one of its members",
                        p.name,
                        r.object.0
                    ),
                }
            }
            if let Some(outline) = &p.outline {
                ensure!(
                    !p.holes
                        .iter()
                        .chain(&p.features)
                        .any(|r| r.same_object(outline)),
                    "part '{}' lists its outline as a hole or feature",
                    p.name
                );
            }
        }
        let mut ids = HashSet::new();
        for s in &self.sheets {
            ensure!(
                !s.id.0.is_empty() && ids.insert(&s.id.0),
                "duplicate or empty sheet ID: {}",
                s.id.0
            );
            ensure!(
                s.size.is_finite() && s.size.cmpgt(DVec2::ZERO).all(),
                "sheet '{}' size must be positive",
                s.name
            );
            ensure!(
                s.thickness.is_finite() && s.thickness > 0.0,
                "sheet '{}' thickness must be positive",
                s.name
            );
            ensure!(
                s.edge_margin.is_finite()
                    && s.edge_margin >= 0.0
                    && 2.0 * s.edge_margin < s.size.min_element(),
                "sheet '{}' margin leaves no usable area",
                s.name
            );
            ensure!(
                s.available >= 1,
                "sheet '{}' must have at least one copy",
                s.name
            );
            for e in &s.exclusions {
                ensure!(
                    e.min.is_finite() && e.max.is_finite() && e.min.cmplt(e.max).all(),
                    "sheet '{}' exclusion '{}' is degenerate",
                    s.name,
                    e.name
                );
            }
        }
        let c = &self.constraints;
        ensure!(
            c.part_spacing.is_finite() && c.part_spacing >= 0.0,
            "part spacing must be finite and non-negative"
        );
        ensure!(
            c.tolerance.is_finite() && c.tolerance > 0.0,
            "tolerance must be finite and positive"
        );
        let mut placed = HashSet::new();
        for pl in &self.placements {
            ensure!(
                placed.insert(&pl.instance),
                "instance {} is placed more than once",
                pl.instance.0
            );
            ensure!(
                pl.position.is_finite() && pl.rotation_deg.is_finite(),
                "placement of {} has a non-finite transform",
                pl.instance.0
            );
            let sheet = self.sheet(&pl.sheet);
            ensure!(
                sheet.is_some(),
                "placement of {} is on missing sheet {}",
                pl.instance.0,
                pl.sheet.0
            );
            ensure!(
                pl.sheet_copy < sheet.unwrap().available,
                "placement of {} uses sheet copy {} but only {} available",
                pl.instance.0,
                pl.sheet_copy,
                sheet.unwrap().available
            );
        }
        Ok(())
    }

    /// Reconcile placements with required instances and part rules. Never drops anything
    /// silently: orphaned and unplaced instances are reported for the user to resolve.
    pub fn report(&self) -> JobReport {
        let mut report = JobReport::default();
        if let Err(e) = self.validate() {
            report.errors.push(format!("{e:#}"));
        }
        let instances = self.instances();
        let required: HashSet<&PartInstanceId> = instances.iter().map(|i| &i.id).collect();
        for pl in &self.placements {
            if !required.contains(&pl.instance) {
                report.orphaned.push(pl.instance.clone());
            }
        }
        for i in &instances {
            match self.placement(&i.id) {
                None => report.unplaced.push(i.id.clone()),
                Some(pl) => {
                    let part = self.part(&i.part).unwrap();
                    if part.outline.is_none() {
                        report.errors.push(format!(
                            "part '{}' has no outline; its placement cannot be checked",
                            part.name
                        ));
                    }
                    if !part.rotations.allows(pl.rotation_deg) {
                        report.errors.push(format!(
                            "instance {} is rotated {}°, which part '{}' does not allow",
                            i.id.0, pl.rotation_deg, part.name
                        ));
                    }
                    if pl.mirrored && !part.allow_mirror {
                        report.errors.push(format!(
                            "instance {} is mirrored but part '{}' forbids mirroring",
                            i.id.0, part.name
                        ));
                    }
                    if let (Some(pg), Some(sheet)) = (part.grain_deg, self.sheet(&pl.sheet)) {
                        if let Some(sg) = sheet.grain_deg {
                            if !grain_aligned(pg, pl.mirrored, pl.rotation_deg, sg) {
                                report.errors.push(format!(
                                    "instance {} grain is not aligned with the sheet grain",
                                    i.id.0
                                ));
                            }
                        }
                    }
                }
            }
        }
        report
    }

    /// Remove placements of instances that no longer exist. Returns what was removed so the
    /// caller can show it; nothing disappears without a record.
    pub fn prune_orphans(&mut self) -> Vec<Placement> {
        let required: HashSet<PartInstanceId> =
            self.instances().into_iter().map(|i| i.id).collect();
        let (keep, orphans): (Vec<_>, Vec<_>) = self
            .placements
            .drain(..)
            .partition(|p| required.contains(&p.instance));
        self.placements = keep;
        orphans
    }

    /// Check that every part still refers to geometry that exists in `scene`.
    pub fn validate_against(&self, scene: &Scene) -> Result<()> {
        self.validate()?;
        let resolved = scene.resolved()?;
        for p in &self.parts {
            match &p.source {
                PartSource::Block { definition } => {
                    let Some(def) = scene.definitions.iter().find(|d| &d.id == definition) else {
                        bail!(
                            "part '{}' refers to missing block definition {}",
                            p.name,
                            definition.0
                        );
                    };
                    for r in p.local_refs() {
                        ensure!(
                            def.objects.iter().any(|o| o.id == r.object),
                            "part '{}' feature {} is not in block '{}'; repair the part",
                            p.name,
                            r.object.0,
                            def.name
                        );
                    }
                }
                PartSource::Loose { members } => {
                    for m in members {
                        ensure!(
                            resolved.iter().any(|o| o.reference.same_object(m)),
                            "part '{}' member {} no longer exists; repair the part",
                            p.name,
                            m.object.0
                        );
                    }
                }
            }
        }
        Ok(())
    }
}
