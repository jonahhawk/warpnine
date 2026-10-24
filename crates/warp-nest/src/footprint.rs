//! Part footprints: the closed outline and holes of a part in its local frame, derived
//! from scene geometry at check time (never stored). The local origin is the outline's
//! bounding-box minimum so placements are meaningful across re-imports.

use crate::{PartDefinition, PartSource, Placement};
use anyhow::{bail, ensure, Context, Result};
use glam::{DMat4, DVec2, DVec3};
use warp_core::scene::{Geometry, Scene};
use warp_core::selection::GeometryRef;
use warp_core::{Aabb2, Polyline2};

/// Z-spread tolerance for calling a loop planar without an approved projection.
pub const PLANAR_TOLERANCE: f64 = 1e-3;

#[derive(Debug, Clone, PartialEq)]
pub struct Footprint {
    /// Counter-clockwise outer loop with bounding-box minimum at the origin.
    pub outline: Polyline2,
    /// Clockwise holes in the same frame.
    pub holes: Vec<Polyline2>,
    /// Translation applied to source coordinates to reach the local frame.
    pub source_offset: DVec2,
    /// Z of the outline plane in source coordinates (0 for projected parts).
    pub source_z: f64,
}
impl Footprint {
    pub fn bbox(&self) -> Aabb2 {
        self.outline.bbox()
    }
    pub fn area(&self) -> f64 {
        self.outline.signed_area()
            - self
                .holes
                .iter()
                .map(|h| h.signed_area().abs())
                .sum::<f64>()
    }
}

/// Resolve one reference to its flattened 2D loop in the part's source frame.
fn flatten(
    part: &PartDefinition,
    geometry: &Geometry,
    transform: DMat4,
    name: &str,
) -> Result<(Polyline2, f64)> {
    let Geometry::Curve { points, closed, .. } = geometry else {
        bail!("part '{}': {name} is not a curve", part.name);
    };
    ensure!(*closed, "part '{}': {name} is not closed", part.name);
    let pts: Vec<DVec3> = points
        .iter()
        .map(|p| transform.transform_point3(*p))
        .collect();
    let (flat, z) = match part.projection {
        Some(plane) => (
            pts.iter().map(|p| plane.project(*p)).collect::<Vec<_>>(),
            0.0,
        ),
        None => {
            let z0 = pts.first().map(|p| p.z).unwrap_or(0.0);
            ensure!(
                pts.iter().all(|p| (p.z - z0).abs() <= PLANAR_TOLERANCE),
                "part '{}': {name} is not planar; approve a projected footprint first",
                part.name
            );
            (pts.iter().map(|p| p.truncate()).collect(), z0)
        }
    };
    let loop_ = Polyline2::new(flat, true);
    ensure!(
        loop_.pts.len() >= 3 && loop_.signed_area().abs() > 1e-9,
        "part '{}': {name} is degenerate",
        part.name
    );
    Ok((loop_, z))
}

/// Build the footprint for a part from the scene. Errors are repair instructions.
pub fn footprint(part: &PartDefinition, scene: &Scene) -> Result<Footprint> {
    let outline_ref = part
        .outline
        .as_ref()
        .with_context(|| format!("part '{}' has no outline", part.name))?;
    // Resolve a part-local reference to (geometry, transform, name).
    let lookup: Box<dyn Fn(&GeometryRef) -> Result<(Geometry, DMat4, String)>> = match &part.source
    {
        PartSource::Block { definition } => {
            let def = scene
                .definitions
                .iter()
                .find(|d| &d.id == definition)
                .with_context(|| {
                    format!(
                        "part '{}' refers to missing block {}",
                        part.name, definition.0
                    )
                })?;
            Box::new(move |r: &GeometryRef| {
                let o = def
                    .objects
                    .iter()
                    .find(|o| o.id == r.object)
                    .with_context(|| {
                        format!(
                            "part '{}' feature {} is not in its block",
                            part.name, r.object.0
                        )
                    })?;
                Ok((o.geometry.clone(), DMat4::IDENTITY, o.name.clone()))
            })
        }
        PartSource::Loose { .. } => {
            let resolved = scene.resolved()?;
            Box::new(move |r: &GeometryRef| {
                let o = resolved
                    .iter()
                    .find(|o| o.reference.same_object(r))
                    .with_context(|| {
                        format!(
                            "part '{}' member {} no longer exists",
                            part.name, r.object.0
                        )
                    })?;
                Ok((o.geometry.clone(), o.transform, o.name.clone()))
            })
        }
    };
    let (g, m, name) = lookup(outline_ref)?;
    let (mut outline, source_z) = flatten(part, &g, m, &name)?;
    if !outline.is_ccw() {
        outline = outline.reversed();
    }
    let min = outline.bbox().min;
    outline.translate(-min);
    let mut holes = vec![];
    for r in &part.holes {
        let (g, m, name) = lookup(r)?;
        let (mut h, z) = flatten(part, &g, m, &name)?;
        ensure!(
            part.projection.is_some() || (z - source_z).abs() <= PLANAR_TOLERANCE,
            "part '{}': hole {name} is not in the outline plane",
            part.name
        );
        h.translate(-min);
        ensure!(
            h.pts.iter().all(|p| outline.contains(*p)),
            "part '{}': hole {name} is not inside the outline",
            part.name
        );
        if h.is_ccw() {
            h = h.reversed();
        }
        holes.push(h);
    }
    Ok(Footprint {
        outline,
        holes,
        source_offset: -min,
        source_z,
    })
}

/// Map a local footprint point onto the sheet for a placement: optional mirror about the
/// local Y axis, then rotation about the local origin, then translation.
pub fn place_point(p: DVec2, placement: &Placement) -> DVec2 {
    let p = if placement.mirrored {
        DVec2::new(-p.x, p.y)
    } else {
        p
    };
    DVec2::from_angle(placement.rotation_deg.to_radians()).rotate(p) + placement.position
}

/// A loop transformed by a placement, re-normalised to counter-clockwise.
pub fn place_loop(l: &Polyline2, placement: &Placement, ccw: bool) -> Polyline2 {
    let out = Polyline2::new(
        l.pts.iter().map(|p| place_point(*p, placement)).collect(),
        true,
    );
    if out.is_ccw() == ccw {
        out
    } else {
        out.reversed()
    }
}

/// The footprint on the sheet: CCW outline and CW holes.
pub fn placed(f: &Footprint, placement: &Placement) -> (Polyline2, Vec<Polyline2>) {
    (
        place_loop(&f.outline, placement, true),
        f.holes
            .iter()
            .map(|h| place_loop(h, placement, false))
            .collect(),
    )
}

/// Sheet-space bounding box of the outline after placement.
pub fn placed_bbox(f: &Footprint, placement: &Placement) -> Aabb2 {
    place_loop(&f.outline, placement, true).bbox()
}

/// The matrix that takes part source coordinates (definition or scene frame) to the
/// sheet frame, for carrying features and operations into CAM later.
pub fn source_to_sheet(f: &Footprint, placement: &Placement) -> DMat4 {
    let mut m = DMat4::from_translation(DVec3::new(
        f.source_offset.x,
        f.source_offset.y,
        -f.source_z,
    ));
    if placement.mirrored {
        m = DMat4::from_scale(DVec3::new(-1.0, 1.0, 1.0)) * m;
    }
    DMat4::from_translation(placement.position.extend(0.0))
        * DMat4::from_rotation_z(placement.rotation_deg.to_radians())
        * m
}
