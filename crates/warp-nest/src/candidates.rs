//! Part discovery: blocks become candidate parts; loose geometry can be grouped
//! explicitly. Both get an outline/hole suggestion with ambiguity reporting rather than a
//! silent guess.

use crate::{PartDefinition, PartId, PartSource};
use anyhow::{Context, Result};
use glam::{DMat4, DVec3};
use warp_core::scene::{Geometry, Scene};
use warp_core::selection::GeometryRef;
use warp_core::Polyline2;

/// Planarity tolerance for treating a curve as a 2D loop in the footprint plane.
pub const PLANAR_TOLERANCE: f64 = 1e-3;

/// What the finder proposes for one part. `outline` is only set when exactly one closed
/// loop contains every other closed loop; otherwise `ambiguities` explains why.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Suggestion {
    pub outline: Option<GeometryRef>,
    pub holes: Vec<GeometryRef>,
    /// Open curves, points and loops nested deeper than a hole.
    pub features: Vec<GeometryRef>,
    pub ambiguities: Vec<String>,
}

/// A block definition that could be a part.
#[derive(Debug, Clone, PartialEq)]
pub struct BlockCandidate {
    pub definition: warp_core::scene::DefinitionId,
    pub name: String,
    /// Resolved instances of this definition in the scene (nested paths included).
    pub instance_count: u32,
    pub suggestion: Suggestion,
}

struct Loop {
    reference: GeometryRef,
    curve: Polyline2,
}

/// Classify a set of references by their XY shape in `transform` coordinates.
fn analyse(items: Vec<(GeometryRef, &Geometry, DMat4, String)>) -> Suggestion {
    let mut s = Suggestion::default();
    let mut loops: Vec<Loop> = vec![];
    for (reference, geometry, m, name) in items {
        match geometry {
            Geometry::Curve { points, closed, .. } => {
                let pts: Vec<DVec3> = points.iter().map(|p| m.transform_point3(*p)).collect();
                let z0 = pts.first().map(|p| p.z).unwrap_or(0.0);
                if pts.iter().any(|p| (p.z - z0).abs() > PLANAR_TOLERANCE) {
                    s.ambiguities.push(format!(
                        "'{name}' is not planar; approve a projected footprint before nesting"
                    ));
                    s.features.push(reference);
                    continue;
                }
                let curve = Polyline2::new(pts.iter().map(|p| p.truncate()).collect(), *closed);
                if *closed && curve.pts.len() >= 3 && curve.signed_area().abs() > 1e-9 {
                    loops.push(Loop { reference, curve });
                } else {
                    s.features.push(reference);
                }
            }
            Geometry::Point(_) => s.features.push(reference),
            Geometry::Mesh(_) => {
                s.ambiguities.push(format!(
                    "'{name}' is a mesh; select a setup plane and approve its footprint"
                ));
                s.features.push(reference);
            }
            Geometry::Unavailable { reason, .. } => s
                .ambiguities
                .push(format!("'{name}' is unavailable: {reason}")),
            Geometry::Instance { .. } => s.ambiguities.push(format!(
                "'{name}' is a nested block; nest the block itself or explode it explicitly"
            )),
        }
    }
    if loops.is_empty() {
        s.ambiguities
            .push("no closed planar loop; an open curve is never used as an outline".into());
        return s;
    }
    // Containment depth by point-in-polygon of a vertex; equal-area duplicates are flagged.
    let contains = |a: &Loop, b: &Loop| -> bool {
        a.curve.signed_area().abs() > b.curve.signed_area().abs()
            && b.curve.pts.iter().all(|p| a.curve.contains(*p))
    };
    let partial = |a: &Loop, b: &Loop| -> bool {
        let inside = b.curve.pts.iter().filter(|p| a.curve.contains(**p)).count();
        inside > 0 && inside < b.curve.pts.len()
    };
    let n = loops.len();
    let depth: Vec<usize> = (0..n)
        .map(|i| {
            (0..n)
                .filter(|&j| j != i && contains(&loops[j], &loops[i]))
                .count()
        })
        .collect();
    for i in 0..n {
        for j in i + 1..n {
            if partial(&loops[i], &loops[j]) || partial(&loops[j], &loops[i]) {
                s.ambiguities.push(format!(
                    "loops '{}' and '{}' cross or touch; repair them before nesting",
                    loops[i].reference.object.0, loops[j].reference.object.0
                ));
            }
        }
    }
    let outer: Vec<usize> = (0..n).filter(|&i| depth[i] == 0).collect();
    if outer.len() > 1 {
        s.ambiguities.push(format!(
            "{} disconnected outer loops; choose the outline or split into separate parts",
            outer.len()
        ));
    }
    let outline = (outer.len() == 1).then(|| outer[0]);
    for i in 0..n {
        if Some(i) == outline {
            continue;
        }
        if outline.is_some() && depth[i] == 1 {
            s.holes.push(loops[i].reference.clone());
        } else {
            s.features.push(loops[i].reference.clone());
        }
    }
    s.outline = outline.map(|i| loops[i].reference.clone());
    s
}

/// Every block definition that contains geometry, with its suggestion in
/// definition-local coordinates and how many times it is placed in the scene.
pub fn block_candidates(scene: &Scene) -> Result<Vec<BlockCandidate>> {
    let resolved = scene.resolved()?;
    let mut out = vec![];
    for def in &scene.definitions {
        if def.objects.is_empty() {
            continue;
        }
        // Count distinct instance paths whose last element is an instance of this block.
        let mut paths = std::collections::HashSet::new();
        for o in &resolved {
            for (k, inst) in o.reference.instances.iter().enumerate() {
                let is_this = scene
                    .objects
                    .iter()
                    .chain(scene.definitions.iter().flat_map(|d| &d.objects))
                    .find(|obj| obj.id.0 == inst.0)
                    .is_some_and(|obj| {
                        matches!(&obj.geometry, Geometry::Instance { definition, .. } if definition == &def.id)
                    });
                if is_this {
                    paths.insert(o.reference.instances[..=k].to_vec());
                }
            }
        }
        let items = def
            .objects
            .iter()
            .map(|o| {
                (
                    GeometryRef::object(o.id.clone()),
                    &o.geometry,
                    DMat4::IDENTITY,
                    o.name.clone(),
                )
            })
            .collect();
        out.push(BlockCandidate {
            definition: def.id.clone(),
            name: def.name.clone(),
            instance_count: paths.len() as u32,
            suggestion: analyse(items),
        });
    }
    Ok(out)
}

/// Suggest roles for an explicit loose group, analysed in scene coordinates.
pub fn loose_suggestion(scene: &Scene, members: &[GeometryRef]) -> Result<Suggestion> {
    let resolved = scene.resolved()?;
    let mut items = vec![];
    for m in members {
        let obj = resolved
            .iter()
            .find(|o| o.reference.same_object(m))
            .with_context(|| format!("unresolved member {}", m.object.0))?;
        items.push((m.clone(), &obj.geometry, obj.transform, obj.name.clone()));
    }
    Ok(analyse(items))
}

/// Build a part from a block candidate, applying its suggestion. The caller decides the
/// quantity; the default is the number of placed instances, or 1 if none.
pub fn part_from_block(id: PartId, c: &BlockCandidate) -> PartDefinition {
    let mut p = PartDefinition::new(
        id,
        c.name.clone(),
        PartSource::Block {
            definition: c.definition.clone(),
        },
    );
    apply(&mut p, &c.suggestion);
    p.quantity = c.instance_count.max(1);
    p
}

/// Build a loose part from members and a suggestion.
pub fn part_from_loose(
    id: PartId,
    name: impl Into<String>,
    members: Vec<GeometryRef>,
    s: &Suggestion,
) -> PartDefinition {
    let mut p = PartDefinition::new(id, name, PartSource::Loose { members });
    apply(&mut p, s);
    p
}

fn apply(p: &mut PartDefinition, s: &Suggestion) {
    p.outline = s.outline.clone();
    p.holes = s.holes.clone();
    p.features = s.features.clone();
}
