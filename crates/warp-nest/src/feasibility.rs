//! Placement feasibility: sheet containment with margins, clamp exclusions and exact
//! part-to-part clearance. Broad-phase bounds first, then real polygon tests on the
//! clearance-inflated outlines. The solver and manual edits share these rules.

use crate::footprint::{footprint, place_loop, Footprint};
use crate::{NestJob, PartId, PartInstanceId, Placement, Sheet, SheetId};
use anyhow::Result;
use geo::{Buffer, Coord, Intersects, LineString, MultiPolygon, Polygon};
use glam::DVec2;
use std::collections::HashMap;
use warp_core::scene::Scene;
use warp_core::{Aabb2, Polyline2};

#[derive(Debug, Clone, PartialEq)]
pub enum Violation {
    /// Two instances on the same sheet copy are closer than the required gap.
    Overlap {
        a: PartInstanceId,
        b: PartInstanceId,
        sheet: SheetId,
        sheet_copy: u32,
        required_gap: f64,
    },
    /// The instance's clearance-inflated outline leaves the usable sheet area.
    OutsideSheet {
        instance: PartInstanceId,
        sheet: SheetId,
    },
    /// The instance's clearance-inflated outline enters a sheet exclusion.
    Exclusion {
        instance: PartInstanceId,
        sheet: SheetId,
        exclusion: String,
    },
    /// A placed part has no computable footprint (see message).
    NoFootprint { part: PartId, reason: String },
}
impl std::fmt::Display for Violation {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Violation::Overlap {
                a,
                b,
                sheet,
                sheet_copy,
                required_gap,
            } => write!(
                f,
                "{} and {} are closer than {required_gap} mm on {} copy {sheet_copy}",
                a.0, b.0, sheet.0
            ),
            Violation::OutsideSheet { instance, sheet } => {
                write!(f, "{} leaves the usable area of {}", instance.0, sheet.0)
            }
            Violation::Exclusion {
                instance,
                sheet,
                exclusion,
            } => write!(
                f,
                "{} enters exclusion '{exclusion}' on {}",
                instance.0, sheet.0
            ),
            Violation::NoFootprint { part, reason } => write!(f, "{}: {reason}", part.0),
        }
    }
}

fn poly(l: &Polyline2) -> Polygon<f64> {
    let mut v: Vec<Coord<f64>> = l.pts.iter().map(|p| Coord { x: p.x, y: p.y }).collect();
    if let Some(&f) = v.first() {
        v.push(f);
    }
    Polygon::new(LineString(v), vec![])
}

/// Outline grown by `d` (round joins); the original when `d` is zero.
fn inflated(l: &Polyline2, d: f64) -> MultiPolygon<f64> {
    let p = poly(l);
    if d <= 1e-12 {
        MultiPolygon::new(vec![p])
    } else {
        p.buffer(d)
    }
}

fn shrink(m: &MultiPolygon<f64>, d: f64) -> MultiPolygon<f64> {
    if d <= 1e-12 {
        return m.clone();
    }
    let mut out = m.buffer(-d);
    out.0.retain(|p| p.exterior().0.len() >= 4);
    out
}

fn mp_bbox(m: &MultiPolygon<f64>) -> Aabb2 {
    let mut b = Aabb2::EMPTY;
    for p in &m.0 {
        for c in p.exterior().0.iter() {
            b.add(DVec2::new(c.x, c.y));
        }
    }
    b
}

fn rect(min: DVec2, max: DVec2) -> Polygon<f64> {
    poly(&Polyline2::rect(min, max))
}

/// Whether every point of the multipolygon lies inside the axis-aligned box (tolerance
/// `tol`). Exact for convex containers because buffered outlines are vertex-sampled arcs.
fn within_box(m: &MultiPolygon<f64>, min: DVec2, max: DVec2, tol: f64) -> bool {
    m.0.iter().all(|p| {
        p.exterior().0.iter().all(|c| {
            c.x >= min.x - tol && c.x <= max.x + tol && c.y >= min.y - tol && c.y <= max.y + tol
        })
    })
}

/// Usable area of a sheet after its edge margin.
pub fn usable(sheet: &Sheet) -> (DVec2, DVec2) {
    (
        DVec2::splat(sheet.edge_margin),
        sheet.size - DVec2::splat(sheet.edge_margin),
    )
}

/// One placed instance with its clearance zones precomputed.
#[derive(Debug, Clone)]
pub struct Body {
    pub instance: PartInstanceId,
    pub sheet: SheetId,
    pub sheet_copy: u32,
    pub extra: f64,
    /// Outline on the sheet, before inflation.
    pub outline: Polyline2,
    /// Outline grown by the part's own extra clearance (sheet/exclusion tests).
    own: MultiPolygon<f64>,
    /// Outline grown by half the job spacing plus extra clearance (pair tests).
    half: MultiPolygon<f64>,
    pub bbox: Aabb2,
    pub half_bbox: Aabb2,
}
impl Body {
    pub fn new(
        instance: PartInstanceId,
        placement: &Placement,
        fp: &Footprint,
        extra: f64,
        spacing: f64,
    ) -> Self {
        let outline = place_loop(&fp.outline, placement, true);
        let own = inflated(&outline, extra);
        let half = inflated(&outline, extra + spacing.max(0.0) * 0.5);
        Body {
            instance,
            sheet: placement.sheet.clone(),
            sheet_copy: placement.sheet_copy,
            extra,
            bbox: outline.bbox(),
            half_bbox: mp_bbox(&half),
            outline,
            own,
            half,
        }
    }
    /// Containment and exclusion violations on `sheet`.
    pub fn sheet_violations(&self, sheet: &Sheet, tol: f64) -> Vec<Violation> {
        let mut out = vec![];
        let (min, max) = usable(sheet);
        if !within_box(&self.own, min, max, tol) {
            out.push(Violation::OutsideSheet {
                instance: self.instance.clone(),
                sheet: sheet.id.clone(),
            });
        }
        for e in &sheet.exclusions {
            // Exclusions are hard: shrink by tolerance so exact touching is allowed.
            let r = rect(e.min + DVec2::splat(tol), e.max - DVec2::splat(tol));
            if (e.max - e.min).cmpgt(DVec2::splat(2.0 * tol)).all() && self.own.intersects(&r) {
                out.push(Violation::Exclusion {
                    instance: self.instance.clone(),
                    sheet: sheet.id.clone(),
                    exclusion: e.name.clone(),
                });
            }
        }
        out
    }
    /// Whether two bodies on the same sheet copy are closer than their required gap.
    pub fn overlaps(&self, other: &Body, tol: f64) -> bool {
        if self.sheet != other.sheet || self.sheet_copy != other.sheet_copy {
            return false;
        }
        // Broad phase: shrink each box by the tolerance so touching at the gap passes.
        let t = DVec2::splat(tol * 0.5);
        if (self.half_bbox.min + t)
            .cmpgt(other.half_bbox.max - t)
            .any()
            || (other.half_bbox.min + t)
                .cmpgt(self.half_bbox.max - t)
                .any()
        {
            return false;
        }
        self.half.intersects(&shrink(&other.half, tol))
    }
}

/// Check every placement of `job` against sheet bounds, exclusions and each other,
/// using precomputed footprints. Parts without a footprint are reported, not skipped.
pub fn check_placements(job: &NestJob, footprints: &HashMap<PartId, Footprint>) -> Vec<Violation> {
    let mut out = vec![];
    let spacing = job.constraints.part_spacing;
    let tol = job.constraints.tolerance;
    let instances = job.instances();
    let mut bodies: Vec<Body> = vec![];
    let mut missing: Vec<&PartId> = vec![];
    for pl in &job.placements {
        let Some(inst) = instances.iter().find(|i| i.id == pl.instance) else {
            continue; // orphaned: reported by NestJob::report
        };
        let Some(part) = job.part(&inst.part) else {
            continue;
        };
        let Some(fp) = footprints.get(&part.id) else {
            if !missing.contains(&&part.id) {
                missing.push(&part.id);
                out.push(Violation::NoFootprint {
                    part: part.id.clone(),
                    reason: "no footprint; set an outline or approve a projection".into(),
                });
            }
            continue;
        };
        bodies.push(Body::new(
            inst.id.clone(),
            pl,
            fp,
            part.extra_clearance,
            spacing,
        ));
    }
    for b in &bodies {
        if let Some(sheet) = job.sheet(&b.sheet) {
            out.extend(b.sheet_violations(sheet, tol));
        }
    }
    for i in 0..bodies.len() {
        for j in i + 1..bodies.len() {
            let (a, b) = (&bodies[i], &bodies[j]);
            if a.overlaps(b, tol) {
                out.push(Violation::Overlap {
                    a: a.instance.clone(),
                    b: b.instance.clone(),
                    sheet: a.sheet.clone(),
                    sheet_copy: a.sheet_copy,
                    required_gap: spacing + a.extra + b.extra,
                });
            }
        }
    }
    out
}

/// Footprints for every part that has one, plus errors for those that do not.
pub fn footprints(job: &NestJob, scene: &Scene) -> (HashMap<PartId, Footprint>, Vec<Violation>) {
    let mut map = HashMap::new();
    let mut errors = vec![];
    for part in &job.parts {
        match footprint(part, scene) {
            Ok(f) => {
                map.insert(part.id.clone(), f);
            }
            Err(e) => errors.push(Violation::NoFootprint {
                part: part.id.clone(),
                reason: format!("{e:#}"),
            }),
        }
    }
    (map, errors)
}

/// Full check from the scene: footprints, then placements.
pub fn check(job: &NestJob, scene: &Scene) -> Result<Vec<Violation>> {
    job.validate()?;
    let (map, mut errors) = footprints(job, scene);
    // Footprint errors already name every part without a footprint.
    errors.extend(
        check_placements(job, &map)
            .into_iter()
            .filter(|v| !matches!(v, Violation::NoFootprint { .. })),
    );
    Ok(errors)
}

/// Bounding box of the outline at a rotation/mirror about the local origin.
pub fn oriented_bbox(fp: &Footprint, rotation_deg: f64, mirrored: bool) -> Aabb2 {
    let pl = Placement {
        instance: PartInstanceId::default(),
        sheet: SheetId::default(),
        sheet_copy: 0,
        position: DVec2::ZERO,
        rotation_deg,
        mirrored,
        locked: false,
    };
    place_loop(&fp.outline, &pl, true).bbox()
}

/// Whether a part can fit the usable area of `sheet` at some allowed rotation,
/// ignoring other parts. Used to report oversize parts before solving.
pub fn fits_sheet(
    part: &crate::PartDefinition,
    fp: &Footprint,
    sheet: &Sheet,
    rotations: &[f64],
) -> bool {
    let (min, max) = usable(sheet);
    let avail = max - min - DVec2::splat(2.0 * part.extra_clearance);
    rotations.iter().any(|r| {
        let size = oriented_bbox(fp, *r, false).size();
        size.x <= avail.x + 1e-9 && size.y <= avail.y + 1e-9
    })
}
