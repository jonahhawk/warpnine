//! Deterministic first solver: bottom-left placement over allowed orientations.
//!
//! Instances are placed largest-first on the first sheet copy with room, at the lowest,
//! then leftmost feasible candidate point. Candidate points come from the usable corner,
//! placed parts' bounding boxes plus the required gap, and exclusion corners. Every
//! candidate is checked with the same feasibility rules as manual edits, so a solver
//! layout never violates clearance, margins or exclusions. Locked placements are kept
//! and validated, never moved. There is no randomness; the same job yields the same layout.

use crate::feasibility::{fits_sheet, oriented_bbox, usable, Body};
use crate::footprint::Footprint;
use crate::{grain_aligned, NestJob, PartId, PartInstanceId, Placement, Sheet};
use glam::DVec2;
use std::collections::HashMap;

#[derive(Debug, Clone, PartialEq)]
pub struct Unplaced {
    pub instance: PartInstanceId,
    pub reason: String,
}

/// Layout quality. Lower sheet count wins; then higher utilization; then a larger
/// contiguous remnant on the last sheet.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Score {
    pub sheets_used: u32,
    /// Net part area (outline minus holes) over the usable area of the used sheets.
    pub utilization: f64,
    /// Fraction of the last used sheet's usable area that is a free full-width strip above
    /// the parts or a free full-height strip to their right, whichever is larger.
    pub remnant: f64,
}
impl Score {
    pub fn better_than(&self, other: &Score) -> bool {
        if self.sheets_used != other.sheets_used {
            return self.sheets_used < other.sheets_used;
        }
        if (self.utilization - other.utilization).abs() > 1e-9 {
            return self.utilization > other.utilization;
        }
        self.remnant > other.remnant + 1e-9
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct Layout {
    pub placements: Vec<Placement>,
    pub unplaced: Vec<Unplaced>,
    pub score: Score,
    /// True when cancellation stopped the solve before every instance was tried.
    pub cancelled: bool,
}

struct Orientation {
    rotation_deg: f64,
    mirrored: bool,
    /// Bounding box of the oriented outline relative to the part origin.
    bbox: warp_core::Aabb2,
}

fn orientations(part: &crate::PartDefinition, fp: &Footprint, sheet: &Sheet) -> Vec<Orientation> {
    let mut out = vec![];
    let mirrors: &[bool] = if part.allow_mirror {
        &[false, true]
    } else {
        &[false]
    };
    for &mirrored in mirrors {
        for r in part.rotations.candidates() {
            if let (Some(pg), Some(sg)) = (part.grain_deg, sheet.grain_deg) {
                if !grain_aligned(pg, mirrored, r, sg) {
                    continue;
                }
            }
            out.push(Orientation {
                rotation_deg: r,
                mirrored,
                bbox: oriented_bbox(fp, r, mirrored),
            });
        }
    }
    out
}

/// Solve `job` with the given footprints. `cancel` is polled between placements; on
/// cancellation the best feasible partial layout so far is returned with the remaining
/// instances listed as unplaced.
pub fn solve(
    job: &NestJob,
    footprints: &HashMap<PartId, Footprint>,
    cancel: &dyn Fn() -> bool,
) -> Layout {
    let spacing = job.constraints.part_spacing.max(0.0);
    let tol = job.constraints.tolerance;
    let mut placements: Vec<Placement> = vec![];
    let mut unplaced: Vec<Unplaced> = vec![];
    let mut bodies: Vec<(Body, f64)> = vec![];

    // Locked placements stay. They still occupy space for everything else.
    let instances = job.instances();
    for pl in job.placements.iter().filter(|p| p.locked) {
        let Some(inst) = instances.iter().find(|i| i.id == pl.instance) else {
            continue;
        };
        let part = job.part(&inst.part).unwrap();
        let Some(fp) = footprints.get(&part.id) else {
            unplaced.push(Unplaced {
                instance: inst.id.clone(),
                reason: format!("locked, but part '{}' has no footprint", part.name),
            });
            continue;
        };
        bodies.push((
            Body::new(inst.id.clone(), pl, fp, part.extra_clearance, spacing),
            fp.area(),
        ));
        placements.push(pl.clone());
    }

    // Largest footprint first; ties by part ID then copy index for determinism.
    let mut order: Vec<&crate::PartInstance> = instances
        .iter()
        .filter(|i| !placements.iter().any(|p| p.instance == i.id))
        .collect();
    order.sort_by(|a, b| {
        let area = |i: &crate::PartInstance| {
            footprints
                .get(&i.part)
                .map(|f| f.bbox().size().x * f.bbox().size().y)
                .unwrap_or(0.0)
        };
        area(b)
            .total_cmp(&area(a))
            .then_with(|| a.part.cmp(&b.part))
            .then(a.index.cmp(&b.index))
    });

    let mut cancelled = false;
    for inst in order {
        if cancel() {
            cancelled = true;
            unplaced.push(Unplaced {
                instance: inst.id.clone(),
                reason: "cancelled before placement".into(),
            });
            continue;
        }
        let part = job.part(&inst.part).unwrap();
        let Some(fp) = footprints.get(&part.id) else {
            unplaced.push(Unplaced {
                instance: inst.id.clone(),
                reason: format!(
                    "part '{}' has no footprint; set an outline or approve a projection",
                    part.name
                ),
            });
            continue;
        };
        let mut reason = None;
        let mut best: Option<(Placement, Body)> = None;
        'sheets: for sheet in &job.sheets {
            let orients = orientations(part, fp, sheet);
            if orients.is_empty() {
                reason.get_or_insert_with(|| {
                    format!(
                        "no allowed rotation aligns part '{}' with the grain of sheet '{}'",
                        part.name, sheet.name
                    )
                });
                continue;
            }
            let rots: Vec<f64> = orients.iter().map(|o| o.rotation_deg).collect();
            if !fits_sheet(part, fp, sheet, &rots) {
                reason.get_or_insert_with(|| {
                    format!(
                        "part '{}' is oversize for sheet '{}'",
                        part.name, sheet.name
                    )
                });
                continue;
            }
            for copy in 0..sheet.available {
                let on_sheet: Vec<&Body> = bodies
                    .iter()
                    .map(|(b, _)| b)
                    .filter(|b| b.sheet == sheet.id && b.sheet_copy == copy)
                    .collect();
                let candidates =
                    candidate_points(sheet, &on_sheet, part.extra_clearance, spacing, tol);
                let mut found: Option<(Placement, Body, DVec2)> = None;
                for o in &orients {
                    for &c in &candidates {
                        // Place so the oriented outline's bbox minimum lands on the candidate.
                        let position = c - o.bbox.min;
                        if let Some((_, _, prev)) = &found {
                            // Bottom-left: only strictly lower, or equal-low and further left, wins.
                            if c.y > prev.y + 1e-9
                                || ((c.y - prev.y).abs() <= 1e-9 && c.x >= prev.x - 1e-9)
                            {
                                continue;
                            }
                        }
                        let pl = Placement {
                            instance: inst.id.clone(),
                            sheet: sheet.id.clone(),
                            sheet_copy: copy,
                            position,
                            rotation_deg: o.rotation_deg,
                            mirrored: o.mirrored,
                            locked: false,
                        };
                        let body =
                            Body::new(inst.id.clone(), &pl, fp, part.extra_clearance, spacing);
                        if !body.sheet_violations(sheet, tol).is_empty() {
                            continue;
                        }
                        if on_sheet.iter().any(|b| body.overlaps(b, tol)) {
                            continue;
                        }
                        found = Some((pl, body, c));
                    }
                }
                if let Some((pl, body, _)) = found {
                    best = Some((pl, body));
                    break 'sheets;
                }
            }
            reason.get_or_insert_with(|| {
                format!("no room for part '{}' on the available sheets", part.name)
            });
        }
        match best {
            Some((pl, body)) => {
                bodies.push((body, fp.area()));
                placements.push(pl);
            }
            None => unplaced.push(Unplaced {
                instance: inst.id.clone(),
                reason: reason.unwrap_or_else(|| "no sheets defined".into()),
            }),
        }
    }
    let score = score(job, &bodies);
    Layout {
        placements,
        unplaced,
        score,
        cancelled,
    }
}

/// Bottom-left candidate points on one sheet copy, deduplicated and sorted by (y, x).
fn candidate_points(
    sheet: &Sheet,
    on_sheet: &[&Body],
    extra: f64,
    spacing: f64,
    tol: f64,
) -> Vec<DVec2> {
    let (min, _) = usable(sheet);
    let origin = min + DVec2::splat(extra);
    let mut pts = vec![origin];
    for b in on_sheet {
        let gap = spacing + b.extra + extra;
        pts.push(DVec2::new(b.bbox.max.x + gap, b.bbox.min.y));
        pts.push(DVec2::new(b.bbox.min.x, b.bbox.max.y + gap));
        pts.push(DVec2::new(origin.x, b.bbox.max.y + gap));
        pts.push(DVec2::new(b.bbox.max.x + gap, origin.y));
    }
    for e in &sheet.exclusions {
        let gap = extra + tol;
        pts.push(DVec2::new(e.max.x + gap, e.min.y.max(origin.y)));
        pts.push(DVec2::new(e.min.x.max(origin.x), e.max.y + gap));
        pts.push(DVec2::new(origin.x, e.max.y + gap));
        pts.push(DVec2::new(e.max.x + gap, origin.y));
    }
    pts.sort_by(|a, b| a.y.total_cmp(&b.y).then(a.x.total_cmp(&b.x)));
    pts.dedup_by(|a, b| a.distance(*b) < 1e-9);
    pts
}

/// Score a set of placed bodies (with their net areas) against the job's sheets.
pub fn score(job: &NestJob, bodies: &[(Body, f64)]) -> Score {
    let mut used: Vec<(&Sheet, u32)> = vec![];
    for (b, _) in bodies {
        if let Some(s) = job.sheet(&b.sheet) {
            if !used.iter().any(|(u, c)| u.id == s.id && *c == b.sheet_copy) {
                used.push((s, b.sheet_copy));
            }
        }
    }
    let usable_area: f64 = used
        .iter()
        .map(|(s, _)| {
            let (min, max) = usable(s);
            (max - min).x * (max - min).y
        })
        .sum();
    let part_area: f64 = bodies.iter().map(|(_, a)| a).sum();
    let remnant = used
        .last()
        .map(|(s, c)| {
            let (min, max) = usable(s);
            let mut extent = min;
            for (b, _) in bodies
                .iter()
                .filter(|(b, _)| b.sheet == s.id && b.sheet_copy == *c)
            {
                extent = extent.max(b.bbox.max);
            }
            let size = max - min;
            let top = (max.y - extent.y).max(0.0) * size.x;
            let right = (max.x - extent.x).max(0.0) * size.y;
            top.max(right) / (size.x * size.y)
        })
        .unwrap_or(0.0);
    Score {
        sheets_used: used.len() as u32,
        utilization: if usable_area > 0.0 {
            part_area / usable_area
        } else {
            0.0
        },
        remnant,
    }
}

/// Apply a layout to the job: locked placements are kept, every other placement is
/// replaced. Returns the instances the layout could not place.
pub fn apply(job: &mut NestJob, layout: &Layout) -> Vec<Unplaced> {
    job.placements = layout.placements.clone();
    layout.unplaced.clone()
}
