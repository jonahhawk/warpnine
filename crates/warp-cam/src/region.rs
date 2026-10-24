//! Planar regions (polygons with holes) and offsetting, built on `geo`.

use geo::{BooleanOps, Buffer, Coord, LineString, MultiLineString, MultiPolygon, Polygon};
use glam::DVec2;
use warp_core::Polyline2;

pub type Region = MultiPolygon<f64>;

fn ls(pts: &[DVec2]) -> LineString<f64> {
    let mut v: Vec<Coord<f64>> = pts.iter().map(|p| Coord { x: p.x, y: p.y }).collect();
    if let Some(&f) = v.first() {
        v.push(f);
    }
    LineString(v)
}

/// Loop points of a ring without the closing duplicate.
pub fn ring_pts(r: &LineString<f64>) -> Vec<DVec2> {
    let mut v: Vec<DVec2> = r.0.iter().map(|c| DVec2::new(c.x, c.y)).collect();
    if v.len() > 1 && v[0].distance(*v.last().unwrap()) < 1e-12 {
        v.pop();
    }
    v
}

/// Build a region from closed curves using even-odd nesting: a curve inside an odd number
/// of others is a hole (island), inside an even number it is a new outer boundary.
pub fn from_curves(curves: &[&Polyline2]) -> Region {
    let closed: Vec<&Polyline2> = curves
        .iter()
        .copied()
        .filter(|c| c.closed && c.pts.len() >= 3)
        .collect();
    let depth: Vec<usize> = closed
        .iter()
        .enumerate()
        .map(|(i, c)| {
            closed
                .iter()
                .enumerate()
                .filter(|(j, o)| {
                    *j != i && o.contains(c.pts[0]) && o.signed_area().abs() > c.signed_area().abs()
                })
                .count()
        })
        .collect();
    let mut polys = Vec::new();
    for (i, c) in closed.iter().enumerate() {
        if depth[i] % 2 != 0 {
            continue;
        }
        let holes: Vec<_> = closed
            .iter()
            .enumerate()
            .filter(|(j, h)| depth[*j] == depth[i] + 1 && c.contains(h.pts[0]))
            .map(|(_, h)| ls(&h.pts))
            .collect();
        polys.push(Polygon::new(ls(&c.pts), holes));
    }
    // Union normalises orientation and merges overlaps.
    let mut out = MultiPolygon::new(vec![]);
    for p in polys {
        out = out.union(&MultiPolygon::new(vec![p]));
    }
    out
}

pub fn rect(min: DVec2, max: DVec2) -> Region {
    MultiPolygon::new(vec![Polygon::new(
        ls(&Polyline2::rect(min, max).pts),
        vec![],
    )])
}

/// Offset a region; positive grows, negative shrinks. Round joins.
pub fn offset(r: &Region, d: f64) -> Region {
    if d.abs() < 1e-9 {
        return r.clone();
    }
    let mut out = r.buffer(d);
    out.0.retain(|p| ring_pts(p.exterior()).len() >= 3);
    out
}

pub fn is_empty(r: &Region) -> bool {
    r.0.is_empty()
}

/// All boundary loops, oriented for climb milling with a CW spindle: exterior rings CCW,
/// hole rings CW. `climb = false` reverses them.
pub fn loops(r: &Region, climb: bool) -> Vec<Vec<DVec2>> {
    let mut out = Vec::new();
    for p in &r.0 {
        let mut e = Polyline2::new(ring_pts(p.exterior()), true);
        if !e.is_ccw() {
            e = e.reversed();
        }
        out.push(if climb { e.pts } else { e.reversed().pts });
        for h in p.interiors() {
            let mut h = Polyline2::new(ring_pts(h), true);
            if h.is_ccw() {
                h = h.reversed();
            }
            out.push(if climb { h.pts } else { h.reversed().pts });
        }
    }
    out.retain(|l| l.len() >= 3);
    out
}

/// Clip straight lines to the inside of a region.
pub fn clip_lines(r: &Region, lines: &[(DVec2, DVec2)]) -> Vec<(DVec2, DVec2)> {
    let mls = MultiLineString::new(
        lines
            .iter()
            .map(|(a, b)| LineString(vec![Coord { x: a.x, y: a.y }, Coord { x: b.x, y: b.y }]))
            .collect(),
    );
    let clipped = r.clip(&mls, false);
    let mut out = Vec::new();
    for l in clipped.0 {
        let pts: Vec<DVec2> = l.0.iter().map(|c| DVec2::new(c.x, c.y)).collect();
        if pts.len() >= 2 {
            out.push((pts[0], *pts.last().unwrap()));
        }
    }
    out
}

/// Concentric offsets of a tool-centre region, outermost first, until it vanishes.
pub fn concentric(r: &Region, stepover: f64, max_rings: usize) -> Vec<Region> {
    let mut out = Vec::new();
    let mut cur = r.clone();
    while !is_empty(&cur) && out.len() < max_rings {
        out.push(cur.clone());
        cur = offset(&cur, -stepover);
    }
    out
}

pub fn difference(a: &Region, b: &Region) -> Region {
    a.difference(b)
}

pub fn intersection(a: &Region, b: &Region) -> Region {
    a.intersection(b)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn island_nesting() {
        let outer = Polyline2::rect(DVec2::ZERO, DVec2::splat(100.0));
        let island = Polyline2::rect(DVec2::splat(40.0), DVec2::splat(60.0));
        let r = from_curves(&[&outer, &island]);
        assert_eq!(r.0.len(), 1);
        assert_eq!(r.0[0].interiors().len(), 1);
        let shrunk = offset(&r, -5.0);
        let ls = loops(&shrunk, true);
        assert_eq!(ls.len(), 2);
        assert!(Polyline2::new(ls[0].clone(), true).is_ccw());
        assert!(!Polyline2::new(ls[1].clone(), true).is_ccw());
    }
}

/// Validate source chains before polygon construction. Crossing/touching boundaries
/// are ambiguous regions; callers must repair them instead of relying on union cleanup.
pub fn validate_chains(curves: &[&Polyline2], closed_required: bool) -> anyhow::Result<()> {
    use anyhow::ensure;
    use geo::{Intersects, Line};
    ensure!(!curves.is_empty(), "No geometry selected");
    let line = |a: DVec2, b: DVec2| Line::new(Coord { x: a.x, y: a.y }, Coord { x: b.x, y: b.y });
    for (i, c) in curves.iter().enumerate() {
        ensure!(
            !closed_required || c.closed,
            "Boundary {} is open; close it or choose an open-path operation",
            i + 1
        );
        ensure!(
            c.pts.len() >= if c.closed { 3 } else { 2 } && c.pts.iter().all(|p| p.is_finite()),
            "Invalid or degenerate chain {}",
            i + 1
        );
        let n = if c.closed {
            c.pts.len()
        } else {
            c.pts.len() - 1
        };
        for j in 0..n {
            ensure!(
                c.pts[j].distance(c.pts[(j + 1) % c.pts.len()]) > 1e-9,
                "Zero-length segment in chain {}",
                i + 1
            );
        }
        if !c.closed {
            continue;
        }
        ensure!(
            c.signed_area().abs() > 1e-12,
            "Degenerate or self-intersecting boundary {}",
            i + 1
        );
        for a in 0..n {
            for b in a + 1..n {
                if b == a + 1 || (a == 0 && b == n - 1) {
                    continue;
                }
                ensure!(
                    !line(c.pts[a], c.pts[(a + 1) % n])
                        .intersects(&line(c.pts[b], c.pts[(b + 1) % n])),
                    "Self-intersecting boundary {}",
                    i + 1
                );
            }
        }
        for other in &curves[..i] {
            if !other.closed {
                continue;
            }
            for a in 0..n {
                for b in 0..other.pts.len() {
                    ensure!(!line(c.pts[a],c.pts[(a+1)%n]).intersects(&line(other.pts[b],other.pts[(b+1)%other.pts.len()])),"Intersecting, touching or duplicate boundaries; repair the selected regions");
                }
            }
        }
    }
    Ok(())
}

pub fn validated(curves: &[&Polyline2]) -> anyhow::Result<Region> {
    validate_chains(curves, true)?;
    let r = from_curves(curves);
    anyhow::ensure!(!is_empty(&r), "No usable closed regions");
    Ok(r)
}

/// Close open pocket walls along the rectangular stock perimeter. Material to the
/// left of each directed wall chain is cleared; reversing the chain selects the other side.
pub fn open_pocket(
    curves: &[&Polyline2],
    min: DVec2,
    max: DVec2,
    tolerance: f64,
) -> anyhow::Result<Region> {
    use anyhow::{ensure, Context};
    validate_chains(curves, false)?;
    let size = max - min;
    let perimeter = 2. * (size.x + size.y);
    let boundary = |p: DVec2| -> Option<(DVec2, f64)> {
        if p.x < min.x - tolerance
            || p.x > max.x + tolerance
            || p.y < min.y - tolerance
            || p.y > max.y + tolerance
        {
            return None;
        }
        let p = p.clamp(min, max);
        if (p.y - min.y).abs() <= tolerance {
            Some((DVec2::new(p.x, min.y), p.x - min.x))
        } else if (p.x - max.x).abs() <= tolerance {
            Some((DVec2::new(max.x, p.y), size.x + p.y - min.y))
        } else if (p.y - max.y).abs() <= tolerance {
            Some((DVec2::new(p.x, max.y), size.x + size.y + max.x - p.x))
        } else if (p.x - min.x).abs() <= tolerance {
            Some((DVec2::new(min.x, p.y), 2. * size.x + size.y + max.y - p.y))
        } else {
            None
        }
    };
    let mut closed = Vec::new();
    for c in curves {
        if c.closed {
            closed.push((*c).clone());
            continue;
        }
        ensure!(
            c.pts
                .iter()
                .all(|p| p.cmpge(min - DVec2::splat(tolerance)).all()
                    && p.cmple(max + DVec2::splat(tolerance)).all()),
            "Open pocket wall lies outside stock"
        );
        let (start, s) =
            boundary(c.pts[0]).context("Open pocket wall start must lie on the stock perimeter")?;
        let (end, e) = boundary(*c.pts.last().unwrap())
            .context("Open pocket wall end must lie on the stock perimeter")?;
        ensure!(
            start.distance(end) > tolerance,
            "Open pocket wall endpoints must be distinct"
        );
        let mut pts = c.pts.clone();
        pts[0] = start;
        *pts.last_mut().unwrap() = end;
        let stop = if s > e { s } else { s + perimeter };
        for lap in [0., perimeter] {
            for (distance, corner) in [
                (0., min),
                (size.x, DVec2::new(max.x, min.y)),
                (size.x + size.y, max),
                (2. * size.x + size.y, DVec2::new(min.x, max.y)),
            ] {
                let distance = distance + lap;
                if distance > e + 1e-9 && distance < stop - 1e-9 {
                    pts.push(corner);
                }
            }
        }
        closed.push(Polyline2::new(pts, true));
    }
    validated(&closed.iter().collect::<Vec<_>>())
}

/// Entire segment coverage, including boundary crossings and narrow islands.
pub fn covers_segment(r: &Region, a: DVec2, b: DVec2) -> bool {
    use geo::{Intersects, Point};
    let length = a.distance(b);
    if length < 1e-9 {
        return r.intersects(&Point::new(a.x, a.y));
    }
    let clipped = clip_lines(r, &[(a, b)]);
    let covered: f64 = clipped.iter().map(|(a, b)| a.distance(*b)).sum();
    (covered - length).abs() <= 1e-8 * length.max(1.0)
}
