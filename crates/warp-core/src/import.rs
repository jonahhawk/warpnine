//! DXF import: flattens supported entities into 2D polylines.

use crate::geom::{bulge_points, chain, tessellate_arc, Polyline2};
use dxf::entities::EntityType;
use glam::DVec2;
use std::path::Path;

pub struct DxfImport {
    pub curves: Vec<Polyline2>,
    pub layers: Vec<String>,
    pub source_units: String,
    /// Entity types that were skipped, with counts, so the UI can warn about them.
    pub skipped: Vec<(String, usize)>,
}

/// Load a DXF file. Arcs, circles, bulges and splines are tessellated to within `tol` (mm).
/// `unitless` is the units assumed when the file does not declare any.
pub fn load_dxf(path: &Path, tol: f64, unitless: crate::Units) -> anyhow::Result<DxfImport> {
    let drawing = dxf::Drawing::load_file(path)?;
    let source_units = format!(
        "{:?} (unitless fallback: {:?})",
        drawing.header.default_drawing_units, unitless
    );
    let scale = match drawing.header.default_drawing_units {
        dxf::enums::Units::Inches => 25.4,
        dxf::enums::Units::Feet => 304.8,
        dxf::enums::Units::Centimeters => 10.0,
        dxf::enums::Units::Meters => 1000.0,
        dxf::enums::Units::Millimeters => 1.0,
        _ => unitless.to_mm(),
    };
    let p2 = |x: f64, y: f64| DVec2::new(x, y) * scale;
    let mut curves = Vec::new();
    let mut layers = Vec::new();
    let mut skipped: std::collections::BTreeMap<String, usize> = Default::default();

    for e in drawing.entities() {
        let before = curves.len();
        match &e.specific {
            EntityType::Line(l) => {
                curves.push(Polyline2::new(
                    vec![p2(l.p1.x, l.p1.y), p2(l.p2.x, l.p2.y)],
                    false,
                ));
            }
            EntityType::Circle(c) => {
                curves.push(Polyline2::circle(
                    p2(c.center.x, c.center.y),
                    c.radius * scale,
                    tol,
                ));
            }
            EntityType::Arc(a) => {
                let pts = tessellate_arc(
                    p2(a.center.x, a.center.y),
                    a.radius * scale,
                    a.start_angle.to_radians(),
                    a.end_angle.to_radians(),
                    true,
                    tol,
                );
                curves.push(Polyline2::new(pts, false));
            }
            EntityType::LwPolyline(pl) => {
                let v = &pl.vertices;
                if v.is_empty() {
                    continue;
                }
                let closed = pl.is_closed();
                let mut pts = vec![p2(v[0].x, v[0].y)];
                let n = v.len();
                let count = if closed { n } else { n - 1 };
                for i in 0..count {
                    let a = p2(v[i].x, v[i].y);
                    let b = p2(v[(i + 1) % n].x, v[(i + 1) % n].y);
                    pts.extend(bulge_points(a, b, v[i].bulge, tol));
                }
                curves.push(Polyline2::new(pts, closed));
            }
            EntityType::Polyline(pl) => {
                let v: Vec<_> = pl.vertices().collect();
                if v.is_empty() {
                    continue;
                }
                let closed = pl.is_closed();
                let mut pts = vec![p2(v[0].location.x, v[0].location.y)];
                let n = v.len();
                let count = if closed { n } else { n - 1 };
                for i in 0..count {
                    let a = p2(v[i].location.x, v[i].location.y);
                    let j = (i + 1) % n;
                    let b = p2(v[j].location.x, v[j].location.y);
                    pts.extend(bulge_points(a, b, v[i].bulge, tol));
                }
                curves.push(Polyline2::new(pts, closed));
            }
            EntityType::Spline(s) => {
                // Control points + knots define the exact curve; fit-point-only splines are
                // interpolated through their fit points.
                let ctrl: Vec<DVec2> = s.control_points.iter().map(|p| p2(p.x, p.y)).collect();
                let pts = if ctrl.len() >= 2 {
                    crate::nurbs::Nurbs::new(
                        s.degree_of_curve.max(1) as usize,
                        s.knot_values.clone(),
                        ctrl,
                        s.weight_values.clone(),
                    )
                    .map(|n| n.tessellate(tol))
                    .unwrap_or_default()
                } else {
                    let fit: Vec<DVec2> = s.fit_points.iter().map(|p| p2(p.x, p.y)).collect();
                    crate::nurbs::through_points(&fit, s.is_closed(), tol)
                };
                if pts.len() >= 2 {
                    let closed = s.is_closed()
                        || s.is_periodic()
                        || pts[0].distance(*pts.last().unwrap()) < 1e-6;
                    curves.push(Polyline2::new(pts, closed));
                }
            }
            EntityType::Ellipse(el) => {
                let c = p2(el.center.x, el.center.y);
                let major = DVec2::new(el.major_axis.x, el.major_axis.y) * scale;
                let minor = major.perp() * el.minor_axis_ratio;
                let (mut a0, a1) = (el.start_parameter, el.end_parameter);
                if a1 <= a0 {
                    a0 -= std::f64::consts::TAU;
                }
                let n = crate::geom::arc_segments(major.length(), a1 - a0, tol).max(16);
                let pts: Vec<_> = (0..=n)
                    .map(|i| {
                        let t = a0 + (a1 - a0) * i as f64 / n as f64;
                        c + major * t.cos() + minor * t.sin()
                    })
                    .collect();
                let closed = (a1 - a0 - std::f64::consts::TAU).abs() < 1e-6;
                curves.push(Polyline2::new(pts, closed));
            }
            other => {
                let name = format!("{other:?}");
                let name = name
                    .split(['(', ' ', '{'])
                    .next()
                    .unwrap_or("?")
                    .to_string();
                *skipped.entry(name).or_default() += 1;
            }
        }
        layers.extend(std::iter::repeat_n(
            e.common.layer.clone(),
            curves.len() - before,
        ));
    }

    let mut by_layer: std::collections::BTreeMap<String, Vec<Polyline2>> = Default::default();
    for (curve, layer) in curves.into_iter().zip(layers) {
        if curve.pts.len() >= 2 {
            by_layer.entry(layer).or_default().push(curve);
        }
    }
    let mut curves = vec![];
    let mut layers = vec![];
    for (layer, input) in by_layer {
        let chained = chain(input, 1e-4 * scale.max(1.0));
        layers.extend(std::iter::repeat_n(layer, chained.len()));
        curves.extend(chained);
    }
    Ok(DxfImport {
        curves,
        layers,
        source_units,
        skipped: skipped.into_iter().collect(),
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use dxf::entities::{Entity, Spline};
    use dxf::Point;

    #[test]
    fn spline_round_trip_through_dxf() {
        let mut d = dxf::Drawing::new();
        d.header.version = dxf::enums::AcadVersion::R2000;
        d.header.default_drawing_units = dxf::enums::Units::Millimeters;
        let mut s = Spline::default();
        s.degree_of_curve = 3;
        s.control_points = vec![
            Point::new(0., 0., 0.),
            Point::new(10., 30., 0.),
            Point::new(20., -30., 0.),
            Point::new(30., 0., 0.),
        ];
        s.knot_values = vec![0., 0., 0., 0., 1., 1., 1., 1.];
        d.add_entity(Entity::new(EntityType::Spline(s)));
        let path = std::env::temp_dir().join(format!("warp_spline_{}.dxf", std::process::id()));
        d.save_file(&path).unwrap();
        let imp = load_dxf(&path, 0.01, crate::Units::Millimeters).unwrap();
        let _ = std::fs::remove_file(&path);
        assert_eq!(imp.curves.len(), 1);
        let c = &imp.curves[0];
        assert!(!c.closed);
        assert!(
            c.pts.len() > 20,
            "spline should be tessellated, got {} points",
            c.pts.len()
        );
        assert!(c.pts[0].distance(DVec2::ZERO) < 1e-6);
        assert!(c.pts.last().unwrap().distance(DVec2::new(30., 0.)) < 1e-6);
        // Bezier midpoint: (0 + 3*10 + 3*20 + 30)/8 = 15, (0 + 90 - 90 + 0)/8 = 0
        assert!(c.pts.iter().any(|p| p.distance(DVec2::new(15., 0.)) < 0.05));
    }
}
