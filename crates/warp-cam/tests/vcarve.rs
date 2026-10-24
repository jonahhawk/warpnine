//! Medial-axis V-carve: the bit stays tangent to the walls, rising in narrow places and
//! coming up to the surface at sharp corners, and never cuts past the boundary.

use glam::{DVec2, DVec3};
use warp_cam::ops::{Common, OpKind, Operation};
use warp_cam::{generate, Project};
use warp_core::Polyline2;
use warp_tools::ToolLibrary;

/// Points every 0.05 mm along the feed moves, at or below the surface, for a V-carve of `shape`
/// with T5 (90° V-bit, Ø12.7), so the allowed depth at a point equals its distance to the wall.
fn carve(shape: Polyline2, max_depth: f64) -> Vec<DVec3> {
    let tools = ToolLibrary::starter();
    let mut p = Project::default();
    p.curves.push(shape);
    let mut c = Common::for_tool(tools.tool(5).unwrap());
    c.curves = vec![0];
    c.heights.bottom = -max_depth;
    let op = Operation {
        name: "v".into(),
        enabled: true,
        common: c,
        kind: OpKind::VCarve {
            max_depth,
            step: 0.1,
        },
    };
    let tp = generate(&op, &p, &tools).unwrap();
    tp.segments(DVec3::new(0., 0., 10.), 0.01)
        .into_iter()
        .filter(|s| !s.2)
        .flat_map(|s| {
            let n = (s.0.distance(s.1) / 0.05).ceil().max(1.0) as usize;
            (0..=n).map(move |k| s.0.lerp(s.1, k as f64 / n as f64))
        })
        .filter(|p| p.z < 1e-9)
        .collect()
}

fn wall_dist(poly: &Polyline2, p: DVec2) -> f64 {
    let n = poly.pts.len();
    (0..n)
        .map(|i| {
            let (a, b) = (poly.pts[i], poly.pts[(i + 1) % n]);
            let t = ((p - a).dot(b - a) / (b - a).length_squared()).clamp(0.0, 1.0);
            p.distance(a + (b - a) * t)
        })
        .fold(f64::INFINITY, f64::min)
}

fn assert_no_gouge(poly: &Polyline2, cuts: &[DVec3]) {
    for p in cuts {
        let allowed = wall_dist(poly, p.truncate());
        assert!(
            -p.z <= allowed + 0.01,
            "gouge at {p}: depth {} > {allowed}",
            -p.z
        );
    }
}

#[test]
fn wedge_rises_to_its_point() {
    // Tip at the origin, 8 mm wide at x = 40.
    let wedge = Polyline2::new(
        vec![
            DVec2::new(0., 0.),
            DVec2::new(40., -4.),
            DVec2::new(40., 4.),
        ],
        true,
    );
    let cuts = carve(wedge.clone(), 5.0);
    assert_no_gouge(&wedge, &cuts);
    // Reaches right into the point, coming up to the surface.
    let tip = cuts
        .iter()
        .map(|p| p.truncate().length())
        .fold(f64::INFINITY, f64::min);
    assert!(tip < 0.15, "closest cut to the tip is {tip} mm away");
    // Along the centreline the depth follows the half-width (tangent to both sides).
    for x in [5.0, 10.0, 20.0, 30.0] {
        let ideal = wall_dist(&wedge, DVec2::new(x, 0.0));
        let z = cuts
            .iter()
            .filter(|p| (p.x - x).abs() < 0.2 && p.y.abs() < 0.05)
            .map(|p| p.z)
            .fold(0.0, f64::min);
        assert!((-z - ideal).abs() < 0.03, "x={x}: depth {} vs {ideal}", -z);
    }
}

#[test]
fn slot_narrower_than_bit_is_cut_on_its_centreline() {
    // 2 mm slot, 12.7 mm bit: one pass down the middle at 1 mm deep, rising into the corners.
    let slot = Polyline2::rect(DVec2::new(0., 0.), DVec2::new(30., 2.));
    let cuts = carve(slot.clone(), 5.0);
    assert_no_gouge(&slot, &cuts);
    let deepest = cuts.iter().map(|p| p.z).fold(0.0, f64::min);
    assert!((deepest + 1.0).abs() < 0.01, "deepest {deepest}");
    assert!(
        cuts.iter()
            .filter(|p| p.z < -0.99)
            .all(|p| (p.y - 1.0).abs() < 0.01),
        "full-depth cuts off centre"
    );
    for corner in [DVec2::new(0., 0.), DVec2::new(30., 2.)] {
        let d = cuts
            .iter()
            .map(|p| p.truncate().distance(corner))
            .fold(f64::INFINITY, f64::min);
        assert!(d < 0.15, "corner {corner} not reached ({d})");
    }
}

#[test]
fn wide_area_bottoms_out_and_lifts_into_corners() {
    let sq = Polyline2::rect(DVec2::new(0., 0.), DVec2::new(30., 30.));
    let cuts = carve(sq.clone(), 2.0);
    assert_no_gouge(&sq, &cuts);
    let deepest = cuts.iter().map(|p| p.z).fold(0.0, f64::min);
    assert!((deepest + 2.0).abs() < 1e-6, "deepest {deepest}");
    // Outline of the flat floor, 2 mm in from the walls.
    assert!(cuts
        .iter()
        .any(|p| (p.x - 2.0).abs() < 1e-3 && (p.y - 15.0).abs() < 1.0 && (p.z + 2.0).abs() < 1e-6));
    let d = cuts
        .iter()
        .map(|p| p.truncate().distance(DVec2::new(30., 30.)))
        .fold(f64::INFINITY, f64::min);
    assert!(d < 0.15, "corner not reached ({d})");
}

#[test]
fn ring_with_hole_follows_its_centre_circle() {
    let tools = ToolLibrary::starter();
    let mut p = Project::default();
    p.curves.push(Polyline2::circle(DVec2::ZERO, 10.0, 0.005));
    p.curves.push(Polyline2::circle(DVec2::ZERO, 6.0, 0.005));
    let mut c = Common::for_tool(tools.tool(5).unwrap());
    c.curves = vec![0, 1];
    c.heights.bottom = -5.0;
    let op = Operation {
        name: "v".into(),
        enabled: true,
        common: c,
        kind: OpKind::VCarve {
            max_depth: 5.0,
            step: 0.1,
        },
    };
    let tp = generate(&op, &p, &tools).unwrap();
    let mut reached_final = false;
    for s in tp
        .segments(DVec3::new(0., 0., 10.), 0.01)
        .into_iter()
        .filter(|s| !s.2 && s.1.z < 1e-9)
    {
        let r = s.1.truncate().length();
        // 4 mm wide band: on the r = 8 circle, 2 mm deep, never deeper than the walls allow.
        let allowed = (r - 6.0).min(10.0 - r);
        assert!(-s.1.z <= allowed + 0.01, "gouge at {}", s.1);
        reached_final |= (s.1.z + 2.).abs() < 0.02;
        assert!(
            (r - 8.0).abs() < 0.05 && s.1.z >= -2.02,
            "off the centre circle: {}",
            s.1
        );
    }
    assert!(
        reached_final,
        "Final pass must reach the analytic two-mm depth"
    );
}
