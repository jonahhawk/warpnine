//! M0 characterization. Known failures are explicit expectations, not ignored tests.
use glam::{DVec2, DVec3};
use std::path::PathBuf;
use warp_cam::toolpath::Move;
use warp_cam::{generate, Project};
use warp_core::{Mesh, Units};
use warp_tools::ToolLibrary;

fn fixtures() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../fixtures/cam-baseline")
}

fn load(name: &str) -> Project {
    Project::load(&fixtures().join(format!("{name}.warp"))).unwrap()
}

fn tools() -> ToolLibrary {
    ToolLibrary::load(&fixtures().join("tools.json")).unwrap()
}

fn same_json_intent(a: &serde_json::Value, b: &serde_json::Value) {
    use serde_json::Value;
    match (a, b) {
        (Value::Number(x), Value::Number(y)) => {
            let (x, y) = (x.as_f64().unwrap(), y.as_f64().unwrap());
            assert!((x - y).abs() <= 1e-12 * x.abs().max(1.0), "{x} != {y}");
        }
        (Value::Array(x), Value::Array(y)) => {
            assert_eq!(x.len(), y.len());
            for (x, y) in x.iter().zip(y) {
                same_json_intent(x, y);
            }
        }
        (Value::Object(x), Value::Object(y)) => {
            // Version 1 adds identity and role metadata; all original fields
            // must retain their numeric/geometric meaning after migration.
            assert!(x.len() <= y.len());
            for (key, value) in x {
                same_json_intent(value, y.get(key).unwrap());
            }
        }
        _ => assert_eq!(a, b),
    }
}

#[test]
fn synthetic_dxf_and_stl_have_expected_geometry() {
    let dxf = warp_core::import::load_dxf(&fixtures().join("planar-mm.dxf"), 0.01, Units::Inches)
        .unwrap();
    assert!(dxf.skipped.is_empty(), "{:?}", dxf.skipped);
    assert_eq!(dxf.curves.len(), 7);
    assert_eq!(dxf.curves.iter().filter(|c| !c.closed).count(), 1);
    let open = dxf.curves.iter().find(|c| !c.closed).unwrap();
    assert!(
        (open.length() - 25.0).abs() < 1e-6,
        "declared mm must override fallback inches"
    );
    let mesh = Mesh::load_stl(&fixtures().join("sculpted-mm.stl")).unwrap();
    assert_eq!(mesh.tris.len(), 600);
    let bbox = mesh.bbox();
    assert!((bbox.min - DVec3::new(0.0, 0.0, -1.25)).length() < 1e-6);
    assert!((bbox.max - DVec3::new(40.0, 30.0, -0.002)).length() < 1e-6);
    for triangle in &mesh.tris {
        for p in triangle {
            let z = -0.002 * ((p.x - 20.0).powi(2) + (p.y - 15.0).powi(2));
            assert!((p.z - z).abs() < 1e-6);
        }
    }
}

#[test]
fn legacy_fixtures_round_trip_without_changing_intent() {
    for name in [
        "pocket-island",
        "open-profile",
        "drill-pattern",
        "sculpted-parallel",
        "projected-open-curve",
    ] {
        let original: serde_json::Value = serde_json::from_str(
            &std::fs::read_to_string(fixtures().join(format!("{name}.warp"))).unwrap(),
        )
        .unwrap();
        assert!(original.get("schema_version").is_none());
        let project = load(name);
        let saved =
            std::env::temp_dir().join(format!("warpnine-m0-{}-{name}.warp", std::process::id()));
        project.save(&saved).unwrap();
        let reloaded = Project::load(&saved).unwrap();
        std::fs::remove_file(saved).unwrap();
        // JSON 2 and 2.0 have the same geometric meaning; decimal/f64 round trips
        // can also differ by a final binary digit, far below machining tolerance.
        same_json_intent(&original, &serde_json::to_value(reloaded).unwrap());
    }
}

#[test]
fn open_profile_follows_analytic_l_at_both_depths() {
    let p = load("open-profile");
    let path = generate(&p.operations[0], &p, &tools()).unwrap();
    let segments = path.segments(DVec3::new(0.0, 0.0, 10.0), 0.01);
    for depth in [-1.0, -2.0] {
        let horizontal_length: f64 = segments
            .iter()
            .filter(|(a, b, rapid, _)| {
                !rapid && (a.z - depth).abs() < 1e-8 && (b.z - depth).abs() < 1e-8
            })
            .map(|(a, b, _, _)| a.distance(*b))
            .sum();
        assert!(
            (horizontal_length - 25.0).abs() < 1e-6,
            "depth {depth}: {horizontal_length}"
        );
    }
    for (a, b, rapid, _) in segments {
        if rapid || b.z >= 0.0 {
            continue;
        }
        for i in 0..=20 {
            let q = a.lerp(b, i as f64 / 20.0);
            let on_horizontal =
                (q.y - 5.0).abs() < 1e-6 && (5.0 - 1e-6..=20.0 + 1e-6).contains(&q.x);
            let on_vertical =
                (q.x - 20.0).abs() < 1e-6 && (5.0 - 1e-6..=15.0 + 1e-6).contains(&q.y);
            assert!(on_horizontal || on_vertical, "off analytic L: {q}");
        }
    }
}

#[test]
fn drill_pattern_preserves_hole_centers_and_depth() {
    let p = load("drill-pattern");
    let path = generate(&p.operations[0], &p, &tools()).unwrap();
    let holes: Vec<_> = path
        .moves
        .iter()
        .filter_map(|m| match m {
            Move::Drill { at, bottom, .. } => Some((*at, *bottom)),
            _ => None,
        })
        .collect();
    assert_eq!(holes.len(), 4);
    for expected in [
        DVec2::new(8.0, 8.0),
        DVec2::new(32.0, 8.0),
        DVec2::new(8.0, 22.0),
        DVec2::new(32.0, 22.0),
    ] {
        assert_eq!(
            holes
                .iter()
                .filter(|(at, z)| at.distance(expected) < 1e-6 && (*z + 2.0).abs() < 1e-8)
                .count(),
            1
        );
    }
}

#[test]
fn pocket_cut_samples_respect_outer_wall_and_island() {
    let p = load("pocket-island");
    let tools = tools();
    let radius = tools.tool(1).unwrap().radius();
    let path = generate(&p.operations[0], &p, &tools).unwrap();
    let mut samples = 0;
    for (a, b, rapid, _) in path.segments(DVec3::new(0.0, 0.0, 10.0), 0.01) {
        if rapid {
            continue;
        }
        let n = (a.distance(b) / 0.05).ceil().max(1.0) as usize;
        for i in 0..=n {
            let q = a.lerp(b, i as f64 / n as f64);
            if q.z >= -1e-6 {
                continue;
            }
            samples += 1;
            assert!(
                q.x >= 2.0 + radius - 0.02
                    && q.x <= 38.0 - radius + 0.02
                    && q.y >= 2.0 + radius - 0.02
                    && q.y <= 28.0 - radius + 0.02,
                "outer wall: {q}"
            );
            // Euclidean distance to the analytic island rectangle; valid at rounded corners too.
            let nearest = q
                .truncate()
                .clamp(DVec2::new(15.0, 10.0), DVec2::new(25.0, 20.0));
            assert!(
                q.truncate().distance(nearest) >= radius - 0.02,
                "island: {q}"
            );
        }
    }
    assert!(samples > 100);
}

#[test]
fn known_m0_projected_open_curve_is_rejected_as_containment() {
    let p = load("projected-open-curve");
    let error = generate(&p.operations[0], &p, &tools()).unwrap_err();
    assert!(format!("{error:#}").contains("boundary curves must be closed"));
    // M1/G08 must replace this characterization with a successful path assertion.
}

#[test]
fn legacy_empty_and_missing_index_selection_are_characterized() {
    let p = load("pocket-island");
    assert_eq!(p.select_curves(&[]).len(), 2);
    assert_eq!(p.select_curves(&[1, 999]).len(), 1);
    // M1 must preserve empty-selection intent, but make unresolved refs repairable.
}
