use std::{
    path::PathBuf,
    sync::atomic::{AtomicU64, Ordering},
};
use warp_core::{interchange::*, scene::*, selection::*};
use warp_import_step::{write_acceptance_fixture, StepImporter};
static NEXT: AtomicU64 = AtomicU64::new(0);
struct Fixture(PathBuf);
impl Fixture {
    fn new(variant: i32) -> Self {
        let p = std::env::temp_dir().join(format!(
            "warp-step-{}-{}.step",
            std::process::id(),
            NEXT.fetch_add(1, Ordering::Relaxed)
        ));
        write_acceptance_fixture(&p, variant).unwrap();
        Self(p)
    }
    fn read(&self) -> ImportResult {
        self.read_at(0.01)
    }
    fn read_at(&self, tolerance_mm: f64) -> ImportResult {
        StepImporter
            .read(
                &self.0,
                &ImportOptions {
                    tolerance_mm,
                    ..Default::default()
                },
            )
            .unwrap()
    }
}
impl Drop for Fixture {
    fn drop(&mut self) {
        let _ = std::fs::remove_file(&self.0);
    }
}
fn rhino_fixture(name: &str) -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../../fixtures/rhino-m5")
        .join(name)
}
fn meshes(scene: &Scene) -> Vec<(&str, &warp_core::Mesh)> {
    scene
        .objects
        .iter()
        .filter_map(|o| match &o.geometry {
            Geometry::Mesh(m) => Some((o.name.as_str(), m)),
            _ => None,
        })
        .collect()
}
fn solids(scene: &Scene) -> Vec<(&str, &str, Option<&ObjectId>)> {
    scene
        .objects
        .iter()
        .filter_map(|o| match &o.geometry {
            Geometry::Unavailable {
                reason,
                represented_by,
            } => Some((o.name.as_str(), reason.as_str(), represented_by.as_ref())),
            _ => None,
        })
        .collect()
}
fn mesh_for<'a>(scene: &'a Scene, solid: &str) -> &'a warp_core::Mesh {
    let (_, reason, by) = solids(scene)
        .into_iter()
        .find(|(n, _, _)| *n == solid)
        .unwrap_or_else(|| panic!("no solid {solid}"));
    let by = by.unwrap_or_else(|| panic!("{solid} is not represented: {reason}"));
    match &scene.objects.iter().find(|o| &o.id == by).unwrap().geometry {
        Geometry::Mesh(m) => m,
        other => panic!("{solid} points at {other:?}"),
    }
}
fn sculpted_project(scene: &Scene) -> warp_cam::Project {
    let root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../fixtures/cam-baseline");
    let mut p = warp_cam::Project::load(&root.join("sculpted-parallel.warp")).unwrap();
    p.scene = Some(scene.clone());
    p.refresh_view().unwrap();
    p
}

#[test]
fn synthetic_solids_and_surface_are_meshed_and_qualified() {
    let imported = Fixture::new(0).read();
    let scene = &imported.scene;
    assert!(imported.summary().starts_with("STEP AP214:"), "{}", imported.summary());
    assert_eq!((imported.unit_scale_mm, imported.source_units.as_str()), (1.0, "MILLIMETRE"));
    let codes: Vec<&str> = imported.diagnostics.iter().map(|d| d.code.as_str()).collect();
    assert!(
        !codes.iter().any(|c| ["mesh_unqualified", "mesh_topology", "unavailable_geometry"].contains(c)),
        "{:?}",
        imported.diagnostics
    );
    assert_eq!(codes.iter().filter(|c| **c == "solid_represented").count(), 3);
    let solids = solids(scene);
    assert_eq!(solids.len(), 3, "{solids:?}");
    let mut names: Vec<&str> = solids.iter().map(|s| s.0).collect();
    names.sort();
    names.dedup();
    assert_eq!(names.len(), 3, "names must be distinct: {names:?}");
    for (name, reason, by) in &solids {
        assert!(reason.contains("Represented by qualified machining mesh"), "{name}: {reason}");
        assert!(by.is_some());
    }
    let slack = imported.source_tolerance_mm;
    assert!(slack < 1e-3, "STEP primitives carry small tolerances: {slack}");

    // Radius-10 sphere at (10,10,10): closed, outward, volume of an inscribed polyhedron at
    // 0.01 mm deviation is within 0.5% of 4/3 pi r^3.
    let mut found = [false; 3];
    for (name, mesh) in meshes(scene) {
        let p = mesh.provenance.as_ref().unwrap_or_else(|| panic!("{name} has no provenance"));
        let m = p.measured.as_ref().unwrap();
        assert_eq!(p.requested_tolerance_mm, Some(0.01));
        assert_eq!(p.exporter.as_deref(), Some("warpnine-occt/7.9.3"));
        assert!(p.deviation_mm().unwrap() <= 0.01 + slack, "{name}: {m:?}");
        assert_eq!(m.outside_trim_samples, 0, "{name}: {m:?}");
        assert!(m.source_samples > 500 && m.mesh_samples > 100, "{name}: {m:?}");
        assert_eq!(p.face_ranges.len(), m.per_face_mm.len());
        assert_eq!(p.face_ranges.last().unwrap().end, mesh.tris.len());
        let q = mesh.analyze(1e-5);
        let b = mesh.bbox();
        if (b.min.x - 0.0).abs() < 0.02 && (b.max.x - 20.0).abs() < 0.02 && b.max.y < 21.0 {
            assert_eq!(p.source_type, "brep");
            assert!(q.closed() && q.oriented(), "{name}: {}", q.describe());
            let v = mesh.signed_volume();
            let exact = 4.0 / 3.0 * std::f64::consts::PI * 1000.0;
            assert!((v - exact).abs() < exact * 0.005, "sphere volume {v} vs {exact}");
            assert!(m.source_to_mesh_mm > 0.001, "a curved solid cannot be met exactly: {m:?}");
            found[0] = true;
        } else if (b.min.x - 40.0).abs() < 0.02 && (b.max.x - 60.0).abs() < 0.02 {
            // 20 x 30 x 15 box minus a radius-4 through hole.
            assert_eq!(p.source_type, "brep");
            assert!(q.closed() && q.oriented(), "{name}: {}", q.describe());
            assert_eq!(p.face_ranges.len(), 7);
            let v = mesh.signed_volume();
            let exact = 9000.0 - std::f64::consts::PI * 16.0 * 15.0;
            assert!((v - exact).abs() < exact * 0.005, "box volume {v} vs {exact}");
            found[1] = true;
        } else if (b.min.x - 70.0).abs() < 0.02 && (b.max.x - 130.0).abs() < 0.02 {
            // Open wavy face interpolating 6x5 points with z = 3 sin(3 pi s) cos(2 pi t):
            // boundary edges, corners at z = 0, sampled peaks of +-2.853 reached and the
            // interpolation overshoot between knots bounded.
            assert_eq!(p.source_type, "surface");
            assert!(!q.closed() && q.boundary_edges > 0, "{name}: {}", q.describe());
            assert!(q.nonmanifold_edges == 0 && q.inconsistent_edges == 0);
            assert!(b.max.z > 2.85 && b.min.z < -2.85, "{b:?}");
            assert!(b.max.z < 4.5 && b.min.z > -4.5, "{b:?}");
            let corner = mesh
                .tris
                .iter()
                .flat_map(|t| t.iter().copied())
                .min_by(|a, c| {
                    let d = |v: &glam::DVec3| (v.x - 70.0).powi(2) + v.y.powi(2);
                    d(a).partial_cmp(&d(c)).unwrap()
                })
                .unwrap();
            assert!(corner.z.abs() < 0.02 && (corner.x - 70.0).abs() < 1e-6 && corner.y.abs() < 1e-6, "{corner:?}");
            assert_eq!(p.face_ranges.len(), 1);
            found[2] = true;
        } else {
            panic!("unexpected mesh {name} with bounds {b:?}");
        }
    }
    assert_eq!(found, [true; 3]);
}

#[test]
fn inch_files_scale_to_millimetres() {
    let imported = Fixture::new(1).read();
    assert_eq!(imported.unit_scale_mm, 25.4);
    assert!(imported.source_units.to_uppercase().contains("INCH"), "{}", imported.source_units);
    let ms = meshes(&imported.scene);
    assert_eq!(ms.len(), 1);
    let b = ms[0].1.bbox();
    let size = b.max - b.min;
    assert!((size.x - 25.4).abs() < 1e-6 && (size.y - 50.8).abs() < 1e-6 && (size.z - 12.7).abs() < 1e-6, "{size:?}");
    assert_eq!(ms[0].1.tris.len(), 12);
    let v = ms[0].1.signed_volume();
    assert!((v - 25.4 * 50.8 * 12.7).abs() < 1e-6, "{v}");
    let src = imported.scene.objects[0].source.as_ref().unwrap();
    assert_eq!(src.original_units.as_deref(), Some(imported.source_units.as_str()));
    assert_eq!(src.approximation_tolerance_mm, Some(0.01));
}

#[test]
fn import_tolerance_controls_density_and_cam_machines_the_solid() {
    let fixture = Fixture::new(0);
    let fine = fixture.read_at(0.01);
    let coarse = fixture.read_at(0.2);
    let sphere_fine = meshes(&fine.scene)
        .into_iter()
        .find(|(_, m)| m.bbox().max.x < 21.0)
        .unwrap()
        .1;
    let sphere_coarse = meshes(&coarse.scene)
        .into_iter()
        .find(|(_, m)| m.bbox().max.x < 21.0)
        .unwrap()
        .1;
    assert!(sphere_coarse.tris.len() * 4 < sphere_fine.tris.len(), "{} vs {}", sphere_coarse.tris.len(), sphere_fine.tris.len());
    let dc = sphere_coarse.provenance.as_ref().unwrap().deviation_mm().unwrap();
    assert!(dc > 0.01 && dc <= 0.2, "coarse deviation {dc}");
    assert!(sphere_coarse.analyze(1e-5).closed());

    // Selecting the STEP solid as a part surface machines its mesh; a tolerance below the
    // measured deviation refuses generation; AllEligible includes every represented solid.
    let scene = &fine.scene;
    let mut p = sculpted_project(scene);
    let (sphere_name, _, _) = solids(scene)
        .into_iter()
        .find(|(n, _, _)| mesh_for(scene, n).bbox().max.x < 21.0)
        .unwrap();
    let solid = scene
        .resolved()
        .unwrap()
        .into_iter()
        .find(|o| o.name == sphere_name)
        .unwrap();
    let control = p.operations[0].common.geometry.as_mut().unwrap();
    control.part_surfaces = Selection {
        mode: SelectionMode::Selected,
        references: vec![solid.reference],
    };
    p.operations[0].common.tolerance = 0.02;
    let (resolved, _) = p.resolve_operation(&p.operations[0]).unwrap();
    assert_eq!(resolved.mesh.as_ref().unwrap().tris.len(), sphere_fine.tris.len());
    p.operations[0].common.tolerance = 0.0005;
    let err = p.resolve_operation(&p.operations[0]).unwrap_err().to_string();
    assert!(err.contains("exceeds the operation tolerance"), "{err}");
    p.operations[0].common.tolerance = 0.02;
    p.operations[0].common.geometry.as_mut().unwrap().part_surfaces = Selection::all();
    let (resolved, _) = p.resolve_operation(&p.operations[0]).unwrap();
    let total: usize = meshes(scene).iter().map(|(_, m)| m.tris.len()).sum();
    assert_eq!(resolved.mesh.unwrap().tris.len(), total);
}

#[test]
fn cancellation_memory_limit_and_bad_files_fail_cleanly() {
    let fixture = Fixture::new(0);
    let cancel = std::sync::Arc::new(std::sync::atomic::AtomicBool::new(true));
    let err = StepImporter
        .read(
            &fixture.0,
            &ImportOptions {
                cancel: Some(cancel),
                ..Default::default()
            },
        )
        .unwrap_err()
        .to_string();
    assert!(err.to_lowercase().contains("cancelled"), "{err}");
    let err = StepImporter
        .read(
            &fixture.0,
            &ImportOptions {
                max_triangles: 100,
                ..Default::default()
            },
        )
        .unwrap_err()
        .to_string();
    assert!(err.contains("memory limit"), "{err}");
    let bogus = std::env::temp_dir().join(format!("warp-step-bogus-{}.step", std::process::id()));
    std::fs::write(&bogus, "this is not a STEP file").unwrap();
    let err = StepImporter.read(&bogus, &ImportOptions::default()).unwrap_err().to_string();
    let _ = std::fs::remove_file(&bogus);
    assert!(err.contains("Not a readable STEP file"), "{err}");
    let err = StepImporter
        .read(&fixture.0, &ImportOptions { tolerance_mm: 0.0, ..Default::default() })
        .unwrap_err()
        .to_string();
    assert!(err.contains("Import tolerance"), "{err}");
}

/// The Rhino-exported STEP of the M5 sculpted solid: WarpNine's own mesh must agree with the
/// Rhino-exported, independently qualified machining mesh from the 3dm fixture.
#[test]
fn rhino_exported_step_matches_the_qualified_3dm_mesh() {
    use warp_import_3dm::OpenNurbsImporter;
    let step = StepImporter
        .read(&rhino_fixture("sculpted-v8.step"), &ImportOptions::default())
        .unwrap();
    assert!(step.summary().starts_with("STEP AP203:"), "{}", step.summary());
    let codes: Vec<&str> = step.diagnostics.iter().map(|d| d.code.as_str()).collect();
    assert_eq!(codes, vec!["solid_represented"], "{:?}", step.diagnostics);
    let names: Vec<&str> = solids(&step.scene).iter().map(|s| s.0).collect();
    assert_eq!(names, vec!["Sculpted solid"], "object names come from the STEP file");
    let mesh = mesh_for(&step.scene, "Sculpted solid");
    let p = mesh.provenance.as_ref().unwrap();
    let m = p.measured.as_ref().unwrap();
    assert!(p.deviation_mm().unwrap() <= 0.01 + step.source_tolerance_mm, "{m:?}");
    // Boundary vertices sit on the trims; a handful may fall just outside the band.
    assert!(m.outside_trim_samples * 100 < m.mesh_samples, "{m:?}");
    let q = mesh.analyze(1e-5);
    assert!(q.closed() && q.oriented(), "{}", q.describe());

    let rhino = OpenNurbsImporter
        .read(&rhino_fixture("sculpted-v8.3dm"), &ImportOptions::default())
        .unwrap();
    let rhino_mesh = meshes(&rhino.scene)
        .into_iter()
        .find(|(n, _)| n.starts_with("Sculpted solid"))
        .unwrap()
        .1;
    let (v_step, v_rhino) = (mesh.signed_volume(), rhino_mesh.signed_volume());
    assert!(
        (v_step - v_rhino).abs() < v_rhino * 0.001,
        "volumes differ: STEP {v_step} vs Rhino {v_rhino}"
    );
    let (bs, br) = (mesh.bbox(), rhino_mesh.bbox());
    assert!((bs.min - br.min).abs().max_element() < 0.02 && (bs.max - br.max).abs().max_element() < 0.02, "{bs:?} vs {br:?}");
    println!(
        "step fixture: {} tris | {} | {:?}",
        mesh.tris.len(),
        p.summary(),
        m
    );

    // The top face alone: an open surface that can be selected for 3D milling.
    let top = StepImporter
        .read(&rhino_fixture("sculpted-top-surface.step"), &ImportOptions::default())
        .unwrap();
    let names: Vec<&str> = solids(&top.scene).iter().map(|s| s.0).collect();
    assert_eq!(names, vec!["Sculpted top"]);
    let surface = mesh_for(&top.scene, "Sculpted top");
    let sp = surface.provenance.as_ref().unwrap();
    assert_eq!(sp.source_type, "surface");
    assert!(sp.deviation_mm().unwrap() <= 0.01 + top.source_tolerance_mm, "{:?}", sp.measured);
    let sq = surface.analyze(1e-5);
    assert!(!sq.closed() && sq.boundary_edges > 0 && sq.nonmanifold_edges == 0, "{}", sq.describe());
    let sb = surface.bbox();
    assert!((sb.max.z - bs.max.z).abs() < 0.02, "top of the surface matches the solid: {sb:?}");
    assert!(sb.min.z > bs.min.z + 10.0, "the surface is the top, not the whole solid: {sb:?}");
    let mut p = sculpted_project(&top.scene);
    let obj = top.scene.resolved().unwrap().into_iter().find(|o| o.name == "Sculpted top").unwrap();
    p.operations[0].common.geometry.as_mut().unwrap().part_surfaces = Selection {
        mode: SelectionMode::Selected,
        references: vec![obj.reference],
    };
    p.operations[0].common.tolerance = 0.02;
    let (resolved, _) = p.resolve_operation(&p.operations[0]).unwrap();
    assert_eq!(resolved.mesh.unwrap().tris.len(), surface.tris.len());
}

#[test]
fn exact_surface_queries_finish_the_rhino_solid_within_tolerance() {
    use glam::{DVec2, DVec3};
    use std::time::Instant;
    use warp_cam::ops::{Common, OpKind, Operation, SurfaceMode};
    use warp_cam::surface::{self, Cutter, MeshSurface, SurfaceQuery};
    use warp_tools::{Tool, ToolLibrary, ToolShape};

    let t_import = Instant::now();
    let step = StepImporter
        .read(&rhino_fixture("sculpted-v8.step"), &ImportOptions::default())
        .unwrap();
    eprintln!("  import {:.2} s", t_import.elapsed().as_secs_f64());
    let mesh = mesh_for(&step.scene, "Sculpted solid").clone();
    let bb = mesh.bbox();
    let mut tool = Tool::new(2, "ball 6", ToolShape::BallEndMill, 6.0);
    tool.flute_length = bb.size().z + 5.0;
    let mut tools = ToolLibrary::default();
    tools.tools.push(tool.clone());
    let mut p = warp_cam::Project::default();
    p.stock.min = bb.min - DVec3::new(0.0, 0.0, 1.0);
    p.stock.max = bb.max;
    p.mesh = Some(mesh.clone());
    // Finish the top inside the footprint of the solid, a radius in from its sides.
    let inset = DVec2::splat(3.5);
    p.curves.push(warp_core::Polyline2::rect(bb.min.truncate() + inset, bb.max.truncate() - inset));
    let surf = MeshSurface::new(&mesh);
    let c = Cutter::new(&tool, 0.0, 0.0);
    // Single-thread cost of one exact contact query on this mesh.
    let t0 = Instant::now();
    let n = 2000;
    let mut acc = 0.0;
    for i in 0..n {
        let f = i as f64 / n as f64;
        let p = bb.min.truncate() + inset + DVec2::new(f * (bb.size().x - 7.0), (f * 7.3).fract() * (bb.size().y - 7.0));
        acc += surf.drop(&c, p);
    }
    eprintln!("exact query: {:.1} us each ({} queries, checksum {acc:.3})", t0.elapsed().as_secs_f64() * 1e6 / n as f64, n);

    let mut report = Vec::new();
    for mode in [SurfaceMode::Exact, SurfaceMode::ZMap] {
        let mut common = Common::for_tool(&tool);
        common.heights.top = bb.max.z;
        common.heights.bottom = bb.min.z;
        common.heights.retract = bb.max.z + 5.0;
        common.heights.clearance = bb.max.z + 15.0;
        common.tolerance = 0.01;
        common.resolution = 0.1;
        common.surface = mode;
        common.curves = vec![0];
        common.entry = warp_cam::Entry::Plunge;
        let op = Operation {
            name: format!("finish {}", mode.label()),
            enabled: true,
            common,
            kind: OpKind::Parallel { stepover: 1.0, angle: 0.0, zigzag: true },
        };
        let t0 = Instant::now();
        let tp = warp_cam::generate(&op, &p, &tools).unwrap();
        let elapsed = t0.elapsed().as_secs_f64();
        let mut dev = surface::PathDeviation::default();
        let mut cut_len = 0.0;
        let t1 = Instant::now();
        for (from, to, rapid, _) in tp.segments(DVec3::new(0.0, 0.0, 50.0), 0.001) {
            if rapid || from.z > bb.max.z + 1e-6 || to.z > bb.max.z + 1e-6 {
                continue;
            }
            cut_len += from.distance(to);
            surface::path_deviation(&surf, &c, &[from, to], 0.25, bb.min.z, &mut dev);
        }
        eprintln!("  deviation measured in {:.2} s", t1.elapsed().as_secs_f64());
        eprintln!(
            "{}: {} triangles, {} moves, {:.0} mm cut in {:.2} s; gouge bound {:.4} mm at {:.2?}, above {:.4} mm at {:.2?} ({} samples)",
            mode.label(),
            mesh.tris.len(),
            tp.moves.len(),
            cut_len,
            elapsed,
            dev.gouge,
            dev.gouge_at,
            dev.above,
            dev.above_at,
            dev.samples
        );
        report.push((mode, dev, elapsed));
    }
    // Chords are accepted within 0.005 mm (3D) of the exact midpoint contact and simplified
    // within another 0.005 mm, so the tool never sits more than the 0.01 mm operation tolerance
    // inside the contact surface, vertical walls included. The Z-map resolves walls only to its
    // 0.1 mm grid.
    let (exact, zmap) = (&report[0].1, &report[1].1);
    assert!(exact.gouge <= 0.01 + 1e-6, "exact mode is inside the surface by {} at {}", exact.gouge, exact.gouge_at);
    assert!(zmap.gouge > 0.02, "Z-map gouge {} at {}", zmap.gouge, zmap.gouge_at);
}
