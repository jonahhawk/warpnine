//! M4 nesting demo: build a nesting job from the native `.3dm` acceptance project, solve it,
//! verify zero violations, save, reopen and write a report.
//!
//! Run `cargo run -p warp-import-3dm --example m3_acceptance` first to produce
//! `target/m3-acceptance/rhino-job.warp`, then `cargo run -p warp-cam --example nest_demo`.
use glam::DVec2;
use std::path::PathBuf;
use warp_cam::Project;
use warp_core::selection::GeometryRef;
use warp_nest::feasibility::{check, footprints};
use warp_nest::solver::{apply, solve};
use warp_nest::*;

fn main() -> anyhow::Result<()> {
    let root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../target");
    let source = root.join("m3-acceptance/rhino-job.warp");
    anyhow::ensure!(
        source.exists(),
        "missing {}; run `cargo run -p warp-import-3dm --example m3_acceptance` first",
        source.display()
    );
    let mut project = Project::load(&source)?;
    let scene = project.scene.as_ref().unwrap();
    let mut job = NestJob::default();
    let mut report = String::new();

    // Block part: the repeated "Part" block (perimeter + hole), eight copies, 90° steps.
    let blocks = candidates::block_candidates(scene)?;
    let part = blocks
        .iter()
        .find(|c| c.name == "Part")
        .ok_or_else(|| anyhow::anyhow!("acceptance project has no 'Part' block"))?;
    anyhow::ensure!(part.suggestion.ambiguities.is_empty(), "{:?}", part.suggestion.ambiguities);
    let mut p = candidates::part_from_block(PartId(job.fresh_id("part")), part);
    p.quantity = 8;
    p.rotations = Rotations::Step { step_deg: 90.0 };
    p.grain_deg = Some(0.0);
    report += &format!(
        "Block part '{}': outline {:?}, {} hole(s), {} placed instance(s) in the scene\n",
        p.name,
        p.outline.as_ref().map(|r| &r.object.0),
        p.holes.len(),
        part.instance_count
    );
    job.parts.push(p);

    // Loose part: the circle, three copies.
    let circle = scene
        .objects
        .iter()
        .find(|o| o.name == "Circle")
        .map(|o| GeometryRef::object(o.id.clone()))
        .ok_or_else(|| anyhow::anyhow!("no Circle object"))?;
    let s = candidates::loose_suggestion(scene, &[circle.clone()])?;
    anyhow::ensure!(s.outline.is_some() && s.ambiguities.is_empty(), "{s:?}");
    let mut p = candidates::part_from_loose(PartId(job.fresh_id("part")), "Disc", vec![circle], &s);
    p.quantity = 3;
    job.parts.push(p);

    // Non-planar loose part: needs an approved projection before it has a footprint.
    let wavy = scene
        .objects
        .iter()
        .find(|o| o.name == "Periodic nonplanar")
        .map(|o| GeometryRef::object(o.id.clone()))
        .ok_or_else(|| anyhow::anyhow!("no nonplanar object"))?;
    let s = candidates::loose_suggestion(scene, &[wavy.clone()])?;
    anyhow::ensure!(
        s.outline.is_none() && s.ambiguities.iter().any(|a| a.contains("not planar")),
        "{s:?}"
    );
    report += &format!("Non-planar part reported: {}\n", s.ambiguities.join("; "));
    let mut p = candidates::part_from_loose(PartId(job.fresh_id("part")), "Wavy", vec![wavy.clone()], &s);
    p.set_outline(wavy);
    p.projection = Some(ProjectionPlane::Xy);
    p.quantity = 2;
    job.parts.push(p);

    // One sheet kind, two copies, with a clamp in the corner.
    let mut sheet = Sheet::new(SheetId(job.fresh_id("sheet")), "Demo 60x40", DVec2::new(60.0, 40.0), 12.0);
    sheet.edge_margin = 2.0;
    sheet.grain_deg = Some(0.0);
    sheet.available = 2;
    sheet.exclusions.push(Exclusion {
        name: "Clamp".into(),
        min: DVec2::ZERO,
        max: DVec2::new(10.0, 10.0),
    });
    job.sheets.push(sheet);
    job.constraints.part_spacing = 2.0;

    let (fps, errors) = footprints(&job, scene);
    anyhow::ensure!(errors.is_empty(), "{errors:?}");
    for part in &job.parts {
        let f = &fps[&part.id];
        report += &format!(
            "Footprint '{}': {:.2} x {:.2} mm, net area {:.1} mm²\n",
            part.name,
            f.bbox().size().x,
            f.bbox().size().y,
            f.area()
        );
    }
    let started = std::time::Instant::now();
    let layout = solve(&job, &fps, &|| false);
    let elapsed = started.elapsed();
    let unplaced = apply(&mut job, &layout);
    let violations = check(&job, scene)?;
    anyhow::ensure!(violations.is_empty(), "solver layout has violations: {violations:?}");
    report += &format!(
        "Solved {} instances in {:.1} ms: {} sheet(s), {:.1}% utilization, remnant {:.1}%, {} unplaced, 0 violations\n",
        job.instances().len(),
        elapsed.as_secs_f64() * 1e3,
        layout.score.sheets_used,
        layout.score.utilization * 100.0,
        layout.score.remnant * 100.0,
        unplaced.len()
    );
    for u in &unplaced {
        report += &format!("  unplaced {}: {}\n", u.instance.0, u.reason);
    }
    for pl in &job.placements {
        report += &format!(
            "  {} on {} copy {} at ({:.2}, {:.2}) rot {:.0}°{}\n",
            pl.instance.0,
            pl.sheet.0,
            pl.sheet_copy,
            pl.position.x,
            pl.position.y,
            pl.rotation_deg,
            if pl.mirrored { " mirrored" } else { "" }
        );
    }

    project.nesting = Some(job.clone());
    let out = root.join("m4-nesting");
    std::fs::create_dir_all(&out)?;
    let path = out.join("nest-demo.warp");
    project.save(&path)?;
    let reopened = Project::load(&path)?;
    if reopened.nesting.as_ref() != Some(&job) {
        std::fs::write(out.join("debug-before.json"), serde_json::to_string_pretty(&job)?)?;
        std::fs::write(out.join("debug-after.json"), serde_json::to_string_pretty(&reopened.nesting)?)?;
        anyhow::bail!("nesting job changed across save/reopen; see debug-*.json");
    }
    let again = check(reopened.nesting.as_ref().unwrap(), reopened.scene.as_ref().unwrap())?;
    anyhow::ensure!(again.is_empty(), "violations after reopen: {again:?}");
    report += &format!("Saved and reopened {} with identical placements\n", path.display());
    std::fs::write(out.join("report.txt"), &report)?;
    print!("{report}");
    Ok(())
}
