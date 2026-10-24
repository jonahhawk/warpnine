//! Reproducible M0 measurement harness; no user tool files or machine connection.
use anyhow::{bail, Context, Result};
use glam::DVec3;
use serde_json::{json, Value};
use std::{path::PathBuf, time::Instant};
use warp_cam::{
    generate,
    post::{post, PostConfig},
    Project,
};
use warp_gcode::Interpreter;
use warp_sim::{Player, SimSegment, Stock};
use warp_tools::ToolLibrary;

fn main() -> Result<()> {
    let output = std::env::args()
        .nth(1)
        .context("usage: cam_baseline <output.json>")?;
    let root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../fixtures/cam-baseline");
    let manifest: Value =
        serde_json::from_str(&std::fs::read_to_string(root.join("benchmark.json"))?)?;
    let tools = ToolLibrary::load(&root.join("tools.json"))?;
    let post_config = PostConfig::mach3();
    let mut results = Vec::new();
    for case in manifest["cases"].as_array().context("manifest cases")? {
        let filename = case["project"].as_str().context("project filename")?;
        let p = Project::load(&root.join(filename))?;
        let op = &p.operations[0];
        for _ in 0..manifest["warmup_runs"].as_u64().unwrap() {
            let _ = generate(op, &p, &tools);
        }
        let mut timings = Vec::new();
        let mut last = None;
        for _ in 0..manifest["measured_runs"].as_u64().unwrap() {
            let start = Instant::now();
            let result = generate(op, &p, &tools);
            timings.push(start.elapsed().as_secs_f64() * 1000.0);
            match (&result, case["expected"].as_str().unwrap()) {
                (Ok(_), "path") => {}
                (Err(e), expected) if expected != "path" && format!("{e:#}").contains(expected) => {
                }
                _ => bail!("{filename}: unexpected generation outcome: {result:?}"),
            }
            last = Some(result);
        }
        let mut sorted = timings.clone();
        sorted.sort_by(f64::total_cmp);
        let mut row = json!({"project": filename, "generation_ms_samples": timings,
                             "generation_ms_median": sorted[sorted.len()/2]});
        match last.context("no measured runs")? {
            Err(e) => {
                row["status"] = json!("known_failure");
                row["error"] = json!(format!("{e:#}"));
                row["issue"] = case["issue"].clone();
            }
            Ok(tp) => {
                let start = DVec3::new(0.0, 0.0, op.common.heights.clearance);
                let segments = tp.segments(
                    start,
                    manifest["inspection_segment_tolerance_mm"]
                        .as_f64()
                        .unwrap(),
                );
                let cut: f64 = segments
                    .iter()
                    .filter(|s| !s.2)
                    .map(|s| s.0.distance(s.1))
                    .sum();
                let rapid: f64 = segments
                    .iter()
                    .filter(|s| s.2)
                    .map(|s| s.0.distance(s.1))
                    .sum();
                let minutes: f64 = segments
                    .iter()
                    .map(|s| {
                        s.0.distance(s.1)
                            / if s.2 {
                                manifest["rapid_estimate_mm_per_min"].as_f64().unwrap()
                            } else {
                                s.3
                            }
                    })
                    .sum();
                let mut stock = Stock::new(
                    p.stock.min,
                    p.stock.max,
                    manifest["simulation_max_cells"].as_u64().unwrap() as usize,
                )
                .map_err(anyhow::Error::msg)?;
                let mut player = Player::new(
                    segments
                        .iter()
                        .enumerate()
                        .map(|(i, s)| SimSegment {
                            from: s.0,
                            to: s.1,
                            rapid: s.2,
                            feed: s.3,
                            tool: tp.tool,
                            source: i,
                        })
                        .collect(),
                );
                player.run_to_end(&mut stock, &|number| tools.tool(number).cloned());
                let code = post(&post_config, &p.name, std::slice::from_ref(&tp), &tools);
                let program = Interpreter::default()
                    .run_str(&code)
                    .map_err(anyhow::Error::msg)?;
                row["status"] = json!("generated");
                row["cut_length_mm"] = json!(cut);
                row["rapid_length_mm"] = json!(rapid);
                row["estimated_motion_minutes"] = json!(minutes);
                row["move_count"] = json!(tp.moves.len());
                row["path_inline_capacity_bytes"] =
                    json!(tp.moves.capacity() * std::mem::size_of::<warp_cam::Move>());
                row["simulation_grid_bytes"] =
                    json!(stock.z.capacity() * std::mem::size_of::<f32>());
                row["simulation_cell_mm"] = json!(stock.cell);
                row["removed_volume_mm3"] = json!(stock.removed_volume());
                row["rapid_collision_count"] = json!(player.collisions.len());
                row["rapid_collision_samples"] = json!(player
                    .collisions
                    .iter()
                    .map(|c| json!({"at_mm": c.at, "reported_depth_mm": c.depth}))
                    .collect::<Vec<_>>());
                row["posted_motion_count"] = json!(program.motions.len());
            }
        }
        results.push(row);
    }
    let report = json!({
        "report_version": 1,
        "environment": {
            "os": std::env::consts::OS, "architecture": std::env::consts::ARCH,
            "cpu": std::env::var("PROCESSOR_IDENTIFIER").unwrap_or_else(|_| "unreported".into()),
            "logical_processors": std::thread::available_parallelism().map(|n| n.get()).unwrap_or(1),
            "build_profile": if cfg!(debug_assertions) { "debug" } else { "release" },
            "rayon_threads": rayon::current_num_threads(),
            "toolchain_and_ram": "see environment.json beside recorded baseline"
        },
        "manifest": manifest, "tools": tools, "post_config": post_config, "results": results
    });
    std::fs::write(&output, serde_json::to_string_pretty(&report)? + "\n")?;
    println!("Wrote {} case measurements to {output}", results.len());
    Ok(())
}
