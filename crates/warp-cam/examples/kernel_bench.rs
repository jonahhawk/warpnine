//! Z-map cutter-location map cost per tool shape on a synthetic surface, or on the first
//! operation of a project: `cargo run --release -p warp-cam --example kernel_bench -- [cell_mm] [size_mm] [project.warp]`.
use glam::{DVec2, DVec3};
use std::time::Instant;
use warp_cam::zmap::{Kernel, ZMap};
use warp_core::Mesh;
use warp_tools::{Tool, ToolShape};

fn main() {
    let cell: f64 = std::env::args().nth(1).map(|s| s.parse().unwrap()).unwrap_or(0.1);
    let size: f64 = std::env::args().nth(2).map(|s| s.parse().unwrap()).unwrap_or(60.0);
    // A bumpy surface: a few pyramids on a slab.
    let mut tris = Vec::new();
    let p = |x: f64, y: f64, z: f64| DVec3::new(x, y, z);
    tris.push([p(0.0, 0.0, 0.0), p(size, 0.0, 0.0), p(size, size, 0.0)]);
    tris.push([p(0.0, 0.0, 0.0), p(size, size, 0.0), p(0.0, size, 0.0)]);
    for k in 0..4 {
        let c = DVec2::new(size * (0.2 + 0.2 * k as f64), size * 0.5);
        let apex = p(c.x, c.y, 8.0);
        let r = 6.0;
        let corners = [p(c.x - r, c.y - r, 0.0), p(c.x + r, c.y - r, 0.0), p(c.x + r, c.y + r, 0.0), p(c.x - r, c.y + r, 0.0)];
        for i in 0..4 {
            tris.push([corners[i], corners[(i + 1) % 4], apex]);
        }
    }
    let mut mesh = Mesh { tris, provenance: None };
    let mut lo = DVec2::splat(-8.0);
    let mut hi = DVec2::splat(size + 8.0);
    let mut floor = -5.0;
    if let Some(path) = std::env::args().nth(3) {
        let p = warp_cam::Project::load(std::path::Path::new(&path)).unwrap();
        let (resolved, op) = p.resolve_operation(&p.operations[0]).unwrap();
        mesh = resolved.mesh.clone().unwrap();
        lo = resolved.stock.min.truncate() - DVec2::splat(8.0);
        hi = resolved.stock.max.truncate() + DVec2::splat(8.0);
        floor = op.common.heights.bottom;
        eprintln!("project mesh: {} triangles, floor {floor}", mesh.tris.len());
    }
    let zm = ZMap::from_mesh(&mesh, lo, hi, cell, floor);
    eprintln!("map {}x{} = {} cells, cell {cell}", zm.nx, zm.ny, zm.nx * zm.ny);
    let tools = [
        Tool::new(1, "flat 12.7", ToolShape::FlatEndMill, 12.7),
        Tool::new(2, "ball 6", ToolShape::BallEndMill, 6.0),
        Tool::new(3, "ball 12.7", ToolShape::BallEndMill, 12.7),
        Tool::new(4, "v90 12.7", ToolShape::VBit { included_angle: 90.0, tip_diameter: 0.0 }, 12.7),
        Tool::new(5, "bull 12.7 r1", ToolShape::BullNose { corner_radius: 1.0 }, 12.7),
    ];
    for t in &tools {
        let k = Kernel::new(t, cell, 0.0, 0.0);
        let flat_rows = k.rows.iter().filter(|r| r.flat.is_some()).count();
        let t0 = Instant::now();
        let cl = zm.cl_map(&k);
        let dt = t0.elapsed().as_secs_f64();
        let ops = (zm.nx * zm.ny) as f64 * k.samples() as f64;
        eprintln!(
            "{:<14} samples {:>6} ({} of {} rows flat): {:.3} s, {:.2} G samples/s, checksum {:.3}",
            t.name,
            k.samples(),
            flat_rows,
            k.rows.len(),
            dt,
            ops / dt / 1e9,
            cl.z.iter().sum::<f64>()
        );
    }
}
