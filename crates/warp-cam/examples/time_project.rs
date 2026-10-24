//! Time every operation of a project with the user's tool library.
//!
//! `cargo run --release -p warp-cam --example time_project -- project.warp [sampling_step_mm] [tool_number]`
//! The optional step and tool number override every operation (the tool's flute length is
//! extended so depth checks do not get in the way of a timing).
use std::time::Instant;
use warp_cam::{generate, Project};
use warp_tools::ToolLibrary;

fn main() -> anyhow::Result<()> {
    let path = std::env::args().nth(1).expect("usage: time_project <project.warp> [sampling_step_mm] [tool_number]");
    let res: Option<f64> = std::env::args().nth(2).map(|s| s.parse().unwrap());
    let tool: Option<u32> = std::env::args().nth(3).map(|s| s.parse().unwrap());
    let mut p = Project::load(std::path::Path::new(&path))?;
    let mut tools = ToolLibrary::load_or_default();
    if let Some(t) = tool {
        tools.tools.iter_mut().find(|x| x.number == t).expect("tool").flute_length = 100.0;
    }
    let ops = p.operations.clone();
    for mut op in ops {
        if let Some(r) = res {
            op.common.resolution = r;
            p.operations.iter_mut().for_each(|o| o.common.resolution = r);
        }
        if let Some(t) = tool {
            op.common.tool = t;
            p.operations.iter_mut().for_each(|o| o.common.tool = t);
        }
        let t = Instant::now();
        let out = generate(&op, &p, &tools);
        match out {
            Ok(tp) => eprintln!("{}: {} moves in {:.2} s (resolution {}, tolerance {})", op.name, tp.moves.len(), t.elapsed().as_secs_f64(), op.common.resolution, op.common.tolerance),
            Err(e) => eprintln!("{}: error {e:#} after {:.2} s", op.name, t.elapsed().as_secs_f64()),
        }
    }
    Ok(())
}
