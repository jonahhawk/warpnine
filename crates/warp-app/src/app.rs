use crate::view3d::{self, Camera, Tri};
use eframe::egui::{self, Color32, Stroke};
use glam::DVec3;
use std::path::PathBuf;
use warp_cam::post::PostConfig;
use warp_cam::{Project, Toolpath};
use warp_machine::{Machine, MachineConfig};
use warp_tools::ToolLibrary;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Tab {
    Cam,
    Nest,
    Simulate,
    Tools,
    Machine,
    Settings,
}

pub struct Show {
    pub model: bool,
    /// Draw mesh triangle edges over the shaded model.
    pub model_edges: bool,
    pub curves: bool,
    pub toolpaths: bool,
    pub rapids: bool,
    pub stock: bool,
    /// Toolpaths over the stock in the Simulate tab (separate from the CAM view setting).
    pub sim_toolpaths: bool,
    pub sim_holder: bool,
}

pub struct SimState {
    pub stock: Option<warp_sim::Stock>,
    pub player: Option<warp_sim::Player>,
    pub playing: bool,
    /// mm of tool travel per frame.
    pub speed: f64,
    pub resolution: usize,
    pub tool_pos: Option<DVec3>,
    pub tool: u32,
    /// Cached stock triangles and the stock revision they were built from.
    pub mesh_cache: Option<(u64, Vec<[DVec3; 3]>)>,
    /// Estimated run time of the simulated program, seconds (no acceleration).
    pub estimate: f64,
    /// Cells per side used to draw the stock (the simulation grid itself can be finer).
    pub display_cells: usize,
    pub mesh_built_at: std::time::Instant,
}

pub struct WarpApp {
    pub tab: Tab,
    pub project: Project,
    pub project_path: Option<PathBuf>,
    pub tools: ToolLibrary,
    pub tools_dirty: bool,
    pub posts: Vec<PostConfig>,
    pub post_idx: usize,
    /// Generated toolpaths, aligned with `project.operations`.
    pub paths: Vec<Option<Result<Toolpath, String>>>,
    pub generating: usize,
    pub jobs: warp_cam::workflow::JobQueue,
    pub job_tokens: std::collections::HashMap<
        warp_core::scene::OperationId,
        (u64, std::sync::Arc<std::sync::atomic::AtomicBool>),
    >,
    pub path_keys: std::collections::HashMap<warp_core::scene::OperationId, Vec<u8>>,
    /// Generation time of the toolpath in `paths`, in seconds, by operation.
    pub path_times: std::collections::HashMap<warp_core::scene::OperationId, f64>,
    pub epoch: u64,
    pub history: warp_cam::workflow::History,
    pub restoring: bool,
    pub selection_role: warp_core::selection::GeometryRole,
    pub preselection: Vec<warp_core::selection::GeometryRef>,
    pub geometry_filter: String,
    pub import_tolerance_mm: f64,
    pub import_max_triangles: usize,
    pub import_report: Option<String>,
    /// Background Rhino import, cancellable from the geometry browser.
    pub import_job: Option<crate::document_ui::ImportJob>,
    /// Mesh topology summaries keyed by object ID, recomputed when the triangle count changes.
    pub mesh_quality: std::collections::HashMap<String, (usize, warp_core::mesh::MeshQuality)>,
    pub layer_filter: Option<warp_core::scene::LayerId>,
    pub drag_history: Option<String>,
    pub scene_rect: Option<egui::Rect>,
    pub selected_op: Option<usize>,
    /// The geometry dialog for the selected operation is open.
    pub geometry_dialog: bool,
    pub cam: Camera,
    pub show: Show,
    pub sim: SimState,
    pub machine: Machine,
    pub machine_cfg: MachineConfig,
    pub jog_active: Option<(usize, f64)>,
    pub jog_step: f64,
    pub jog_speed: f64,
    pub mdi: String,
    pub selected_tool: Option<u32>,
    pub selected_holder: Option<String>,
    pub status: String,
    pub gcode_preview: String,
    pub uc100_probe: Option<String>,
    /// Camera for the 3D tool preview, and which tool it was last fitted to.
    pub tool_cam: Camera,
    pub tool_cam_fit: Option<u32>,
    /// Display copy of the model (simplified if huge), keyed by `model_key`.
    pub model_view: Option<(u64, Vec<[DVec3; 3]>)>,
    /// STL waiting for the user to confirm its units.
    pub pending_stl: Option<PendingStl>,
    pub nest: crate::nest_ui::NestState,
}

pub struct PendingStl {
    pub name: String,
    pub mesh: warp_core::Mesh,
    /// Index into `STL_UNITS`.
    pub unit: usize,
}

/// STL files carry no units: what one file unit can mean, in mm.
pub const STL_UNITS: [(&str, f64); 5] = [
    ("mm", 1.0),
    ("cm", 10.0),
    ("m", 1000.0),
    ("inch", 25.4),
    ("foot", 304.8),
];

/// Cheap fingerprint of a mesh, to know when the display copy is stale.
fn model_key(m: &warp_core::Mesh) -> u64 {
    use std::hash::{Hash, Hasher};
    let mut h = std::collections::hash_map::DefaultHasher::new();
    m.tris.len().hash(&mut h);
    for t in [m.tris.first(), m.tris.get(m.tris.len() / 2), m.tris.last()]
        .into_iter()
        .flatten()
    {
        for v in t {
            v.x.to_bits().hash(&mut h);
            v.y.to_bits().hash(&mut h);
            v.z.to_bits().hash(&mut h);
        }
    }
    h.finish()
}

impl WarpApp {
    pub fn new(_cc: &eframe::CreationContext<'_>) -> Self {
        let machine_cfg = MachineConfig::load(&MachineConfig::default_path())
            .unwrap_or_else(|_| MachineConfig::simulator());
        let mut project = Project {
            name: "Untitled".into(),
            post: "Mach3 / Mach4".into(),
            ..Default::default()
        };
        project.stock = warp_cam::Stock::default();
        project.ensure_document().expect("empty document");
        let mut cam = Camera::default();
        cam.fit(project.stock.min, project.stock.max);
        let mut tools = ToolLibrary::load_or_default();
        let seeded = tools.seed_starter_sets();
        WarpApp {
            tab: Tab::Cam,
            project,
            project_path: None,
            tools,
            tools_dirty: !seeded.is_empty(),
            posts: PostConfig::presets(),
            post_idx: 0,
            paths: vec![],
            generating: 0,
            jobs: Default::default(),
            job_tokens: Default::default(),
            path_keys: Default::default(),
            path_times: Default::default(),
            epoch: 0,
            history: Default::default(),
            restoring: false,
            selection_role: Default::default(),
            preselection: vec![],
            geometry_filter: String::new(),
            import_tolerance_mm: 0.01,
            import_max_triangles: 2_000_000,
            import_report: None,
            import_job: None,
            mesh_quality: Default::default(),
            layer_filter: None,
            drag_history: None,
            scene_rect: None,
            selected_op: None,
            geometry_dialog: false,
            cam,
            show: Show {
                model: true,
                model_edges: false,
                curves: true,
                toolpaths: true,
                rapids: true,
                stock: true,
                sim_toolpaths: true,
                sim_holder: false,
            },
            sim: SimState {
                stock: None,
                player: None,
                playing: false,
                speed: 20.0,
                resolution: 400,
                tool_pos: None,
                tool: 0,
                mesh_cache: None,
                estimate: 0.0,
                display_cells: 300,
                mesh_built_at: std::time::Instant::now(),
            },
            machine: Machine::spawn(machine_cfg.clone()),
            machine_cfg,
            jog_active: None,
            jog_step: 0.0,
            jog_speed: 1000.0,
            mdi: String::new(),
            selected_tool: None,
            selected_holder: None,
            status: "Ready".into(),
            gcode_preview: String::new(),
            uc100_probe: None,
            tool_cam: Camera::default(),
            tool_cam_fit: None,
            model_view: None,
            pending_stl: None,
            nest: Default::default(),
        }
    }

    pub fn post_config(&self) -> &PostConfig {
        &self.posts[self.post_idx.min(self.posts.len() - 1)]
    }

    pub fn sync_paths_len(&mut self) {
        self.paths
            .resize_with(self.project.operations.len(), || None);
    }

    /// Generate one operation in a background thread.
    pub fn generate(&mut self, i: usize, ctx: &egui::Context) {
        self.project.commit_stock();
        if let Err(e) = self.project.ensure_document() {
            self.status = e.to_string();
            return;
        }
        self.sync_paths_len();
        let Some(op) = self.project.operations.get(i).cloned() else {
            return;
        };
        let key = match warp_cam::workflow::input_key(&op, &self.project, &self.tools) {
            Ok(k) => k,
            Err(e) => {
                self.paths[i] = Some(Err(format!("{e:#}")));
                self.status = format!("{e:#}");
                return;
            }
        };
        if let Some((_, token)) = self.job_tokens.remove(&op.common.id) {
            token.store(true, std::sync::atomic::Ordering::Relaxed);
        }
        self.epoch += 1;
        let token = std::sync::Arc::new(std::sync::atomic::AtomicBool::new(false));
        let id = op.common.id.clone();
        let job = warp_cam::workflow::Job {
            id: id.clone(),
            epoch: self.epoch,
            key,
            op,
            project: self.project.clone(),
            tools: self.tools.clone(),
            cancelled: token.clone(),
        };
        match self.jobs.submit(job) {
            Ok(()) => {
                self.job_tokens.insert(id, (self.epoch, token));
                self.paths[i] = None;
            }
            Err(e) => self.status = e,
        }
        self.generating = self.job_tokens.len();
        ctx.request_repaint();
    }

    pub fn generate_all(&mut self, ctx: &egui::Context) {
        for i in 0..self.project.operations.len() {
            if self.project.operations[i].enabled
                && self.project.operations[i].common.setup == self.project.active_setup
            {
                self.generate(i, ctx);
            }
        }
    }

    fn poll_generation(&mut self) {
        while let Ok(job) = self.jobs.rx.try_recv() {
            if !self.job_tokens.get(&job.id).is_some_and(|(epoch, token)| {
                *epoch == job.epoch && !token.load(std::sync::atomic::Ordering::Relaxed)
            }) {
                continue;
            }
            self.job_tokens.remove(&job.id);
            self.sync_paths_len();
            if let Some(i) = self
                .project
                .operations
                .iter()
                .position(|o| o.common.id == job.id)
            {
                if warp_cam::workflow::input_key(
                    &self.project.operations[i],
                    &self.project,
                    &self.tools,
                )
                .ok()
                .as_ref()
                    != Some(&job.key)
                {
                    continue;
                }
                let r = job.result;
                let secs = job.elapsed.as_secs_f64();
                self.status = match &r {
                    Ok(tp) => {
                        let warn = match tp.warnings.len() {
                            0 => String::new(),
                            1 => ", 1 warning".into(),
                            n => format!(", {n} warnings"),
                        };
                        format!("{}: {:.0} mm of cutting, generated in {secs:.1} s{warn}", tp.name, tp.cut_length())
                    }
                    Err(e) => format!("Error: {e}"),
                };
                self.paths[i] = Some(r);
                self.path_times.insert(job.id.clone(), secs);
                self.path_keys.insert(job.id, job.key);
            }
        }
        self.generating = self.job_tokens.len();
    }

    pub fn ready_paths(&self) -> Vec<Toolpath> {
        self.project
            .operations
            .iter()
            .zip(&self.paths)
            .filter(|(o, _)| o.enabled && o.common.setup == self.project.active_setup)
            .filter(|(o, _)| {
                self.path_keys.get(&o.common.id).is_some_and(|key| {
                    warp_cam::workflow::input_key(o, &self.project, &self.tools)
                        .ok()
                        .as_ref()
                        == Some(key)
                })
            })
            .filter_map(|(_, p)| p.as_ref().and_then(|r| r.as_ref().ok()).cloned())
            .collect()
    }

    pub fn post_program(&mut self) -> String {
        if !self.all_paths_ready() || self.project.post != self.post_config().name {
            return String::new();
        }
        let result = warp_cam::post::post_checked(
            self.post_config(),
            &self.project.name,
            &self.ready_paths(),
            &self.tools,
        );
        match result {
            Ok(code) => {
                self.status = "G-code motion verification passed for the current operations".into();
                code
            }
            Err(e) => {
                self.status = format!("Cannot post: {e}");
                String::new()
            }
        }
    }

    // ---------- file actions ----------

    pub fn import_dxf(&mut self) {
        let Some(path) = rfd::FileDialog::new()
            .add_filter("DXF", &["dxf"])
            .pick_file()
        else {
            return;
        };
        match warp_core::import::load_dxf(&path, 0.01, self.project.display_units) {
            Ok(imp) => {
                let n = imp.curves.len();
                for (i, c) in imp.curves.iter().enumerate() {
                    let scene = self.project.scene.as_mut().unwrap();
                    let id = scene.add(
                        format!(
                            "{} / curve {}",
                            path.file_name().unwrap_or_default().to_string_lossy(),
                            i + 1
                        ),
                        warp_core::scene::Geometry::polyline(c),
                    );
                    let layer_name = &imp.layers[i];
                    let layer = warp_core::scene::LayerId(format!("dxf/{layer_name}"));
                    if !scene.layers.iter().any(|l| l.id == layer) {
                        scene.layers.push(warp_core::scene::Layer {
                            id: layer.clone(),
                            name: layer_name.clone(),
                            visible: true,
                        });
                    }
                    let obj = scene.objects.iter_mut().find(|o| o.id == id).unwrap();
                    obj.layer = layer;
                    obj.source = Some(warp_core::scene::Source {
                        file: path.to_string_lossy().into(),
                        uuid: None,
                        units: warp_core::Units::Millimeters,
                        original_units: Some(imp.source_units.clone()),
                        tolerance_mm: Some(0.01),
                        approximation_tolerance_mm: Some(0.01),
                    });
                }
                let _ = self.project.refresh_view();
                self.project.fit_stock(5.0);
                self.cam.fit(self.project.stock.min, self.project.stock.max);
                self.status = format!("Imported {n} curves from {}", path.display());
                if !imp.skipped.is_empty() {
                    let s: Vec<String> = imp
                        .skipped
                        .iter()
                        .map(|(k, n)| format!("{k}×{n}"))
                        .collect();
                    self.status += &format!(" (skipped: {})", s.join(", "));
                }
            }
            Err(e) => self.status = format!("DXF import failed: {e:#}"),
        }
    }

    pub fn import_stl(&mut self) {
        let Some(path) = rfd::FileDialog::new()
            .add_filter("STL", &["stl"])
            .pick_file()
        else {
            return;
        };
        match warp_core::Mesh::load_stl(&path) {
            Ok(m) if m.tris.is_empty() => {
                self.status = format!("{} contains no triangles", path.display())
            }
            Ok(m) => {
                // Guess the units: the first whose result is a plausible CNC part (20 mm to 1.5 m).
                let size = m.bbox().size().max_element();
                let unit = [0, 3, 2, 1, 4]
                    .into_iter()
                    .find(|&u| (20.0..=1500.0).contains(&(size * STL_UNITS[u].1)))
                    .unwrap_or(0);
                let name = path
                    .file_name()
                    .map(|n| n.to_string_lossy().into_owned())
                    .unwrap_or_default();
                self.pending_stl = Some(PendingStl {
                    name,
                    mesh: m,
                    unit,
                });
            }
            Err(e) => self.status = format!("STL import failed: {e:#}"),
        }
    }

    /// Units dialog for a freshly loaded STL.
    fn stl_dialog(&mut self, ctx: &egui::Context) {
        let Some(pend) = &mut self.pending_stl else {
            return;
        };
        let raw = pend.mesh.bbox().size();
        let mut done = None;
        egui::Modal::new(egui::Id::unique("stl_units")).show(ctx, |ui| {
            ui.heading(format!("Import {}", pend.name));
            ui.label(format!(
                "{} triangles. STL files don't record units: what is one unit in this file?",
                pend.mesh.tris.len()
            ));
            ui.add_space(4.0);
            for (i, (name, f)) in STL_UNITS.iter().enumerate() {
                let s = raw * *f;
                let txt = format!(
                    "{name}  →  {} × {} × {}",
                    crate::units::fmt_len(s.x),
                    crate::units::fmt_len(s.y),
                    crate::units::fmt_len(s.z)
                );
                ui.radio_value(&mut pend.unit, i, txt);
            }
            if raw.z < 1e-6 * raw.max_element().max(1e-9) {
                ui.colored_label(
                    Color32::YELLOW,
                    "This model is flat (no thickness): it is a single surface, not a solid.",
                );
            }
            ui.add_space(4.0);
            ui.horizontal(|ui| {
                if ui.button("Import").clicked() {
                    done = Some(true);
                }
                if ui.button("Cancel").clicked() {
                    done = Some(false);
                }
            });
        });
        match done {
            Some(true) => {
                let mut p = self.pending_stl.take().unwrap();
                p.mesh.scale(STL_UNITS[p.unit].1);
                p.mesh.place_top_at_zero();
                let n = p.mesh.tris.len();
                let size = p.mesh.bbox().size();
                self.project
                    .scene
                    .as_mut()
                    .unwrap()
                    .add(p.name, warp_core::scene::Geometry::Mesh(p.mesh));
                let _ = self.project.refresh_view();
                self.project.fit_stock(5.0);
                self.cam.fit(self.project.stock.min, self.project.stock.max);
                self.show.model = true;
                self.status = format!(
                    "Loaded {n} triangles ({} × {} × {}), placed with its top at Z0",
                    crate::units::fmt_len(size.x),
                    crate::units::fmt_len(size.y),
                    crate::units::fmt_len(size.z)
                );
            }
            Some(false) => self.pending_stl = None,
            None => {}
        }
    }

    pub fn open_project(&mut self) {
        let Some(path) = rfd::FileDialog::new()
            .add_filter("WarpNine project", &["warp"])
            .pick_file()
        else {
            return;
        };
        self.open_project_path(path);
    }

    pub fn open_project_path(&mut self, path: PathBuf) {
        match Project::load(&path) {
            Ok(p) => {
                self.cancel_jobs();
                self.project = p;
                self.history = Default::default();
                self.restoring = true;
                self.selected_op = None;
                self.preselection.clear();
                self.path_keys.clear();
                self.paths.clear();
                self.sync_paths_len();
                self.post_idx = self
                    .posts
                    .iter()
                    .position(|c| c.name == self.project.post)
                    .unwrap_or(0);
                let inches = self.project.display_units == warp_core::Units::Inches;
                for p in &mut self.posts {
                    p.inches = inches;
                }
                self.cam.fit(self.project.stock.min, self.project.stock.max);
                self.project_path = Some(path);
                self.status = if self.posts.iter().any(|p| p.name == self.project.post) {
                    "Project opened".into()
                } else {
                    format!(
                        "Project opened; unresolved post '{}'. Select a post before export.",
                        self.project.post
                    )
                };
            }
            Err(e) => self.status = format!("Open failed: {e:#}"),
        }
    }

    pub fn save_project(&mut self, save_as: bool) {
        let path = match (&self.project_path, save_as) {
            (Some(p), false) => p.clone(),
            _ => match rfd::FileDialog::new()
                .add_filter("WarpNine project", &["warp"])
                .set_file_name(format!("{}.warp", self.project.name))
                .save_file()
            {
                Some(p) => p,
                None => return,
            },
        };
        match self.project.save(&path) {
            Ok(()) => {
                self.status = format!("Saved {}", path.display());
                self.project_path = Some(path);
            }
            Err(e) => self.status = format!("Save failed: {e:#}"),
        }
    }

    pub fn export_gcode(&mut self) {
        if !self.all_paths_ready() || self.project.post != self.post_config().name {
            self.status =
                "Generate every enabled operation and select a resolved post before exporting"
                    .into();
            return;
        }
        let code = self.post_program();
        if code.is_empty() {
            return;
        }
        let ext = self.post_config().file_extension.clone();
        let Some(path) = rfd::FileDialog::new()
            .add_filter("G-code", &[ext.as_str(), "nc", "tap", "ngc", "gcode"])
            .set_file_name(format!("{}.{ext}", self.project.name))
            .save_file()
        else {
            return;
        };
        match std::fs::write(&path, code) {
            Ok(()) => self.status = format!("Wrote {}", path.display()),
            Err(e) => self.status = format!("Write failed: {e}"),
        }
    }

    // ---------- simulation ----------

    /// Allocate a fresh simulation stock. Returns false (with a status message) if it doesn't fit in memory.
    fn new_sim_stock(&mut self) -> bool {
        let s = self.project.stock;
        // Free the old grid first so its memory is available for the new one.
        self.sim.stock = None;
        self.sim.mesh_cache = None;
        match warp_sim::Stock::new(s.min, s.max, self.sim.resolution) {
            Ok(st) => {
                self.sim.stock = Some(st);
                true
            }
            Err(e) => {
                self.sim.player = None;
                self.status = e;
                false
            }
        }
    }

    pub fn sim_reset(&mut self) {
        if !self.all_paths_ready() {
            self.status = "Generate every enabled operation in this setup before simulating".into();
            return;
        }
        if !self.new_sim_stock() {
            return;
        }
        let s = &self.project.stock;
        let mut segs = Vec::new();
        let mut cur = DVec3::new(s.min.x, s.min.y, s.max.z + 20.0);
        for tp in self.ready_paths() {
            for (from, to, rapid, feed) in tp.segments(cur, 0.02) {
                segs.push(warp_sim::SimSegment {
                    from,
                    to,
                    rapid,
                    feed,
                    tool: tp.tool,
                    source: 0,
                });
                cur = to;
            }
        }
        let rapid = self.machine_cfg.rapid_rate.max(1.0);
        self.sim.estimate = segs
            .iter()
            .map(|s| s.from.distance(s.to) / if s.rapid { rapid } else { s.feed.max(1.0) } * 60.0)
            .sum();
        self.status = format!("Simulation ready: {} segments", segs.len());
        self.sim.player = Some(warp_sim::Player::new(segs));
        self.sim.tool_pos = None;
        self.sim.mesh_cache = None;
    }

    /// Simulate a G-code program (e.g. one loaded from disk).
    pub fn sim_gcode(&mut self, code: &str) {
        match warp_gcode::Interpreter::default().run_str(code) {
            Ok(prog) => {
                let mut segs = Vec::new();
                for m in prog.motions.iter().filter(|m| m.is_move()) {
                    let mut prev = m.from;
                    for (p, _) in m.linearize(0.02) {
                        segs.push(warp_sim::SimSegment {
                            from: prev,
                            to: p,
                            rapid: m.kind == warp_gcode::MotionKind::Rapid,
                            feed: m.feed,
                            tool: m.tool,
                            source: m.line,
                        });
                        prev = p;
                    }
                }
                if !self.new_sim_stock() {
                    return;
                }
                self.sim.player = Some(warp_sim::Player::new(segs));
                self.sim.mesh_cache = None;
                self.status = format!(
                    "Loaded program for simulation ({} warnings)",
                    prog.warnings.len()
                );
            }
            Err(e) => self.status = format!("G-code error: {e}"),
        }
    }

    fn sim_tick(&mut self) {
        if !self.sim.playing {
            return;
        }
        let tools = self.tools.clone();
        let lookup = move |n: u32| tools.tool(n).cloned();
        if let (Some(stock), Some(player)) = (self.sim.stock.as_mut(), self.sim.player.as_mut()) {
            if let Some(p) = player.step(stock, &lookup, self.sim.speed) {
                self.sim.tool_pos = Some(p);
                self.sim.tool = player.current().map(|s| s.tool).unwrap_or(0);
            }
            if player.done() {
                self.sim.playing = false;
            }
        }
    }

    pub fn sim_finish(&mut self) {
        let tools = self.tools.clone();
        let lookup = move |n: u32| tools.tool(n).cloned();
        if let (Some(stock), Some(player)) = (self.sim.stock.as_mut(), self.sim.player.as_mut()) {
            player.run_to_end(stock, &lookup);
            self.sim.tool_pos = player.segments.last().map(|s| s.to);
        }
        self.sim.playing = false;
    }

    fn stock_tris(&mut self) -> Vec<[DVec3; 3]> {
        let Some(stock) = &self.sim.stock else {
            return vec![];
        };
        if let Some((rev, tris)) = &self.sim.mesh_cache {
            // Rebuilding scans the whole grid, so during playback refresh at most ~4×/s.
            let fresh = self.sim.mesh_built_at.elapsed() < std::time::Duration::from_millis(250);
            if *rev == stock.revision || (self.sim.playing && fresh) {
                return tris.clone();
            }
        }
        self.sim.mesh_built_at = std::time::Instant::now();
        // Downsample for drawing; keep the minimum height in each block so cuts stay visible.
        let step = (stock.nx.max(stock.ny) / self.sim.display_cells.max(10)).max(1);
        let (nx, ny) = (stock.nx.div_ceil(step), stock.ny.div_ceil(step));
        let h = |bi: usize, bj: usize| -> f64 {
            let mut m = f32::INFINITY;
            for j in bj * step..((bj + 1) * step).min(stock.ny) {
                for i in bi * step..((bi + 1) * step).min(stock.nx) {
                    m = m.min(stock.height(i, j));
                }
            }
            m as f64
        };
        let hs: Vec<f64> = (0..ny)
            .flat_map(|j| (0..nx).map(move |i| (i, j)))
            .map(|(i, j)| h(i, j))
            .collect();
        let pos = |i: usize, j: usize| {
            let p = stock.pos((i * step).min(stock.nx - 1), (j * step).min(stock.ny - 1));
            DVec3::new(p.x, p.y, hs[j * nx + i])
        };
        let mut tris = Vec::with_capacity(nx * ny * 2);
        for j in 0..ny - 1 {
            for i in 0..nx - 1 {
                let (a, b, c, d) = (pos(i, j), pos(i + 1, j), pos(i + 1, j + 1), pos(i, j + 1));
                tris.push([a, b, c]);
                tris.push([a, c, d]);
            }
        }
        // Side walls so the block reads as a solid.
        let bottom = stock.bottom as f64;
        let mut wall = |a: DVec3, b: DVec3| {
            tris.push([
                DVec3::new(a.x, a.y, bottom),
                DVec3::new(b.x, b.y, bottom),
                b,
            ]);
            tris.push([DVec3::new(a.x, a.y, bottom), b, a]);
        };
        for i in 0..nx - 1 {
            wall(pos(i, 0), pos(i + 1, 0));
            wall(pos(i, ny - 1), pos(i + 1, ny - 1));
        }
        for j in 0..ny - 1 {
            wall(pos(0, j), pos(0, j + 1));
            wall(pos(nx - 1, j), pos(nx - 1, j + 1));
        }
        self.sim.mesh_cache = Some((stock.revision, tris.clone()));
        tris
    }

    // ---------- 3D scene ----------

    pub fn scene(&mut self, ui: &mut egui::Ui, sim_view: bool) {
        let rect = ui.available_rect_before_wrap();
        self.scene_rect = Some(rect);
        let resp = self.cam.interact(ui, rect);
        let p = self.cam.proj(rect);
        let painter = ui.painter_at(rect);
        let dark = ui.visuals().dark_mode;
        painter.rect_filled(
            rect,
            0.0,
            if dark {
                Color32::from_gray(28)
            } else {
                Color32::from_gray(235)
            },
        );

        let stock = self.project.stock;
        if sim_view && self.sim.stock.is_some() {
            let tris = self.stock_tris();
            let col = Color32::from_rgb(196, 160, 110);
            view3d::paint_tris(
                &painter,
                &p,
                tris.into_iter().map(|v| Tri { v, color: col }),
                None,
            );
        } else {
            if self.show.model {
                if let Some(m) = &self.project.mesh {
                    let key = model_key(m);
                    if self.model_view.as_ref().is_none_or(|v| v.0 != key) {
                        self.model_view = Some((key, view3d::decimate(&m.tris, 100_000)));
                    }
                    let col = Color32::from_rgb(150, 170, 200);
                    let edges = self.show.model_edges.then(|| {
                        Stroke::new(
                            1.0,
                            if dark {
                                Color32::from_rgb(20, 30, 45)
                            } else {
                                Color32::from_rgb(40, 50, 70)
                            },
                        )
                    });
                    let tris = &self.model_view.as_ref().unwrap().1;
                    view3d::paint_tris(
                        &painter,
                        &p,
                        tris.iter().map(|t| Tri { v: *t, color: col }),
                        edges,
                    );
                }
            }
            if self.show.stock {
                view3d::box_wire(
                    &painter,
                    &p,
                    stock.min,
                    stock.max,
                    Stroke::new(1.0, Color32::from_rgb(200, 150, 60)),
                );
            }
        }
        view3d::axes(
            &painter,
            &p,
            (stock.max - stock.min).truncate().max_element() * 0.15,
        );

        // Curves (selected ones highlighted for the current operation).
        let sel = self.selected_curve_indices();
        if self.show.curves {
            for (i, c) in self.project.curves.iter().enumerate() {
                if !self.project.view_visible.get(i).copied().unwrap_or(true) {
                    continue;
                }
                let mut pts = self
                    .project
                    .view_points
                    .get(i)
                    .cloned()
                    .unwrap_or_else(|| c.pts.iter().map(|q| q.extend(0.0)).collect());
                if sel.contains(&i) {
                    if let Some(r) = self.selected_references().iter().find(|r| {
                        self.project
                            .curve_refs
                            .get(i)
                            .is_some_and(|v| v.same_object(r))
                    }) {
                        if c.closed && r.start_vertex < pts.len() {
                            pts.rotate_left(r.start_vertex);
                        }
                        if r.reversed {
                            if c.closed && pts.len() > 1 {
                                pts[1..].reverse();
                            } else {
                                pts.reverse();
                            }
                        }
                    }
                    if pts.len() > 1 {
                        let a = p.to_screen(pts[0]);
                        let b = p.to_screen(pts[1]);
                        painter.circle_filled(a, 4.0, Color32::YELLOW);
                        painter.arrow(a, (b - a) * 0.35, Stroke::new(2.0, Color32::YELLOW));
                    }
                }
                if c.closed && !pts.is_empty() {
                    pts.push(pts[0]);
                }
                let col = if sel.contains(&i) {
                    Color32::from_rgb(255, 140, 0)
                } else if dark {
                    Color32::WHITE
                } else {
                    Color32::BLACK
                };
                view3d::polyline(
                    &painter,
                    &p,
                    &pts,
                    Stroke::new(if sel.contains(&i) { 2.5 } else { 1.2 }, col),
                );
            }
        }

        if (sim_view && self.show.sim_toolpaths) || (!sim_view && self.show.toolpaths) {
            for (k, r) in self.paths.iter().enumerate() {
                if self
                    .project
                    .operations
                    .get(k)
                    .is_none_or(|o| o.common.setup != self.project.active_setup || !o.enabled)
                {
                    continue;
                }
                let Some(Ok(tp)) = r else { continue };
                let highlighted = self.selected_op == Some(k);
                let feed_col = if highlighted {
                    Color32::from_rgb(40, 200, 255)
                } else {
                    Color32::from_rgb(40, 120, 220)
                };
                let mut run: Vec<DVec3> = Vec::new();
                let mut run_rapid = false;
                let flush = |run: &mut Vec<DVec3>, rapid: bool| {
                    if rapid && !self.show.rapids {
                        run.clear();
                        return;
                    }
                    let s = if rapid {
                        Stroke::new(1.0, Color32::from_rgba_unmultiplied(255, 70, 70, 160))
                    } else {
                        Stroke::new(if highlighted { 1.6 } else { 1.0 }, feed_col)
                    };
                    view3d::polyline(&painter, &p, run, s);
                    run.clear();
                };
                let start = DVec3::new(stock.min.x, stock.min.y, stock.max.z + 20.0);
                for (a, b, rapid, _) in tp.segments(start, 0.05) {
                    if run.is_empty() || rapid != run_rapid {
                        flush(&mut run, run_rapid);
                        run.push(a);
                        run_rapid = rapid;
                    }
                    run.push(b);
                }
                flush(&mut run, run_rapid);
            }
        }

        if sim_view {
            if let Some(tip) = self.sim.tool_pos {
                let fallback =
                    warp_tools::Tool::new(0, "?", warp_tools::ToolShape::FlatEndMill, 3.0);
                let t = self.tools.tool(self.sim.tool).unwrap_or(&fallback);
                let holder = if self.show.sim_holder {
                    t.holder.as_deref().and_then(|id| self.tools.holder(id))
                } else {
                    None
                };
                view3d::tool_solid(&painter, &p, t, holder, tip);
            }
            if let Some(pl) = &self.sim.player {
                for c in &pl.collisions {
                    painter.circle_stroke(p.to_screen(c.at), 6.0, Stroke::new(2.0, Color32::RED));
                }
            }
        }

        // Stable instance-aware picks; no operation is needed for preselection.
        if !sim_view && self.show.curves && resp.clicked() {
            if let Some(mp) = resp.interact_pointer_pos() {
                let mut best: Option<(usize, f32)> = None;
                for (i, c) in self.project.curves.iter().enumerate() {
                    if !self.project.view_visible.get(i).copied().unwrap_or(true) {
                        continue;
                    }
                    let Some(points) = self.project.view_points.get(i) else {
                        continue;
                    };
                    let mut pts = points.clone();
                    if c.closed && !pts.is_empty() {
                        pts.push(pts[0]);
                    }
                    for pair in pts.windows(2) {
                        let d = dist_to_segment(mp, p.to_screen(pair[0]), p.to_screen(pair[1]));
                        if d < 8.0 && best.is_none_or(|(_, bd)| d < bd) {
                            best = Some((i, d));
                        }
                    }
                }
                if let Some((i, _)) = best {
                    if let Some(r) = self.project.curve_refs.get(i).cloned() {
                        self.toggle_reference(r);
                    }
                }
            }
        }

        // View buttons overlay.
        let mut child = ui.new_child(
            egui::UiBuilder::new()
                .max_rect(rect.shrink(6.0))
                .layout(egui::Layout::left_to_right(egui::Align::Min)),
        );
        if child.small_button("Iso").clicked() {
            self.cam.iso();
        }
        if child.small_button("Top").clicked() {
            self.cam.top();
        }
        if child.small_button("Front").clicked() {
            self.cam.front();
        }
        if child.small_button("Fit").clicked() {
            self.cam.fit(stock.min, stock.max);
        }
        if !sim_view && self.project.mesh.is_some() {
            child.add_space(8.0);
            child.toggle_value(&mut self.show.model, "Model");
            child.add_enabled_ui(self.show.model, |ui| {
                ui.toggle_value(&mut self.show.model_edges, "Edges")
            });
        }
        child.label(
            egui::RichText::new(
                "drag: orbit · right-drag/shift: pan · wheel: zoom · click curve: select",
            )
            .weak()
            .small(),
        );
    }
}

fn dist_to_segment(p: egui::Pos2, a: egui::Pos2, b: egui::Pos2) -> f32 {
    let ab = b - a;
    let t = ((p - a).dot(ab) / ab.length_sq().max(1e-6)).clamp(0.0, 1.0);
    (a + ab * t - p).length()
}

impl eframe::App for WarpApp {
    fn ui(&mut self, ui: &mut egui::Ui, _frame: &mut eframe::Frame) {
        let ctx = ui.ctx().clone();
        // Avoid serializing large CAD scenes on idle frames and camera drags.
        let record_frame = ctx.input(|i| {
            i.events
                .iter()
                .any(|e| !matches!(e, egui::Event::PointerMoved(_)))
                || i.pointer.primary_down()
                    && i.pointer
                        .interact_pos()
                        .is_some_and(|p| self.scene_rect.is_none_or(|r| !r.contains(p)))
        });
        let before_document = if record_frame {
            self.project.document_json().unwrap_or_default()
        } else {
            String::new()
        };
        let before_tools = if record_frame {
            serde_json::to_string(&self.tools).unwrap_or_default()
        } else {
            String::new()
        };
        let before_post = if record_frame {
            serde_json::to_string(self.post_config()).unwrap_or_default()
        } else {
            String::new()
        };
        self.restoring = false;
        crate::units::set(self.project.display_units);
        self.poll_generation();
        self.poll_import();
        self.sim_tick();
        if self.sim.playing
            || self.generating > 0
            || self.tab == Tab::Machine
            || self.nest.running.is_some()
            || self.import_job.is_some()
        {
            ctx.request_repaint_after(std::time::Duration::from_millis(16));
        }

        egui::Panel::top("menu").show(ui, |ui| {
            egui::MenuBar::new().ui(ui, |ui| {
                ui.menu_button("File", |ui| {
                    if ui.button("New project").clicked() {
                        let units = self.project.display_units;
                        self.cancel_jobs();
                        self.project = Project {
                            name: "Untitled".into(),
                            post: self.post_config().name.clone(),
                            display_units: units,
                            ..Default::default()
                        };
                        let _ = self.project.ensure_document();
                        self.history = Default::default();
                        self.restoring = true;
                        self.selected_op = None;
                        self.preselection.clear();
                        self.path_keys.clear();
                        self.paths.clear();
                        self.project_path = None;
                    }
                    if ui.button("Open project…").clicked() {
                        self.open_project();
                    }
                    if ui.button("Save project").clicked() {
                        self.save_project(false);
                    }
                    if ui.button("Save project as…").clicked() {
                        self.save_project(true);
                    }
                    ui.separator();
                    if ui.button("Import DXF…").clicked() {
                        self.import_dxf();
                    }
                    if ui.button("Import STL…").clicked() {
                        self.import_stl();
                    }
                    if ui.button("Import 3dm / STEP…").clicked() {
                        self.import_model(None);
                    }
                    ui.separator();
                    if ui.button("Export G-code…").clicked() {
                        self.export_gcode();
                    }
                    if ui.button("Simulate G-code file…").clicked() {
                        if let Some(p) = rfd::FileDialog::new().pick_file() {
                            if let Ok(s) = std::fs::read_to_string(&p) {
                                self.sim_gcode(&s);
                                self.tab = Tab::Simulate;
                            }
                        }
                    }
                });
                ui.menu_button("Edit", |ui| {
                    if ui
                        .add_enabled(self.history.can_undo(), egui::Button::new("Undo (Ctrl+Z)"))
                        .clicked()
                    {
                        self.undo_document(false);
                    }
                    if ui
                        .add_enabled(self.history.can_redo(), egui::Button::new("Redo (Ctrl+Y)"))
                        .clicked()
                    {
                        self.undo_document(true);
                    }
                });
                ui.menu_button("View", |ui| {
                    ui.checkbox(&mut self.show.model, "Model");
                    ui.checkbox(&mut self.show.model_edges, "Model mesh edges");
                    ui.checkbox(&mut self.show.curves, "Curves");
                    ui.checkbox(&mut self.show.toolpaths, "Toolpaths");
                    ui.checkbox(&mut self.show.rapids, "Rapid moves");
                    ui.checkbox(&mut self.show.stock, "Stock box");
                });
                ui.separator();
                let before = self.project.display_units;
                ui.label("Units:");
                ui.selectable_value(
                    &mut self.project.display_units,
                    warp_core::Units::Millimeters,
                    "mm",
                );
                ui.selectable_value(
                    &mut self.project.display_units,
                    warp_core::Units::Inches,
                    "inch",
                );
                if self.project.display_units != before {
                    crate::units::set(self.project.display_units);
                    // Post in the same units by default (still changeable in post settings).
                    for p in &mut self.posts {
                        p.inches = self.project.display_units == warp_core::Units::Inches;
                    }
                }
                ui.separator();
                for (t, name) in [
                    (Tab::Cam, "CAM"),
                    (Tab::Nest, "Nesting"),
                    (Tab::Simulate, "Simulate"),
                    (Tab::Tools, "Tool library"),
                    (Tab::Machine, "Machine"),
                    (Tab::Settings, "Settings"),
                ] {
                    ui.selectable_value(&mut self.tab, t, name);
                }
            });
        });
        egui::Panel::bottom("status").show(ui, |ui| {
            ui.horizontal(|ui| {
                if self.generating > 0 {
                    ui.spinner();
                    ui.label(format!("{} jobs running/queued", self.generating));
                }
                ui.label(&self.status);
            });
        });

        self.stl_dialog(&ctx);
        match self.tab {
            Tab::Cam => self.cam_tab(ui),
            Tab::Nest => self.nest_tab(ui),
            Tab::Simulate => self.sim_tab(ui),
            Tab::Tools => self.tools_tab(ui),
            Tab::Machine => self.machine_tab(ui),
            Tab::Settings => self.settings_tab(ui),
        }
        if !ctx.egui_wants_keyboard_input() {
            if ctx.input_mut(|i| i.consume_key(egui::Modifiers::CTRL, egui::Key::Z)) {
                self.undo_document(false);
            }
            if ctx.input_mut(|i| i.consume_key(egui::Modifiers::CTRL, egui::Key::Y)) {
                self.undo_document(true);
            }
        }
        if record_frame {
            if let Err(e) = self.project.ensure_document() {
                self.status = format!("{e:#}");
            }
            self.project.commit_stock();
            let after_document = self.project.document_json().unwrap_or_default();
            if after_document != before_document {
                if !self.restoring {
                    if ctx.input(|i| i.pointer.primary_down()) {
                        if self.drag_history.is_none() {
                            self.drag_history = Some(before_document);
                        }
                    } else {
                        self.history
                            .record(self.drag_history.take().unwrap_or(before_document));
                    }
                }
                if let Err(e) = self.project.refresh_view() {
                    self.status = format!("{e:#}");
                }
                self.invalidate_document();
            } else if !ctx.input(|i| i.pointer.primary_down()) {
                if let Some(before) = self.drag_history.take() {
                    self.history.record(before);
                }
            }
            if before_tools != serde_json::to_string(&self.tools).unwrap_or_default() {
                self.invalidate_document();
            }
            if before_post != serde_json::to_string(self.post_config()).unwrap_or_default() {
                self.gcode_preview.clear();
            }
        }
    }

    fn on_exit(&mut self) {
        if self.tools_dirty {
            let _ = self.tools.save(&ToolLibrary::default_path());
        }
        self.machine.send(warp_machine::Command::Stop);
        self.machine.send(warp_machine::Command::Disconnect);
    }
}

impl WarpApp {
    fn sim_tab(&mut self, ui: &mut egui::Ui) {
        egui::Panel::left("sim_controls").default_size(260.0).show(ui, |ui| {
            ui.heading("Simulation");
            ui.label("Material removal of the generated toolpaths on the stock.");
            ui.add(egui::Slider::new(&mut self.sim.resolution, 100..=50_000).logarithmic(true).text("grid cells"))
                .on_hover_text("Simulation cells along the longer side of the stock. Applied on Reset.");
            let st = self.project.stock;
            let (cell, nx, ny) = warp_sim::Stock::grid_dims(st.min, st.max, self.sim.resolution);
            let gb = warp_sim::Stock::bytes_for(nx, ny) as f64 / 1e9;
            let info = format!("{nx} × {ny} cells, {}/cell, {gb:.2} GB", crate::units::fmt_len(cell));
            if gb > 4.0 {
                ui.colored_label(Color32::RED, format!("{info} — may exceed available memory"));
            } else if gb > 1.0 {
                ui.colored_label(Color32::YELLOW, format!("{info} — large; playback will be slow"));
            } else {
                ui.label(info);
            }
            ui.add(egui::Slider::new(&mut self.sim.display_cells, 100..=1500).logarithmic(true).text("display detail"))
                .on_hover_text("Cells per side drawn in the 3D view. Higher shows finer detail but redraws slower.");
            if ui.button("Redraw").clicked() {
                self.sim.mesh_cache = None;
            }
            ui.checkbox(&mut self.show.sim_toolpaths, "Show toolpaths");
            ui.checkbox(&mut self.show.rapids, "Show rapid moves");
            ui.checkbox(&mut self.show.sim_holder, "Show tool holder");
            if ui.button("⟲ Reset with current toolpaths").clicked() {
                self.sim_reset();
            }
            ui.horizontal(|ui| {
                let label = if self.sim.playing { "⏸ Pause" } else { "▶ Play" };
                if ui.button(label).clicked() {
                    if self.sim.player.is_none() {
                        self.sim_reset();
                    }
                    self.sim.playing = !self.sim.playing;
                }
                if ui.button("⏭ To end").clicked() {
                    if self.sim.player.is_none() {
                        self.sim_reset();
                    }
                    self.sim_finish();
                }
            });
            ui.add(egui::Slider::new(&mut self.sim.speed, 0.5..=2000.0).logarithmic(true).text("mm / frame"));
            if let Some(pl) = &self.sim.player {
                let frac = if pl.segments.is_empty() { 1.0 } else { pl.index as f32 / pl.segments.len() as f32 };
                ui.add(egui::ProgressBar::new(frac).show_percentage());
                if !pl.collisions.is_empty() {
                    ui.colored_label(Color32::RED, format!("⚠ {} rapid moves cut material", pl.collisions.len()));
                    for c in pl.collisions.iter().take(10) {
                        ui.label(format!("  at X{:.2} Y{:.2} Z{:.2}, {:.2} mm deep", c.at.x, c.at.y, c.at.z, c.depth));
                    }
                }
            }
            if let Some(s) = &self.sim.stock {
                ui.label(format!("Removed: {:.1} cm³", s.removed_volume() / 1000.0));
                ui.label(format!("Grid cell: {}", crate::units::fmt_len(s.cell)));
            }
            if self.sim.estimate > 0.0 {
                ui.label(format!("Estimated run time: {}", fmt_secs(self.sim.estimate)));
            }
        });
        egui::CentralPanel::default().show(ui, |ui| self.scene(ui, true));
    }
}

pub fn fmt_secs(s: f64) -> String {
    let s = s.round() as u64;
    format!("{}:{:02}:{:02}", s / 3600, (s / 60) % 60, s % 60)
}
