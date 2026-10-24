use crate::app::WarpApp;
use eframe::egui::{self, Ui};
use std::sync::{atomic::AtomicBool, Arc};
use warp_core::interchange::ImportResult;
use warp_core::{scene::*, selection::*};

/// A model import (3dm or STEP) running on a worker thread. The native reader polls `cancel`.
pub struct ImportJob {
    pub path: std::path::PathBuf,
    pub cancel: Arc<AtomicBool>,
    pub rx: std::sync::mpsc::Receiver<anyhow::Result<ImportResult>>,
    pub started: std::time::Instant,
}

/// Browser line for a mesh: topology summary plus provenance, cached per object.
pub fn mesh_info_label(
    cache: &mut std::collections::HashMap<String, (usize, warp_core::mesh::MeshQuality)>,
    ui: &mut Ui,
    id: &ObjectId,
    mesh: &warp_core::Mesh,
) {
    let entry = cache
        .entry(id.0.clone())
        .or_insert_with(|| (mesh.tris.len(), mesh.analyze(1e-4)));
    if entry.0 != mesh.tris.len() {
        *entry = (mesh.tris.len(), mesh.analyze(1e-4));
    }
    let quality = &entry.1;
    let mut text = quality.describe();
    let mut bad = !quality.oriented();
    if let Some(p) = &mesh.provenance {
        text.push_str(" · ");
        text.push_str(&p.summary());
        if p.meets_claim(0.0) == Some(false) {
            bad = true;
        }
    }
    let colour = if bad {
        egui::Color32::LIGHT_RED
    } else {
        egui::Color32::GRAY
    };
    ui.colored_label(colour, text);
}

/// Which native importer handles a path, by extension.
#[derive(Clone, Copy)]
pub enum ImportKind {
    Rhino,
    Step,
}
pub fn import_kind(path: &std::path::Path) -> Option<ImportKind> {
    let ext = path.extension()?.to_string_lossy().to_lowercase();
    match ext.as_str() {
        "3dm" => Some(ImportKind::Rhino),
        "step" | "stp" => Some(ImportKind::Step),
        _ => None,
    }
}

impl WarpApp {
    /// Imports a Rhino `.3dm` or a STEP `.step`/`.stp` file on a worker thread.
    pub fn import_model(&mut self, source: Option<std::path::PathBuf>) {
        use warp_core::interchange::{GeometryImporter, ImportOptions};
        if self.import_job.is_some() {
            self.status = "A model import is already running; wait or cancel it".into();
            return;
        }
        let path = match source {
            Some(p) => p,
            None => match rfd::FileDialog::new()
                .add_filter("CAD models (3dm, STEP)", &["3dm", "step", "stp"])
                .add_filter("Rhino / openNURBS", &["3dm"])
                .add_filter("STEP", &["step", "stp"])
                .pick_file()
            {
                Some(p) => p,
                None => return,
            },
        };
        let Some(kind) = import_kind(&path) else {
            self.status = format!(
                "Cannot import {}: only .3dm and .step/.stp files are supported",
                path.display()
            );
            return;
        };
        let cancel = Arc::new(AtomicBool::new(false));
        let options = ImportOptions {
            tolerance_mm: self.import_tolerance_mm,
            max_triangles: self.import_max_triangles,
            cancel: Some(cancel.clone()),
        };
        let (tx, rx) = std::sync::mpsc::channel();
        let worker_path = path.clone();
        std::thread::spawn(move || {
            let result = match kind {
                ImportKind::Rhino => warp_import_3dm::OpenNurbsImporter.read(&worker_path, &options),
                ImportKind::Step => warp_import_step::StepImporter.read(&worker_path, &options),
            };
            let _ = tx.send(result);
        });
        self.status = format!("Importing {}…", path.display());
        self.import_job = Some(ImportJob {
            path,
            cancel,
            rx,
            started: std::time::Instant::now(),
        });
    }

    pub fn cancel_import(&mut self) {
        if let Some(job) = &self.import_job {
            job.cancel
                .store(true, std::sync::atomic::Ordering::Relaxed);
        }
    }

    pub fn poll_import(&mut self) {
        let Some(job) = &self.import_job else {
            return;
        };
        let result = match job.rx.try_recv() {
            Ok(r) => r,
            Err(std::sync::mpsc::TryRecvError::Empty) => return,
            Err(std::sync::mpsc::TryRecvError::Disconnected) => {
                Err(anyhow::anyhow!("import worker stopped without a result"))
            }
        };
        let job = self.import_job.take().unwrap();
        match result {
            Ok(imported) => self.apply_import(imported, job.started.elapsed()),
            Err(e) => {
                self.status = format!("Model import failed: {e:#}");
                self.import_report = Some(self.status.clone());
            }
        }
    }

    fn apply_import(&mut self, imported: ImportResult, elapsed: std::time::Duration) {
        let mut candidate = self.project.clone();
        let result = imported
            .merge_into(candidate.scene.as_mut().unwrap())
            .and_then(|_| candidate.refresh_view());
        if let Err(e) = result {
            self.status = format!("Model import failed: {e:#}");
            return;
        }
        self.project = candidate;
        self.preselection.clear();
        self.layer_filter = None;
        self.mesh_quality.clear();
        // Preserve explicit setup stock and placement during reimport.
        let mut bounds = warp_core::Aabb3::EMPTY;
        for points in &self.project.view_points {
            for p in points {
                bounds.add(*p);
            }
        }
        if let Some(mesh) = &self.project.mesh {
            let b = mesh.bbox();
            if !b.is_empty() {
                bounds.add(b.min);
                bounds.add(b.max);
            }
        }
        if !bounds.is_empty() {
            self.cam.fit(bounds.min, bounds.max);
        }
        let summary = imported.summary();
        let mut report = format!(
            "{summary} in {:.1} s\nSource: {}\nModel tolerance: {} mm; curve approximation: {} mm",
            elapsed.as_secs_f64(),
            imported.source_file,
            imported.source_tolerance_mm,
            imported.approximation_tolerance_mm
        );
        for o in imported.scene.objects.iter().chain(
            imported
                .scene
                .definitions
                .iter()
                .flat_map(|d| &d.objects),
        ) {
            if let Geometry::Mesh(m) = &o.geometry {
                if let Some(p) = &m.provenance {
                    report.push_str(&format!("\nMachining mesh '{}': {}", o.name, p.summary()));
                }
            }
        }
        for d in imported.diagnostics {
            report.push_str(&format!("\n{}: {} ({})", d.code, d.message, d.object));
        }
        self.import_report = Some(report);
        self.status = summary;
        self.invalidate_document();
    }

    pub fn cancel_jobs(&mut self) {
        for (_, token) in self.job_tokens.values() {
            token.store(true, std::sync::atomic::Ordering::Relaxed);
        }
        self.job_tokens.clear();
        self.generating = 0;
    }
    pub fn all_paths_ready(&self) -> bool {
        let count = self
            .project
            .operations
            .iter()
            .filter(|o| o.enabled && o.common.setup == self.project.active_setup)
            .count();
        count > 0 && self.ready_paths().len() == count
    }
    pub fn invalidate_document(&mut self) {
        self.sync_paths_len();
        for (i, op) in self.project.operations.iter().enumerate() {
            let key = warp_cam::workflow::input_key(op, &self.project, &self.tools).ok();
            if key.as_ref() != self.path_keys.get(&op.common.id) {
                self.paths[i] = None;
                self.path_keys.remove(&op.common.id);
            }
        }
        let existing: std::collections::HashSet<_> = self
            .project
            .operations
            .iter()
            .map(|o| o.common.id.clone())
            .collect();
        self.job_tokens.retain(|id, (_, token)| {
            if !existing.contains(id) {
                token.store(true, std::sync::atomic::Ordering::Relaxed);
                false
            } else {
                true
            }
        });
        self.generating = self.job_tokens.len();
        self.sim.playing = false;
        self.sim.player = None;
        self.sim.stock = None;
        self.sim.mesh_cache = None;
        self.gcode_preview.clear();
        self.model_view = None;
        self.nest_invalidate();
    }
    pub fn undo_document(&mut self, redo: bool) {
        self.project.commit_stock();
        let Ok(current) = self.project.document_json() else {
            return;
        };
        let text = if redo {
            self.history.redo(current)
        } else {
            self.history.undo(current)
        };
        if let Some(text) = text {
            match warp_cam::Project::from_json(&text) {
                Ok(p) => {
                    self.cancel_jobs();
                    self.project = p;
                    self.paths.clear();
                    self.path_keys.clear();
                    self.selected_op = None;
                    self.restoring = true;
                    self.drag_history = None;
                    self.invalidate_document();
                    self.status = if redo { "Redone" } else { "Undone" }.into();
                }
                Err(e) => self.status = format!("History restore failed: {e:#}"),
            }
        }
    }
    pub fn selected_references(&self) -> Vec<GeometryRef> {
        self.selected_op
            .and_then(|i| self.project.operations.get(i))
            .and_then(|o| o.common.geometry.as_ref())
            .map(|g| g.get(self.selection_role).references.clone())
            .unwrap_or_else(|| self.preselection.clone())
    }
    pub fn selected_curve_indices(&self) -> Vec<usize> {
        let refs = self.selected_references();
        self.project
            .curve_refs
            .iter()
            .enumerate()
            .filter_map(|(i, r)| refs.iter().any(|s| s.same_object(r)).then_some(i))
            .collect()
    }
    pub fn toggle_reference(&mut self, r: GeometryRef) {
        if let Some(g) = self
            .selected_op
            .and_then(|i| self.project.operations.get_mut(i))
            .and_then(|o| o.common.geometry.as_mut())
        {
            g.get_mut(self.selection_role).toggle(r);
        } else if let Some(i) = self.preselection.iter().position(|p| p.same_object(&r)) {
            self.preselection.remove(i);
        } else {
            self.preselection.push(r);
        }
    }
    pub fn setup_ui(&mut self, ui: &mut Ui) {
        let mut chosen = self.project.active_setup.clone();
        egui::ComboBox::from_id_salt("active_setup")
            .selected_text(
                self.project
                    .current_setup()
                    .map(|s| s.name.as_str())
                    .unwrap_or("Missing setup"),
            )
            .show_ui(ui, |ui| {
                for s in &self.project.setups {
                    ui.selectable_value(&mut chosen, s.id.clone(), &s.name);
                }
            });
        if chosen != self.project.active_setup {
            let _ = self.project.switch_setup(chosen);
            self.selected_op = None;
            self.invalidate_document();
        }
        if ui.small_button("Add setup").clicked() {
            self.project.commit_stock();
            let mut s = self.project.current_setup().unwrap().clone();
            s.id = SetupId(self.project.fresh_id("setup"));
            s.name = format!("Setup {}", self.project.setups.len() + 1);
            self.project.setups.push(s.clone());
            let _ = self.project.switch_setup(s.id);
            self.selected_op = None;
        }
        egui::CollapsingHeader::new("Setup coordinates").show(ui, |ui| {
            let s = self
                .project
                .setups
                .iter_mut()
                .find(|s| s.id == self.project.active_setup)
                .unwrap();
            ui.text_edit_singleline(&mut s.name);
            ui.horizontal(|ui| {
                ui.label("Work offset");
                ui.add(egui::DragValue::new(&mut s.wcs).range(54..=59).prefix("G"));
            });
            ui.label("Setup origin in model coordinates");
            ui.horizontal(|ui| {
                for v in [&mut s.origin.x, &mut s.origin.y, &mut s.origin.z] {
                    crate::units::len(ui, v);
                }
            });
            ui.label("Setup orientation X / Y / Z (degrees)");
            ui.horizontal(|ui| {
                for v in [
                    &mut s.rotation_deg.x,
                    &mut s.rotation_deg.y,
                    &mut s.rotation_deg.z,
                ] {
                    ui.add(egui::DragValue::new(v).speed(0.5).suffix("°"));
                }
            });
            ui.label("Stock and cut heights are in setup coordinates. Export one setup at a time.");
            ui.label(format!(
                "{} fixture meshes (bounding-box envelope checks)",
                s.fixtures.references.len()
            ));
            if ui.button("Assign preselected meshes as fixtures").clicked() {
                s.fixtures = Selection {
                    mode: SelectionMode::Selected,
                    references: self.preselection.clone(),
                };
            }
            if ui.button("Clear fixtures").clicked() {
                s.fixtures = Selection::default();
            }
        });
    }
    /// The geometry roles an operation of this kind uses, in display order. Roles that hold
    /// references or a non-default mode are always shown too so nothing stored is hidden.
    pub fn roles_for(kind: &warp_cam::OpKind, g: &ControlGeometry) -> Vec<GeometryRole> {
        use warp_cam::OpKind;
        let mut roles: Vec<GeometryRole> = match kind {
            OpKind::ProjectCurves { .. } => vec![
                GeometryRole::Drive,
                GeometryRole::PartSurfaces,
                GeometryRole::Containment,
                GeometryRole::Avoid,
            ],
            k if k.is_3d() => vec![
                GeometryRole::PartSurfaces,
                GeometryRole::Containment,
                GeometryRole::Avoid,
                GeometryRole::CheckSurfaces,
            ],
            k => vec![warp_cam::document::primary_role(k), GeometryRole::Starts],
        };
        for role in GeometryRole::ALL {
            let s = g.get(role);
            let default = match role {
                GeometryRole::PartSurfaces => Selection::all(),
                GeometryRole::Containment => Selection::stock(),
                _ => Selection::default(),
            };
            if !roles.contains(&role) && *s != default {
                roles.push(role);
            }
        }
        roles
    }

    /// Which scene objects a role accepts.
    fn eligible(role: GeometryRole, geometry: &Geometry) -> bool {
        match role {
            GeometryRole::PartSurfaces | GeometryRole::CheckSurfaces => matches!(
                geometry,
                Geometry::Mesh(_)
                    | Geometry::Unavailable {
                        represented_by: Some(_),
                        ..
                    }
            ),
            GeometryRole::Holes | GeometryRole::Starts => {
                matches!(geometry, Geometry::Curve { .. } | Geometry::Point(_))
            }
            _ => matches!(geometry, Geometry::Curve { .. }),
        }
    }

    fn selection_summary(s: &Selection) -> String {
        match s.mode {
            SelectionMode::AllEligible => "all eligible objects".into(),
            SelectionMode::StockBoundary => "stock boundary".into(),
            SelectionMode::Selected => match s.references.len() {
                0 => "none".into(),
                1 => "1 object".into(),
                n => format!("{n} objects"),
            },
        }
    }

    /// Operation panel: one line per role and the button that opens the geometry dialog.
    /// The objects themselves are listed only in the dialog.
    pub fn control_geometry_ui(&mut self, ui: &mut Ui) {
        let Some(op) = self
            .selected_op
            .and_then(|i| self.project.operations.get_mut(i))
        else {
            return;
        };
        let g = op.common.geometry.as_mut().unwrap();
        ui.horizontal(|ui| {
            ui.heading("Geometry");
            if ui.button("Select geometry…").clicked() {
                self.geometry_dialog = true;
            }
        });
        if let Some(message) = &g.review {
            ui.colored_label(egui::Color32::YELLOW, message);
            if ui.button("I reviewed the geometry roles").clicked() {
                g.review = None;
            }
        }
        let roles = Self::roles_for(&op.kind, g);
        if !roles.contains(&self.selection_role) {
            self.selection_role = roles[0];
        }
        egui::Grid::new("geometry_summary").num_columns(2).show(ui, |ui| {
            for role in roles {
                ui.label(role.label());
                ui.label(Self::selection_summary(g.get(role)));
                ui.end_row();
            }
        });
    }

    /// Non-modal window for choosing the geometry of the selected operation, role by role.
    pub fn geometry_dialog(&mut self, ctx: &egui::Context) {
        if !self.geometry_dialog {
            return;
        }
        let Some(i) = self.selected_op.filter(|&i| i < self.project.operations.len()) else {
            self.geometry_dialog = false;
            return;
        };
        let title = format!("Geometry: {}", self.project.operations[i].name);
        let mut open = true;
        egui::Window::new(title)
            .id(egui::Id::unique("geometry_dialog"))
            .open(&mut open)
            .resizable(true)
            .default_size([460.0, 560.0])
            .show(ctx, |ui| self.geometry_dialog_body(ui, i));
        self.geometry_dialog = open;
    }

    fn geometry_dialog_body(&mut self, ui: &mut Ui, i: usize) {
        let kind = self.project.operations[i].kind.clone();
        let objects = self
            .project
            .scene
            .as_ref()
            .unwrap()
            .resolved()
            .unwrap_or_default();
        let g = self.project.operations[i].common.geometry.as_mut().unwrap();
        let roles = Self::roles_for(&kind, g);
        if !roles.contains(&self.selection_role) {
            self.selection_role = roles[0];
        }
        ui.horizontal_wrapped(|ui| {
            for role in &roles {
                ui.selectable_value(&mut self.selection_role, *role, role.label());
            }
        });
        let role = self.selection_role;
        ui.separator();
        let s = g.get_mut(role);
        ui.horizontal(|ui| {
            ui.radio_value(&mut s.mode, SelectionMode::AllEligible, "All eligible objects")
                .on_hover_text("Every object of the right type in the scene, including ones added later");
            ui.radio_value(&mut s.mode, SelectionMode::Selected, "Selected objects");
            if role == GeometryRole::Containment
                || matches!(kind, warp_cam::OpKind::Facing { .. }) && role == GeometryRole::Regions
            {
                ui.radio_value(&mut s.mode, SelectionMode::StockBoundary, "Stock boundary");
            }
        });
        if matches!(role, GeometryRole::CheckSurfaces | GeometryRole::Starts) {
            ui.label("Stored for future use; assigning this role currently blocks generation.");
        }
        let filter = self.geometry_filter.to_lowercase();
        let shown: Vec<&ResolvedObject> = objects
            .iter()
            .filter(|o| Self::eligible(role, &o.geometry))
            .filter(|o| self.layer_filter.as_ref().is_none_or(|l| *l == o.layer))
            .filter(|o| filter.is_empty() || o.name.to_lowercase().contains(&filter))
            .collect();
        ui.horizontal(|ui| {
            ui.label("Filter");
            ui.text_edit_singleline(&mut self.geometry_filter);
            if ui.small_button("Check all shown").clicked() {
                s.mode = SelectionMode::Selected;
                for o in &shown {
                    if !s.references.iter().any(|r| r.same_object(&o.reference)) {
                        s.references.push(o.reference.clone());
                    }
                }
            }
            if ui.small_button("Clear").clicked() {
                s.references.clear();
            }
        });
        ui.label(format!(
            "{} eligible object{} · {} checked{}",
            shown.len(),
            if shown.len() == 1 { "" } else { "s" },
            s.references.len(),
            if s.mode == SelectionMode::Selected { "" } else { " (not used in this mode)" }
        ));
        // References that no longer resolve stay visible so they can be removed.
        let mut remove = None;
        let mut up = None;
        for (k, r) in s.references.iter().enumerate() {
            if !objects.iter().any(|o| o.reference.same_object(r)) {
                ui.horizontal(|ui| {
                    ui.colored_label(
                        egui::Color32::LIGHT_RED,
                        format!("Missing {} — remove and pick a replacement", r.object.0),
                    );
                    if ui.small_button("×").clicked() {
                        remove = Some(k);
                    }
                });
            }
        }
        let mut toggle = None;
        egui::ScrollArea::vertical()
            .id_salt("geometry_dialog_objects")
            .auto_shrink([false, false])
            .show(ui, |ui| {
                for o in shown {
                    let kind_label = match &o.geometry {
                        Geometry::Curve { closed: true, .. } => "closed curve",
                        Geometry::Curve { .. } => "open curve",
                        Geometry::Mesh(_) => "mesh",
                        Geometry::Point(_) => "point",
                        Geometry::Unavailable { .. } => "solid (machined as its mesh)",
                        _ => continue,
                    };
                    let pos = s.references.iter().position(|r| r.same_object(&o.reference));
                    ui.push_id(format!("{:?}", o.reference), |ui| {
                        ui.horizontal(|ui| {
                            let mut checked = pos.is_some();
                            if ui.checkbox(&mut checked, "").changed() {
                                toggle = Some(o.reference.clone());
                            }
                            ui.label(format!("{} · {kind_label}", o.name));
                            if let Some(k) = pos {
                                if k > 0 && ui.small_button("↑").on_hover_text("Machine earlier").clicked() {
                                    up = Some(k);
                                }
                            }
                        });
                        if let (Some(k), Geometry::Curve { points, closed, .. }) = (pos, &o.geometry) {
                            let r = &mut s.references[k];
                            ui.indent("curve_opts", |ui| {
                                ui.horizontal(|ui| {
                                    ui.checkbox(&mut r.reversed, "Reverse");
                                    if *closed {
                                        ui.label("Start vertex");
                                        ui.add(
                                            egui::DragValue::new(&mut r.start_vertex)
                                                .range(0..=points.len().saturating_sub(1)),
                                        );
                                    }
                                });
                            });
                        }
                    });
                }
            });
        if let Some(r) = toggle {
            s.toggle(r);
        }
        if let Some(k) = remove {
            s.references.remove(k);
        } else if let Some(k) = up {
            s.references.swap(k, k - 1);
        }
        ui.separator();
        ui.horizontal(|ui| {
            ui.label("Clicking a curve in the viewport toggles it in the highlighted role.");
            if ui.button("Close").clicked() {
                self.geometry_dialog = false;
            }
        });
    }
    pub fn geometry_browser(&mut self, ui: &mut Ui) {
        ui.horizontal(|ui| {
            let running = self.import_job.is_some();
            if ui
                .add_enabled(!running, egui::Button::new("Import 3dm / STEP…"))
                .clicked()
            {
                self.import_model(None);
            }
            ui.add(
                egui::DragValue::new(&mut self.import_tolerance_mm)
                    .range(0.000001..=10.0)
                    .speed(0.001)
                    .suffix(" mm"),
            )
            .on_hover_text(
                "Maximum curve approximation error in model millimeters, including block scale",
            );
            let mut millions = self.import_max_triangles as f64 / 1e6;
            if ui
                .add(
                    egui::DragValue::new(&mut millions)
                        .range(0.01..=50.0)
                        .speed(0.1)
                        .suffix(" M tris"),
                )
                .on_hover_text("Memory bound: the import fails before exceeding this many mesh triangles")
                .changed()
            {
                self.import_max_triangles = (millions * 1e6).round().max(1.0) as usize;
            }
        });
        if let Some(job) = &self.import_job {
            let (path, elapsed) = (job.path.clone(), job.started.elapsed().as_secs_f64());
            ui.horizontal(|ui| {
                ui.spinner();
                ui.label(format!(
                    "Importing {}… {elapsed:.0} s",
                    path.file_name()
                        .map(|f| f.to_string_lossy().into_owned())
                        .unwrap_or_default()
                ));
                if ui.button("Cancel import").clicked() {
                    self.cancel_import();
                }
            });
        }
        let sources: std::collections::BTreeSet<_> = self
            .project
            .scene
            .as_ref()
            .unwrap()
            .objects
            .iter()
            .chain(
                self.project
                    .scene
                    .as_ref()
                    .unwrap()
                    .definitions
                    .iter()
                    .flat_map(|d| &d.objects),
            )
            .filter_map(|o| o.source.as_ref())
            .filter(|s| import_kind(std::path::Path::new(&s.file)).is_some())
            .map(|s| s.file.clone())
            .collect();
        if !sources.is_empty() {
            ui.menu_button("Reimport source", |ui| {
                for source in sources {
                    if ui
                        .add_enabled(self.import_job.is_none(), egui::Button::new(&source))
                        .clicked()
                    {
                        self.import_model(Some(source.into()));
                        ui.close();
                    }
                }
            });
        }
        if let Some(report) = &self.import_report {
            ui.collapsing("Import report", |ui| {
                ui.label(report);
            });
        }
        ui.horizontal(|ui| {
            if ui.button("Import DXF…").clicked() {
                self.import_dxf();
            }
            if ui.button("Import STL…").clicked() {
                self.import_stl();
            }
        });
        ui.text_edit_singleline(&mut self.geometry_filter)
            .on_hover_text("Filter by object name or type: curve, mesh, point");
        egui::ComboBox::from_id_salt("layer_filter")
            .selected_text(
                self.layer_filter
                    .as_ref()
                    .map(|l| l.0.as_str())
                    .unwrap_or("All layers"),
            )
            .show_ui(ui, |ui| {
                ui.selectable_value(&mut self.layer_filter, None, "All layers");
                for l in &self.project.scene.as_ref().unwrap().layers {
                    ui.selectable_value(&mut self.layer_filter, Some(l.id.clone()), &l.name);
                }
            });
        egui::CollapsingHeader::new("Layers").show(ui, |ui| {
            for layer in &mut self.project.scene.as_mut().unwrap().layers {
                ui.horizontal(|ui| {
                    ui.checkbox(&mut layer.visible, "");
                    ui.text_edit_singleline(&mut layer.name);
                });
            }
        });
        ui.horizontal(|ui| {
            if ui.small_button("Show all").clicked() {
                for o in &mut self.project.scene.as_mut().unwrap().objects {
                    o.visible = true;
                }
            }
            if ui.small_button("Isolate selected").clicked() {
                let refs = self.selected_references();
                for o in &mut self.project.scene.as_mut().unwrap().objects {
                    o.visible = refs.iter().any(|r| {
                        r.instances
                            .first()
                            .map(|i| i.0.as_str())
                            .unwrap_or(&r.object.0)
                            == o.id.0
                    });
                }
            }
            if ui
                .small_button("Preselect")
                .on_hover_text("Pick geometry before creating a new operation")
                .clicked()
            {
                self.selected_op = None;
            }
        });
        let refs = self.selected_references();
        let objects = self
            .project
            .scene
            .as_ref()
            .unwrap()
            .resolved()
            .unwrap_or_default();
        let filter = self.geometry_filter.to_lowercase();
        let mut toggle = None;
        let mut delete = None;
        let mut visible = None;
        egui::ScrollArea::vertical()
            .id_salt("scene_objects")
            .max_height(240.0)
            .show(ui, |ui| {
                for obj in objects {
                    let kind = match &obj.geometry {
                        Geometry::Curve { closed, .. } => {
                            if *closed {
                                "closed curve"
                            } else {
                                "open curve"
                            }
                        }
                        Geometry::Mesh(_) => "mesh",
                        Geometry::Point(_) => "point",
                        Geometry::Unavailable { .. } => "unavailable",
                        _ => continue,
                    };
                    if self.layer_filter.as_ref().is_some_and(|l| *l != obj.layer)
                        || !format!("{} {kind}", obj.name)
                            .to_lowercase()
                            .contains(&filter)
                    {
                        continue;
                    }
                    let instance_label = obj
                        .reference
                        .instances
                        .iter()
                        .filter_map(|id| {
                            let scene = self.project.scene.as_ref().unwrap();
                            scene
                                .objects
                                .iter()
                                .chain(scene.definitions.iter().flat_map(|d| &d.objects))
                                .find(|o| o.id.0 == id.0)
                                .map(|o| o.name.clone())
                        })
                        .collect::<Vec<_>>()
                        .join(" / ");
                    ui.push_id(format!("{:?}", obj.reference), |ui| {
                        ui.horizontal(|ui| {
                            let mut picked = refs.iter().any(|r| r.same_object(&obj.reference));
                            if ui.checkbox(&mut picked, "").changed() {
                                toggle = Some(obj.reference.clone());
                            }
                            ui.label(format!(
                                "{} · {kind}{}",
                                obj.name,
                                if obj.reference.instances.is_empty() {
                                    String::new()
                                } else {
                                    format!(" · {instance_label}")
                                }
                            ));
                        });
                        if let Geometry::Unavailable {
                            reason,
                            represented_by,
                        } = &obj.geometry
                        {
                            let colour = if represented_by.is_some() {
                                egui::Color32::LIGHT_GREEN
                            } else {
                                egui::Color32::YELLOW
                            };
                            ui.colored_label(colour, reason);
                        }
                        if let Geometry::Mesh(mesh) = &obj.geometry {
                            mesh_info_label(
                                &mut self.mesh_quality,
                                ui,
                                &obj.reference.object,
                                mesh,
                            );
                        }
                        ui.horizontal(|ui| {
                            let root = ObjectId(
                                obj.reference
                                    .instances
                                    .first()
                                    .map(|i| i.0.clone())
                                    .unwrap_or(obj.reference.object.0.clone()),
                            );
                            if ui
                                .small_button(if obj.visible { "Hide" } else { "Show" })
                                .clicked()
                            {
                                visible = Some((root.clone(), !obj.visible));
                            }
                            if ui.small_button("Delete object / instance").clicked() {
                                delete = Some(root);
                            }
                        });
                    });
                }
            });
        if let Some(r) = toggle {
            self.toggle_reference(r);
        }
        if let Some(id) = delete {
            self.project
                .scene
                .as_mut()
                .unwrap()
                .objects
                .retain(|o| o.id != id);
        }
        if let Some((id, on)) = visible {
            if let Some(o) = self
                .project
                .scene
                .as_mut()
                .unwrap()
                .objects
                .iter_mut()
                .find(|o| o.id == id)
            {
                o.visible = on;
            }
        }
    }
}
