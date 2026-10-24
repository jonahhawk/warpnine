//! Nesting workspace: parts, sheets, constraints, a draggable sheet preview and the
//! background solver. Manual edits are validated with the same rules as the solver.

use crate::app::WarpApp;
use crate::units;
use eframe::egui::{self, Color32, Stroke, Ui};
use glam::DVec2;
use std::collections::HashMap;
use std::sync::{
    atomic::{AtomicBool, Ordering},
    Arc,
};
use warp_core::scene::Geometry;
use warp_nest::feasibility::{check_placements, footprints, oriented_bbox, usable, Violation};
use warp_nest::footprint::{placed, Footprint};
use warp_nest::job::{input_key, NestQueue, NestTask};
use warp_nest::solver::Layout;
use warp_nest::*;

pub struct NestState {
    pub queue: NestQueue,
    pub epoch: u64,
    /// Running solve: epoch, input key and cancel token.
    pub running: Option<(u64, Vec<u8>, Arc<AtomicBool>)>,
    /// Footprints and footprint errors for the current document; dropped on any edit.
    pub footprints: Option<(HashMap<PartId, Footprint>, Vec<Violation>)>,
    pub violations: Vec<Violation>,
    pub selected_part: Option<PartId>,
    pub selected_sheet: Option<SheetId>,
    pub sheet_copy: u32,
    pub selected_instance: Option<PartInstanceId>,
    /// Drag in progress: instance and accumulated sheet-space delta (committed on release).
    pub drag: Option<(PartInstanceId, DVec2)>,
    pub last: Option<Layout>,
}
impl Default for NestState {
    fn default() -> Self {
        Self {
            queue: NestQueue::default(),
            epoch: 0,
            running: None,
            footprints: None,
            violations: vec![],
            selected_part: None,
            selected_sheet: None,
            sheet_copy: 0,
            selected_instance: None,
            drag: None,
            last: None,
        }
    }
}

fn rotation_label(r: &Rotations) -> String {
    match r {
        Rotations::Step { step_deg } if *step_deg <= 0.0 => "Fixed 0°".into(),
        Rotations::Step { step_deg } => format!("{step_deg:.0}° steps"),
        Rotations::Fixed(list) => format!(
            "Fixed {}",
            list.iter()
                .map(|a| format!("{a:.0}°"))
                .collect::<Vec<_>>()
                .join("/")
        ),
    }
}

impl WarpApp {
    fn nest_job(&mut self) -> &mut NestJob {
        self.project.nesting.get_or_insert_with(NestJob::default)
    }

    /// Drop cached footprints and violations; called on every document change.
    pub fn nest_invalidate(&mut self) {
        self.nest.footprints = None;
        self.nest.drag = None;
    }

    fn nest_refresh(&mut self) {
        let Some(job) = &self.project.nesting else {
            self.nest.footprints = None;
            self.nest.violations.clear();
            return;
        };
        if self.nest.footprints.is_none() {
            let scene = self.project.scene.as_ref().unwrap();
            self.nest.footprints = Some(footprints(job, scene));
        }
        let (fps, errors) = self.nest.footprints.as_ref().unwrap();
        let mut v = errors.clone();
        v.extend(
            check_placements(job, fps)
                .into_iter()
                .filter(|v| !matches!(v, Violation::NoFootprint { .. })),
        );
        let report = job.report();
        for e in report.errors {
            v.push(Violation::NoFootprint {
                part: PartId("job".into()),
                reason: e,
            });
        }
        self.nest.violations = v;
    }

    pub fn nest_poll(&mut self) {
        while let Ok(out) = self.nest.queue.rx.try_recv() {
            let Some((epoch, key, token)) = &self.nest.running else {
                continue;
            };
            if out.epoch != *epoch || token.load(Ordering::Relaxed) {
                continue;
            }
            if &out.key != key {
                continue;
            }
            self.nest.running = None;
            self.nest_refresh();
            let Some((fps, _)) = &self.nest.footprints else {
                continue;
            };
            let Some(job) = &self.project.nesting else {
                continue;
            };
            // Only apply if nothing relevant changed while the solver ran.
            if input_key(job, fps) != out.key {
                self.status = "Nesting result discarded: the job changed while it ran".into();
                continue;
            }
            let before = self.project.document_json().unwrap_or_default();
            let unplaced =
                warp_nest::solver::apply(self.project.nesting.as_mut().unwrap(), &out.layout);
            self.history.record(before);
            let s = out.layout.score;
            self.status = format!(
                "Nested: {} sheet(s), {:.1}% utilization, {} unplaced{}",
                s.sheets_used,
                s.utilization * 100.0,
                unplaced.len(),
                if out.layout.cancelled {
                    " (cancelled, partial)"
                } else {
                    ""
                }
            );
            self.nest.last = Some(out.layout);
            self.nest.footprints = None;
        }
    }

    fn nest_start(&mut self, ctx: &egui::Context) {
        self.nest_refresh();
        let Some((fps, _)) = self.nest.footprints.clone() else {
            return;
        };
        let job = self.nest_job().clone();
        if let Err(e) = job.validate() {
            self.status = format!("{e:#}");
            return;
        }
        if let Some((_, _, token)) = self.nest.running.take() {
            token.store(true, Ordering::Relaxed);
        }
        self.nest.epoch += 1;
        let token = Arc::new(AtomicBool::new(false));
        let key = input_key(&job, &fps);
        match self.nest.queue.submit(NestTask {
            epoch: self.nest.epoch,
            key: key.clone(),
            job,
            footprints: fps,
            cancelled: token.clone(),
        }) {
            Ok(()) => {
                self.nest.running = Some((self.nest.epoch, key, token));
                self.status = "Nesting…".into();
            }
            Err(e) => self.status = e,
        }
        ctx.request_repaint();
    }

    pub fn nest_tab(&mut self, ui: &mut Ui) {
        let ctx = ui.ctx().clone();
        self.nest_poll();
        if self.nest.running.is_some() {
            ctx.request_repaint_after(std::time::Duration::from_millis(100));
        }
        if self.project.nesting.is_some() {
            self.nest_refresh();
        }
        egui::Panel::left("nest_left")
            .default_size(340.0)
            .resizable(true)
            .show(ui, |ui| {
                egui::ScrollArea::vertical().show(ui, |ui| {
                    self.nest_parts_ui(ui);
                    ui.separator();
                    self.nest_sheets_ui(ui);
                    ui.separator();
                    self.nest_run_ui(ui, &ctx);
                });
            });
        egui::CentralPanel::default().show(ui, |ui| self.nest_canvas(ui));
    }

    fn nest_parts_ui(&mut self, ui: &mut Ui) {
        ui.heading("Parts");
        let blocks =
            candidates::block_candidates(self.project.scene.as_ref().unwrap()).unwrap_or_default();
        ui.horizontal(|ui| {
            ui.add_enabled_ui(!blocks.is_empty(), |ui| {
                ui.menu_button("Add block part", |ui| {
                    for c in &blocks {
                        let label = format!(
                            "{} ({} placed{})",
                            c.name,
                            c.instance_count,
                            if c.suggestion.ambiguities.is_empty() {
                                ""
                            } else {
                                ", needs review"
                            }
                        );
                        if ui.button(label).clicked() {
                            let id = PartId(self.nest_job().fresh_id("part"));
                            let part = candidates::part_from_block(id.clone(), c);
                            self.status = if c.suggestion.ambiguities.is_empty() {
                                format!("Added part '{}'", part.name)
                            } else {
                                c.suggestion.ambiguities.join("; ")
                            };
                            self.nest_job().parts.push(part);
                            self.nest.selected_part = Some(id);
                            ui.close();
                        }
                    }
                });
            });
            let members = self.preselection.clone();
            if ui
                .add_enabled(
                    !members.is_empty(),
                    egui::Button::new("Add loose part from preselection"),
                )
                .on_hover_text("Preselect curves in the CAM geometry browser, then group them here")
                .clicked()
            {
                let suggestion =
                    candidates::loose_suggestion(self.project.scene.as_ref().unwrap(), &members);
                match suggestion {
                    Ok(s) => {
                        let id = PartId(self.nest_job().fresh_id("part"));
                        let name = format!("Part {}", id.0.rsplit('/').next().unwrap_or("?"));
                        let part = candidates::part_from_loose(id.clone(), name, members, &s);
                        self.status = if s.ambiguities.is_empty() {
                            format!("Added part '{}'", part.name)
                        } else {
                            s.ambiguities.join("; ")
                        };
                        self.nest_job().parts.push(part);
                        self.nest.selected_part = Some(id);
                        self.preselection.clear();
                    }
                    Err(e) => self.status = format!("{e:#}"),
                }
            }
        });
        let Some(job) = &self.project.nesting else {
            ui.label("No nesting job yet. Add a part to begin.");
            return;
        };
        let mut remove = None;
        for p in &job.parts {
            let selected = self.nest.selected_part.as_ref() == Some(&p.id);
            let warn = if p.outline.is_none() {
                " ⚠ no outline"
            } else {
                ""
            };
            if ui
                .selectable_label(selected, format!("{} ×{}{}", p.name, p.quantity, warn))
                .clicked()
            {
                self.nest.selected_part = Some(p.id.clone());
            }
        }
        let Some(pid) = self.nest.selected_part.clone() else {
            return;
        };
        let scene = self.project.scene.as_ref().unwrap();
        // Closed-curve candidates for outline/holes, in the part's own frame.
        let candidates_for =
            |p: &PartDefinition| -> Vec<(warp_core::selection::GeometryRef, String)> {
                match &p.source {
                    PartSource::Block { definition } => scene
                        .definitions
                        .iter()
                        .find(|d| &d.id == definition)
                        .map(|d| {
                            d.objects
                                .iter()
                                .filter(|o| {
                                    matches!(&o.geometry, Geometry::Curve { closed: true, .. })
                                })
                                .map(|o| {
                                    (
                                        warp_core::selection::GeometryRef::object(o.id.clone()),
                                        o.name.clone(),
                                    )
                                })
                                .collect()
                        })
                        .unwrap_or_default(),
                    PartSource::Loose { members } => {
                        let resolved = scene.resolved().unwrap_or_default();
                        members
                            .iter()
                            .filter_map(|m| {
                                resolved
                                    .iter()
                                    .find(|o| o.reference.same_object(m))
                                    .filter(|o| {
                                        matches!(&o.geometry, Geometry::Curve { closed: true, .. })
                                    })
                                    .map(|o| (m.clone(), o.name.clone()))
                            })
                            .collect()
                    }
                }
            };
        let job = self.project.nesting.as_mut().unwrap();
        let Some(p) = job.parts.iter_mut().find(|p| p.id == pid) else {
            return;
        };
        let options = candidates_for(p);
        egui::Grid::new("nest_part").num_columns(2).show(ui, |ui| {
            ui.label("Name");
            ui.text_edit_singleline(&mut p.name);
            ui.end_row();
            ui.label("Quantity");
            ui.add(egui::DragValue::new(&mut p.quantity).range(1..=10_000));
            ui.end_row();
            ui.label("Outline");
            let cur = p
                .outline
                .as_ref()
                .and_then(|r| options.iter().find(|(o, _)| o.same_object(r)))
                .map(|(_, n)| n.clone())
                .unwrap_or_else(|| "(none)".into());
            egui::ComboBox::from_id_salt("nest_outline")
                .selected_text(cur)
                .show_ui(ui, |ui| {
                    for (r, name) in &options {
                        let is = p.outline.as_ref().is_some_and(|o| o.same_object(r));
                        if ui.selectable_label(is, name).clicked() {
                            p.set_outline(r.clone());
                        }
                    }
                });
            ui.end_row();
            ui.label("Holes");
            ui.vertical(|ui| {
                for (r, name) in &options {
                    if p.outline.as_ref().is_some_and(|o| o.same_object(r)) {
                        continue;
                    }
                    let mut is = p.holes.iter().any(|h| h.same_object(r));
                    if ui.checkbox(&mut is, name).changed() {
                        if is {
                            p.holes.push(r.clone());
                        } else {
                            p.holes.retain(|h| !h.same_object(r));
                        }
                    }
                }
            });
            ui.end_row();
            ui.label("Rotations");
            egui::ComboBox::from_id_salt("nest_rot")
                .selected_text(rotation_label(&p.rotations))
                .show_ui(ui, |ui| {
                    for (label, r) in [
                        ("Fixed 0°", Rotations::Step { step_deg: 0.0 }),
                        ("0° / 180°", Rotations::Step { step_deg: 180.0 }),
                        ("90° steps", Rotations::Step { step_deg: 90.0 }),
                        ("45° steps", Rotations::Step { step_deg: 45.0 }),
                        ("15° steps", Rotations::Step { step_deg: 15.0 }),
                    ] {
                        if ui.selectable_label(p.rotations == r, label).clicked() {
                            p.rotations = r;
                        }
                    }
                });
            ui.end_row();
            ui.label("Mirror allowed");
            ui.checkbox(&mut p.allow_mirror, "");
            ui.end_row();
            ui.label("Grain");
            ui.horizontal(|ui| {
                let mut has = p.grain_deg.is_some();
                if ui.checkbox(&mut has, "").changed() {
                    p.grain_deg = has.then_some(0.0);
                }
                if let Some(g) = &mut p.grain_deg {
                    ui.add(egui::DragValue::new(g).suffix("°").speed(1.0));
                }
            });
            ui.end_row();
            ui.label("Extra clearance");
            units::len(ui, &mut p.extra_clearance);
            ui.end_row();
            ui.label("Projection");
            egui::ComboBox::from_id_salt("nest_proj")
                .selected_text(match p.projection {
                    None => "Planar only",
                    Some(ProjectionPlane::Xy) => "XY",
                    Some(ProjectionPlane::Xz) => "XZ",
                    Some(ProjectionPlane::Yz) => "YZ",
                })
                .show_ui(ui, |ui| {
                    ui.selectable_value(&mut p.projection, None, "Planar only");
                    ui.selectable_value(&mut p.projection, Some(ProjectionPlane::Xy), "XY");
                    ui.selectable_value(&mut p.projection, Some(ProjectionPlane::Xz), "XZ");
                    ui.selectable_value(&mut p.projection, Some(ProjectionPlane::Yz), "YZ");
                });
            ui.end_row();
        });
        if ui.button("Remove part").clicked() {
            remove = Some(pid.clone());
        }
        if let Some(id) = remove {
            job.parts.retain(|p| p.id != id);
            let orphans = job.prune_orphans();
            self.nest.selected_part = None;
            self.status = format!("Removed part and {} placement(s)", orphans.len());
        }
    }

    fn nest_sheets_ui(&mut self, ui: &mut Ui) {
        ui.heading("Sheets");
        if ui.button("Add sheet").clicked() {
            let id = SheetId(self.nest_job().fresh_id("sheet"));
            let n = self.nest_job().sheets.len() + 1;
            let mut sheet = Sheet::new(
                id.clone(),
                format!("Sheet {n}"),
                DVec2::new(1220.0, 2440.0),
                18.0,
            );
            sheet.edge_margin = 10.0;
            self.nest_job().sheets.push(sheet);
            self.nest.selected_sheet = Some(id);
        }
        let Some(job) = self.project.nesting.as_mut() else {
            return;
        };
        for s in &job.sheets {
            let selected = self.nest.selected_sheet.as_ref() == Some(&s.id);
            if ui
                .selectable_label(
                    selected,
                    format!(
                        "{} {}×{} ×{}",
                        s.name,
                        units::fmt_len(s.size.x),
                        units::fmt_len(s.size.y),
                        s.available
                    ),
                )
                .clicked()
            {
                self.nest.selected_sheet = Some(s.id.clone());
                self.nest.sheet_copy = 0;
            }
        }
        let Some(sid) = self.nest.selected_sheet.clone() else {
            return;
        };
        let Some(s) = job.sheets.iter_mut().find(|s| s.id == sid) else {
            return;
        };
        egui::Grid::new("nest_sheet").num_columns(2).show(ui, |ui| {
            ui.label("Name");
            ui.text_edit_singleline(&mut s.name);
            ui.end_row();
            ui.label("Width");
            units::len_speed(ui, &mut s.size.x, 1.0);
            ui.end_row();
            ui.label("Height");
            units::len_speed(ui, &mut s.size.y, 1.0);
            ui.end_row();
            ui.label("Thickness");
            units::len(ui, &mut s.thickness);
            ui.end_row();
            ui.label("Edge margin");
            units::len(ui, &mut s.edge_margin);
            ui.end_row();
            ui.label("Grain");
            ui.horizontal(|ui| {
                let mut has = s.grain_deg.is_some();
                if ui.checkbox(&mut has, "").changed() {
                    s.grain_deg = has.then_some(0.0);
                }
                if let Some(g) = &mut s.grain_deg {
                    ui.add(egui::DragValue::new(g).suffix("°").speed(1.0));
                }
            });
            ui.end_row();
            ui.label("Available");
            ui.add(egui::DragValue::new(&mut s.available).range(1..=1000));
            ui.end_row();
        });
        ui.label("Exclusions (clamps)");
        let mut remove = None;
        for (i, e) in s.exclusions.iter_mut().enumerate() {
            ui.horizontal(|ui| {
                ui.text_edit_singleline(&mut e.name);
                units::len_speed(ui, &mut e.min.x, 1.0);
                units::len_speed(ui, &mut e.min.y, 1.0);
                ui.label("to");
                units::len_speed(ui, &mut e.max.x, 1.0);
                units::len_speed(ui, &mut e.max.y, 1.0);
                if ui.small_button("✖").clicked() {
                    remove = Some(i);
                }
            });
        }
        if let Some(i) = remove {
            s.exclusions.remove(i);
        }
        if ui.small_button("Add exclusion").clicked() {
            s.exclusions.push(Exclusion {
                name: "Clamp".into(),
                min: DVec2::ZERO,
                max: DVec2::splat(50.0),
            });
        }
        if ui.button("Remove sheet").clicked() {
            job.sheets.retain(|s| s.id != sid);
            job.placements.retain(|p| p.sheet != sid);
            self.nest.selected_sheet = None;
        }
    }

    fn nest_run_ui(&mut self, ui: &mut Ui, ctx: &egui::Context) {
        ui.heading("Nest");
        let Some(job) = self.project.nesting.as_mut() else {
            return;
        };
        egui::Grid::new("nest_constraints")
            .num_columns(2)
            .show(ui, |ui| {
                ui.label("Part spacing");
                units::len(ui, &mut job.constraints.part_spacing);
                ui.end_row();
                ui.label("Tolerance");
                units::len(ui, &mut job.constraints.tolerance);
                ui.end_row();
                ui.label("Seed");
                ui.add(egui::DragValue::new(&mut job.constraints.seed));
                ui.end_row();
            });
        ui.horizontal(|ui| {
            if self.nest.running.is_some() {
                ui.spinner();
                if ui.button("Cancel").clicked() {
                    if let Some((_, _, token)) = &self.nest.running {
                        token.store(true, Ordering::Relaxed);
                    }
                }
            } else if ui.button("Nest all").clicked() {
                self.nest_start(ctx);
            }
            if ui
                .button("Clear unlocked")
                .on_hover_text("Remove every placement that is not locked")
                .clicked()
            {
                if let Some(job) = self.project.nesting.as_mut() {
                    job.placements.retain(|p| p.locked);
                }
            }
        });
        let Some(job) = self.project.nesting.as_ref() else {
            return;
        };
        let report = job.report();
        if let Some(l) = &self.nest.last {
            ui.label(format!(
                "Last nest: {} sheet(s), {:.1}% utilization, remnant {:.0}%",
                l.score.sheets_used,
                l.score.utilization * 100.0,
                l.score.remnant * 100.0
            ));
        }
        if !report.orphaned.is_empty() {
            ui.colored_label(
                Color32::YELLOW,
                format!(
                    "{} placement(s) refer to removed copies",
                    report.orphaned.len()
                ),
            );
            if ui.small_button("Drop orphaned placements").clicked() {
                let n = self.project.nesting.as_mut().unwrap().prune_orphans().len();
                self.status = format!("Dropped {n} orphaned placement(s)");
            }
        }
        if !report.unplaced.is_empty() {
            ui.label(format!("Unplaced: {}", report.unplaced.len()));
            let reasons: HashMap<&PartInstanceId, &str> = self
                .nest
                .last
                .as_ref()
                .map(|l| {
                    l.unplaced
                        .iter()
                        .map(|u| (&u.instance, u.reason.as_str()))
                        .collect()
                })
                .unwrap_or_default();
            let mut place = None;
            egui::ScrollArea::vertical()
                .id_salt("nest_unplaced")
                .max_height(160.0)
                .show(ui, |ui| {
                    for id in &report.unplaced {
                        ui.horizontal(|ui| {
                            ui.label(&id.0);
                            if let Some(r) = reasons.get(id) {
                                ui.label(*r).on_hover_text(*r);
                            }
                            if ui
                                .small_button("Place")
                                .on_hover_text("Put it at the sheet origin to drag into place")
                                .clicked()
                            {
                                place = Some(id.clone());
                            }
                        });
                    }
                });
            if let Some(id) = place {
                self.nest_place_manually(&id);
            }
        }
        if !self.nest.violations.is_empty() {
            ui.colored_label(
                Color32::LIGHT_RED,
                format!("{} violation(s)", self.nest.violations.len()),
            );
            egui::ScrollArea::vertical()
                .id_salt("nest_violations")
                .max_height(160.0)
                .show(ui, |ui| {
                    for v in &self.nest.violations {
                        ui.label(v.to_string());
                    }
                });
        } else if self
            .project
            .nesting
            .as_ref()
            .is_some_and(|j| !j.placements.is_empty())
        {
            ui.colored_label(
                Color32::LIGHT_GREEN,
                "No clearance, margin or exclusion violations",
            );
        }
    }

    /// Put an unplaced instance at the usable origin of the current sheet copy.
    fn nest_place_manually(&mut self, id: &PartInstanceId) {
        self.nest_refresh();
        let Some(job) = self.project.nesting.as_ref() else {
            return;
        };
        let Some(sheet) = self
            .nest
            .selected_sheet
            .as_ref()
            .and_then(|s| job.sheet(s))
            .or(job.sheets.first())
        else {
            self.status = "Add a sheet first".into();
            return;
        };
        let Some(inst) = job.instances().into_iter().find(|i| &i.id == id) else {
            return;
        };
        let part = job.part(&inst.part).unwrap();
        let Some(fp) = self
            .nest
            .footprints
            .as_ref()
            .and_then(|(f, _)| f.get(&part.id))
        else {
            self.status = format!(
                "Part '{}' has no footprint; set an outline first",
                part.name
            );
            return;
        };
        let (min, _) = usable(sheet);
        let position = min + DVec2::splat(part.extra_clearance) - oriented_bbox(fp, 0.0, false).min;
        let pl = Placement {
            instance: id.clone(),
            sheet: sheet.id.clone(),
            sheet_copy: self.nest.sheet_copy.min(sheet.available.saturating_sub(1)),
            position,
            rotation_deg: 0.0,
            mirrored: false,
            locked: false,
        };
        self.nest.selected_sheet = Some(sheet.id.clone());
        self.nest.selected_instance = Some(id.clone());
        self.project.nesting.as_mut().unwrap().placements.push(pl);
    }

    fn nest_canvas(&mut self, ui: &mut Ui) {
        let Some(job) = self.project.nesting.as_ref() else {
            ui.label("Add parts and a sheet to start nesting.");
            return;
        };
        if self.nest.selected_sheet.is_none() {
            self.nest.selected_sheet = job.sheets.first().map(|s| s.id.clone());
        }
        let Some(sheet) = self
            .nest
            .selected_sheet
            .as_ref()
            .and_then(|s| job.sheet(s))
            .cloned()
        else {
            ui.label("Add a sheet to preview placements.");
            return;
        };
        ui.horizontal(|ui| {
            ui.label("Sheet:");
            let mut pick = None;
            egui::ComboBox::from_id_salt("nest_sheet_pick")
                .selected_text(&sheet.name)
                .show_ui(ui, |ui| {
                    for s in &job.sheets {
                        if ui.selectable_label(s.id == sheet.id, &s.name).clicked() {
                            pick = Some(s.id.clone());
                        }
                    }
                });
            if let Some(p) = pick {
                self.nest.selected_sheet = Some(p);
                self.nest.sheet_copy = 0;
            }
            ui.label("Copy:");
            for c in 0..sheet.available.min(64) {
                let used = job
                    .placements
                    .iter()
                    .filter(|p| p.sheet == sheet.id && p.sheet_copy == c)
                    .count();
                ui.selectable_value(&mut self.nest.sheet_copy, c, format!("{} ({used})", c + 1));
            }
            ui.label("Drag to move · R rotate · M mirror · L lock · Delete unplace");
        });
        let rect = ui.available_rect_before_wrap();
        let resp = ui.allocate_rect(rect, egui::Sense::click_and_drag());
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
        let pad = 20.0;
        let scale = ((rect.width() - 2.0 * pad) / sheet.size.x as f32)
            .min((rect.height() - 2.0 * pad) / sheet.size.y as f32)
            .max(1e-4);
        let origin = egui::pos2(
            rect.center().x - sheet.size.x as f32 * scale * 0.5,
            rect.center().y + sheet.size.y as f32 * scale * 0.5,
        );
        let to_screen =
            |p: DVec2| egui::pos2(origin.x + p.x as f32 * scale, origin.y - p.y as f32 * scale);
        let from_screen = |p: egui::Pos2| {
            DVec2::new(
                ((p.x - origin.x) / scale) as f64,
                ((origin.y - p.y) / scale) as f64,
            )
        };
        let sheet_col = if dark {
            Color32::from_gray(60)
        } else {
            Color32::from_gray(250)
        };
        painter.rect_filled(
            egui::Rect::from_two_pos(to_screen(DVec2::ZERO), to_screen(sheet.size)),
            0.0,
            sheet_col,
        );
        let (umin, umax) = usable(&sheet);
        painter.rect_stroke(
            egui::Rect::from_two_pos(to_screen(umin), to_screen(umax)),
            0.0,
            Stroke::new(1.0, Color32::from_gray(140)),
            egui::StrokeKind::Inside,
        );
        for e in &sheet.exclusions {
            painter.rect_filled(
                egui::Rect::from_two_pos(to_screen(e.min), to_screen(e.max)),
                0.0,
                Color32::from_rgba_unmultiplied(220, 60, 60, 70),
            );
            painter.text(
                to_screen((e.min + e.max) * 0.5),
                egui::Align2::CENTER_CENTER,
                &e.name,
                egui::FontId::proportional(12.0),
                Color32::from_rgb(220, 80, 80),
            );
        }
        if let Some(g) = sheet.grain_deg {
            let c = to_screen(sheet.size * 0.5);
            let d = egui::vec2(g.to_radians().cos() as f32, -(g.to_radians().sin() as f32)) * 30.0;
            painter.arrow(c - d, d * 2.0, Stroke::new(1.0, Color32::from_gray(120)));
        }

        let fps = self
            .nest
            .footprints
            .as_ref()
            .map(|(f, _)| f.clone())
            .unwrap_or_default();
        let violating: std::collections::HashSet<PartInstanceId> = self
            .nest
            .violations
            .iter()
            .flat_map(|v| match v {
                Violation::Overlap { a, b, .. } => vec![a.clone(), b.clone()],
                Violation::OutsideSheet { instance, .. }
                | Violation::Exclusion { instance, .. } => vec![instance.clone()],
                Violation::NoFootprint { .. } => vec![],
            })
            .collect();
        let instances = job.instances();
        let pointer = resp.interact_pointer_pos().or(resp.hover_pos());
        let mut hit: Option<PartInstanceId> = None;
        let mut shapes = vec![];
        for pl in job
            .placements
            .iter()
            .filter(|p| p.sheet == sheet.id && p.sheet_copy == self.nest.sheet_copy)
        {
            let Some(inst) = instances.iter().find(|i| i.id == pl.instance) else {
                continue;
            };
            let Some(fp) = fps.get(&inst.part) else {
                continue;
            };
            let mut shown = pl.clone();
            if let Some((id, d)) = &self.nest.drag {
                if id == &pl.instance {
                    shown.position += *d;
                }
            }
            let (outline, holes) = placed(fp, &shown);
            if let Some(p) = pointer {
                if outline.contains(from_screen(p)) {
                    hit = Some(pl.instance.clone());
                }
            }
            let selected = self.nest.selected_instance.as_ref() == Some(&pl.instance);
            let col = if violating.contains(&pl.instance) {
                Color32::from_rgb(230, 70, 70)
            } else if pl.locked {
                Color32::from_rgb(200, 160, 60)
            } else if dark {
                Color32::from_rgb(120, 190, 255)
            } else {
                Color32::from_rgb(30, 90, 180)
            };
            let width = if selected { 3.0 } else { 1.5 };
            shapes.push((outline, holes, col, width, pl.instance.clone(), pl.locked));
        }
        for (outline, holes, col, width, id, locked) in &shapes {
            let pts: Vec<egui::Pos2> = outline.pts.iter().map(|p| to_screen(*p)).collect();
            painter.add(egui::Shape::closed_line(pts, Stroke::new(*width, *col)));
            for h in holes {
                let pts: Vec<egui::Pos2> = h.pts.iter().map(|p| to_screen(*p)).collect();
                painter.add(egui::Shape::closed_line(pts, Stroke::new(1.0, *col)));
            }
            let b = outline.bbox();
            painter.text(
                to_screen(b.center()),
                egui::Align2::CENTER_CENTER,
                format!("{}{}", if *locked { "🔒 " } else { "" }, id.0),
                egui::FontId::proportional(11.0),
                *col,
            );
        }

        // Interaction: select on click, move on drag, commit on release.
        if resp.drag_started() {
            if let Some(id) = &hit {
                let locked = job.placement(id).is_some_and(|p| p.locked);
                self.nest.selected_instance = Some(id.clone());
                if !locked {
                    self.nest.drag = Some((id.clone(), DVec2::ZERO));
                }
            }
        } else if resp.clicked() {
            self.nest.selected_instance = hit.clone();
        }
        if resp.dragged() {
            if let Some((_, d)) = &mut self.nest.drag {
                let dd = resp.drag_delta();
                *d += DVec2::new((dd.x / scale) as f64, -(dd.y / scale) as f64);
            }
        }
        let mut commit: Option<(PartInstanceId, DVec2)> = None;
        if resp.drag_stopped() {
            commit = self.nest.drag.take();
        }
        let key = |k: egui::Key| ui.input(|i| i.key_pressed(k));
        let shift = ui.input(|i| i.modifiers.shift);
        let selected = self.nest.selected_instance.clone();
        let hovered = resp.hovered() || resp.has_focus();
        let job = self.project.nesting.as_mut().unwrap();
        if let Some((id, d)) = commit {
            if let Some(p) = job.placements.iter_mut().find(|p| p.instance == id) {
                p.position += d;
            }
        }
        if hovered && !ui.ctx().egui_wants_keyboard_input() {
            if let Some(id) = selected {
                let part = instances
                    .iter()
                    .find(|i| i.id == id)
                    .and_then(|i| job.part(&i.part))
                    .cloned();
                if let (Some(part), Some(pl)) =
                    (part, job.placements.iter_mut().find(|p| p.instance == id))
                {
                    if !pl.locked {
                        if key(egui::Key::R) {
                            let c = part.rotations.candidates();
                            if !c.is_empty() {
                                let cur = normalize_deg(pl.rotation_deg);
                                let i = c.iter().position(|a| (a - cur).abs() < 1e-6).unwrap_or(0);
                                let n = if shift {
                                    (i + c.len() - 1) % c.len()
                                } else {
                                    (i + 1) % c.len()
                                };
                                // Rotate about the footprint centre so the part stays put.
                                if let Some(fp) = fps.get(&part.id) {
                                    let before = oriented_bbox(fp, pl.rotation_deg, pl.mirrored);
                                    let after = oriented_bbox(fp, c[n], pl.mirrored);
                                    pl.position += before.center() - after.center();
                                }
                                pl.rotation_deg = c[n];
                            }
                        }
                        if key(egui::Key::M) && part.allow_mirror {
                            if let Some(fp) = fps.get(&part.id) {
                                let before = oriented_bbox(fp, pl.rotation_deg, pl.mirrored);
                                let after = oriented_bbox(fp, pl.rotation_deg, !pl.mirrored);
                                pl.position += before.center() - after.center();
                            }
                            pl.mirrored = !pl.mirrored;
                        }
                    }
                    if key(egui::Key::L) {
                        pl.locked = !pl.locked;
                    }
                    if key(egui::Key::Delete) {
                        job.placements.retain(|p| p.instance != id);
                        self.nest.selected_instance = None;
                    }
                }
            }
        }
        if resp.hovered() && hit.is_some() {
            ui.ctx().set_cursor_icon(egui::CursorIcon::Grab);
        }
        if self.nest.drag.is_some() {
            ui.ctx().request_repaint();
        }
    }
}
