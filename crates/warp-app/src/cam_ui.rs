//! CAM tab: geometry, stock, operation list and full parameter editing.

use crate::app::{Tab, WarpApp};
use crate::units;
use eframe::egui::{self, DragValue, Ui};
use glam::DVec2;
use warp_cam::ops::{
    default_resolution_for, Common, CutOrder, DrillSource, Entry, OpKind, Operation, PocketPattern, Side,
    SurfaceMode, Tabs,
};
use warp_cam::zmap::ZMapWork;
use warp_cam::toolpath::DrillCycle;
use warp_tools::{Coolant, ToolLibrary, ToolUnits};

/// Colour for relaxed-check warnings on generated toolpaths.
pub const WARNING: egui::Color32 = egui::Color32::from_rgb(235, 170, 40);

/// Thousands-grouped integer for the cost readout.
fn group(n: usize) -> String {
    let s = n.to_string();
    let mut out = String::new();
    for (i, ch) in s.chars().enumerate() {
        if i > 0 && (s.len() - i) % 3 == 0 {
            out.push(' ');
        }
        out.push(ch);
    }
    out
}

fn num(ui: &mut Ui, label: &str, v: &mut f64, speed: f64, suffix: &str) -> bool {
    ui.label(label);
    let r = ui
        .add(
            DragValue::new(v)
                .speed(speed)
                .suffix(suffix)
                .max_decimals(4),
        )
        .changed();
    ui.end_row();
    r
}

/// Length row (stored in mm, shown in display units).
fn mm(ui: &mut Ui, label: &str, v: &mut f64) -> bool {
    ui.label(label);
    let r = units::len(ui, v);
    ui.end_row();
    r
}

fn feed(ui: &mut Ui, label: &str, v: &mut f64) -> bool {
    ui.label(label);
    let r = units::feed(ui, v);
    ui.end_row();
    r
}

fn side_combo(ui: &mut Ui, id: &str, side: &mut Side) {
    ui.label("Side");
    egui::ComboBox::from_id_salt(id)
        .selected_text(format!("{side:?}"))
        .show_ui(ui, |ui| {
            ui.selectable_value(side, Side::Outside, "Outside");
            ui.selectable_value(side, Side::Inside, "Inside");
            ui.selectable_value(side, Side::On, "On");
        });
    ui.end_row();
}

/// `extent` is the XY size of the stock, used to predict the Z-map cost of 3D operations.
pub fn edit_common(ui: &mut Ui, c: &mut Common, tools: &ToolLibrary, kind: &OpKind, extent: DVec2) {
    let is_3d = kind.is_3d();
    egui::Grid::new("common")
        .num_columns(2)
        .striped(true)
        .show(ui, |ui| {
            ui.label("Tool");
            let cur = tools
                .tool(c.tool)
                .map(|t| format!("T{} {}", t.number, t.name))
                .unwrap_or(format!("T{} (missing)", c.tool));
            let mut picked = None;
            egui::ComboBox::from_id_salt("tool")
                .selected_text(cur)
                .width(200.0)
                .show_ui(ui, |ui| {
                    for division in ToolUnits::ALL {
                        if tools.in_units(division).next().is_none() {
                            continue;
                        }
                        ui.weak(division.label());
                        for t in tools.in_units(division) {
                            if ui
                                .selectable_label(
                                    t.number == c.tool,
                                    format!(
                                        "T{} {} ({} Ø{})",
                                        t.number,
                                        t.name,
                                        t.shape.label(),
                                        crate::tools_ui::fmt_tool_len(t, t.diameter)
                                    ),
                                )
                                .clicked()
                            {
                                picked = Some(t.clone());
                            }
                        }
                    }
                });
            ui.end_row();
            if let Some(t) = picked {
                c.tool = t.number;
                c.feeds = warp_cam::Feeds::from_tool_data(&t.cutting);
                c.stepdown = t.cutting.stepdown;
            }
            ui.label("");
            if ui.small_button("Reload feeds from tool").clicked() {
                if let Some(t) = tools.tool(c.tool) {
                    c.feeds = warp_cam::Feeds::from_tool_data(&t.cutting);
                    c.stepdown = t.cutting.stepdown;
                }
            }
            ui.end_row();

            ui.strong("Heights");
            ui.end_row();
            mm(ui, "Clearance Z", &mut c.heights.clearance);
            mm(ui, "Retract Z", &mut c.heights.retract);
            mm(ui, "Feed height (above top)", &mut c.heights.feed_height);
            mm(ui, "Top Z", &mut c.heights.top);
            mm(ui, "Bottom Z", &mut c.heights.bottom);
            mm(ui, "Stepdown", &mut c.stepdown);

            ui.strong("Feeds & speeds");
            ui.end_row();
            num(ui, "Spindle", &mut c.feeds.spindle_rpm, 50.0, " rpm");
            ui.label("Spindle direction");
            ui.horizontal(|ui| {
                ui.radio_value(&mut c.feeds.clockwise, true, "CW (M3)");
                ui.radio_value(&mut c.feeds.clockwise, false, "CCW (M4)");
            });
            ui.end_row();
            feed(ui, "Cut feed", &mut c.feeds.feed);
            feed(ui, "Plunge feed", &mut c.feeds.plunge);
            feed(ui, "Ramp feed", &mut c.feeds.ramp);
            ui.label("Coolant");
            egui::ComboBox::from_id_salt("coolant")
                .selected_text(format!("{:?}", c.feeds.coolant))
                .show_ui(ui, |ui| {
                    for k in [Coolant::Off, Coolant::Flood, Coolant::Mist, Coolant::Air] {
                        ui.selectable_value(&mut c.feeds.coolant, k, format!("{k:?}"));
                    }
                });
            ui.end_row();

            ui.strong("Cutting");
            ui.end_row();
            mm(ui, "Stock to leave (walls)", &mut c.stock_xy);
            mm(ui, "Stock to leave (floors)", &mut c.stock_z);
            ui.label("Direction");
            ui.horizontal(|ui| {
                ui.radio_value(&mut c.climb, true, "Climb");
                ui.radio_value(&mut c.climb, false, "Conventional");
            });
            ui.end_row();
            ui.label("Entry");
            ui.horizontal(|ui| {
                let mut ramp = matches!(c.entry, Entry::Ramp { .. });
                ui.radio_value(&mut ramp, false, "Plunge");
                ui.radio_value(&mut ramp, true, "Ramp");
                match (&mut c.entry, ramp) {
                    (Entry::Ramp { angle }, true) => {
                        ui.add(
                            DragValue::new(angle)
                                .speed(0.1)
                                .range(0.1..=45.0)
                                .suffix("°"),
                        );
                    }
                    (_, true) => c.entry = Entry::Ramp { angle: 3.0 },
                    (_, false) => c.entry = Entry::Plunge,
                }
            });
            ui.end_row();
            let mut helix = c.helix_entry.is_some();
            ui.label("Pocket helix entry");
            ui.checkbox(&mut helix, "");
            ui.end_row();
            if helix && c.helix_entry.is_none() {
                c.helix_entry = Some(warp_cam::ops::HelixEntry {
                    radius: 1.0,
                    pitch: 0.5,
                });
            }
            if !helix {
                c.helix_entry = None;
            }
            if let Some(e) = &mut c.helix_entry {
                mm(ui, "Helix radius", &mut e.radius);
                mm(ui, "Helix pitch", &mut e.pitch);
            }
            ui.label("Tolerance");
            units::len_speed(ui, &mut c.tolerance, 0.001);
            ui.end_row();
            if is_3d {
                ui.label("Surface sampling");
                ui.horizontal(|ui| {
                    units::len_speed(ui, &mut c.resolution, 0.01);
                    let d = default_resolution_for(kind);
                    if (c.resolution - d).abs() > 1e-9 && ui.small_button("Default").on_hover_text(format!("{}", units::fmt_len(d))).clicked() {
                        c.resolution = d;
                    }
                });
                ui.end_row();
                ui.label("Surface queries");
                egui::ComboBox::from_id_salt("surface_mode")
                    .selected_text(c.surface.label())
                    .show_ui(ui, |ui| {
                        for m in [SurfaceMode::Auto, SurfaceMode::Exact, SurfaceMode::ZMap] {
                            ui.selectable_value(&mut c.surface, m, m.label());
                        }
                    });
                ui.end_row();
                if c.surface.resolve(kind) == SurfaceMode::ZMap {
                    if let Some(tool) = tools.tool(c.tool) {
                        let w = ZMapWork::estimate(tool, c.resolution, c.stock_xy, extent);
                        ui.label("Z-map work");
                        let text = if w.flat {
                            format!("{} cells × {} rows", group(w.cells), w.rows)
                        } else {
                            format!("{} cells × {} samples", group(w.cells), group(w.samples))
                        };
                        let level = if w.visits > 2e11 {
                            "very slow"
                        } else if w.visits > 2e10 {
                            "slow"
                        } else {
                            "fast"
                        };
                        ui.label(format!("{text} ({level})")).on_hover_text(
                            "Predicted drop-cutter cost: the padded grid size times the footprint samples under                              the tool. Doubling the sampling step divides it by about 16 for ball, bull-nose and                              V tools and by 8 for flat tools. Roughing with stock to leave does not need a fine step.",
                        );
                        ui.end_row();
                    }
                }
            }
        });
}

pub fn edit_kind(ui: &mut Ui, k: &mut OpKind) {
    egui::Grid::new("kind")
        .num_columns(2)
        .striped(true)
        .show(ui, |ui| match k {
            OpKind::OpenPocket { stepover } => {
                mm(ui, "Stepover", stepover);
                ui.label("Select closed extents or open walls ending on stock edges. Clear the left side of each wall; reverse the chain to change sides.");
            }
            OpKind::FaceTop { stepover, angle } => {
                mm(ui, "Stepover", stepover);
                ui.label("Raster angle");
                ui.add(DragValue::new(angle).suffix("°"));
                ui.end_row();
            }
            OpKind::Roughing2D {
                stepover,
                inside_out,
            } => {
                mm(ui, "Stepover", stepover);
                ui.label("Inside out");
                ui.checkbox(inside_out, "");
                ui.end_row();
            }
            OpKind::HolePocket { stepover, pitch } => {
                mm(ui, "Radial stepover", stepover);
                mm(ui, "Helix pitch", pitch);
            }
            OpKind::HoleProfile { pitch, spring_pass } => {
                mm(ui, "Helix pitch", pitch);
                ui.label("Spring pass");
                ui.checkbox(spring_pass, "");
                ui.end_row();
            }
            OpKind::Slot => {
                ui.label("Centreline slot width equals tool diameter");
                ui.end_row();
            }
            OpKind::Facing {
                stepover,
                angle,
                overlap,
                zigzag,
            } => {
                mm(ui, "Stepover", stepover);
                num(ui, "Angle", angle, 1.0, "°");
                mm(ui, "Overlap past stock", overlap);
                ui.label("Zigzag");
                ui.checkbox(zigzag, "");
                ui.end_row();
            }
            OpKind::Profile {
                side,
                finish_passes,
                finish_stepover,
                spring_pass,
                lead_radius,
                tabs,
            } => {
                side_combo(ui, "pside", side);
                ui.label("Finish passes");
                ui.add(DragValue::new(finish_passes).range(0..=20));
                ui.end_row();
                mm(ui, "Finish stepover", finish_stepover);
                ui.label("Spring pass");
                ui.checkbox(spring_pass, "");
                ui.end_row();
                mm(ui, "Lead-in/out arc radius", lead_radius);
                ui.label("Tabs");
                let mut on = tabs.is_some();
                ui.checkbox(&mut on, "");
                ui.end_row();
                if on && tabs.is_none() {
                    *tabs = Some(Tabs {
                        count: 4,
                        width: 6.0,
                        height: 1.5,
                    });
                } else if !on {
                    *tabs = None;
                }
                if let Some(t) = tabs {
                    ui.label("  Tab count");
                    ui.add(DragValue::new(&mut t.count).range(1..=100));
                    ui.end_row();
                    mm(ui, "  Tab width", &mut t.width);
                    mm(ui, "  Tab height", &mut t.height);
                }
            }
            OpKind::Pocket {
                pattern,
                stepover,
                finish_pass,
                inside_out,
                order,
            } => {
                ui.label("Multiple pockets");
                ui.horizontal(|ui| {
                    ui.radio_value(order, CutOrder::LayerFirst, "Layer first")
                        .on_hover_text("Cut all pockets one stepdown at a time");
                    ui.radio_value(order, CutOrder::DepthFirst, "Depth first")
                        .on_hover_text("Finish each pocket to full depth before the next");
                });
                ui.end_row();
                ui.label("Pattern");
                ui.horizontal(|ui| {
                    let mut zig = matches!(pattern, PocketPattern::ZigZag { .. });
                    ui.radio_value(&mut zig, false, "Offset");
                    ui.radio_value(&mut zig, true, "Zigzag");
                    match (&mut *pattern, zig) {
                        (PocketPattern::ZigZag { angle }, true) => {
                            ui.add(DragValue::new(angle).suffix("°"));
                        }
                        (_, true) => *pattern = PocketPattern::ZigZag { angle: 0.0 },
                        (_, false) => *pattern = PocketPattern::Offset,
                    }
                });
                ui.end_row();
                mm(ui, "Stepover", stepover);
                ui.label("Finish pass");
                ui.checkbox(finish_pass, "");
                ui.end_row();
                ui.label("Start inside, cut outward");
                ui.checkbox(inside_out, "");
                ui.end_row();
            }
            OpKind::Drill {
                cycle,
                peck,
                dwell,
                source,
                tip_compensation,
                breakthrough,
            } => {
                ui.label("Cycle");
                egui::ComboBox::from_id_salt("cycle")
                    .selected_text(format!("{cycle:?}"))
                    .show_ui(ui, |ui| {
                        for (c, n) in [
                            (DrillCycle::Simple, "Simple (G81)"),
                            (DrillCycle::Dwell, "Dwell (G82)"),
                            (DrillCycle::Peck, "Peck (G83)"),
                            (DrillCycle::ChipBreak, "Chip break (G73)"),
                            (DrillCycle::BoreFeedOut, "Bore (G85)"),
                            (DrillCycle::BoreDwellFeedOut, "Bore + dwell (G89)"),
                        ] {
                            ui.selectable_value(cycle, c, n);
                        }
                    });
                ui.end_row();
                mm(ui, "Peck depth", peck);
                num(ui, "Dwell", dwell, 0.05, " s");
                ui.label("Tip compensation");
                ui.checkbox(tip_compensation, "full diameter reaches bottom");
                ui.end_row();
                mm(ui, "Breakthrough", breakthrough);
                match source {
                    DrillSource::CircleCenters {
                        min_diameter,
                        max_diameter,
                    } => {
                        mm(ui, "Min hole Ø", min_diameter);
                        mm(ui, "Max hole Ø", max_diameter);
                    }
                    DrillSource::CurveStarts => {}
                }
                ui.label("Points from");
                ui.horizontal(|ui| {
                    let mut circ = matches!(source, DrillSource::CircleCenters { .. });
                    ui.radio_value(&mut circ, true, "Circle centres");
                    ui.radio_value(&mut circ, false, "Curve starts");
                    if circ && !matches!(source, DrillSource::CircleCenters { .. }) {
                        *source = DrillSource::CircleCenters {
                            min_diameter: 0.0,
                            max_diameter: 1000.0,
                        };
                    } else if !circ {
                        *source = DrillSource::CurveStarts;
                    }
                });
                ui.end_row();
            }
            OpKind::Engrave => {
                ui.label("Follows selected curves at each depth level.");
                ui.end_row();
            }
            OpKind::VCarve { max_depth, step } => {
                mm(ui, "Max depth", max_depth);
                mm(ui, "Resolution", step);
            }
            OpKind::HelicalBore {
                min_diameter,
                max_diameter,
                pitch,
                stepover,
            } => {
                mm(ui, "Min hole Ø", min_diameter);
                mm(ui, "Max hole Ø", max_diameter);
                mm(ui, "Helix pitch", pitch);
                mm(ui, "Radial stepover", stepover);
            }
            OpKind::ThreadMill {
                pitch,
                internal,
                major_diameter,
                right_hand,
            } => {
                mm(ui, "Pitch", pitch);
                mm(ui, "Major Ø (0 = circle Ø)", major_diameter);
                ui.label("Type");
                ui.horizontal(|ui| {
                    ui.radio_value(internal, true, "Internal");
                    ui.radio_value(internal, false, "External");
                });
                ui.end_row();
                ui.label("Hand");
                ui.horizontal(|ui| {
                    ui.radio_value(right_hand, true, "Right");
                    ui.radio_value(right_hand, false, "Left");
                });
                ui.end_row();
            }
            OpKind::Chamfer {
                width,
                extra_depth,
                side,
            } => {
                mm(ui, "Chamfer width", width);
                mm(ui, "Extra tip depth", extra_depth);
                side_combo(ui, "cside", side);
            }
            OpKind::Roughing3D {
                stepover,
                finish_contour,
            } => {
                mm(ui, "Stepover", stepover);
                ui.label("Contour pass per level");
                ui.checkbox(finish_contour, "");
                ui.end_row();
            }
            OpKind::Parallel {
                stepover,
                angle,
                zigzag,
            } => {
                mm(ui, "Stepover", stepover);
                num(ui, "Angle", angle, 1.0, "°");
                ui.label("Zigzag");
                ui.checkbox(zigzag, "");
                ui.end_row();
            }
            OpKind::Waterline { stepdown } => {
                mm(ui, "Z step", stepdown);
            }
            OpKind::Spiral { stepover } => {
                mm(ui, "Stepover", stepover);
            }
            OpKind::Radial { angle_step } => {
                num(ui, "Angle step", angle_step, 0.1, "°");
            }
            OpKind::ProjectCurves { depth } => {
                mm(ui, "Depth below surface", depth);
            }
        });
}

impl WarpApp {
    pub fn cam_tab(&mut self, ui: &mut Ui) {
        let ctx = ui.ctx().clone();
        egui::Panel::left("cam_left")
            .default_size(300.0)
            .resizable(true)
            .show(ui, |ui| {
                egui::ScrollArea::vertical().show(ui, |ui| {
                    self.setup_ui(ui);

                    ui.horizontal(|ui| {
                        ui.label("Project");
                        ui.text_edit_singleline(&mut self.project.name);
                    });
                    egui::CollapsingHeader::new("Stock")
                        .default_open(true)
                        .show(ui, |ui| {
                            let s = &mut self.project.stock;
                            egui::Grid::new("stock").num_columns(4).show(ui, |ui| {
                                ui.label("");
                                ui.label("X");
                                ui.label("Y");
                                ui.label("Z");
                                ui.end_row();
                                ui.label("Min");
                                for v in [&mut s.min.x, &mut s.min.y, &mut s.min.z] {
                                    units::len_speed(ui, v, 0.5);
                                }
                                ui.end_row();
                                ui.label("Max");
                                for v in [&mut s.max.x, &mut s.max.y, &mut s.max.z] {
                                    units::len_speed(ui, v, 0.5);
                                }
                                ui.end_row();
                            });
                            if ui.button("Fit to geometry (+5 mm margin)").clicked() {
                                self.project.fit_stock(5.0);
                            }
                        });
                    egui::CollapsingHeader::new("Geometry browser")
                        .default_open(true)
                        .show(ui, |ui| {
                            self.geometry_browser(ui);
                        });

                    ui.separator();
                    ui.heading("Operations in active setup");
                    ui.menu_button("➕ Add operation", |ui| {
                        for kind in OpKind::all_defaults() {
                            let label = format!(
                                "{}{}",
                                if kind.is_3d() { "3D · " } else { "2D · " },
                                kind.label()
                            );
                            if ui.button(label).clicked() {
                                let tool = self
                                    .tools
                                    .tools
                                    .iter()
                                    .find(|t| {
                                        matches!(
                                            (&kind, t.shape),
                                            (
                                                OpKind::Drill { .. },
                                                warp_tools::ToolShape::Drill { .. }
                                            ) | (
                                                OpKind::VCarve { .. } | OpKind::Chamfer { .. },
                                                warp_tools::ToolShape::VBit { .. }
                                            )
                                        )
                                    })
                                    .or(self.tools.tools.first())
                                    .cloned();
                                let Some(tool) = tool else { continue };
                                let mut common = Common::for_tool(&tool);
                                common.resolution = default_resolution_for(&kind);
                                common.heights.top = self.project.stock.max.z;
                                common.heights.bottom = if kind.is_3d() {
                                    self.project.stock.min.z
                                } else {
                                    self.project.stock.max.z - 3.0
                                };
                                common.heights.retract = self.project.stock.max.z + 5.0;
                                common.heights.clearance = self.project.stock.max.z + 15.0;
                                common.setup = self.project.active_setup.clone();
                                common.id = warp_core::scene::OperationId(
                                    self.project.fresh_id("operation"),
                                );
                                let mut geometry = warp_core::selection::ControlGeometry::default();
                                let role = warp_cam::document::primary_role(&kind);
                                if !self.preselection.is_empty() {
                                    *geometry.get_mut(role) = warp_core::selection::Selection {
                                        mode: warp_core::selection::SelectionMode::Selected,
                                        references: self.preselection.clone(),
                                    };
                                }
                                common.geometry = Some(geometry);
                                self.selection_role = role;
                                let n = self.project.operations.len() + 1;
                                self.project.operations.push(Operation {
                                    name: format!("{n}. {}", kind.label()),
                                    enabled: true,
                                    common,
                                    kind,
                                });
                                self.selected_op = Some(self.project.operations.len() - 1);
                                self.sync_paths_len();
                            }
                        }
                    });
                    self.sync_paths_len();
                    let mut action: Option<(&str, usize)> = None;
                    for i in 0..self.project.operations.len() {
                        let op = &mut self.project.operations[i];
                        if op.common.setup != self.project.active_setup {
                            continue;
                        }
                        ui.horizontal(|ui| {
                            ui.checkbox(&mut op.enabled, "");
                            let state = match &self.paths[i] {
                                None => "○",
                                Some(Ok(tp)) if tp.warnings.is_empty() => "●",
                                Some(Ok(_)) | Some(Err(_)) => "⚠",
                            };
                            if ui
                                .selectable_label(
                                    self.selected_op == Some(i),
                                    format!("{state} {}  T{}", op.name, op.common.tool),
                                )
                                .clicked()
                            {
                                self.selected_op = Some(i);
                                self.selection_role = warp_cam::document::primary_role(&op.kind);
                            }
                            ui.with_layout(
                                egui::Layout::right_to_left(egui::Align::Center),
                                |ui| {
                                    if ui.small_button("🗑").on_hover_text("Delete").clicked() {
                                        action = Some(("del", i));
                                    }
                                    if ui.small_button("⧉").on_hover_text("Duplicate").clicked() {
                                        action = Some(("dup", i));
                                    }
                                    if ui.small_button("⏷").clicked() {
                                        action = Some(("down", i));
                                    }
                                    if ui.small_button("⏶").clicked() {
                                        action = Some(("up", i));
                                    }
                                },
                            );
                        });
                        match &self.paths[i] {
                            Some(Err(e)) => {
                                ui.colored_label(egui::Color32::LIGHT_RED, e);
                            }
                            Some(Ok(tp)) => {
                                for w in &tp.warnings {
                                    ui.colored_label(WARNING, w);
                                }
                            }
                            None => {}
                        }
                    }
                    if let Some((a, i)) = action {
                        let duplicate_id =
                            warp_core::scene::OperationId(self.project.fresh_id("operation"));
                        let ops = &mut self.project.operations;
                        match a {
                            "del" => {
                                ops.remove(i);
                                self.paths.remove(i);
                                self.selected_op = None;
                            }
                            "dup" => {
                                let mut o = ops[i].clone();
                                o.name += " copy";
                                o.common.id = duplicate_id;
                                ops.insert(i + 1, o);
                                self.paths.insert(i + 1, None);
                                self.selected_op = Some(i + 1);
                            }
                            "up" if i > 0 => {
                                ops.swap(i, i - 1);
                                self.paths.swap(i, i - 1);
                                self.selected_op = Some(i - 1);
                            }
                            "down" if i + 1 < ops.len() => {
                                ops.swap(i, i + 1);
                                self.paths.swap(i, i + 1);
                                self.selected_op = Some(i + 1);
                            }
                            _ => {}
                        }
                    }
                    ui.separator();
                    ui.horizontal(|ui| {
                        if ui.button("Cancel jobs").clicked() {
                            self.cancel_jobs();
                        }
                        if ui.button("⚙ Generate all").clicked() {
                            self.generate_all(&ctx);
                        }
                    });
                    ui.horizontal(|ui| {
                        ui.label("Post");
                        egui::ComboBox::from_id_salt("post")
                            .selected_text(self.post_config().name.clone())
                            .show_ui(ui, |ui| {
                                for (i, p) in self.posts.iter().enumerate() {
                                    if ui
                                        .selectable_value(&mut self.post_idx, i, &p.name)
                                        .clicked()
                                    {
                                        self.project.post = p.name.clone();
                                    }
                                }
                            });
                    });
                    ui.horizontal(|ui| {
                        if ui.button("💾 Export G-code…").clicked() {
                            self.export_gcode();
                        }
                        if ui.button("👁 Preview G-code").clicked() {
                            self.gcode_preview = self.post_program();
                        }
                    });
                    if ui.button("➡ Send to machine").clicked() {
                        let code = self.post_program();
                        if !code.is_empty() {
                            self.machine.send(warp_machine::Command::Load {
                                name: self.project.name.clone(),
                                gcode: code,
                            });
                            self.tab = Tab::Machine;
                        }
                    }
                    egui::CollapsingHeader::new("Post-processor settings").show(ui, |ui| {
                        let p = &mut self.posts[self.post_idx];
                        egui::Grid::new("postcfg").num_columns(2).show(ui, |ui| {
                            ui.label("Output inches");
                            ui.checkbox(&mut p.inches, "");
                            ui.end_row();
                            ui.label("Decimals");
                            ui.add(DragValue::new(&mut p.decimals).range(3..=8));
                            ui.end_row();
                            ui.label("Line numbers");
                            ui.checkbox(&mut p.line_numbers, "");
                            ui.end_row();
                            ui.label("Arcs (G2/G3)");
                            ui.checkbox(&mut p.arcs, "");
                            ui.end_row();
                            ui.label("Canned drill cycles");
                            ui.checkbox(&mut p.canned_cycles, "");
                            ui.end_row();
                            ui.label("Tool change");
                            ui.text_edit_singleline(&mut p.tool_change);
                            ui.end_row();
                            ui.label("Tool length comp (G43)");
                            ui.checkbox(&mut p.tool_length_comp, "");
                            ui.end_row();
                            ui.label("Spindle delay (s)");
                            ui.add(DragValue::new(&mut p.spindle_delay).speed(0.1));
                            ui.end_row();
                            ui.label("Program end");
                            ui.text_edit_singleline(&mut p.program_end);
                            ui.end_row();
                        });
                        ui.label("Preamble (one line each)");
                        let mut pre = p.preamble.join("\n");
                        if ui.text_edit_multiline(&mut pre).changed() {
                            p.preamble = pre.lines().map(str::to_string).collect();
                        }
                    });
                });
            });

        egui::Panel::right("cam_right")
            .default_size(340.0)
            .resizable(true)
            .show(ui, |ui| {
                egui::ScrollArea::vertical().show(ui, |ui| {
                    let Some(i) = self
                        .selected_op
                        .filter(|&i| i < self.project.operations.len())
                    else {
                        ui.label("Select or add an operation to edit its parameters.");
                        if !self.gcode_preview.is_empty() {
                            ui.separator();
                            self.preview_ui(ui);
                        }
                        return;
                    };
                    self.control_geometry_ui(ui);
                    let before = self.project.operations[i].clone();
                    {
                        let op = &mut self.project.operations[i];
                        ui.horizontal(|ui| {
                            ui.label("Name");
                            ui.text_edit_singleline(&mut op.name);
                        });
                        ui.heading(op.kind.label());
                        edit_kind(ui, &mut op.kind);
                        ui.separator();
                        let extent = (self.project.stock.max - self.project.stock.min).truncate();
                        let kind = op.kind.clone();
                        edit_common(ui, &mut op.common, &self.tools, &kind, extent);
                    }
                    if self.project.operations[i] != before {
                        // Parameters changed: the old toolpath is stale.
                        self.paths[i] = None;
                    }
                    ui.separator();
                    if ui.button("⚙ Generate").clicked() {
                        self.generate(i, &ctx);
                    }
                    if let Some(Ok(tp)) = &self.paths[i] {
                        let time = self
                            .path_times
                            .get(&self.project.operations[i].common.id)
                            .map(|s| format!(", generated in {s:.1} s"))
                            .unwrap_or_default();
                        ui.label(format!(
                            "{} moves, {} cutting{time}",
                            tp.moves.len(),
                            units::fmt_len(tp.cut_length())
                        ));
                        for w in &tp.warnings {
                            ui.colored_label(WARNING, format!("⚠ {w}"));
                        }
                    }
                    if !self.gcode_preview.is_empty() {
                        ui.separator();
                        self.preview_ui(ui);
                    }
                });
            });

        egui::CentralPanel::default().show(ui, |ui| self.scene(ui, false));
        self.geometry_dialog(&ctx);
    }

    fn preview_ui(&mut self, ui: &mut Ui) {
        ui.horizontal(|ui| {
            ui.strong("G-code preview");
            if ui.small_button("Close").clicked() {
                self.gcode_preview.clear();
            }
        });
        egui::ScrollArea::vertical()
            .id_salt("gprev")
            .max_height(400.0)
            .show(ui, |ui| {
                ui.add(
                    egui::TextEdit::multiline(&mut self.gcode_preview.as_str())
                        .code_editor()
                        .desired_width(f32::INFINITY),
                );
            });
    }
}
