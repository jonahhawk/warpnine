//! Tool library tab: tools, holders, and a profile preview.

use crate::app::WarpApp;
use crate::units;
use eframe::egui::{self, Color32, DragValue, Stroke, Ui};
use warp_tools::{Coolant, Holder, HolderKind, HolderSegment, Tool, ToolLibrary, ToolMaterial, ToolShape, ToolUnits, STARTER_IMPERIAL, STARTER_METRIC};

/// A length of a tool in the tool's own division (inches for imperial tools), regardless of the
/// project's display units.
pub fn fmt_tool_len(t: &Tool, mm: f64) -> String {
    match t.units {
        ToolUnits::Metric => format!("{mm:.3} mm"),
        ToolUnits::Imperial => format!("{:.4} in", mm / 25.4),
    }
}

fn row_num(ui: &mut Ui, label: &str, v: &mut f64, suffix: &str) -> bool {
    ui.label(label);
    let c = ui.add(DragValue::new(v).speed(0.05).suffix(suffix).max_decimals(4)).changed();
    ui.end_row();
    c
}

fn row_len(ui: &mut Ui, label: &str, v: &mut f64) -> bool {
    ui.label(label);
    let c = units::len(ui, v);
    ui.end_row();
    c
}

fn row_feed(ui: &mut Ui, label: &str, v: &mut f64) -> bool {
    ui.label(label);
    let c = units::feed(ui, v);
    ui.end_row();
    c
}

fn row_text(ui: &mut Ui, label: &str, v: &mut String) -> bool {
    ui.label(label);
    let c = ui.text_edit_singleline(v).changed();
    ui.end_row();
    c
}

fn shape_params(ui: &mut Ui, s: &mut ToolShape) -> bool {
    let mut c = false;
    match s {
        ToolShape::BullNose { corner_radius } => c |= row_len(ui, "Corner radius", corner_radius),
        ToolShape::VBit { included_angle, tip_diameter } | ToolShape::Engraver { included_angle, tip_diameter } => {
            c |= row_num(ui, "Included angle", included_angle, "°");
            c |= row_len(ui, "Tip diameter", tip_diameter);
        }
        ToolShape::ChamferMill { angle, tip_diameter } => {
            c |= row_num(ui, "Included angle", angle, "°");
            c |= row_len(ui, "Tip diameter", tip_diameter);
        }
        ToolShape::Drill { point_angle } | ToolShape::SpotDrill { point_angle } => c |= row_num(ui, "Point angle", point_angle, "°"),
        ToolShape::TaperedBall { taper_angle, tip_radius } => {
            c |= row_num(ui, "Taper (per side)", taper_angle, "°");
            c |= row_len(ui, "Tip radius", tip_radius);
        }
        ToolShape::ThreadMill { pitch } | ToolShape::Tap { pitch } => c |= row_len(ui, "Pitch", pitch),
        ToolShape::SlotCutter { thickness } => c |= row_len(ui, "Thickness", thickness),
        ToolShape::FlatEndMill | ToolShape::BallEndMill | ToolShape::FaceMill => {}
    }
    c
}

/// Filled side silhouette of the tool and holder, with key dimensions.
fn draw_profile(ui: &mut Ui, t: &Tool, holder: Option<&Holder>) {
    use crate::view3d::{tool_profile, ToolPart};
    let (rect, _) = ui.allocate_exact_size(egui::vec2(240.0, 320.0), egui::Sense::hover());
    let p = ui.painter_at(rect);
    p.rect_filled(rect, 4.0, ui.visuals().extreme_bg_color);
    let prof = tool_profile(t, holder);
    let height = prof.iter().map(|q| q.1).fold(0.0, f64::max).max(1.0);
    let width = prof.iter().map(|q| q.0 * 2.0).fold(0.0, f64::max).max(1.0);
    let scale = ((rect.height() - 40.0) as f64 / height).min((rect.width() - 60.0) as f64 / width);
    let base = egui::pos2(rect.center().x, rect.bottom() - 24.0);
    let pt = |x: f64, z: f64| egui::pos2(base.x + (x * scale) as f32, base.y - (z * scale) as f32);
    // Each profile step is a convex trapezoid between -r and +r.
    for w in prof.windows(2) {
        let ((r0, z0, _), (r1, z1, part)) = (w[0], w[1]);
        let fill = match part {
            ToolPart::Cutter => Color32::from_rgb(225, 190, 90),
            ToolPart::Shank => Color32::from_rgb(170, 175, 185),
            ToolPart::Holder => Color32::from_rgb(110, 115, 125),
        };
        let quad = vec![pt(-r0, z0), pt(r0, z0), pt(r1, z1), pt(-r1, z1)];
        p.add(egui::Shape::convex_polygon(quad, fill, Stroke::NONE));
    }
    let outline = Stroke::new(1.0, ui.visuals().text_color());
    let right: Vec<_> = prof.iter().map(|q| pt(q.0, q.1)).collect();
    let left: Vec<_> = prof.iter().map(|q| pt(-q.0, q.1)).collect();
    p.add(egui::Shape::line(right, outline));
    p.add(egui::Shape::line(left, outline));
    // Dimensions: cutting diameter at the bottom, flute length on the side.
    let dim = ui.visuals().weak_text_color();
    let font = egui::FontId::proportional(11.0);
    p.text(pt(0.0, 0.0) + egui::vec2(0.0, 4.0), egui::Align2::CENTER_TOP, format!("Ø {}", units::fmt_len(t.diameter)), font.clone(), dim);
    let fl = t.flute_length.max(t.profile_height(t.radius()).unwrap_or(0.0));
    let x = pt(t.radius(), 0.0).x + 8.0;
    p.line_segment([egui::pos2(x, pt(0.0, 0.0).y), egui::pos2(x, pt(0.0, fl).y)], Stroke::new(1.0, dim));
    p.text(egui::pos2(x + 3.0, pt(0.0, fl * 0.5).y), egui::Align2::LEFT_CENTER, units::fmt_len(fl), font, dim);
}

impl WarpApp {
    pub fn tools_tab(&mut self, ui: &mut Ui) {
        egui::Panel::left("tool_list").default_size(340.0).resizable(true).show(ui, |ui| {
            ui.horizontal(|ui| {
                ui.menu_button("➕ New tool", |ui| {
                    for units in ToolUnits::ALL {
                        ui.menu_button(units.label(), |ui| {
                            for shape in ToolShape::all_defaults() {
                                if ui.button(shape.label()).clicked() {
                                    let n = self.tools.next_number();
                                    let mut t = match units {
                                        ToolUnits::Metric => {
                                            let d = if matches!(shape, ToolShape::VBit { .. }) { 12.7 } else { 6.0 };
                                            Tool::new(n, format!("New {}", shape.label()), shape, d)
                                        }
                                        ToolUnits::Imperial => {
                                            let d = if matches!(shape, ToolShape::VBit { .. }) { 0.5 } else { 0.25 };
                                            Tool::imperial(n, format!("New {}", shape.label()), shape, d)
                                        }
                                    };
                                    if let ToolShape::SlotCutter { thickness } = &mut t.shape {
                                        *thickness *= units.to_mm();
                                    }
                                    self.tools.upsert(t);
                                    self.selected_tool = Some(n);
                                    self.tools_dirty = true;
                                    ui.close();
                                }
                            }
                        });
                    }
                });
                ui.menu_button("Starter sets", |ui| {
                    let sets = [
                        ("Metric (T1-T9)", STARTER_METRIC, ToolLibrary::starter_metric as fn() -> ToolLibrary),
                        ("Imperial (T101-T114)", STARTER_IMPERIAL, ToolLibrary::starter_imperial),
                    ];
                    for (label, name, set) in sets {
                        if ui.button(label).on_hover_text("Adds the tools of this set that are missing; tools on a taken number get the next free number").clicked() {
                            let added = self.tools.merge(set());
                            if !self.tools.seeded.iter().any(|s| s == name) {
                                self.tools.seeded.push(name.into());
                                self.tools_dirty = true;
                            }
                            self.status = if added.is_empty() {
                                format!("{label} is already in the library")
                            } else {
                                self.tools_dirty = true;
                                format!("Added {} tools from the {label} starter set", added.len())
                            };
                            ui.close();
                        }
                    }
                });
                if ui.button("💾 Save library").clicked() {
                    let path = ToolLibrary::default_path();
                    self.status = match self.tools.save(&path) {
                        Ok(()) => {
                            self.tools_dirty = false;
                            format!("Saved tool library to {}", path.display())
                        }
                        Err(e) => format!("Save failed: {e:#}"),
                    };
                }
            });
            ui.horizontal(|ui| {
                if ui.button("Import…").clicked() {
                    if let Some(p) = rfd::FileDialog::new().add_filter("Tool library", &["json"]).pick_file() {
                        match ToolLibrary::load(&p) {
                            Ok(lib) => {
                                for t in lib.tools {
                                    self.tools.upsert(t);
                                }
                                for h in lib.holders {
                                    self.tools.upsert_holder(h);
                                }
                                self.tools_dirty = true;
                            }
                            Err(e) => self.status = format!("Import failed: {e:#}"),
                        }
                    }
                }
                if ui.button("Export…").clicked() {
                    if let Some(p) = rfd::FileDialog::new().add_filter("Tool library", &["json"]).set_file_name("tools.json").save_file() {
                        if let Err(e) = self.tools.save(&p) {
                            self.status = format!("Export failed: {e:#}");
                        }
                    }
                }
            });
            if self.tools_dirty {
                ui.colored_label(Color32::YELLOW, "Unsaved changes (saved automatically on exit)");
            }
            ui.separator();
            ui.heading("Tools");
            egui::ScrollArea::vertical().id_salt("tools").max_height(ui.available_height() * 0.6).show(ui, |ui| {
                for division in ToolUnits::ALL {
                    let count = self.tools.in_units(division).count();
                    let header = format!("{} ({count})", division.label());
                    egui::CollapsingHeader::new(header).id_salt(division.label()).default_open(true).show(ui, |ui| {
                        if count == 0 {
                            ui.weak("No tools. Use New tool or Starter sets.");
                            return;
                        }
                        egui::Grid::new(("tooltable", division.label())).striped(true).num_columns(4).show(ui, |ui| {
                            ui.strong("T");
                            ui.strong("Name");
                            ui.strong("Type");
                            ui.strong("Ø");
                            ui.end_row();
                            for t in self.tools.in_units(division) {
                                let sel = self.selected_tool == Some(t.number);
                                if ui.selectable_label(sel, format!("T{}", t.number)).clicked() {
                                    self.selected_tool = Some(t.number);
                                }
                                ui.label(&t.name);
                                ui.label(t.shape.label());
                                ui.label(fmt_tool_len(t, t.diameter));
                                ui.end_row();
                            }
                        });
                    });
                }
            });
            ui.separator();
            ui.horizontal(|ui| {
                ui.heading("Holders");
                if ui.small_button("➕").clicked() {
                    let id = format!("holder{}", self.tools.holders.len() + 1);
                    self.tools.upsert_holder(Holder {
                        id: id.clone(),
                        name: "New holder".into(),
                        kind: HolderKind::Collet,
                        segments: vec![HolderSegment { length: 30.0, lower_diameter: 30.0, upper_diameter: 30.0 }],
                        gauge_length: 30.0,
                        notes: String::new(),
                    });
                    self.selected_holder = Some(id);
                    self.selected_tool = None;
                    self.tools_dirty = true;
                }
            });
            for h in &self.tools.holders {
                if ui.selectable_label(self.selected_holder.as_deref() == Some(&h.id), format!("{} ({:?})", h.name, h.kind)).clicked() {
                    self.selected_holder = Some(h.id.clone());
                    self.selected_tool = None;
                }
            }
        });

        egui::CentralPanel::default().show(ui, |ui| {
            egui::ScrollArea::vertical().show(ui, |ui| {
                if let Some(n) = self.selected_tool {
                    self.tool_editor(ui, n);
                } else if let Some(id) = self.selected_holder.clone() {
                    self.holder_editor(ui, &id);
                } else {
                    ui.label("Select a tool or holder to edit it.");
                }
            });
        });
    }

    fn tool_editor(&mut self, ui: &mut Ui, number: u32) {
        let Some(mut t) = self.tools.tool(number).cloned() else { return };
        let holders: Vec<(String, String)> = self.tools.holders.iter().map(|h| (h.id.clone(), h.name.clone())).collect();
        let mut changed = false;
        let mut delete = false;
        ui.horizontal_top(|ui| {
            ui.vertical(|ui| {
                egui::Grid::new("tooled").num_columns(2).striped(true).show(ui, |ui| {
                    ui.label("Number");
                    let mut num = t.number;
                    if ui.add(DragValue::new(&mut num).range(1..=9999)).changed() && self.tools.tool(num).is_none() {
                        t.number = num;
                        changed = true;
                    }
                    ui.end_row();
                    changed |= row_text(ui, "Name", &mut t.name);
                    ui.label("Division");
                    ui.horizontal(|ui| {
                        for u in ToolUnits::ALL {
                            changed |= ui.radio_value(&mut t.units, u, u.label()).changed();
                        }
                    });
                    ui.end_row();
                    ui.label("Type");
                    egui::ComboBox::from_id_salt("shape").selected_text(t.shape.label()).show_ui(ui, |ui| {
                        for s in ToolShape::all_defaults() {
                            if ui.selectable_label(std::mem::discriminant(&s) == std::mem::discriminant(&t.shape), s.label()).clicked() {
                                t.shape = s;
                                changed = true;
                            }
                        }
                    });
                    ui.end_row();
                    changed |= shape_params(ui, &mut t.shape);
                    ui.strong("Geometry");
                    ui.end_row();
                    changed |= row_len(ui, "Diameter", &mut t.diameter);
                    changed |= row_len(ui, "Shank diameter", &mut t.shank_diameter);
                    changed |= row_len(ui, "Flute length", &mut t.flute_length);
                    changed |= row_len(ui, "Overall length", &mut t.overall_length);
                    changed |= row_len(ui, "Stickout", &mut t.stickout);
                    ui.label("Flutes");
                    changed |= ui.add(DragValue::new(&mut t.flutes).range(1..=16)).changed();
                    ui.end_row();
                    ui.label("Material");
                    egui::ComboBox::from_id_salt("mat").selected_text(format!("{:?}", t.material)).show_ui(ui, |ui| {
                        for m in [ToolMaterial::Carbide, ToolMaterial::Hss, ToolMaterial::Cobalt, ToolMaterial::Diamond, ToolMaterial::Ceramic] {
                            changed |= ui.selectable_value(&mut t.material, m, format!("{m:?}")).changed();
                        }
                    });
                    ui.end_row();
                    changed |= row_text(ui, "Coating", &mut t.coating);
                    changed |= row_text(ui, "Manufacturer", &mut t.manufacturer);
                    changed |= row_text(ui, "Part number", &mut t.part_number);
                    ui.label("Holder");
                    let cur = t.holder.as_ref().and_then(|id| holders.iter().find(|h| &h.0 == id)).map(|h| h.1.clone()).unwrap_or("None".into());
                    egui::ComboBox::from_id_salt("holder").selected_text(cur).show_ui(ui, |ui| {
                        changed |= ui.selectable_value(&mut t.holder, None, "None").changed();
                        for (id, name) in &holders {
                            changed |= ui.selectable_value(&mut t.holder, Some(id.clone()), name).changed();
                        }
                    });
                    ui.end_row();
                    ui.label("Length offset (H)");
                    changed |= ui.add(DragValue::new(&mut t.length_offset)).changed();
                    ui.end_row();
                    ui.label("Diameter offset (D)");
                    changed |= ui.add(DragValue::new(&mut t.diameter_offset)).changed();
                    ui.end_row();

                    ui.strong("Default cutting data");
                    ui.end_row();
                    let c = &mut t.cutting;
                    ui.label("Spindle");
                    changed |= ui.add(DragValue::new(&mut c.spindle_rpm).speed(50.0).suffix(" rpm")).changed();
                    ui.end_row();
                    ui.label("Direction");
                    ui.horizontal(|ui| {
                        changed |= ui.radio_value(&mut c.clockwise, true, "CW").changed();
                        changed |= ui.radio_value(&mut c.clockwise, false, "CCW").changed();
                    });
                    ui.end_row();
                    changed |= row_feed(ui, "Feed", &mut c.feed_rate);
                    changed |= row_feed(ui, "Plunge", &mut c.plunge_rate);
                    changed |= row_feed(ui, "Ramp", &mut c.ramp_rate);
                    changed |= row_len(ui, "Stepdown", &mut c.stepdown);
                    ui.label("Stepover");
                    let mut pct = c.stepover * 100.0;
                    if ui.add(DragValue::new(&mut pct).range(1.0..=100.0).suffix(" %")).changed() {
                        c.stepover = pct / 100.0;
                        changed = true;
                    }
                    ui.end_row();
                    ui.label("Coolant");
                    egui::ComboBox::from_id_salt("tcool").selected_text(format!("{:?}", c.coolant)).show_ui(ui, |ui| {
                        for k in [Coolant::Off, Coolant::Flood, Coolant::Mist, Coolant::Air] {
                            changed |= ui.selectable_value(&mut c.coolant, k, format!("{k:?}")).changed();
                        }
                    });
                    ui.end_row();
                    let chip = c.feed_rate / (c.spindle_rpm.max(1.0) * t.flutes.max(1) as f64);
                    ui.label("Chip load");
                    ui.label(format!("{}/tooth", units::fmt_len(chip)));
                    ui.end_row();
                });
                ui.label("Notes");
                changed |= ui.text_edit_multiline(&mut t.notes).changed();
                ui.horizontal(|ui| {
                    if ui.button("Duplicate").clicked() {
                        let mut d = t.clone();
                        d.number = self.tools.next_number();
                        d.name += " copy";
                        self.selected_tool = Some(d.number);
                        self.tools.upsert(d);
                        self.tools_dirty = true;
                    }
                    if ui.button("🗑 Delete").clicked() {
                        delete = true;
                    }
                });
            });
            let holder = t.holder.as_ref().and_then(|id| self.tools.holder(id)).cloned();
            ui.vertical(|ui| {
                draw_profile(ui, &t, holder.as_ref());
                // Rotatable 3D view (drag to orbit, wheel to zoom).
                let (rect, _) = ui.allocate_exact_size(egui::vec2(240.0, 260.0), egui::Sense::hover());
                let total = crate::view3d::tool_profile(&t, holder.as_ref()).iter().map(|q| q.1).fold(0.0, f64::max).max(1.0);
                if self.tool_cam_fit != Some(t.number) {
                    self.tool_cam.target = glam::DVec3::new(0.0, 0.0, total * 0.5);
                    self.tool_cam.span = total * 1.2;
                    self.tool_cam.pitch = 15f64.to_radians();
                    self.tool_cam_fit = Some(t.number);
                }
                let mut child = ui.new_child(egui::UiBuilder::new().max_rect(rect));
                self.tool_cam.interact(&mut child, rect);
                let pr = self.tool_cam.proj(rect);
                let painter = ui.painter_at(rect);
                painter.rect_filled(rect, 4.0, ui.visuals().extreme_bg_color);
                crate::view3d::tool_solid(&painter, &pr, &t, holder.as_ref(), glam::DVec3::ZERO);
            });
        });
        if delete {
            self.tools.tools.retain(|x| x.number != number);
            self.selected_tool = None;
            self.tools_dirty = true;
        } else if changed {
            self.tools.tools.retain(|x| x.number != number);
            self.selected_tool = Some(t.number);
            self.tools.upsert(t);
            self.tools_dirty = true;
        }
    }

    fn holder_editor(&mut self, ui: &mut Ui, id: &str) {
        let Some(mut h) = self.tools.holder(id).cloned() else { return };
        let mut changed = false;
        egui::Grid::new("holder").num_columns(2).show(ui, |ui| {
            changed |= row_text(ui, "Name", &mut h.name);
            ui.label("Kind");
            egui::ComboBox::from_id_salt("hk").selected_text(format!("{:?}", h.kind)).show_ui(ui, |ui| {
                use HolderKind::*;
                for k in [Collet, Er11, Er16, Er20, Er25, Er32, Er40, Bt30, Bt40, Cat40, Hsk63, Iso20, Iso30, R8, Ttc, ShrinkFit, Hydraulic, EndMillHolder, Router] {
                    changed |= ui.selectable_value(&mut h.kind, k, format!("{k:?}")).changed();
                }
            });
            ui.end_row();
            changed |= row_len(ui, "Gauge length", &mut h.gauge_length);
        });
        ui.strong("Segments (from tool end upward)");
        let mut remove = None;
        egui::Grid::new("segs").num_columns(4).show(ui, |ui| {
            ui.label("Length");
            ui.label("Lower Ø");
            ui.label("Upper Ø");
            ui.label("");
            ui.end_row();
            for (i, s) in h.segments.iter_mut().enumerate() {
                changed |= units::len_speed(ui, &mut s.length, 0.1);
                changed |= units::len_speed(ui, &mut s.lower_diameter, 0.1);
                changed |= units::len_speed(ui, &mut s.upper_diameter, 0.1);
                if ui.small_button("🗑").clicked() {
                    remove = Some(i);
                }
                ui.end_row();
            }
        });
        if let Some(i) = remove {
            h.segments.remove(i);
            changed = true;
        }
        if ui.button("➕ Segment").clicked() {
            let last = h.segments.last().copied().unwrap_or(HolderSegment { length: 10.0, lower_diameter: 30.0, upper_diameter: 30.0 });
            h.segments.push(last);
            changed = true;
        }
        ui.label("Notes");
        changed |= ui.text_edit_multiline(&mut h.notes).changed();
        if ui.button("🗑 Delete holder").clicked() {
            self.tools.holders.retain(|x| x.id != id);
            for t in &mut self.tools.tools {
                if t.holder.as_deref() == Some(id) {
                    t.holder = None;
                }
            }
            self.selected_holder = None;
            self.tools_dirty = true;
            return;
        }
        if changed {
            self.tools.upsert_holder(h);
            self.tools_dirty = true;
        }
    }
}
