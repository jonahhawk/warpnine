//! Machine tab: DRO, jogging, MDI, program run controls, and machine setup.

use crate::app::WarpApp;
use crate::units;
use eframe::egui::{self, Color32, DragValue, RichText, Ui};
use warp_machine::{Command, DeviceKind, MachineConfig, RunState, AXES, AXIS_NAMES};

impl WarpApp {
    pub fn machine_tab(&mut self, ui: &mut Ui) {
        let snap = self.machine.snapshot();
        let connected = !matches!(snap.run, RunState::Disconnected);

        egui::Panel::left("machine_left").default_size(380.0).resizable(true).show(ui, |ui| {
            egui::ScrollArea::vertical().show(ui, |ui| {
                // ---- connection & E-stop ----
                ui.horizontal(|ui| {
                    if connected {
                        if ui.button("Disconnect").clicked() {
                            self.machine.send(Command::Disconnect);
                        }
                        ui.label(RichText::new(format!("● {}", snap.device)).color(Color32::GREEN));
                    } else {
                        if ui.button("Connect").clicked() {
                            self.machine.send(Command::SetConfig(Box::new(self.machine_cfg.clone())));
                            self.machine.send(Command::Connect);
                        }
                        ui.label(RichText::new("○ disconnected").color(Color32::GRAY));
                    }
                });
                let estop = matches!(snap.run, RunState::EStop);
                let estop_btn = egui::Button::new(RichText::new(if estop { "RESET E-STOP" } else { "E-STOP" }).size(22.0).strong().color(Color32::WHITE))
                    .fill(if estop { Color32::from_rgb(120, 90, 0) } else { Color32::from_rgb(190, 20, 20) })
                    .min_size(egui::vec2(ui.available_width(), 48.0));
                if ui.add(estop_btn).clicked() {
                    self.machine.send(Command::EStop(!estop));
                }
                let state_text = match &snap.run {
                    RunState::Paused(why) => format!("PAUSED — {why}"),
                    RunState::Alarm(a) => format!("ALARM — {a}"),
                    s => format!("{s:?}").to_uppercase(),
                };
                ui.label(RichText::new(state_text).strong());

                // ---- DRO ----
                ui.separator();
                let work = snap.work_pos();
                egui::Grid::new("dro").num_columns(4).spacing([12.0, 6.0]).show(ui, |ui| {
                    ui.label("");
                    ui.label(format!("Work (G{})", 54 + snap.wcs_index));
                    ui.label("Machine");
                    ui.label("");
                    ui.end_row();
                    for a in 0..AXES {
                        if !self.machine_cfg.axes[a].enabled {
                            continue;
                        }
                        ui.label(RichText::new(AXIS_NAMES[a]).size(22.0).strong());
                        ui.label(RichText::new(dro(work[a], a)).size(22.0).monospace().color(Color32::from_rgb(90, 220, 120)));
                        ui.label(RichText::new(dro(snap.status.pos[a], a)).monospace());
                        if ui.button(format!("Zero {}", AXIS_NAMES[a])).clicked() {
                            self.machine.send(Command::ZeroWork(Some(a)));
                        }
                        ui.end_row();
                    }
                });
                ui.horizontal(|ui| {
                    if ui.button("Zero all").clicked() {
                        self.machine.send(Command::ZeroWork(None));
                    }
                    if ui.button("Go to work zero").clicked() {
                        // Retract to machine Z0 (top of travel once homed) before moving in XY.
                        self.machine.send(Command::Mdi("G53 G0 Z0\nG0 X0 Y0".into()));
                    }
                    if ui.button("Set machine zero").on_hover_text("Declare current position as machine 0 (manual homing)").clicked() {
                        self.machine.send(Command::SetMachineZero);
                    }
                });
                ui.horizontal(|ui| {
                    ui.label("Work offset");
                    for i in 0..6 {
                        if ui.selectable_label(snap.wcs_index == i, format!("G{}", 54 + i)).clicked() {
                            self.machine.send(Command::SelectWcs(i));
                        }
                    }
                });
                ui.label(format!("Tool T{}   Spindle {:?} {:.0} rpm   Coolant {}", snap.tool, snap.spindle, snap.rpm, if snap.coolant.flood { "flood" } else if snap.coolant.mist { "mist" } else { "off" }));

                // ---- jog ----
                ui.separator();
                ui.strong("Jog");
                ui.horizontal(|ui| {
                    ui.label("Step");
                    ui.selectable_value(&mut self.jog_step, 0.0, "Cont.");
                    // Steps are stored in mm; inch setups get inch-friendly increments.
                    let steps: Vec<f64> = if units::inches() { [0.0001, 0.001, 0.01, 0.1].iter().map(|s| s * 25.4).collect() } else { self.machine_cfg.jog_steps.clone() };
                    for s in steps {
                        let label = format!("{}", (s / units::factor() * 1e6).round() / 1e6);
                        ui.selectable_value(&mut self.jog_step, s, label);
                    }
                    ui.label(units::len_suffix());
                });
                ui.horizontal(|ui| {
                    ui.label("Speed");
                    units::feed(ui, &mut self.jog_speed);
                    self.jog_speed = self.jog_speed.clamp(1.0, 20_000.0);
                });
                let mut want: Option<(usize, f64)> = None;
                egui::Grid::new("jog").num_columns(3).show(ui, |ui| {
                    let jog_btn = |ui: &mut Ui, label: &str, axis: usize, dir: f64, want: &mut Option<(usize, f64)>| {
                        let r = ui.add(egui::Button::new(RichText::new(label).size(16.0)).min_size(egui::vec2(64.0, 40.0)));
                        if self.jog_step > 0.0 {
                            if r.clicked() {
                                self.machine.send(Command::JogStep { axis, distance: dir * self.jog_step, feed: self.jog_speed });
                            }
                        } else if r.is_pointer_button_down_on() {
                            *want = Some((axis, dir * self.jog_speed));
                        }
                    };
                    ui.label("");
                    jog_btn(ui, "Y+", 1, 1.0, &mut want);
                    jog_btn(ui, "Z+", 2, 1.0, &mut want);
                    ui.end_row();
                    jog_btn(ui, "X−", 0, -1.0, &mut want);
                    ui.label("");
                    jog_btn(ui, "X+", 0, 1.0, &mut want);
                    ui.end_row();
                    ui.label("");
                    jog_btn(ui, "Y−", 1, -1.0, &mut want);
                    jog_btn(ui, "Z−", 2, -1.0, &mut want);
                    ui.end_row();
                    if self.machine_cfg.axes[3].enabled {
                        jog_btn(ui, "A−", 3, -1.0, &mut want);
                        ui.label("");
                        jog_btn(ui, "A+", 3, 1.0, &mut want);
                        ui.end_row();
                    }
                });
                // Keyboard jog (arrows = XY, PgUp/PgDn = Z) when no text field has focus.
                if self.jog_step == 0.0 && !ui.ctx().egui_wants_keyboard_input() {
                    ui.input(|i| {
                        let k = |key| i.key_down(key);
                        use egui::Key::*;
                        let keys = [(ArrowRight, 0, 1.0), (ArrowLeft, 0, -1.0), (ArrowUp, 1, 1.0), (ArrowDown, 1, -1.0), (PageUp, 2, 1.0), (PageDown, 2, -1.0)];
                        for (key, axis, dir) in keys {
                            if k(key) {
                                want = Some((axis, dir * self.jog_speed));
                            }
                        }
                    });
                }
                if want != self.jog_active {
                    if let Some((axis, _)) = self.jog_active {
                        self.machine.send(Command::Jog { axis, velocity: 0.0 });
                    }
                    if let Some((axis, v)) = want {
                        self.machine.send(Command::Jog { axis, velocity: v });
                    }
                    self.jog_active = want;
                }

                // ---- MDI ----
                ui.separator();
                ui.strong("MDI");
                ui.horizontal(|ui| {
                    let r = ui.add(egui::TextEdit::singleline(&mut self.mdi).hint_text("G0 X0 Y0").desired_width(240.0));
                    let enter = r.lost_focus() && ui.input(|i| i.key_pressed(egui::Key::Enter));
                    if (ui.button("Send").clicked() || enter) && !self.mdi.trim().is_empty() {
                        self.machine.send(Command::Mdi(std::mem::take(&mut self.mdi)));
                        r.request_focus();
                    }
                });

                // ---- setup ----
                ui.separator();
                egui::CollapsingHeader::new("Machine setup").show(ui, |ui| self.machine_setup(ui, connected));
            });
        });

        egui::Panel::bottom("machine_log").default_size(140.0).resizable(true).show(ui, |ui| {
            egui::ScrollArea::vertical().stick_to_bottom(true).show(ui, |ui| {
                for l in &snap.log {
                    ui.label(RichText::new(l).monospace().small());
                }
            });
        });

        egui::CentralPanel::default().show(ui, |ui| {
            ui.horizontal(|ui| {
                ui.heading(if snap.program_name.is_empty() { "No program".to_string() } else { snap.program_name.clone() });
                if ui.button("📂 Load G-code…").clicked() {
                    if let Some(p) = rfd::FileDialog::new().add_filter("G-code", &["nc", "tap", "ngc", "gcode", "cnc", "txt"]).pick_file() {
                        match std::fs::read_to_string(&p) {
                            Ok(code) => self.machine.send(Command::Load { name: p.file_name().unwrap_or_default().to_string_lossy().into(), gcode: code }),
                            Err(e) => self.status = format!("{e}"),
                        }
                    }
                }
            });
            ui.horizontal(|ui| {
                let big = |t: &str, c: Color32| egui::Button::new(RichText::new(t).size(18.0).strong()).fill(c).min_size(egui::vec2(140.0, 44.0));
                if ui.add(big("▶ Cycle start", Color32::from_rgb(20, 120, 40))).clicked() {
                    self.machine.send(Command::CycleStart);
                }
                if ui.add(big("⏸ Feed hold", Color32::from_rgb(160, 120, 0))).clicked() {
                    self.machine.send(Command::FeedHold);
                }
                if ui.add(big("■ Stop", Color32::from_rgb(110, 30, 30))).clicked() {
                    self.machine.send(Command::Stop);
                }
            });
            ui.horizontal(|ui| {
                let mut ovr = snap.feed_override * 100.0;
                ui.label("Feed override");
                if ui.add(egui::Slider::new(&mut ovr, 10.0..=200.0).suffix(" %")).changed() {
                    self.machine.send(Command::FeedOverride(ovr / 100.0));
                }
                ui.label("(applies to moves not yet buffered)");
            });
            ui.horizontal(|ui| {
                ui.label(format!("Line {} / {}", snap.line, snap.program.len()));
                if snap.line > 0 && matches!(snap.run, RunState::Idle) && ui.button("Run from this line").clicked() {
                    self.machine.send(Command::RunFromLine(snap.line));
                }
            });
            ui.separator();
            let row_h = ui.text_style_height(&egui::TextStyle::Monospace);
            let n = snap.program.len();
            egui::ScrollArea::vertical().auto_shrink(false).show_rows(ui, row_h, n, |ui, range| {
                for i in range {
                    let line_no = i + 1;
                    let text = format!("{line_no:>6}  {}", snap.program[i]);
                    let rt = RichText::new(text).monospace();
                    if line_no == snap.line {
                        ui.label(rt.background_color(Color32::from_rgb(40, 80, 140)).color(Color32::WHITE));
                    } else {
                        ui.label(rt);
                    }
                }
            });
        });
    }

    fn machine_setup(&mut self, ui: &mut Ui, connected: bool) {
        let c = &mut self.machine_cfg;
        ui.horizontal(|ui| {
            ui.label("Name");
            ui.text_edit_singleline(&mut c.name);
        });
        ui.horizontal(|ui| {
            ui.label("Device");
            ui.radio_value(&mut c.device, DeviceKind::Simulator, "Simulator");
            ui.radio_value(&mut c.device, DeviceKind::Uc100, "UC100 (USB)");
        });
        if c.device == DeviceKind::Uc100 {
            ui.horizontal(|ui| {
                ui.label("UC100.dll");
                let mut s = c.uc100_dll.as_ref().map(|p| p.display().to_string()).unwrap_or_default();
                if ui.text_edit_singleline(&mut s).changed() {
                    c.uc100_dll = (!s.is_empty()).then(|| s.into());
                }
                if ui.button("…").clicked() {
                    if let Some(p) = rfd::FileDialog::new().add_filter("DLL", &["dll"]).pick_file() {
                        c.uc100_dll = Some(p);
                    }
                }
            });
            #[cfg(windows)]
            if ui.button("Probe DLL").clicked() {
                self.uc100_probe = Some(match warp_machine::uc100::find_dll(c.uc100_dll.as_deref()) {
                    None => "UC100.dll not found. Install UCCNC (cncdrive.com) or browse to the DLL.".into(),
                    Some(p) => match warp_machine::uc100::probe(&p) {
                        Ok(r) => {
                            let missing: Vec<_> = r.iter().filter(|x| !x.1).map(|x| x.0).collect();
                            if missing.is_empty() {
                                format!("{}\n✔ loaded, all {} required functions present", p.display(), r.len())
                            } else {
                                format!("{}\n✘ missing: {}", p.display(), missing.join(", "))
                            }
                        }
                        Err(e) => format!("{e:#}"),
                    },
                });
            }
            if let Some(p) = &self.uc100_probe {
                ui.label(RichText::new(p).monospace().small());
            }
            ui.checkbox(&mut c.uc100_demo, "Demo mode (CNC Drive's simulated controller, no board needed)");
            ui.label(
                RichText::new("Close UCCNC/Mach3 first: only one program can use the board. After Connect the board is in reset; press RESET E-STOP to enable motion.")
                    .small(),
            );
        }
        ui.horizontal(|ui| {
            if ui.button("Preset: G540 + UC100").clicked() {
                *c = MachineConfig::g540_uc100();
            }
            if ui.button("Preset: Simulator").clicked() {
                *c = MachineConfig::simulator();
            }
        });
        ui.label("Axes");
        egui::ScrollArea::horizontal().show(ui, |ui| {
            egui::Grid::new("axes").striped(true).num_columns(9).show(ui, |ui| {
                let u = units::len_suffix().trim();
                for h in ["".to_string(), "On".into(), format!("Steps/{u}"), "Vmax".into(), format!("Accel {u}/s²"), "Invert".into(), "Step/Dir pin".into(), "Soft limits".into(), "Min / Max".into()] {
                    ui.strong(h);
                }
                ui.end_row();
                for (i, a) in c.axes.iter_mut().enumerate() {
                    ui.label(AXIS_NAMES[i]);
                    ui.checkbox(&mut a.enabled, "");
                    let rotary = i == 3; // A axis is in degrees regardless of units
                    if rotary {
                        ui.add(DragValue::new(&mut a.steps_per_unit).speed(0.1).suffix("/°"));
                        ui.add(DragValue::new(&mut a.max_velocity).speed(10.0).suffix(" °/min"));
                        ui.add(DragValue::new(&mut a.acceleration).speed(5.0).suffix(" °/s²"));
                    } else {
                        units::per_len(ui, &mut a.steps_per_unit);
                        units::feed(ui, &mut a.max_velocity);
                        units::len_speed(ui, &mut a.acceleration, 5.0);
                    }
                    ui.checkbox(&mut a.invert_direction, "");
                    ui.horizontal(|ui| {
                        ui.add(DragValue::new(&mut a.step_pin).range(1..=17));
                        ui.add(DragValue::new(&mut a.dir_pin).range(1..=17));
                    });
                    ui.checkbox(&mut a.soft_limits, "");
                    ui.horizontal(|ui| {
                        if rotary {
                            ui.add(DragValue::new(&mut a.min).speed(1.0));
                            ui.add(DragValue::new(&mut a.max).speed(1.0));
                        } else {
                            units::len_speed(ui, &mut a.min, 1.0);
                            units::len_speed(ui, &mut a.max, 1.0);
                        }
                    });
                    ui.end_row();
                }
            });
        });
        let pin = |ui: &mut Ui, label: &str, p: &mut Option<u32>| {
            ui.horizontal(|ui| {
                let mut on = p.is_some();
                ui.checkbox(&mut on, label);
                match (on, p.as_mut()) {
                    (true, Some(v)) => {
                        ui.add(DragValue::new(v).range(1..=17));
                    }
                    (true, None) => *p = Some(1),
                    (false, _) => *p = None,
                }
            });
        };
        pin(ui, "E-stop input pin", &mut c.estop_pin);
        pin(ui, "Spindle CW output pin", &mut c.spindle_cw_pin);
        pin(ui, "Spindle CCW output pin", &mut c.spindle_ccw_pin);
        pin(ui, "Flood output pin", &mut c.flood_pin);
        pin(ui, "Mist output pin", &mut c.mist_pin);
        pin(ui, "Charge pump pin", &mut c.charge_pump_pin);
        ui.horizontal(|ui| {
            ui.label("Rapid rate");
            units::feed(ui, &mut c.rapid_rate);
            ui.label("Arc tolerance");
            units::len_speed(ui, &mut c.arc_tolerance, 0.001);
        });
        ui.horizontal(|ui| {
            if ui.button("Apply").clicked() {
                if connected {
                    self.status = "Disconnect before changing the device".into();
                }
                self.machine.send(Command::SetConfig(Box::new(self.machine_cfg.clone())));
            }
            if ui.button("💾 Save machine config").clicked() {
                let p = MachineConfig::default_path();
                self.status = match self.machine_cfg.save(&p) {
                    Ok(()) => format!("Saved {}", p.display()),
                    Err(e) => format!("Save failed: {e:#}"),
                };
            }
        });
    }
}

/// DRO text for an axis value stored in mm (A axis is degrees and never converted).
fn dro(v: f64, axis: usize) -> String {
    if axis == 3 {
        format!("{v:+10.3}°")
    } else if units::inches() {
        format!("{:+10.4}", v / 25.4)
    } else {
        format!("{v:+10.3}")
    }
}
