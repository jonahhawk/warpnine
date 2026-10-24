//! Settings tab: project-level choices that are not operation parameters, starting with the
//! safety checks the user may relax. Everything here is saved in the project file so the
//! toolpaths it produces are reproducible from the file alone.

use crate::app::WarpApp;
use eframe::egui::{self, Ui};

impl WarpApp {
    pub fn settings_tab(&mut self, ui: &mut Ui) {
        egui::CentralPanel::default().show(ui, |ui| {
            ui.heading("Settings");
            ui.label("Saved with the project, so a project reopened later generates the same toolpaths.");
            ui.add_space(12.0);

            ui.strong("Safety checks");
            let before = self.project.checks;
            ui.checkbox(
                &mut self.project.checks.allow_depth_beyond_flute,
                "Allow cut depth beyond the tool's flute length",
            );
            ui.indent("flute_note", |ui| {
                ui.label(
                    "Off: an operation whose depth exceeds the flute length is refused. \
                     On: it generates, and the operation shows a warning naming how far the cut \
                     reaches beyond the flutes, for reduced-shank tools or pockets wider than the \
                     shank. Drill depth includes tip and breakthrough; V-carve is no longer limited \
                     to the flute length. Stickout and holder clearance are not checked.",
                );
            });
            if self.project.checks != before {
                // Refused operations may generate now; operations generated under a relaxed
                // check may be refused now. Either way their results no longer describe the
                // project, so drop them; valid paths are untouched.
                for p in &mut self.paths {
                    let stale = match p {
                        Some(Err(_)) => true,
                        Some(Ok(tp)) => !tp.warnings.is_empty(),
                        None => false,
                    };
                    if stale {
                        *p = None;
                    }
                }
            }
        });
    }
}
