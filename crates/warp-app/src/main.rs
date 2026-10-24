#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")]

mod app;
mod cam_ui;
mod document_ui;
mod machine_ui;
mod nest_ui;
mod settings_ui;
mod tools_ui;
mod units;
mod view3d;

fn main() -> eframe::Result {
    env_logger::init();
    let options = eframe::NativeOptions {
        viewport: eframe::egui::ViewportBuilder::default()
            .with_inner_size([1500.0, 920.0])
            .with_title("WarpNine CAM / CNC"),
        ..Default::default()
    };
    // `warpnine [project.warp|model.3dm|model.step] [--tab cam|nesting|simulate|tools|machine]`
    let args: Vec<String> = std::env::args().skip(1).collect();
    let mut project = None;
    let mut tab = None;
    let mut i = 0;
    while i < args.len() {
        if args[i] == "--tab" && i + 1 < args.len() {
            tab = Some(args[i + 1].to_lowercase());
            i += 2;
        } else {
            project = Some(std::path::PathBuf::from(&args[i]));
            i += 1;
        }
    }
    eframe::run_native(
        "WarpNine",
        options,
        Box::new(move |cc| {
            let mut app = app::WarpApp::new(cc);
            if let Some(p) = project {
                if document_ui::import_kind(&p).is_some() {
                    app.import_model(Some(p));
                } else {
                    app.open_project_path(p);
                }
            }
            app.tab = match tab.as_deref() {
                Some("nesting") | Some("nest") => app::Tab::Nest,
                Some("simulate") => app::Tab::Simulate,
                Some("tools") => app::Tab::Tools,
                Some("machine") => app::Tab::Machine,
                _ => app::Tab::Cam,
            };
            Ok(Box::new(app))
        }),
    )
}
