//! Display units. Everything is stored in millimetres; these helpers convert at the UI edge.
//!
//! The active units are kept in a thread-local set once per frame, so every editor uses the
//! project's units without threading a parameter through each widget function.

use eframe::egui::{DragValue, Ui};
use std::cell::Cell;
use warp_core::Units;

thread_local! {
    static UNITS: Cell<Units> = const { Cell::new(Units::Millimeters) };
}

pub fn set(u: Units) {
    UNITS.with(|c| c.set(u));
}

pub fn get() -> Units {
    UNITS.with(|c| c.get())
}

pub fn inches() -> bool {
    get() == Units::Inches
}

/// mm per display unit.
pub fn factor() -> f64 {
    get().to_mm()
}

pub fn len_suffix() -> &'static str {
    if inches() { " in" } else { " mm" }
}

pub fn feed_suffix() -> &'static str {
    if inches() { " in/min" } else { " mm/min" }
}

/// Format a length given in mm.
pub fn fmt_len(mm: f64) -> String {
    if inches() { format!("{:.4} in", mm / 25.4) } else { format!("{mm:.3} mm") }
}

fn drag(ui: &mut Ui, v: &mut f64, speed_mm: f64, suffix: &str, decimals: usize) -> bool {
    let f = factor();
    let mut d = *v / f;
    let changed = ui.add(DragValue::new(&mut d).speed(speed_mm / f).suffix(suffix).max_decimals(decimals)).changed();
    if changed {
        *v = d * f;
    }
    changed
}

/// Drag editor for a length stored in mm.
pub fn len(ui: &mut Ui, v: &mut f64) -> bool {
    drag(ui, v, 0.05, len_suffix(), if inches() { 5 } else { 4 })
}

/// Drag editor for a length stored in mm, with a custom drag speed (mm per pixel).
pub fn len_speed(ui: &mut Ui, v: &mut f64, speed_mm: f64) -> bool {
    drag(ui, v, speed_mm, len_suffix(), if inches() { 5 } else { 4 })
}

/// Drag editor for a feed rate stored in mm/min.
pub fn feed(ui: &mut Ui, v: &mut f64) -> bool {
    drag(ui, v, 5.0, feed_suffix(), if inches() { 2 } else { 1 })
}

/// Drag editor for a "per length" quantity stored per mm (e.g. steps/mm).
pub fn per_len(ui: &mut Ui, v: &mut f64) -> bool {
    let f = factor();
    let mut d = *v * f;
    let changed = ui.add(DragValue::new(&mut d).speed(f)).changed();
    if changed {
        *v = d / f;
    }
    changed
}
