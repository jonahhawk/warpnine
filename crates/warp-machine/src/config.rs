use serde::{Deserialize, Serialize};
use std::path::{Path, PathBuf};

pub const AXES: usize = 4;
pub const AXIS_NAMES: [&str; AXES] = ["X", "Y", "Z", "A"];

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct AxisConfig {
    pub enabled: bool,
    /// Motor steps per mm (or per degree for A).
    pub steps_per_unit: f64,
    /// mm/min
    pub max_velocity: f64,
    /// mm/s²
    pub acceleration: f64,
    pub invert_direction: bool,
    /// Parallel-port style pin numbers on the UC100's DB25 connector.
    pub step_pin: u32,
    pub dir_pin: u32,
    pub soft_limits: bool,
    pub min: f64,
    pub max: f64,
    pub home_pin: Option<u32>,
    pub home_to_max: bool,
}

impl AxisConfig {
    fn new(step_pin: u32, dir_pin: u32, min: f64, max: f64) -> Self {
        AxisConfig {
            enabled: true,
            // G540: 10 microsteps × 200 steps/rev = 2000 steps/rev; with a 5 mm pitch screw = 400.
            steps_per_unit: 400.0,
            max_velocity: 2500.0,
            acceleration: 200.0,
            invert_direction: false,
            step_pin,
            dir_pin,
            soft_limits: false,
            min,
            max,
            home_pin: None,
            home_to_max: false,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum DeviceKind {
    Simulator,
    Uc100,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct MachineConfig {
    pub name: String,
    pub device: DeviceKind,
    pub axes: [AxisConfig; AXES],
    /// Rapid traverse rate used for estimates when axes differ (mm/min).
    pub rapid_rate: f64,
    pub estop_pin: Option<u32>,
    pub estop_active_low: bool,
    pub spindle_cw_pin: Option<u32>,
    pub spindle_ccw_pin: Option<u32>,
    pub flood_pin: Option<u32>,
    pub mist_pin: Option<u32>,
    pub charge_pump_pin: Option<u32>,
    pub uc100_dll: Option<PathBuf>,
    /// Use CNC Drive's built-in demo controller instead of the USB board (no hardware needed).
    #[serde(default)]
    pub uc100_demo: bool,
    /// Chord tolerance for linearising arcs before streaming (mm).
    pub arc_tolerance: f64,
    pub jog_speeds: Vec<f64>,
    pub jog_steps: Vec<f64>,
}

impl MachineConfig {
    /// Gecko G540 behind a UC100. Step/dir pins are the G540's fixed DB25 assignment
    /// (X 2/3, Y 4/5, Z 6/7, A 8/9). Inputs/outputs depend on wiring: fill them in from the
    /// G540 manual for your setup. Steps-per-unit assume 5 mm pitch screws; adjust.
    pub fn g540_uc100() -> Self {
        MachineConfig {
            name: "G540 via UC100".into(),
            device: DeviceKind::Uc100,
            axes: [
                AxisConfig::new(2, 3, 0.0, 600.0),
                AxisConfig::new(4, 5, 0.0, 400.0),
                AxisConfig::new(6, 7, -100.0, 0.0),
                AxisConfig { enabled: false, steps_per_unit: 2000.0 / 360.0, ..AxisConfig::new(8, 9, -1e6, 1e6) },
            ],
            rapid_rate: 2500.0,
            estop_pin: None,
            estop_active_low: false,
            spindle_cw_pin: None,
            spindle_ccw_pin: None,
            flood_pin: None,
            mist_pin: None,
            charge_pump_pin: None,
            uc100_dll: None,
            uc100_demo: false,
            arc_tolerance: 0.005,
            jog_speeds: vec![100.0, 500.0, 1500.0, 2500.0],
            jog_steps: vec![0.01, 0.1, 1.0, 10.0],
        }
    }

    pub fn simulator() -> Self {
        MachineConfig { name: "Simulator".into(), device: DeviceKind::Simulator, ..Self::g540_uc100() }
    }

    pub fn default_path() -> PathBuf {
        let base = std::env::var_os("APPDATA")
            .map(PathBuf::from)
            .unwrap_or_else(|| std::env::var_os("HOME").map(PathBuf::from).unwrap_or_default().join(".config"));
        base.join("WarpNine").join("machine.json")
    }

    pub fn load(path: &Path) -> anyhow::Result<Self> {
        Ok(serde_json::from_str(&std::fs::read_to_string(path)?)?)
    }

    pub fn save(&self, path: &Path) -> anyhow::Result<()> {
        if let Some(d) = path.parent() {
            std::fs::create_dir_all(d)?;
        }
        std::fs::write(path, serde_json::to_string_pretty(self)?)?;
        Ok(())
    }
}
