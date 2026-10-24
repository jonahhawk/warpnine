//! CNC Drive UC100 USB motion controller, through CNC Drive's native `UC100.dll`.
//!
//! The DLL ships in `C:\UCCNC\API\DLL` with a UCCNC install and needs no licence. Every signature
//! and struct below is transcribed from the vendor's `API\Interface\UC100.h`, cross-checked
//! against `UC100_wrapper.cs`: cdecl calling convention, C++ `bool` = 1 byte, natural alignment.
//! `UC100.dll` is 64-bit (`UC100_x86.dll` is the 32-bit build).
//!
//! The controller does its own trajectory planning: we configure the axes in mm (steps/mm,
//! mm/s, mm/s²), then stream absolute linear moves tagged with the source line number.
//! A board ID <= 0 opens CNC Drive's built-in demo controller (no hardware), which is how the
//! tests below exercise these bindings.

use crate::config::{MachineConfig, AXES};
use crate::device::{DeviceStatus, MotionDevice};
use anyhow::{bail, Context, Result};
use std::collections::VecDeque;
use std::path::{Path, PathBuf};

/// Return codes (`enum ReturnVal`).
fn describe(code: i32) -> &'static str {
    match code {
        0 => "ok",
        1 => "device not opened",
        2 => "device not found",
        3 => "firmware error",
        4 => "USB communication error",
        5 => "motion in progress",
        6 => "homing in progress",
        7 => "controller is in e-stop/reset",
        8 => "limit input active",
        9 => "parameter error",
        10 => "command buffer full",
        11 => "firmware update in progress",
        12 => "probe input active",
        13 => "no laser data",
        14 => "timeout",
        _ => "unknown error",
    }
}

const OK: i32 = 0;
const ESTOP: i32 = 7;
const BUFFER_FULL: i32 = 10;
/// `DeviceType::UC100`.
const TYPE_UC100: i32 = 1;
/// The DLL has six axes (X Y Z A B C); WarpNine drives the first `AXES`.
const DLL_AXES: usize = 6;
/// Moves we hold locally while the controller's own buffer is full.
const LOCAL_QUEUE: usize = 64;

#[repr(C)]
#[derive(Debug, Clone, Copy, Default)]
pub struct AxisSetting {
    pub axis: i32,
    pub enable: bool,
    pub step_pin: i32,
    pub dir_pin: i32,
    pub step_neg: bool,
    pub dir_neg: bool,
    /// Units/s²
    pub max_accel: f64,
    /// Units/s
    pub max_vel: f64,
    /// Steps per unit.
    pub step_per: f64,
    pub home_pin: i32,
    pub home_neg: bool,
    pub limit_p_pin: i32,
    pub limit_p_neg: bool,
    pub limit_n_pin: i32,
    pub limit_n_neg: bool,
    pub soft_limit_p: f64,
    pub soft_limit_n: f64,
    pub slave_axis: i32,
    pub backlash_on: bool,
    pub backlash_dist: f64,
    pub comp_accel: f64,
    pub enable_pin: i32,
    pub enable_pin_neg: bool,
    pub enable_delay: i32,
    pub current_hi_low_pin: i32,
    pub current_hi_low_pin_neg: bool,
    pub home_back_off: f64,
    pub rotary_axis: bool,
    pub rotary_rollover: bool,
}

#[repr(C)]
#[derive(Debug, Clone, Copy, Default)]
pub struct Stat {
    pub idle: bool,
    pub jog: bool,
    pub dwell: bool,
    pub backlash: bool,
    pub home: bool,
    pub probe: bool,
    pub estop: bool,
    pub soft_limit: bool,
    pub hard_limit: bool,
    pub puffer: i32,
    /// Units/s
    pub feed: f64,
    pub spindle_rpm: f64,
    pub current_id: i32,
    pub limit_override: bool,
    pub programmed_feed: f64,
    pub mpg1_jog_on: bool,
    pub mpg2_jog_on: bool,
    pub thc_on_waiting: bool,
    pub sync_thread: bool,
    pub spindle_on: bool,
    pub spindle_dir: bool,
    pub laser_running: bool,
    pub l_data_valid: bool,
    pub thc_on: bool,
    pub thc_anti_dive: bool,
    pub thc_anti_dive_enable: bool,
    pub thc_delay_enable: bool,
    pub thc_anti_down_enable: bool,
    pub probe_active: bool,
}

#[repr(C)]
#[derive(Debug, Clone, Copy, Default)]
pub struct TrajParam {
    pub stop_angle: f64,
    pub constant_vel: bool,
    pub units: bool,
    pub linear_err: f64,
    pub max_linear_unification_length: f64,
    pub max_linear_add_length: f64,
    pub corner_error: f64,
    /// Communication buffer length in seconds (0.05–1).
    pub puffer_size: f64,
}

#[repr(C)]
#[derive(Debug, Clone, Copy, Default)]
pub struct Versio {
    pub fw_version: i32,
    pub hw_version: i32,
    pub api_version: i32,
    pub serial_number: i32,
    pub device_type: i32,
}

type D = f64;

macro_rules! api {
    ($($name:ident : fn($($arg:ty),*);)*) => {
        /// Exports WarpNine uses.
        pub const REQUIRED_EXPORTS: &[&str] = &[$(stringify!($name)),*];

        // GetAxisSetting is only used by the layout test.
        #[allow(non_snake_case, dead_code)]
        struct Api {
            $($name: unsafe extern "C" fn($($arg),*) -> i32,)*
            _lib: libloading::Library,
            _ftdi: Option<libloading::Library>,
        }

        impl Api {
            fn load(path: &Path) -> Result<Api> {
                let ftdi = load_ftdi(path);
                let lib = load_lib(path)?;
                // SAFETY: each type matches the declaration in the vendor's UC100.h.
                unsafe {
                    Ok(Api {
                        $($name: *lib.get(concat!(stringify!($name), "\0").as_bytes())
                            .with_context(|| format!("{} has no export {}", path.display(), stringify!($name)))?,)*
                        _lib: lib,
                        _ftdi: ftdi,
                    })
                }
            }
        }
    };
}

api! {
    ListDevices: fn(*mut i32);
    DeviceInfo: fn(i32, *mut i32, *mut i32);
    Open: fn(i32);
    Close: fn();
    GetVersio: fn(*mut Versio);
    SetAxisSetting: fn(*const AxisSetting);
    GetAxisSetting: fn(*mut AxisSetting);
    SetTrajParam: fn(*const TrajParam);
    GetTrajParam: fn(*mut TrajParam);
    SetEstopSetting: fn(i32, bool);
    SetChargePumpSetting: fn(i32, bool, i32, bool, bool);
    SetSoftLimit: fn(bool);
    SetEstopState: fn();
    Stop: fn();
    GetStatus: fn(*mut Stat);
    GetAxisPosition: fn(*mut D, *mut D, *mut D, *mut D, *mut D, *mut D);
    SetAxisPosition: fn(D, D, D, D, D, D);
    AddLinearMove: fn(D, D, D, D, D, D, D, i32);
    JogOnSpeed: fn(i32, bool, D);
    JogOff: fn(i32);
    SetFeedHold: fn(bool);
    GetInput: fn(*mut u64);
    SetOutputBit: fn(i32);
    ClearOutputBit: fn(i32);
}

pub fn default_dll_locations() -> Vec<PathBuf> {
    // Next to warpnine.exe first, for a portable copy (e.g. on a USB stick).
    let exe_dir = std::env::current_exe().ok().and_then(|p| p.parent().map(|d| d.join("UC100.dll")));
    exe_dir
        .into_iter()
        .chain(["C:\\UCCNC\\API\\DLL\\UC100.dll", "C:\\UCCNC\\API\\UC100.dll", "C:\\UCCNC\\UC100.dll"].iter().map(PathBuf::from))
        .collect()
}

pub fn find_dll(configured: Option<&Path>) -> Option<PathBuf> {
    configured.map(Path::to_path_buf).filter(|p| p.exists()).or_else(|| default_dll_locations().into_iter().find(|p| p.exists()))
}

/// UC100.dll imports `FTD2XX.dll` (FTDI's USB library). The FTDI driver installs it into
/// System32, but UCCNC also keeps a copy next to UCCNC.exe, which isn't on the DLL search path
/// for other programs. Loading it first by full path lets UC100.dll's import resolve to it.
fn load_ftdi(uc100: &Path) -> Option<libloading::Library> {
    let mut dirs: Vec<PathBuf> = uc100.ancestors().skip(1).take(3).map(Path::to_path_buf).collect();
    dirs.push("C:\\UCCNC".into());
    // The system copy (from the installed FTDI driver) wins when present.
    // SAFETY: FTDI's library; loading it only runs its DllMain.
    if let Ok(l) = unsafe { libloading::Library::new("ftd2xx.dll") } {
        return Some(l);
    }
    dirs.iter().map(|d| d.join("ftd2xx.dll")).filter(|p| p.exists()).find_map(|p| unsafe { libloading::Library::new(&p) }.ok())
}

fn load_lib(path: &Path) -> Result<libloading::Library> {
    // SAFETY: loading runs the vendor DLL's DllMain; the user installed it via UCCNC.
    unsafe { libloading::Library::new(path) }.with_context(|| {
        format!(
            "loading {} (it needs 64-bit FTD2XX.dll from the FTDI USB driver or C:\\UCCNC, \
             and the Visual C++ 2008 x64 runtime)",
            path.display()
        )
    })
}

/// Load the DLL and report which required exports are present.
pub fn probe(path: &Path) -> Result<Vec<(&'static str, bool)>> {
    let _ftdi = load_ftdi(path);
    let lib = load_lib(path)?;
    Ok(REQUIRED_EXPORTS
        .iter()
        .map(|name| {
            let sym = format!("{name}\0");
            // SAFETY: only resolving the address, never calling it.
            let found = unsafe { lib.get::<*const ()>(sym.as_bytes()) }.is_ok();
            (*name, found)
        })
        .collect())
}

#[derive(Default)]
pub struct Uc100Device {
    api: Option<Api>,
    /// mm/min used for G0 moves.
    rapid: f64,
    /// Max velocity per axis (mm/min), for converting jog speeds to the DLL's percentage.
    max_vel: [f64; AXES],
    /// Moves refused with "buffer full", retried on the next poll.
    pending: VecDeque<([f64; AXES], f64, i32)>,
    jogging: bool,
    /// Board is in its reset (e-stop) state; motion commands are refused.
    reset: bool,
}

impl Uc100Device {
    fn api(&self) -> Result<&Api> {
        self.api.as_ref().context("UC100 not connected")
    }

    fn call(&self, what: &str, f: impl FnOnce(&Api) -> i32) -> Result<()> {
        let code = f(self.api()?);
        // While in reset every call reports ESTOP but still completes; we track reset ourselves.
        if code != OK && code != ESTOP {
            bail!("UC100 {what}: {} (code {code})", describe(code));
        }
        Ok(())
    }

    /// Send pending moves until the controller's buffer fills.
    fn flush(&mut self) -> Result<()> {
        while let Some(&(t, feed, id)) = self.pending.front() {
            let api = self.api()?;
            // SAFETY: plain value arguments, signature from UC100.h.
            let code = unsafe { (api.AddLinearMove)(t[0], t[1], t[2], t[3], 0.0, 0.0, feed / 60.0, id) };
            match code {
                OK => {
                    self.pending.pop_front();
                }
                BUFFER_FULL => break,
                c => {
                    self.pending.clear();
                    bail!("UC100 AddLinearMove: {} (code {c})", describe(c));
                }
            }
        }
        Ok(())
    }

    fn configure(&self, cfg: &MachineConfig) -> Result<()> {
        for i in 0..DLL_AXES {
            let mut s = AxisSetting { axis: i as i32, ..Default::default() };
            if let Some(a) = cfg.axes.get(i) {
                s.enable = a.enabled;
                s.step_pin = a.step_pin as i32;
                s.dir_pin = a.dir_pin as i32;
                s.dir_neg = a.invert_direction;
                s.max_accel = a.acceleration;
                s.max_vel = a.max_velocity / 60.0;
                s.step_per = a.steps_per_unit;
                s.home_pin = a.home_pin.unwrap_or(0) as i32;
                // WarpNine checks soft limits itself before streaming; the board's are disabled below.
                s.soft_limit_p = a.max;
                s.soft_limit_n = a.min;
                s.comp_accel = a.acceleration;
                s.rotary_axis = i >= 3;
            }
            self.call("SetAxisSetting", |api| unsafe { (api.SetAxisSetting)(&s) })?;
        }
        self.call("SetSoftLimit", |api| unsafe { (api.SetSoftLimit)(false) })?;
        let (pin, neg) = (cfg.estop_pin.unwrap_or(0) as i32, cfg.estop_active_low);
        self.call("SetEstopSetting", |api| unsafe { (api.SetEstopSetting)(pin, neg) })?;
        if let Some(p) = cfg.charge_pump_pin {
            self.call("SetChargePumpSetting", |api| unsafe { (api.SetChargePumpSetting)(p as i32, false, 0, false, false) })?;
        }
        // Constant-velocity blending through small direction changes; exact stop at sharp corners.
        let mut tp = TrajParam::default();
        self.call("GetTrajParam", |api| unsafe { (api.GetTrajParam)(&mut tp) })?;
        tp.constant_vel = true;
        tp.stop_angle = 30.0;
        tp.corner_error = cfg.arc_tolerance.max(0.001);
        tp.linear_err = cfg.arc_tolerance.max(0.001);
        if tp.puffer_size <= 0.0 {
            tp.puffer_size = 0.2;
        }
        self.call("SetTrajParam", |api| unsafe { (api.SetTrajParam)(&tp) })?;
        Ok(())
    }

    /// Open board `board` (<= 0 = CNC Drive's demo controller) and configure it from `cfg`.
    pub fn open(&mut self, cfg: &MachineConfig, board: i32) -> Result<()> {
        let path = find_dll(cfg.uc100_dll.as_deref())
            .context("UC100.dll not found; install UCCNC from cncdrive.com or set its path in Machine setup")?;
        let api = Api::load(&path)?;
        // SAFETY (all calls in this file): argument types and pointer targets match UC100.h;
        // out-pointers point at live, correctly laid out locals.
        if board > 0 {
            let mut count = 0;
            let c = unsafe { (api.ListDevices)(&mut count) };
            if c != OK || count < board {
                bail!("No UC100 found on USB (devices: {count}). Check the cable and that the FTDI driver is installed.");
            }
            let (mut kind, mut serial) = (0, 0);
            let c = unsafe { (api.DeviceInfo)(board, &mut kind, &mut serial) };
            if c != OK {
                bail!("UC100 DeviceInfo: {}", describe(c));
            }
            if kind != TYPE_UC100 {
                bail!("device {board} is a CNC Drive board of type {kind}, not a UC100");
            }
            log::info!("UC100 serial {serial:X}");
        }
        let c = unsafe { (api.Open)(board) };
        if c != OK {
            bail!("UC100 Open: {} (code {c}). Close UCCNC or Mach3 if either is using the board.", describe(c));
        }
        self.api = Some(api);
        if let Err(e) = self.configure(cfg) {
            self.disconnect();
            return Err(e);
        }
        self.rapid = cfg.rapid_rate;
        self.max_vel = std::array::from_fn(|i| cfg.axes[i].max_velocity.max(1.0));
        self.pending.clear();
        Ok(())
    }

    pub fn version(&self) -> Result<Versio> {
        let mut v = Versio::default();
        self.call("GetVersio", |api| unsafe { (api.GetVersio)(&mut v) })?;
        Ok(v)
    }
}

impl Drop for Uc100Device {
    fn drop(&mut self) {
        self.disconnect();
    }
}

impl MotionDevice for Uc100Device {
    fn name(&self) -> String {
        "UC100".into()
    }

    fn connect(&mut self, cfg: &MachineConfig) -> Result<()> {
        self.open(cfg, if cfg.uc100_demo { 0 } else { 1 })?;
        // Start in reset, like Mach3/UCCNC: the operator releases it with Reset.
        self.set_estop(true)
    }

    fn starts_in_estop(&self) -> bool {
        true
    }

    fn disconnect(&mut self) {
        if let Some(api) = self.api.take() {
            unsafe {
                (api.Stop)();
                (api.Close)();
            }
        }
        self.pending.clear();
    }

    fn poll(&mut self) -> Result<DeviceStatus> {
        self.flush()?;
        let mut st = Stat::default();
        self.call("GetStatus", |api| unsafe { (api.GetStatus)(&mut st) })?;
        let mut p = [0.0f64; DLL_AXES];
        self.call("GetAxisPosition", |api| unsafe {
            let [x, y, z, a, b, c] = &mut p;
            (api.GetAxisPosition)(x, y, z, a, b, c)
        })?;
        let mut inputs = 0u64;
        self.call("GetInput", |api| unsafe { (api.GetInput)(&mut inputs) })?;
        if st.hard_limit {
            bail!("hardware limit switch active");
        }
        Ok(DeviceStatus {
            pos: std::array::from_fn(|i| p[i]),
            moving: !st.idle || !self.pending.is_empty() || self.jogging,
            buffer_free: LOCAL_QUEUE.saturating_sub(self.pending.len()),
            current_id: st.current_id.max(0) as u64,
            estop: st.estop,
            inputs,
        })
    }

    fn queue_linear(&mut self, target: [f64; AXES], feed: f64, rapid: bool, id: u64) -> Result<()> {
        if self.reset {
            bail!("UC100 is in reset: release E-stop first");
        }
        let feed = if rapid { self.rapid } else { feed };
        self.pending.push_back((target, feed.max(0.1), id.min(i32::MAX as u64) as i32));
        self.flush()
    }

    fn stop(&mut self) -> Result<()> {
        self.pending.clear();
        self.jogging = false;
        self.call("Stop", |api| unsafe { (api.Stop)() })
    }

    fn feed_hold(&mut self, hold: bool) -> Result<()> {
        self.call("SetFeedHold", |api| unsafe { (api.SetFeedHold)(hold) })
    }

    fn jog(&mut self, axis: usize, velocity: f64) -> Result<()> {
        if velocity == 0.0 {
            self.jogging = false;
            return self.call("JogOff", |api| unsafe { (api.JogOff)(axis as i32) });
        }
        if self.reset {
            bail!("UC100 is in reset: release E-stop first");
        }
        // The DLL takes jog speed as 1–100 % of the axis' max velocity, and Dir = true is the
        // negative direction (checked against the demo controller in the test below).
        let pct = (velocity.abs() / self.max_vel[axis] * 100.0).clamp(1.0, 100.0);
        self.jogging = true;
        self.call("JogOnSpeed", |api| unsafe { (api.JogOnSpeed)(axis as i32, velocity < 0.0, pct) })
    }

    fn set_estop(&mut self, on: bool) -> Result<()> {
        self.pending.clear();
        self.jogging = false;
        if on {
            self.reset = true;
            self.call("SetEstopState", |api| unsafe { (api.SetEstopState)() })
        } else {
            // Stop() clears the reset condition (UC100.h: "can be cleared with the Stop() function").
            let code = unsafe { (self.api()?.Stop)() };
            if code != OK {
                bail!("UC100 could not leave reset: {} (code {code}). Check the e-stop input.", describe(code));
            }
            self.reset = false;
            Ok(())
        }
    }

    fn set_output(&mut self, pin: u32, on: bool) -> Result<()> {
        if on {
            self.call("SetOutputBit", |api| unsafe { (api.SetOutputBit)(pin as i32) })
        } else {
            self.call("ClearOutputBit", |api| unsafe { (api.ClearOutputBit)(pin as i32) })
        }
    }

    fn set_position(&mut self, pos: [f64; AXES]) -> Result<()> {
        self.call("SetAxisPosition", |api| unsafe { (api.SetAxisPosition)(pos[0], pos[1], pos[2], pos[3], 0.0, 0.0) })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::{Duration, Instant};

    /// Runs CNC Drive's demo controller through the real DLL (skipped if UCCNC isn't installed).
    #[test]
    fn demo_controller_moves() {
        let cfg = MachineConfig { uc100_demo: true, ..MachineConfig::g540_uc100() };
        if find_dll(None).is_none() {
            eprintln!("UC100.dll not installed; skipping");
            return;
        }
        let mut d = Uc100Device::default();
        d.connect(&cfg).unwrap();
        let v = d.version().unwrap();
        eprintln!("version {v:?}");
        // Struct layout check: read back what `configure` wrote.
        let mut y = AxisSetting { axis: 1, ..Default::default() };
        d.call("GetAxisSetting", |api| unsafe { (api.GetAxisSetting)(&mut y) }).unwrap();
        eprintln!("Y axis {y:?}");
        assert!(y.enable && y.step_pin == 4 && y.dir_pin == 5 && y.step_per == 400.0, "{y:?}");
        assert!((y.max_vel - 2500.0 / 60.0).abs() < 1e-9 && y.max_accel == 200.0 && y.soft_limit_p == 400.0, "{y:?}");
        assert!(d.queue_linear([1.0, 0.0, 0.0, 0.0], 600.0, false, 1).is_err(), "moves must be refused while in reset");
        d.set_estop(false).unwrap();
        d.set_position([0.0; AXES]).unwrap();
        d.queue_linear([5.0, -3.0, -1.0, 0.0], 1200.0, false, 7).unwrap();
        let t0 = Instant::now();
        let mut seen_id = false;
        let s = loop {
            let s = d.poll().unwrap();
            seen_id |= s.current_id == 7;
            if !s.moving && t0.elapsed() > Duration::from_millis(200) {
                break s;
            }
            assert!(t0.elapsed() < Duration::from_secs(10), "move never finished: {s:?}");
            std::thread::sleep(Duration::from_millis(20));
        };
        eprintln!("final {s:?} seen_id={seen_id}");
        for (got, want) in s.pos.iter().zip([5.0, -3.0, -1.0, 0.0]) {
            assert!((got - want).abs() < 0.01, "{:?}", s.pos);
        }
        // Jog +X then -X briefly; position must follow the sign.
        let mut x = 5.0;
        for v in [600.0, -600.0] {
            d.jog(0, v).unwrap();
            std::thread::sleep(Duration::from_millis(300));
            d.jog(0, 0.0).unwrap();
            std::thread::sleep(Duration::from_millis(400));
            let nx = d.poll().unwrap().pos[0];
            eprintln!("jog {v}: {x} -> {nx}");
            assert!((nx - x) * v > 0.0, "jog {v} moved {x} -> {nx}");
            x = nx;
        }
        d.disconnect();
    }
}
