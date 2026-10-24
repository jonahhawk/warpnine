//! Machine control thread: owns the motion device, streams programs, handles jog/MDI/E-stop.

use crate::config::{DeviceKind, MachineConfig, AXES, AXIS_NAMES};
use crate::device::{DeviceStatus, MotionDevice, SimDevice};
use crossbeam_channel::{unbounded, Receiver, Sender};
use glam::DVec3;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};
use warp_gcode::{CoolantState, Interpreter, MachineState, MotionKind, SpindleState};

#[derive(Debug, Clone)]
pub enum Command {
    Connect,
    Disconnect,
    SetConfig(Box<MachineConfig>),
    /// Continuous jog, signed mm/min; 0 stops.
    Jog { axis: usize, velocity: f64 },
    /// Incremental jog.
    JogStep { axis: usize, distance: f64, feed: f64 },
    Mdi(String),
    Load { name: String, gcode: String },
    CycleStart,
    FeedHold,
    Stop,
    EStop(bool),
    /// Set the active work offset so the current position reads 0 on `axis` (all if None).
    ZeroWork(Option<usize>),
    SelectWcs(usize),
    /// Declare the current position as machine zero (manual homing).
    SetMachineZero,
    FeedOverride(f64),
    /// Start running from a given source line (1-based).
    RunFromLine(usize),
}

#[derive(Debug, Clone, PartialEq)]
pub enum RunState {
    Disconnected,
    Idle,
    Running,
    /// Waiting for Cycle Start (M0, tool change …) with a reason.
    Paused(String),
    Hold,
    EStop,
    Alarm(String),
}

#[derive(Debug, Clone)]
pub struct Snapshot {
    pub device: String,
    pub run: RunState,
    pub status: DeviceStatus,
    pub wcs: [[f64; AXES]; 6],
    pub wcs_index: usize,
    pub program_name: String,
    pub program: Arc<Vec<String>>,
    pub line: usize,
    pub tool: u32,
    pub spindle: SpindleState,
    pub rpm: f64,
    pub coolant: CoolantState,
    pub feed_override: f64,
    pub log: Vec<String>,
}

impl Snapshot {
    pub fn work_pos(&self) -> [f64; AXES] {
        let w = self.wcs[self.wcs_index];
        std::array::from_fn(|i| self.status.pos[i] - w[i])
    }
}

#[derive(Debug, Clone)]
enum Item {
    Move { target: [f64; AXES], feed: f64, rapid: bool, line: usize },
    Sync { kind: MotionKind, line: usize },
}

pub struct Machine {
    tx: Sender<Command>,
    pub snapshot: Arc<Mutex<Snapshot>>,
}

impl Machine {
    pub fn spawn(cfg: MachineConfig) -> Machine {
        let (tx, rx) = unbounded();
        let snapshot = Arc::new(Mutex::new(Snapshot {
            device: String::new(),
            run: RunState::Disconnected,
            status: DeviceStatus::default(),
            wcs: [[0.0; AXES]; 6],
            wcs_index: 0,
            program_name: String::new(),
            program: Arc::new(vec![]),
            line: 0,
            tool: 0,
            spindle: SpindleState::Off,
            rpm: 0.0,
            coolant: CoolantState::default(),
            feed_override: 1.0,
            log: vec![],
        }));
        let snap = snapshot.clone();
        std::thread::Builder::new()
            .name("machine".into())
            .spawn(move || Worker::new(cfg, rx, snap).run())
            .expect("spawn machine thread");
        Machine { tx, snapshot }
    }

    pub fn send(&self, c: Command) {
        let _ = self.tx.send(c);
    }

    pub fn snapshot(&self) -> Snapshot {
        self.snapshot.lock().unwrap().clone()
    }
}

struct Worker {
    cfg: MachineConfig,
    rx: Receiver<Command>,
    snap: Arc<Mutex<Snapshot>>,
    dev: Option<Box<dyn MotionDevice>>,
    items: Vec<Item>,
    next: usize,
    wait_until: Option<Instant>,
    modal: MachineState,
    status: DeviceStatus,
}

impl Worker {
    fn new(cfg: MachineConfig, rx: Receiver<Command>, snap: Arc<Mutex<Snapshot>>) -> Self {
        Worker { cfg, rx, snap, dev: None, items: vec![], next: 0, wait_until: None, modal: MachineState::default(), status: DeviceStatus::default() }
    }

    fn log(&self, msg: impl Into<String>) {
        let msg = msg.into();
        log::info!("{msg}");
        let mut s = self.snap.lock().unwrap();
        s.log.push(msg);
        let n = s.log.len();
        if n > 500 {
            s.log.drain(..n - 500);
        }
    }

    fn set_run(&self, r: RunState) {
        self.snap.lock().unwrap().run = r;
    }

    fn run_state(&self) -> RunState {
        self.snap.lock().unwrap().run.clone()
    }

    fn run(mut self) {
        loop {
            match self.rx.recv_timeout(Duration::from_millis(10)) {
                Ok(c) => {
                    if let Err(e) = self.handle(c) {
                        self.log(format!("error: {e:#}"));
                    }
                    while let Ok(c) = self.rx.try_recv() {
                        if let Err(e) = self.handle(c) {
                            self.log(format!("error: {e:#}"));
                        }
                    }
                }
                Err(crossbeam_channel::RecvTimeoutError::Timeout) => {}
                Err(crossbeam_channel::RecvTimeoutError::Disconnected) => return,
            }
            if let Err(e) = self.tick() {
                self.log(format!("device error: {e:#}"));
                if let Some(d) = self.dev.as_mut() {
                    let _ = d.stop();
                }
                self.set_run(RunState::Alarm(format!("{e:#}")));
            }
        }
    }

    fn dev(&mut self) -> anyhow::Result<&mut Box<dyn MotionDevice>> {
        self.dev.as_mut().ok_or_else(|| anyhow::anyhow!("not connected"))
    }

    fn idle(&self) -> bool {
        matches!(self.run_state(), RunState::Idle)
    }

    fn handle(&mut self, c: Command) -> anyhow::Result<()> {
        match c {
            Command::Connect => {
                let mut d: Box<dyn MotionDevice> = match self.cfg.device {
                    DeviceKind::Simulator => Box::new(SimDevice::default()),
                    #[cfg(windows)]
                    DeviceKind::Uc100 => Box::new(crate::uc100::Uc100Device::default()),
                    #[cfg(not(windows))]
                    DeviceKind::Uc100 => anyhow::bail!("the UC100 is only supported on Windows"),
                };
                d.connect(&self.cfg)?;
                self.snap.lock().unwrap().device = d.name();
                self.log(format!("connected to {}", d.name()));
                let estop = d.starts_in_estop();
                self.dev = Some(d);
                if estop {
                    self.set_run(RunState::EStop);
                    self.log("controller is in reset: press RESET E-STOP to enable motion");
                } else {
                    self.set_run(RunState::Idle);
                }
            }
            Command::Disconnect => {
                if let Some(mut d) = self.dev.take() {
                    let _ = d.stop();
                    d.disconnect();
                }
                self.items.clear();
                self.set_run(RunState::Disconnected);
            }
            Command::SetConfig(cfg) => {
                anyhow::ensure!(self.dev.is_none() || self.idle(), "stop the machine before changing configuration");
                self.cfg = *cfg;
            }
            Command::Jog { axis, velocity } => {
                if velocity != 0.0 {
                    anyhow::ensure!(self.idle(), "jog only when idle");
                }
                let v = velocity.clamp(-self.cfg.axes[axis].max_velocity, self.cfg.axes[axis].max_velocity);
                self.dev()?.jog(axis, v)?;
            }
            Command::JogStep { axis, distance, feed } => {
                anyhow::ensure!(self.idle(), "jog only when idle");
                let mut t = self.status.pos;
                t[axis] += distance;
                self.check_limits(&t)?;
                self.dev()?.queue_linear(t, feed, false, 0)?;
            }
            Command::Mdi(line) => {
                anyhow::ensure!(self.idle(), "MDI only when idle");
                self.log(format!("MDI> {line}"));
                self.load_items(&line)?;
                self.set_run(RunState::Running);
            }
            Command::Load { name, gcode } => {
                anyhow::ensure!(!matches!(self.run_state(), RunState::Running | RunState::Hold), "stop before loading");
                // Validate now so errors show at load time.
                let mut it = Interpreter::new(self.modal.clone());
                let prog = it.run_str(&gcode)?;
                for w in prog.warnings.iter().take(20) {
                    self.log(format!("warning: {w}"));
                }
                let mut s = self.snap.lock().unwrap();
                s.program_name = name;
                s.program = Arc::new(gcode.lines().map(str::to_string).collect());
                s.line = 0;
            }
            Command::CycleStart | Command::RunFromLine(_) => match self.run_state() {
                RunState::Hold => {
                    self.dev()?.feed_hold(false)?;
                    self.set_run(RunState::Running);
                }
                RunState::Paused(_) => self.set_run(RunState::Running),
                RunState::Idle => {
                    let start_line = if let Command::RunFromLine(l) = c { l } else { 1 };
                    let src = self.snap.lock().unwrap().program.join("\n");
                    anyhow::ensure!(!src.is_empty(), "no program loaded");
                    self.load_items(&src)?;
                    if start_line > 1 {
                        // Skip motion before the start line but keep modal sync items (spindle, tool).
                        let first = self.items.iter().position(|i| matches!(i, Item::Move { line, .. } if *line >= start_line)).unwrap_or(self.items.len());
                        let mut kept: Vec<Item> = self.items[..first].iter().filter(|i| matches!(i, Item::Sync { .. })).cloned().collect();
                        kept.extend_from_slice(&self.items[first..]);
                        self.items = kept;
                    }
                    self.set_run(RunState::Running);
                    self.log("cycle start");
                }
                other => anyhow::bail!("cannot start while {other:?}"),
            },
            Command::FeedHold => {
                if self.run_state() == RunState::Running {
                    self.dev()?.feed_hold(true)?;
                    self.set_run(RunState::Hold);
                }
            }
            Command::Stop => {
                if let Some(d) = self.dev.as_mut() {
                    d.stop()?;
                    d.feed_hold(false)?;
                }
                self.items.clear();
                self.next = 0;
                self.sync_outputs(SpindleState::Off, CoolantState::default())?;
                if self.dev.is_some() && !matches!(self.run_state(), RunState::EStop) {
                    self.set_run(RunState::Idle);
                }
                self.log("stopped");
            }
            Command::EStop(on) => {
                if let Some(d) = self.dev.as_mut() {
                    d.set_estop(on)?;
                }
                if on {
                    self.items.clear();
                    self.set_run(RunState::EStop);
                    self.log("E-STOP");
                } else if self.dev.is_some() {
                    self.set_run(RunState::Idle);
                    self.log("E-stop reset");
                }
            }
            Command::ZeroWork(axis) => {
                anyhow::ensure!(self.idle(), "zero only when idle");
                let mut s = self.snap.lock().unwrap();
                let i = s.wcs_index;
                for a in 0..AXES {
                    if axis.is_none_or(|x| x == a) {
                        s.wcs[i][a] = self.status.pos[a];
                    }
                }
                self.modal.wcs[i] = DVec3::new(s.wcs[i][0], s.wcs[i][1], s.wcs[i][2]);
            }
            Command::SelectWcs(i) => {
                self.snap.lock().unwrap().wcs_index = i.min(5);
                self.modal.wcs_index = i.min(5);
            }
            Command::SetMachineZero => {
                anyhow::ensure!(self.idle(), "only when idle");
                self.dev()?.set_position([0.0; AXES])?;
                self.log("machine zero set at current position");
            }
            Command::FeedOverride(f) => self.snap.lock().unwrap().feed_override = f.clamp(0.1, 2.0),
        }
        Ok(())
    }

    fn check_limits(&self, t: &[f64; AXES]) -> anyhow::Result<()> {
        for (i, a) in self.cfg.axes.iter().enumerate() {
            if a.enabled && a.soft_limits && (t[i] < a.min - 1e-6 || t[i] > a.max + 1e-6) {
                anyhow::bail!("{} {:.3} is outside soft limits [{:.3}, {:.3}]", AXIS_NAMES[i], t[i], a.min, a.max);
            }
        }
        Ok(())
    }

    fn load_items(&mut self, src: &str) -> anyhow::Result<()> {
        let mut state = self.modal.clone();
        {
            let s = self.snap.lock().unwrap();
            state.wcs_index = s.wcs_index;
            for (k, w) in s.wcs.iter().enumerate() {
                state.wcs[k] = DVec3::new(w[0], w[1], w[2]);
            }
        }
        state.pos = DVec3::new(self.status.pos[0], self.status.pos[1], self.status.pos[2]);
        state.a = self.status.pos[3];
        let mut it = Interpreter::new(state);
        let prog = it.run_str(src)?;
        let mut items = Vec::new();
        for m in &prog.motions {
            match m.kind {
                MotionKind::Rapid | MotionKind::Feed | MotionKind::Arc { .. } => {
                    for (p, a) in m.linearize(self.cfg.arc_tolerance) {
                        let target = [p.x, p.y, p.z, a];
                        self.check_limits(&target).map_err(|e| anyhow::anyhow!("line {}: {e}", m.line))?;
                        items.push(Item::Move { target, feed: m.feed, rapid: m.kind == MotionKind::Rapid, line: m.line });
                    }
                }
                k => items.push(Item::Sync { kind: k, line: m.line }),
            }
        }
        // Persist modal changes from MDI (G10, G54..G59, units…). Programs persist WCS edits too.
        self.modal = it.state.clone();
        {
            let mut s = self.snap.lock().unwrap();
            s.wcs_index = it.state.wcs_index;
            for k in 0..6 {
                s.wcs[k][0] = it.state.wcs[k].x;
                s.wcs[k][1] = it.state.wcs[k].y;
                s.wcs[k][2] = it.state.wcs[k].z;
            }
        }
        self.items = items;
        self.next = 0;
        Ok(())
    }

    fn sync_outputs(&mut self, spindle: SpindleState, coolant: CoolantState) -> anyhow::Result<()> {
        let pins = [
            (self.cfg.spindle_cw_pin, spindle == SpindleState::Cw),
            (self.cfg.spindle_ccw_pin, spindle == SpindleState::Ccw),
            (self.cfg.flood_pin, coolant.flood),
            (self.cfg.mist_pin, coolant.mist),
        ];
        if let Some(d) = self.dev.as_mut() {
            for (pin, on) in pins {
                if let Some(p) = pin {
                    d.set_output(p, on)?;
                }
            }
        }
        let mut s = self.snap.lock().unwrap();
        s.spindle = spindle;
        s.coolant = coolant;
        Ok(())
    }

    fn tick(&mut self) -> anyhow::Result<()> {
        let Some(d) = self.dev.as_mut() else { return Ok(()) };
        self.status = d.poll()?;
        {
            let mut s = self.snap.lock().unwrap();
            s.status = self.status;
            if self.status.current_id > 0 {
                s.line = self.status.current_id as usize;
            }
        }
        if self.status.estop && self.run_state() != RunState::EStop {
            self.items.clear();
            self.set_run(RunState::EStop);
            self.log("E-stop input triggered");
            return Ok(());
        }
        if self.run_state() != RunState::Running {
            return Ok(());
        }
        if let Some(t) = self.wait_until {
            if Instant::now() < t {
                return Ok(());
            }
            self.wait_until = None;
        }
        let ovr = self.snap.lock().unwrap().feed_override;
        let mut free = self.status.buffer_free;
        while self.next < self.items.len() {
            match self.items[self.next].clone() {
                Item::Move { target, feed, rapid, line } => {
                    if free == 0 {
                        return Ok(());
                    }
                    self.dev()?.queue_linear(target, feed * ovr, rapid, line as u64)?;
                    free -= 1;
                    self.next += 1;
                }
                Item::Sync { kind, line } => {
                    // Synchronous M-codes wait for motion to finish.
                    let queued_this_tick = free < self.status.buffer_free;
                    if self.status.moving || queued_this_tick {
                        return Ok(());
                    }
                    self.next += 1;
                    self.snap.lock().unwrap().line = line;
                    match kind {
                        MotionKind::Spindle { state, rpm } => {
                            let c = self.snap.lock().unwrap().coolant;
                            self.snap.lock().unwrap().rpm = rpm;
                            self.sync_outputs(state, c)?;
                        }
                        MotionKind::Coolant(c) => {
                            let s = self.snap.lock().unwrap().spindle;
                            self.sync_outputs(s, c)?;
                        }
                        MotionKind::Dwell { seconds } => {
                            self.wait_until = Some(Instant::now() + Duration::from_secs_f64(seconds.max(0.0)));
                            return Ok(());
                        }
                        MotionKind::ToolChange { tool } => {
                            // Manual tool change: stop spindle/coolant; the program's following
                            // M3/M8 turn them back on after Cycle Start.
                            self.snap.lock().unwrap().tool = tool;
                            self.sync_outputs(SpindleState::Off, CoolantState::default())?;
                            self.set_run(RunState::Paused(format!("Insert tool T{tool}, set its length, then Cycle Start")));
                            return Ok(());
                        }
                        MotionKind::Pause { optional } => {
                            self.set_run(RunState::Paused(if optional { "M1 optional stop".into() } else { "M0 program pause".into() }));
                            return Ok(());
                        }
                        MotionKind::End => {
                            self.sync_outputs(SpindleState::Off, CoolantState::default())?;
                            self.next = self.items.len();
                        }
                        _ => {}
                    }
                }
            }
        }
        if !self.status.moving && self.next >= self.items.len() {
            self.items.clear();
            self.next = 0;
            self.set_run(RunState::Idle);
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn wait_for(m: &Machine, f: impl Fn(&Snapshot) -> bool) -> Snapshot {
        let t0 = Instant::now();
        loop {
            let s = m.snapshot();
            if f(&s) {
                return s;
            }
            assert!(t0.elapsed() < Duration::from_secs(10), "timeout; state {:?}, log {:?}", s.run, s.log);
            std::thread::sleep(Duration::from_millis(5));
        }
    }

    #[test]
    fn runs_program_on_simulator() {
        let mut cfg = MachineConfig::simulator();
        cfg.rapid_rate = 60_000.0;
        let m = Machine::spawn(cfg);
        m.send(Command::Connect);
        wait_for(&m, |s| s.run == RunState::Idle);
        m.send(Command::Load { name: "t".into(), gcode: "G21 G90\nG0 X5 Y5\nG1 Z-1 F30000\nM0\nG1 X10 F30000\nM30".into() });
        m.send(Command::CycleStart);
        wait_for(&m, |s| matches!(s.run, RunState::Paused(_)));
        m.send(Command::CycleStart);
        let s = wait_for(&m, |s| s.run == RunState::Idle && s.status.pos[0] > 9.99);
        assert!((s.status.pos[2] + 1.0).abs() < 1e-9);
    }

    #[test]
    fn work_zero_and_mdi() {
        let mut cfg = MachineConfig::simulator();
        cfg.rapid_rate = 60_000.0;
        let m = Machine::spawn(cfg);
        m.send(Command::Connect);
        wait_for(&m, |s| s.run == RunState::Idle);
        m.send(Command::Mdi("G0 X20 Y10".into()));
        wait_for(&m, |s| s.run == RunState::Idle && s.status.pos[0] > 19.99);
        m.send(Command::ZeroWork(None));
        m.send(Command::Mdi("G0 X1".into()));
        let s = wait_for(&m, |s| s.run == RunState::Idle && s.status.pos[0] > 20.99);
        assert!((s.work_pos()[0] - 1.0).abs() < 1e-9);
    }
}
