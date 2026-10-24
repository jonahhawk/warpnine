use crate::{parse::Block, GcodeError};
use glam::DVec3;
use serde::{Deserialize, Serialize};
use std::f64::consts::TAU;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Default)]
pub enum Plane {
    #[default]
    XY,
    XZ,
    YZ,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Default)]
pub enum SpindleState {
    #[default]
    Off,
    Cw,
    Ccw,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Default)]
pub struct CoolantState {
    pub flood: bool,
    pub mist: bool,
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub enum MotionKind {
    Rapid,
    Feed,
    Arc { center: DVec3, cw: bool, plane: Plane },
    Dwell { seconds: f64 },
    ToolChange { tool: u32 },
    Spindle { state: SpindleState, rpm: f64 },
    Coolant(CoolantState),
    /// M0 (mandatory) or M1 (optional stop).
    Pause { optional: bool },
    End,
}

/// One interpreted step, in machine coordinates (mm). `a` is the rotary axis in degrees.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Motion {
    pub kind: MotionKind,
    pub from: DVec3,
    pub to: DVec3,
    pub a_from: f64,
    pub a_to: f64,
    /// mm/min (0 for rapids).
    pub feed: f64,
    pub line: usize,
    pub tool: u32,
}

impl Motion {
    pub fn is_move(&self) -> bool {
        matches!(self.kind, MotionKind::Rapid | MotionKind::Feed | MotionKind::Arc { .. })
    }

    /// Polyline approximation (excluding the start point) within chord tolerance `tol`.
    pub fn linearize(&self, tol: f64) -> Vec<(DVec3, f64)> {
        match self.kind {
            MotionKind::Arc { center, cw, plane } => {
                let (u, v, w) = plane_axes(plane);
                let s = self.from - center;
                let e = self.to - center;
                let r = (s.dot(u).powi(2) + s.dot(v).powi(2)).sqrt();
                let a0 = s.dot(v).atan2(s.dot(u));
                let mut a1 = e.dot(v).atan2(e.dot(u));
                // In G18 the XZ plane is viewed from +Y, so G2/G3 sense flips relative to (X,Z) math.
                let cw = if plane == Plane::XZ { !cw } else { cw };
                if cw {
                    while a1 >= a0 - 1e-12 {
                        a1 -= TAU;
                    }
                } else {
                    while a1 <= a0 + 1e-12 {
                        a1 += TAU;
                    }
                }
                let sweep = a1 - a0;
                let max_step = 2.0 * (1.0 - (tol / r.max(1e-9)).min(1.0)).acos();
                let n = ((sweep.abs() / max_step.max(1e-3)).ceil() as usize).clamp(2, 20_000);
                let (w0, w1) = (self.from.dot(w), self.to.dot(w));
                (1..=n)
                    .map(|i| {
                        let t = i as f64 / n as f64;
                        let ang = a0 + sweep * t;
                        let p = center.dot(u) * u + center.dot(v) * v
                            + (u * ang.cos() + v * ang.sin()) * r
                            + w * (w0 + (w1 - w0) * t);
                        (if i == n { self.to } else { p }, self.a_from + (self.a_to - self.a_from) * t)
                    })
                    .collect()
            }
            _ => vec![(self.to, self.a_to)],
        }
    }

    pub fn length(&self) -> f64 {
        if !self.is_move() {
            return 0.0;
        }
        let mut prev = self.from;
        let mut len = 0.0;
        for (p, _) in self.linearize(0.01) {
            len += prev.distance(p);
            prev = p;
        }
        len
    }
}

fn plane_axes(p: Plane) -> (DVec3, DVec3, DVec3) {
    match p {
        Plane::XY => (DVec3::X, DVec3::Y, DVec3::Z),
        Plane::XZ => (DVec3::X, DVec3::Z, DVec3::Y),
        Plane::YZ => (DVec3::Y, DVec3::Z, DVec3::X),
    }
}

/// Modal state of the interpreter.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MachineState {
    pub pos: DVec3,
    pub a: f64,
    pub inches: bool,
    pub absolute: bool,
    pub arc_absolute: bool,
    pub plane: Plane,
    pub motion_mode: i32,
    pub feed: f64,
    pub rpm: f64,
    pub tool: u32,
    pub spindle: SpindleState,
    pub coolant: CoolantState,
    /// G54..G59 offsets (index 0 = G54).
    pub wcs: [DVec3; 6],
    pub wcs_index: usize,
    pub g92: DVec3,
    /// Active tool length offset (G43).
    pub tlo: f64,
    /// Tool lengths by H number for G43.
    pub tool_lengths: std::collections::BTreeMap<u32, f64>,
    pub canned_return_initial: bool,
}

impl Default for MachineState {
    fn default() -> Self {
        MachineState {
            pos: DVec3::ZERO,
            a: 0.0,
            inches: false,
            absolute: true,
            arc_absolute: false,
            plane: Plane::XY,
            motion_mode: 0,
            feed: 0.0,
            rpm: 0.0,
            tool: 0,
            spindle: SpindleState::Off,
            coolant: CoolantState::default(),
            wcs: [DVec3::ZERO; 6],
            wcs_index: 0,
            g92: DVec3::ZERO,
            tlo: 0.0,
            tool_lengths: Default::default(),
            canned_return_initial: true,
        }
    }
}

impl MachineState {
    /// Offset from program coordinates to machine coordinates.
    pub fn offset(&self) -> DVec3 {
        self.wcs[self.wcs_index] + self.g92 + DVec3::new(0.0, 0.0, self.tlo)
    }

    /// Current position in program (work) coordinates.
    pub fn work_pos(&self) -> DVec3 {
        self.pos - self.offset()
    }
}

#[derive(Debug, Clone, Default)]
pub struct Program {
    pub motions: Vec<Motion>,
    pub warnings: Vec<String>,
}

impl Program {
    /// Bounding box of all feed and rapid moves in machine coordinates.
    pub fn bounds(&self) -> Option<(DVec3, DVec3)> {
        let mut it = self.motions.iter().filter(|m| m.is_move());
        let first = it.next()?;
        let (mut lo, mut hi) = (first.from.min(first.to), first.from.max(first.to));
        for m in it {
            for (p, _) in m.linearize(0.05) {
                lo = lo.min(p);
                hi = hi.max(p);
            }
        }
        Some((lo, hi))
    }

    /// Estimated run time in seconds (ignores acceleration).
    pub fn estimate_seconds(&self, rapid_rate: f64) -> f64 {
        self.motions
            .iter()
            .map(|m| match m.kind {
                MotionKind::Rapid => m.length() / rapid_rate.max(1.0) * 60.0,
                MotionKind::Feed | MotionKind::Arc { .. } => m.length() / m.feed.max(1.0) * 60.0,
                MotionKind::Dwell { seconds } => seconds,
                _ => 0.0,
            })
            .sum()
    }
}

pub struct Interpreter {
    pub state: MachineState,
    pub block_delete: bool,
}

impl Default for Interpreter {
    fn default() -> Self {
        Self::new(MachineState::default())
    }
}

impl Interpreter {
    pub fn new(state: MachineState) -> Self {
        Interpreter { state, block_delete: true }
    }

    pub fn run_str(&mut self, src: &str) -> Result<Program, GcodeError> {
        let blocks = crate::parse(src)?;
        self.run(&blocks)
    }

    pub fn run(&mut self, blocks: &[Block]) -> Result<Program, GcodeError> {
        let mut prog = Program::default();
        for b in blocks {
            if b.block_delete && self.block_delete {
                continue;
            }
            let ended = self.exec(b, &mut prog)?;
            if ended {
                break;
            }
        }
        Ok(prog)
    }

    /// Execute one block, appending motions. Returns true at program end.
    pub fn exec(&mut self, b: &Block, prog: &mut Program) -> Result<bool, GcodeError> {
        let line = b.line;
        let sem = |msg: &str| GcodeError::Semantic { line, msg: msg.to_string() };
        let st = &mut self.state;
        let mut non_modal: Option<i32> = None;
        let mut machine_coords = false;
        let mut events: Vec<MotionKind> = Vec::new();

        // --- modal settings ---
        // Units first so F and axis words on the same line use the new units.
        for g in b.codes('G') {
            match g {
                200 => st.inches = true,
                210 => st.inches = false,
                _ => {}
            }
        }
        if let Some(f) = b.get('F') {
            st.feed = f * if st.inches { 25.4 } else { 1.0 };
        }
        if let Some(s) = b.get('S') {
            st.rpm = s;
            if st.spindle != SpindleState::Off {
                events.push(MotionKind::Spindle { state: st.spindle, rpm: s });
            }
        }
        let new_tool = b.get('T').map(|t| t as u32);

        for g in b.codes('G') {
            match g {
                0 | 10 | 20 | 30 | 800 | 810 | 820 | 830 | 730 | 850 | 860 | 890 => {
                    st.motion_mode = g;
                }
                40 => non_modal = Some(g),
                100 => non_modal = Some(100),
                170 => st.plane = Plane::XY,
                180 => st.plane = Plane::XZ,
                190 => st.plane = Plane::YZ,
                200 => st.inches = true,
                210 => st.inches = false,
                280 | 300 => non_modal = Some(g),
                400 | 410 | 420 => {
                    if g != 400 {
                        prog.warnings.push(format!("line {line}: cutter compensation G{} ignored; CAM output is already compensated", g / 10));
                    }
                }
                430 => {
                    let h = b.get('H').map(|h| h as u32).unwrap_or(st.tool);
                    st.tlo = *st.tool_lengths.get(&h).unwrap_or(&0.0);
                }
                490 => st.tlo = 0.0,
                530 => machine_coords = true,
                540..=590 if g % 10 == 0 => st.wcs_index = ((g - 540) / 10) as usize,
                610 | 611 | 640 => {} // path control modes: handled by the motion controller
                900 => st.absolute = true,
                910 => st.absolute = false,
                901 => st.arc_absolute = true,
                911 => st.arc_absolute = false,
                920 => non_modal = Some(920),
                921 | 922 => st.g92 = DVec3::ZERO,
                930 => return Err(sem("inverse time feed (G93) is not supported")),
                940 => {}
                950 => return Err(sem("feed per revolution (G95) is not supported")),
                980 => st.canned_return_initial = true,
                990 => st.canned_return_initial = false,
                _ => prog.warnings.push(format!("line {line}: unsupported G{} ignored", g as f64 / 10.0)),
            }
        }

        let mut stop = false;
        for m in b.codes('M') {
            match m {
                0 => events.push(MotionKind::Pause { optional: false }),
                10 => events.push(MotionKind::Pause { optional: true }),
                20 | 300 => stop = true,
                30 | 40 => {
                    st.spindle = if m == 30 { SpindleState::Cw } else { SpindleState::Ccw };
                    events.push(MotionKind::Spindle { state: st.spindle, rpm: st.rpm });
                }
                50 => {
                    st.spindle = SpindleState::Off;
                    events.push(MotionKind::Spindle { state: st.spindle, rpm: st.rpm });
                }
                60 => {
                    if let Some(t) = new_tool.or(Some(st.tool)) {
                        st.tool = t;
                        events.push(MotionKind::ToolChange { tool: t });
                    }
                }
                70 => {
                    st.coolant.mist = true;
                    events.push(MotionKind::Coolant(st.coolant));
                }
                80 => {
                    st.coolant.flood = true;
                    events.push(MotionKind::Coolant(st.coolant));
                }
                90 => {
                    st.coolant = CoolantState::default();
                    events.push(MotionKind::Coolant(st.coolant));
                }
                _ => prog.warnings.push(format!("line {line}: unsupported M{} ignored", m / 10)),
            }
        }

        let (pos, a) = (st.pos, st.a);
        let tool = st.tool;
        for e in events {
            prog.motions.push(Motion { kind: e, from: pos, to: pos, a_from: a, a_to: a, feed: 0.0, line, tool });
        }

        // --- non-modal commands ---
        let scale = if self.state.inches { 25.4 } else { 1.0 };
        let axis_words = ['X', 'Y', 'Z'].iter().any(|c| b.has(*c)) || b.has('A');
        match non_modal {
            Some(40) => {
                let secs = b.get('P').unwrap_or(0.0);
                let st = &self.state;
                prog.motions.push(Motion { kind: MotionKind::Dwell { seconds: secs }, from: st.pos, to: st.pos, a_from: st.a, a_to: st.a, feed: 0.0, line, tool: st.tool });
                return Ok(stop);
            }
            Some(100) => {
                // G10 L2 Pn X Y Z: set work offset n so that the given values are the offset itself.
                // G10 L20 Pn: set offset so the current position reads the given values.
                let l = b.get('L').unwrap_or(2.0) as i32;
                let p = b.get('P').unwrap_or(1.0) as usize;
                if !(1..=6).contains(&p) {
                    return Err(sem("G10 P must be 1..6"));
                }
                let st = &mut self.state;
                let mut off = st.wcs[p - 1];
                for (i, c) in ['X', 'Y', 'Z'].iter().enumerate() {
                    if let Some(v) = b.get(*c) {
                        off[i] = match l {
                            20 => st.pos[i] - v * scale - st.g92[i],
                            _ => v * scale,
                        };
                    }
                }
                st.wcs[p - 1] = off;
                return Ok(stop);
            }
            Some(920) => {
                let st = &mut self.state;
                let base = st.wcs[st.wcs_index] + DVec3::new(0.0, 0.0, st.tlo);
                for (i, c) in ['X', 'Y', 'Z'].iter().enumerate() {
                    if let Some(v) = b.get(*c) {
                        st.g92[i] = st.pos[i] - base[i] - v * scale;
                    }
                }
                return Ok(stop);
            }
            Some(280) | Some(300) => {
                // Optional intermediate point, then home (machine zero) for named axes (all if none).
                let st = &self.state;
                let mut mid = st.pos;
                let mut home = st.pos;
                let any = ['X', 'Y', 'Z'].iter().any(|c| b.has(*c));
                for (i, c) in ['X', 'Y', 'Z'].iter().enumerate() {
                    if let Some(v) = b.get(*c) {
                        mid[i] = if st.absolute { v * scale + st.offset()[i] } else { st.pos[i] + v * scale };
                        home[i] = 0.0;
                    } else if !any {
                        home[i] = 0.0;
                    }
                }
                let mut push = |to: DVec3, st: &mut MachineState| {
                    prog.motions.push(Motion { kind: MotionKind::Rapid, from: st.pos, to, a_from: st.a, a_to: st.a, feed: 0.0, line, tool: st.tool });
                    st.pos = to;
                };
                push(mid, &mut self.state);
                push(home, &mut self.state);
                return Ok(stop);
            }
            _ => {}
        }

        // --- motion ---
        if new_tool.is_some() && !b.codes('M').any(|m| m == 60) {
            // T without M6 just preselects; nothing to emit.
        }
        let st = &mut self.state;
        let arc_words = matches!(st.motion_mode, 20 | 30)
            && ['I', 'J', 'K', 'R'].iter().any(|c| b.has(*c));
        if !axis_words && !arc_words {
            if stop {
                let (pos, a) = (st.pos, st.a);
                prog.motions.push(Motion { kind: MotionKind::End, from: pos, to: pos, a_from: a, a_to: a, feed: 0.0, line, tool: st.tool });
            }
            return Ok(stop);
        }

        let off = if machine_coords { DVec3::ZERO } else { st.offset() };
        let mut target = st.pos;
        for (i, c) in ['X', 'Y', 'Z'].iter().enumerate() {
            if let Some(v) = b.get(*c) {
                target[i] = if st.absolute || machine_coords { v * scale + off[i] } else { st.pos[i] + v * scale };
            }
        }
        let a_target = match b.get('A') {
            Some(v) if st.absolute => v,
            Some(v) => st.a + v,
            None => st.a,
        };
        let mode = if machine_coords { 0 } else { st.motion_mode };
        let mk = |kind, from, to, st: &MachineState, feed| Motion { kind, from, to, a_from: st.a, a_to: a_target, feed, line, tool: st.tool };

        match mode {
            0 => prog.motions.push(mk(MotionKind::Rapid, st.pos, target, st, 0.0)),
            10 => {
                if st.feed <= 0.0 {
                    return Err(sem("feed move with F=0"));
                }
                prog.motions.push(mk(MotionKind::Feed, st.pos, target, st, st.feed));
            }
            20 | 30 => {
                if st.feed <= 0.0 {
                    return Err(sem("arc move with F=0"));
                }
                let cw = mode == 20;
                let (u, v, _) = plane_axes(st.plane);
                let center = if let Some(r) = b.get('R') {
                    let r = r * scale;
                    let s = DVec3::new(st.pos.dot(u), st.pos.dot(v), 0.0);
                    let e = DVec3::new(target.dot(u), target.dot(v), 0.0);
                    let d = e - s;
                    let dl = d.length();
                    if dl < 1e-9 || dl > 2.0 * r.abs() + 1e-6 {
                        return Err(sem("arc radius too small for endpoints"));
                    }
                    let h = (r * r - dl * dl / 4.0).max(0.0).sqrt();
                    let mid = (s + e) * 0.5;
                    let perp = DVec3::new(-d.y, d.x, 0.0) / dl;
                    // For G2 with positive R the centre is to the right of travel (short arc).
                    let sign = if cw == (r > 0.0) { -1.0 } else { 1.0 };
                    let c2 = mid + perp * h * sign;
                    let mut c = st.pos;
                    c += u * (c2.x - st.pos.dot(u)) + v * (c2.y - st.pos.dot(v));
                    c
                } else {
                    let ij = match st.plane {
                        Plane::XY => ('I', 'J'),
                        Plane::XZ => ('I', 'K'),
                        Plane::YZ => ('J', 'K'),
                    };
                    let (ou, ov) = (b.get(ij.0).unwrap_or(0.0) * scale, b.get(ij.1).unwrap_or(0.0) * scale);
                    if st.arc_absolute {
                        let mut c = st.pos;
                        c += u * (ou + off.dot(u) - st.pos.dot(u)) + v * (ov + off.dot(v) - st.pos.dot(v));
                        c
                    } else {
                        st.pos + u * ou + v * ov
                    }
                };
                let rs = ((st.pos - center).dot(u).powi(2) + (st.pos - center).dot(v).powi(2)).sqrt();
                let re = ((target - center).dot(u).powi(2) + (target - center).dot(v).powi(2)).sqrt();
                if (rs - re).abs() > 0.01_f64.max(rs * 1e-3) {
                    prog.warnings.push(format!("line {line}: arc start/end radius mismatch ({rs:.4} vs {re:.4})"));
                }
                prog.motions.push(mk(MotionKind::Arc { center, cw, plane: st.plane }, st.pos, target, st, st.feed));
            }
            800 => return Err(sem("axis words with G80 active and no motion mode")),
            810 | 820 | 830 | 730 | 850 | 860 | 890 => {
                if !st.absolute {
                    return Err(sem("canned cycles in G91 are not supported"));
                }
                let r = b.get('R').ok_or_else(|| sem("canned cycle needs R"))? * scale + off.z;
                let initial_z = st.pos.z;
                let depth = target.z;
                let peck = b.get('Q').map(|q| q * scale).unwrap_or(0.0);
                let dwell = b.get('P').unwrap_or(0.0);
                let feed = st.feed;
                let xy = DVec3::new(target.x, target.y, st.pos.z);
                let mut emit = |kind: MotionKind, to: DVec3, f: f64, st: &mut MachineState| {
                    prog.motions.push(Motion { kind, from: st.pos, to, a_from: st.a, a_to: st.a, feed: f, line, tool: st.tool });
                    st.pos = to;
                };
                if st.pos.z < r {
                    // Never traverse XY below the retract plane.
                    let up = DVec3::new(st.pos.x, st.pos.y, r);
                    emit(MotionKind::Rapid, up, 0.0, st);
                }
                let xy = DVec3::new(xy.x, xy.y, st.pos.z);
                emit(MotionKind::Rapid, xy, 0.0, st);
                let rz = DVec3::new(target.x, target.y, r);
                if st.pos.z != r {
                    emit(MotionKind::Rapid, rz, 0.0, st);
                }
                let bottom = DVec3::new(target.x, target.y, depth);
                if matches!(mode, 830 | 730) && peck > 0.0 {
                    let mut z = r;
                    while z > depth + 1e-9 {
                        z = (z - peck).max(depth);
                        emit(MotionKind::Feed, DVec3::new(target.x, target.y, z), feed, st);
                        if z > depth + 1e-9 {
                            if mode == 830 {
                                emit(MotionKind::Rapid, rz, 0.0, st);
                                emit(MotionKind::Rapid, DVec3::new(target.x, target.y, z + 0.5_f64.min(peck)), 0.0, st);
                            } else {
                                emit(MotionKind::Rapid, DVec3::new(target.x, target.y, z + 0.5_f64.min(peck)), 0.0, st);
                            }
                        }
                    }
                } else {
                    emit(MotionKind::Feed, bottom, feed, st);
                }
                if matches!(mode, 820 | 890) && dwell > 0.0 {
                    let p = st.pos;
                    emit(MotionKind::Dwell { seconds: dwell }, p, 0.0, st);
                }
                let ret_z = if st.canned_return_initial { initial_z.max(r) } else { r };
                let ret = DVec3::new(target.x, target.y, ret_z);
                if matches!(mode, 850 | 890) {
                    emit(MotionKind::Feed, ret, feed, st);
                } else {
                    emit(MotionKind::Rapid, ret, 0.0, st);
                }
                st.a = a_target;
                return Ok(stop);
            }
            _ => return Err(sem("unknown motion mode")),
        }
        st.pos = target;
        st.a = a_target;
        if stop {
            let (pos, a) = (st.pos, st.a);
            prog.motions.push(Motion { kind: MotionKind::End, from: pos, to: pos, a_from: a, a_to: a, feed: 0.0, line, tool: st.tool });
        }
        Ok(stop)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn run(src: &str) -> Program {
        Interpreter::default().run_str(src).unwrap()
    }

    #[test]
    fn basic_moves_and_units() {
        let p = run("G21 G90\nG0 X10 Y0\nG1 Z-1 F100\nG20 G1 X1 F100");
        let moves: Vec<_> = p.motions.iter().filter(|m| m.is_move()).collect();
        assert_eq!(moves.len(), 3);
        assert!((moves[2].to.x - 25.4).abs() < 1e-9);
        assert!((moves[2].feed - 2540.0).abs() < 1e-9);
    }

    #[test]
    fn arc_ij_and_r_agree() {
        let a = run("G0 X0 Y0\nG2 X10 Y0 I5 J0 F100");
        let b = run("G0 X0 Y0\nG2 X10 Y0 R5 F100");
        let ca = match a.motions[1].kind { MotionKind::Arc { center, .. } => center, _ => panic!() };
        let cb = match b.motions[1].kind { MotionKind::Arc { center, .. } => center, _ => panic!() };
        assert!(ca.distance(cb) < 1e-9, "{ca} vs {cb}");
        // CW from (0,0) to (10,0) around (5,0) goes over the top (+Y).
        let pts = a.motions[1].linearize(0.01);
        assert!(pts[pts.len() / 2].0.y > 4.9);
    }

    #[test]
    fn work_offsets() {
        let p = run("G10 L2 P2 X100 Y50 Z-20\nG55 G0 X1 Y1 Z1");
        let m = p.motions.last().unwrap();
        assert_eq!(m.to, DVec3::new(101.0, 51.0, -19.0));
    }

    #[test]
    fn peck_drill_expands() {
        let p = run("G0 Z5\nG83 X1 Y1 Z-3 R1 Q1 F100\nG80");
        let feeds = p.motions.iter().filter(|m| m.kind == MotionKind::Feed).count();
        assert_eq!(feeds, 4);
        assert_eq!(p.motions.last().unwrap().to.z, 5.0);
    }
}
