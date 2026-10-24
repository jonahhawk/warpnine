//! Configurable G-code post-processor with presets for common controllers.

use crate::toolpath::{arc_points, DrillCycle, Move, Toolpath};
use glam::DVec3;
use serde::{Deserialize, Serialize};
use std::fmt::Write;
use warp_tools::{Coolant, ToolLibrary};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum CommentStyle {
    Parens,
    Semicolon,
    None,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PostConfig {
    pub name: String,
    pub file_extension: String,
    pub inches: bool,
    pub decimals: usize,
    pub feed_decimals: usize,
    pub line_numbers: bool,
    pub line_start: u32,
    pub line_step: u32,
    pub comments: CommentStyle,
    pub preamble: Vec<String>,
    pub postamble: Vec<String>,
    /// Output G2/G3. If false, arcs are linearised to `arc_tolerance`.
    pub arcs: bool,
    /// Use G90.1 absolute IJ instead of incremental IJ.
    pub arc_ij_absolute: bool,
    pub arc_tolerance: f64,
    /// Emit canned drilling cycles (G81/G82/G83/G73/G85/G89). If false, expand to moves.
    pub canned_cycles: bool,
    /// `{T}` = tool number. Empty = no tool change output.
    pub tool_change: String,
    pub tool_length_comp: bool,
    pub spindle_cw: String,
    pub spindle_ccw: String,
    pub spindle_off: String,
    /// Seconds to dwell after starting the spindle (0 = none).
    pub spindle_delay: f64,
    pub modal_motion: bool,
    pub modal_axes: bool,
    /// Rapid to this machine Z (G53) before tool changes; `None` = use clearance in work coords.
    pub tool_change_z_g53: Option<f64>,
    pub program_end: String,
}

impl PostConfig {
    pub fn mach3() -> Self {
        PostConfig {
            name: "Mach3 / Mach4".into(),
            file_extension: "tap".into(),
            inches: false,
            decimals: 4,
            feed_decimals: 1,
            line_numbers: false,
            line_start: 10,
            line_step: 10,
            comments: CommentStyle::Parens,
            preamble: vec!["G90 G94 G91.1 G40 G49 G17".into(), "G21".into()],
            postamble: vec![],
            arcs: true,
            arc_ij_absolute: false,
            arc_tolerance: 0.005,
            canned_cycles: true,
            tool_change: "T{T} M6".into(),
            tool_length_comp: true,
            spindle_cw: "M3".into(),
            spindle_ccw: "M4".into(),
            spindle_off: "M5".into(),
            spindle_delay: 0.0,
            modal_motion: true,
            modal_axes: true,
            tool_change_z_g53: None,
            program_end: "M30".into(),
        }
    }

    pub fn uccnc() -> Self {
        PostConfig {
            name: "UCCNC".into(),
            ..Self::mach3()
        }
    }

    pub fn linuxcnc() -> Self {
        PostConfig {
            name: "LinuxCNC".into(),
            file_extension: "ngc".into(),
            preamble: vec!["G17 G21 G40 G49 G54 G80 G90 G94 G64 P0.01".into()],
            postamble: vec![],
            program_end: "M2".into(),
            ..Self::mach3()
        }
    }

    pub fn grbl() -> Self {
        PostConfig {
            name: "GRBL".into(),
            file_extension: "nc".into(),
            preamble: vec!["G90 G94 G17 G21".into()],
            canned_cycles: false,
            tool_change: String::new(),
            tool_length_comp: false,
            comments: CommentStyle::Parens,
            program_end: "M2".into(),
            ..Self::mach3()
        }
    }

    pub fn turbocnc() -> Self {
        PostConfig {
            name: "TurboCNC".into(),
            file_extension: "cnc".into(),
            preamble: vec!["G90 G17 G21".into()],
            canned_cycles: false,
            tool_length_comp: false,
            comments: CommentStyle::Semicolon,
            line_numbers: true,
            ..Self::mach3()
        }
    }

    pub fn presets() -> Vec<PostConfig> {
        vec![
            Self::mach3(),
            Self::uccnc(),
            Self::linuxcnc(),
            Self::grbl(),
            Self::turbocnc(),
        ]
    }
}

struct Writer<'a> {
    cfg: &'a PostConfig,
    out: String,
    line: u32,
    last_motion: Option<&'static str>,
    pos: Option<DVec3>,
    feed: Option<f64>,
}

impl Writer<'_> {
    fn emit(&mut self, s: &str) {
        if s.is_empty() {
            return;
        }
        if self.cfg.line_numbers {
            let _ = write!(self.out, "N{} ", self.line);
            self.line += self.cfg.line_step;
        }
        self.out.push_str(s);
        self.out.push('\n');
    }

    fn comment(&mut self, s: &str) {
        let s = s.replace(['(', ')'], "");
        match self.cfg.comments {
            CommentStyle::Parens => self.emit(&format!("({s})")),
            CommentStyle::Semicolon => self.emit(&format!("; {s}")),
            CommentStyle::None => {}
        }
    }

    fn num(&self, v: f64) -> String {
        let v = if self.cfg.inches { v / 25.4 } else { v };
        let s = format!("{:.*}", self.cfg.decimals, v);
        let s = if s.contains('.') {
            s.trim_end_matches('0').trim_end_matches('.').to_string()
        } else {
            s
        };
        if s == "-0" {
            "0".into()
        } else {
            s
        }
    }

    fn feed_word(&mut self, f: f64) -> String {
        if self.feed.is_some_and(|c| (c - f).abs() < 1e-9) {
            return String::new();
        }
        self.feed = Some(f);
        let f = if self.cfg.inches { f / 25.4 } else { f };
        format!(" F{:.*}", self.cfg.feed_decimals, f)
    }

    fn motion(&mut self, g: &'static str) -> String {
        if self.cfg.modal_motion && self.last_motion == Some(g) {
            String::new()
        } else {
            self.last_motion = Some(g);
            g.to_string()
        }
    }

    fn axes(&mut self, to: DVec3, force: bool) -> String {
        let mut s = String::new();
        for (i, a) in ['X', 'Y', 'Z'].iter().enumerate() {
            let changed = self
                .pos
                .map(|p| self.num(p[i]) != self.num(to[i]))
                .unwrap_or(true);
            if force || !self.cfg.modal_axes || changed {
                let _ = write!(s, " {a}{}", self.num(to[i]));
            }
        }
        s
    }

    fn linear(&mut self, g: &'static str, to: DVec3, feed: Option<f64>) {
        let ax = self.axes(to, false);
        if ax.is_empty() {
            return;
        }
        let m = self.motion(g);
        let f = feed.map(|f| self.feed_word(f)).unwrap_or_default();
        let s = format!("{m}{ax}{f}");
        self.emit(s.trim_start());
        self.pos = Some(to);
    }
}

/// Validate configured post capabilities before exposing output to export or machine loading.
pub fn validate(cfg: &PostConfig, paths: &[Toolpath], tools: &ToolLibrary) -> anyhow::Result<()> {
    use anyhow::ensure;
    ensure!(!paths.is_empty(), "No generated operations to post");
    ensure!(
        (3..=8).contains(&cfg.decimals),
        "Output requires 3–8 decimal places"
    );
    ensure!(
        cfg.arc_tolerance.is_finite() && cfg.arc_tolerance > 0.,
        "Arc tolerance must be positive"
    );
    ensure!(
        cfg.feed_decimals <= 8,
        "Feed precision must be at most eight decimals"
    );
    ensure!(
        cfg.spindle_delay.is_finite() && cfg.spindle_delay >= 0.,
        "Invalid spindle delay"
    );
    ensure!(
        cfg.tool_change_z_g53.is_none_or(f64::is_finite),
        "Invalid machine tool-change height"
    );
    if cfg.name == "GRBL" || cfg.name == "TurboCNC" {
        ensure!(
            !cfg.arc_ij_absolute,
            "This preset supports incremental arc centres only"
        );
        ensure!(
            !cfg.canned_cycles,
            "This preset requires expanded drilling cycles"
        );
    }
    let distinct: std::collections::BTreeSet<_> = paths.iter().map(|p| p.tool).collect();
    ensure!(
        distinct.len() <= 1 || !cfg.tool_change.trim().is_empty(),
        "This post has no tool-change command; export each tool separately"
    );
    for p in paths {
        ensure!(tools.tool(p.tool).is_some(), "Unresolved tool T{}", p.tool);
        ensure!(
            (54..=59).contains(&p.wcs),
            "This post supports G54–G59 work offsets"
        );
        ensure!(!p.moves.is_empty(), "Empty operation {}", p.name);
        ensure!(p.rpm.is_finite() && p.rpm > 0., "Invalid spindle speed");
        ensure!(
            matches!(
                p.moves.iter().find(|m| !matches!(m, Move::Comment(_))),
                Some(Move::Rapid(_))
            ),
            "Operation must start with a clearance rapid"
        );
        for m in &p.moves {
            match m {
                Move::Rapid(to) => ensure!(to.is_finite(), "Nonfinite rapid endpoint"),
                Move::Linear { to, feed } => ensure!(
                    to.is_finite() && feed.is_finite() && *feed > 0.,
                    "Invalid feed move"
                ),
                Move::Arc {
                    to, center, feed, ..
                } => ensure!(
                    to.is_finite() && center.is_finite() && feed.is_finite() && *feed > 0.,
                    "Invalid arc move"
                ),
                Move::Drill {
                    at,
                    r_plane,
                    bottom,
                    clearance,
                    peck,
                    dwell,
                    feed,
                    ..
                } => {
                    ensure!(
                        at.is_finite()
                            && [*r_plane, *bottom, *clearance, *peck, *dwell, *feed]
                                .iter()
                                .all(|v| v.is_finite()),
                        "Nonfinite drill parameters"
                    );
                    ensure!(
                        clearance >= r_plane
                            && r_plane > bottom
                            && *peck >= 0.
                            && *dwell >= 0.
                            && *feed > 0.,
                        "Invalid drilling heights/feed"
                    );
                    ensure!(
                        *peck == 0. || (r_plane - bottom) / peck < 100_000.,
                        "Excessive peck count"
                    );
                }
                Move::Comment(_) => {}
            }
        }
        ensure!(
            if p.clockwise {
                !cfg.spindle_cw.trim().is_empty()
            } else {
                !cfg.spindle_ccw.trim().is_empty()
            },
            "Post does not support requested spindle direction"
        );
    }
    Ok(())
}
pub fn post_checked(
    cfg: &PostConfig,
    name: &str,
    paths: &[Toolpath],
    tools: &ToolLibrary,
) -> anyhow::Result<String> {
    validate(cfg, paths, tools)?;
    let code = post(cfg, name, paths, tools);
    crate::verification::verify_post(cfg, paths, &code)?;
    Ok(code)
}

/// Generate a complete program from toolpaths.
pub fn post(
    cfg: &PostConfig,
    program_name: &str,
    paths: &[Toolpath],
    tools: &ToolLibrary,
) -> String {
    let mut w = Writer {
        cfg,
        out: String::new(),
        line: cfg.line_start,
        last_motion: None,
        pos: None,
        feed: None,
    };
    if cfg.comments != CommentStyle::None {
        w.out.push_str("%\n");
    }
    w.comment(&format!("{program_name} - generated by WarpNine"));
    let mut seen = std::collections::BTreeSet::new();
    for p in paths {
        if seen.insert(p.tool) {
            let desc = tools
                .tool(p.tool)
                .map(|t| format!("{} D={:.3}", t.name, t.diameter))
                .unwrap_or_default();
            w.comment(&format!("T{} {desc}", p.tool));
        }
    }
    let mut pre = cfg.preamble.clone();
    if cfg.inches {
        for l in &mut pre {
            *l = l.replace("G21", "G20");
        }
    }
    for l in &pre {
        w.emit(l);
    }
    // Establish output modes after custom preamble text, independently of its defaults.
    w.emit(if cfg.inches {
        "G20 G90 G94 G17"
    } else {
        "G21 G90 G94 G17"
    });
    if cfg.arc_ij_absolute {
        w.emit("G90.1");
    } else if cfg.name != "GRBL" && cfg.name != "TurboCNC" {
        w.emit("G91.1");
    }

    let mut cur_tool: Option<u32> = None;
    let mut spindle: Option<(f64, bool)> = None;
    let mut coolant = Coolant::Off;
    let mut current_wcs = None;
    for p in paths {
        w.comment(&p.name);
        if current_wcs != Some(p.wcs) {
            w.emit(&format!("G{}", p.wcs));
            w.pos = None;
            current_wcs = Some(p.wcs);
        }
        if cur_tool != Some(p.tool) {
            if spindle.is_some() {
                w.emit(&cfg.spindle_off);
                spindle = None;
            }
            if coolant != Coolant::Off {
                w.emit("M9");
                coolant = Coolant::Off;
            }
            if !cfg.tool_change.is_empty() {
                if let Some(z) = cfg.tool_change_z_g53 {
                    w.emit(&format!("G53 G0 Z{}", w.num(z)));
                    w.last_motion = Some("G0");
                    w.pos = None;
                }
                w.emit(&cfg.tool_change.replace("{T}", &p.tool.to_string()));
                if cfg.tool_length_comp {
                    let h = tools
                        .tool(p.tool)
                        .map(|t| t.length_offset)
                        .unwrap_or(p.tool);
                    w.emit(&format!("G43 H{h}"));
                }
            }
            cur_tool = Some(p.tool);
        }
        if spindle != Some((p.rpm, p.clockwise)) {
            let m = if p.clockwise {
                &cfg.spindle_cw
            } else {
                &cfg.spindle_ccw
            };
            w.emit(&format!("S{} {m}", p.rpm.round()));
            if cfg.spindle_delay > 0.0 {
                w.emit(&format!("G4 P{}", cfg.spindle_delay));
            }
            spindle = Some((p.rpm, p.clockwise));
        }
        if p.coolant != coolant {
            match p.coolant {
                Coolant::Flood => w.emit("M8"),
                Coolant::Mist | Coolant::Air => w.emit("M7"),
                Coolant::Off => w.emit("M9"),
            }
            coolant = p.coolant;
        }
        for m in &p.moves {
            match m {
                Move::Comment(c) => w.comment(c),
                Move::Rapid(to) => {
                    // Establish clearance before moving laterally from an unknown position.
                    if w.pos.is_none() {
                        let s = format!("G0 Z{}", w.num(to.z));
                        w.emit(&s);
                        let s = format!("G0 X{} Y{}", w.num(to.x), w.num(to.y));
                        w.emit(&s);
                        w.last_motion = Some("G0");
                        w.pos = Some(*to);
                    } else {
                        w.linear("G0", *to, None);
                    }
                }
                Move::Linear { to, feed } => w.linear("G1", *to, Some(*feed)),
                Move::Arc {
                    to,
                    center,
                    cw,
                    feed,
                } => {
                    let from = w.pos.unwrap_or(*to);
                    if cfg.arcs {
                        let g = if *cw { "G2" } else { "G3" };
                        // Always output a G2/G3 word: modal arcs confuse some controllers.
                        w.last_motion = Some(g);
                        let ax = w.axes(*to, false);
                        let ij = if cfg.arc_ij_absolute {
                            *center
                        } else {
                            *center - from.truncate()
                        };
                        let f = w.feed_word(*feed);
                        let s = format!("{g}{ax} I{} J{}{f}", w.num(ij.x), w.num(ij.y));
                        w.emit(&s);
                        w.pos = Some(*to);
                    } else {
                        for pt in arc_points(from, *to, *center, *cw, cfg.arc_tolerance) {
                            w.linear("G1", pt, Some(*feed));
                        }
                    }
                }
                Move::Drill {
                    at,
                    r_plane,
                    bottom,
                    clearance,
                    peck,
                    dwell,
                    cycle,
                    feed,
                } => {
                    if cfg.canned_cycles {
                        let (g, use_q, use_p) = match cycle {
                            DrillCycle::Simple => ("G81", false, false),
                            DrillCycle::Dwell => ("G82", false, true),
                            DrillCycle::Peck if *peck > 0.0 => ("G83", true, false),
                            DrillCycle::ChipBreak if *peck > 0.0 => ("G73", true, false),
                            DrillCycle::Peck | DrillCycle::ChipBreak => ("G81", false, false),
                            DrillCycle::BoreFeedOut => ("G85", false, false),
                            DrillCycle::BoreDwellFeedOut => ("G89", false, true),
                        };
                        // Initial Z (G98 return) is the clearance plane.
                        if w.pos
                            .map(|p| (p.z - clearance).abs() > 1e-9)
                            .unwrap_or(true)
                        {
                            let p = DVec3::new(at.x, at.y, *clearance);
                            w.linear("G0", p, None);
                        }
                        let mut s = format!(
                            "G98 {g} X{} Y{} Z{} R{}",
                            w.num(at.x),
                            w.num(at.y),
                            w.num(*bottom),
                            w.num(*r_plane)
                        );
                        if use_q {
                            let _ = write!(s, " Q{}", w.num(*peck));
                        }
                        if use_p {
                            let _ = write!(s, " P{}", dwell);
                        }
                        w.feed = None;
                        s += &w.feed_word(*feed);
                        w.emit(&s);
                        w.emit("G80");
                        w.last_motion = None;
                        w.pos = Some(DVec3::new(at.x, at.y, *clearance));
                    } else {
                        let tp = Toolpath {
                            moves: vec![m.clone()],
                            ..p.clone()
                        };
                        let start = w.pos.unwrap_or(DVec3::new(at.x, at.y, *clearance));
                        let mut dwelled = false;
                        for (_, to, rapid, f) in tp.segments(start, cfg.arc_tolerance) {
                            if rapid {
                                w.linear("G0", to, None);
                            } else {
                                w.linear("G1", to, Some(f));
                            }
                            if !dwelled
                                && !rapid
                                && (to.z - bottom).abs() < 1e-9
                                && *dwell > 0.0
                                && matches!(cycle, DrillCycle::Dwell | DrillCycle::BoreDwellFeedOut)
                            {
                                w.emit(&format!("G4 P{dwell}"));
                                w.last_motion = None;
                                dwelled = true;
                            }
                        }
                    }
                }
            }
        }
    }
    if spindle.is_some() {
        w.emit(&cfg.spindle_off);
    }
    if coolant != Coolant::Off {
        w.emit("M9");
    }
    for l in &cfg.postamble {
        w.emit(l);
    }
    w.emit(&cfg.program_end);
    if cfg.comments != CommentStyle::None {
        w.out.push_str("%\n");
    }
    w.out
}
