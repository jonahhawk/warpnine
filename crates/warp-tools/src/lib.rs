//! Cutting tools, holders, and a JSON-backed library. All lengths are millimetres.

use serde::{Deserialize, Serialize};
use std::path::Path;

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub enum ToolShape {
    FlatEndMill,
    BallEndMill,
    /// Flat end mill with a corner radius.
    BullNose { corner_radius: f64 },
    /// V-bit; `included_angle` in degrees (e.g. 60, 90). `tip_diameter` 0 for a sharp point.
    VBit { included_angle: f64, tip_diameter: f64 },
    /// Twist drill; `point_angle` in degrees (typically 118 or 135).
    Drill { point_angle: f64 },
    /// Chamfer mill; `angle` is the included angle in degrees.
    ChamferMill { angle: f64, tip_diameter: f64 },
    /// Spot / centre drill.
    SpotDrill { point_angle: f64 },
    /// Tapered ball end mill; `taper_angle` is per side in degrees.
    TaperedBall { taper_angle: f64, tip_radius: f64 },
    /// Single-point engraver / drag tool.
    Engraver { included_angle: f64, tip_diameter: f64 },
    FaceMill,
    ThreadMill { pitch: f64 },
    SlotCutter { thickness: f64 },
    Tap { pitch: f64 },
}

impl ToolShape {
    pub fn label(&self) -> &'static str {
        match self {
            ToolShape::FlatEndMill => "Flat end mill",
            ToolShape::BallEndMill => "Ball end mill",
            ToolShape::BullNose { .. } => "Bull nose",
            ToolShape::VBit { .. } => "V-bit",
            ToolShape::Drill { .. } => "Drill",
            ToolShape::ChamferMill { .. } => "Chamfer mill",
            ToolShape::SpotDrill { .. } => "Spot drill",
            ToolShape::TaperedBall { .. } => "Tapered ball",
            ToolShape::Engraver { .. } => "Engraver",
            ToolShape::FaceMill => "Face mill",
            ToolShape::ThreadMill { .. } => "Thread mill",
            ToolShape::SlotCutter { .. } => "Slot cutter",
            ToolShape::Tap { .. } => "Tap",
        }
    }

    pub fn all_defaults() -> Vec<ToolShape> {
        vec![
            ToolShape::FlatEndMill,
            ToolShape::BallEndMill,
            ToolShape::BullNose { corner_radius: 1.0 },
            ToolShape::VBit { included_angle: 90.0, tip_diameter: 0.0 },
            ToolShape::Drill { point_angle: 118.0 },
            ToolShape::ChamferMill { angle: 90.0, tip_diameter: 0.0 },
            ToolShape::SpotDrill { point_angle: 90.0 },
            ToolShape::TaperedBall { taper_angle: 5.0, tip_radius: 0.5 },
            ToolShape::Engraver { included_angle: 30.0, tip_diameter: 0.1 },
            ToolShape::FaceMill,
            ToolShape::ThreadMill { pitch: 1.0 },
            ToolShape::SlotCutter { thickness: 2.0 },
            ToolShape::Tap { pitch: 1.0 },
        ]
    }
}

/// Which division of the library a tool belongs to. Dimensions are always stored in
/// millimetres; the division records the system the tool was specified in, so inch tools stay
/// grouped, named and edited as inch tools.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize, Default)]
pub enum ToolUnits {
    #[default]
    Metric,
    Imperial,
}

impl ToolUnits {
    pub const ALL: [ToolUnits; 2] = [ToolUnits::Metric, ToolUnits::Imperial];

    pub fn label(&self) -> &'static str {
        match self {
            ToolUnits::Metric => "Metric",
            ToolUnits::Imperial => "Imperial",
        }
    }

    /// Millimetres per unit of this system.
    pub fn to_mm(&self) -> f64 {
        match self {
            ToolUnits::Metric => 1.0,
            ToolUnits::Imperial => 25.4,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Default)]
pub enum ToolMaterial {
    #[default]
    Carbide,
    Hss,
    Cobalt,
    Diamond,
    Ceramic,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Default)]
pub enum Coolant {
    #[default]
    Off,
    Flood,
    Mist,
    Air,
}

/// Default cutting parameters stored with a tool; operations copy these and may override them.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct CuttingData {
    pub spindle_rpm: f64,
    /// Cutting feed, mm/min.
    pub feed_rate: f64,
    /// Plunge feed, mm/min.
    pub plunge_rate: f64,
    /// Ramp feed, mm/min.
    pub ramp_rate: f64,
    /// Axial depth per pass, mm.
    pub stepdown: f64,
    /// Radial stepover as a fraction of diameter (0..1).
    pub stepover: f64,
    pub coolant: Coolant,
    pub clockwise: bool,
}

impl Default for CuttingData {
    fn default() -> Self {
        CuttingData {
            spindle_rpm: 18000.0,
            feed_rate: 1000.0,
            plunge_rate: 300.0,
            ramp_rate: 600.0,
            stepdown: 1.0,
            stepover: 0.4,
            coolant: Coolant::Off,
            clockwise: true,
        }
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Tool {
    /// Tool number used in `T` words. Unique within a library.
    pub number: u32,
    pub name: String,
    pub shape: ToolShape,
    /// Library division (metric or imperial). Libraries saved before this field existed load as
    /// metric.
    #[serde(default)]
    pub units: ToolUnits,
    pub diameter: f64,
    pub shank_diameter: f64,
    pub flute_length: f64,
    pub overall_length: f64,
    /// Length the tool protrudes from the holder.
    pub stickout: f64,
    pub flutes: u32,
    pub material: ToolMaterial,
    pub coating: String,
    pub manufacturer: String,
    pub part_number: String,
    pub holder: Option<String>,
    /// Tool length offset (H register).
    pub length_offset: u32,
    /// Diameter / cutter comp offset (D register).
    pub diameter_offset: u32,
    pub cutting: CuttingData,
    pub notes: String,
}

impl Tool {
    pub fn new(number: u32, name: impl Into<String>, shape: ToolShape, diameter: f64) -> Self {
        Tool {
            number,
            name: name.into(),
            shape,
            units: ToolUnits::Metric,
            diameter,
            shank_diameter: diameter,
            flute_length: diameter * 3.0,
            overall_length: diameter * 3.0 + 25.0,
            stickout: diameter * 3.0 + 5.0,
            flutes: 2,
            material: ToolMaterial::Carbide,
            coating: String::new(),
            manufacturer: String::new(),
            part_number: String::new(),
            holder: None,
            length_offset: number,
            diameter_offset: number,
            cutting: CuttingData::default(),
            notes: String::new(),
        }
    }

    /// An imperial tool specified in inches: `diameter_in` and the shape's lengths are converted
    /// to millimetres; the default shank, flute and overall lengths scale from the diameter.
    pub fn imperial(number: u32, name: impl Into<String>, shape: ToolShape, diameter_in: f64) -> Self {
        let mut t = Tool::new(number, name, shape, diameter_in * 25.4);
        t.units = ToolUnits::Imperial;
        t
    }

    pub fn radius(&self) -> f64 {
        self.diameter * 0.5
    }

    /// Height of the cutting profile above the tool tip at radial distance `r` from the axis,
    /// or `None` if `r` is outside the cutting diameter. This single function drives both
    /// drop-cutter toolpath generation and material-removal simulation for every tool shape.
    pub fn profile_height(&self, r: f64) -> Option<f64> {
        let rad = self.radius();
        if r > rad + 1e-9 {
            return None;
        }
        let cone = |half_angle_deg: f64, tip_d: f64| {
            let tip_r = tip_d * 0.5;
            if r <= tip_r {
                0.0
            } else {
                (r - tip_r) / half_angle_deg.to_radians().tan().max(1e-6)
            }
        };
        Some(match self.shape {
            ToolShape::FlatEndMill | ToolShape::FaceMill | ToolShape::ThreadMill { .. } | ToolShape::SlotCutter { .. } => 0.0,
            ToolShape::BallEndMill => rad - (rad * rad - r * r).max(0.0).sqrt(),
            ToolShape::BullNose { corner_radius } => {
                let cr = corner_radius.min(rad);
                let flat = rad - cr;
                if r <= flat {
                    0.0
                } else {
                    let d = r - flat;
                    cr - (cr * cr - d * d).max(0.0).sqrt()
                }
            }
            ToolShape::VBit { included_angle, tip_diameter }
            | ToolShape::ChamferMill { angle: included_angle, tip_diameter }
            | ToolShape::Engraver { included_angle, tip_diameter } => cone(included_angle * 0.5, tip_diameter),
            ToolShape::Drill { point_angle } | ToolShape::SpotDrill { point_angle } => cone(point_angle * 0.5, 0.0),
            ToolShape::Tap { .. } => 0.0,
            ToolShape::TaperedBall { taper_angle, tip_radius } => {
                let tr = tip_radius.min(rad);
                let a = taper_angle.to_radians();
                // Tangent point between the ball and the cone.
                let rt = tr * a.cos();
                if r <= rt {
                    tr - (tr * tr - r * r).max(0.0).sqrt()
                } else {
                    let zt = tr - tr * a.sin();
                    zt + (r - rt) / a.tan().max(1e-6)
                }
            }
        })
    }

    /// Depth at which a pointed tool reaches `width` (for V-carving / chamfering).
    pub fn depth_for_width(&self, width: f64) -> Option<f64> {
        self.profile_height(width * 0.5)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Default)]
pub enum HolderKind {
    #[default]
    Collet,
    Er11,
    Er16,
    Er20,
    Er25,
    Er32,
    Er40,
    Bt30,
    Bt40,
    Cat40,
    Hsk63,
    Iso20,
    Iso30,
    R8,
    Ttc,
    ShrinkFit,
    Hydraulic,
    EndMillHolder,
    Router,
}

/// One frustum segment of a holder, stacked from the tool end upward.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct HolderSegment {
    pub length: f64,
    pub lower_diameter: f64,
    pub upper_diameter: f64,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Holder {
    pub id: String,
    pub name: String,
    pub kind: HolderKind,
    pub segments: Vec<HolderSegment>,
    /// Gauge length from spindle face to holder nose.
    pub gauge_length: f64,
    pub notes: String,
}

impl Holder {
    pub fn length(&self) -> f64 {
        self.segments.iter().map(|s| s.length).sum()
    }

    pub fn max_diameter(&self) -> f64 {
        self.segments.iter().map(|s| s.lower_diameter.max(s.upper_diameter)).fold(0.0, f64::max)
    }
}

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct ToolLibrary {
    pub tools: Vec<Tool>,
    pub holders: Vec<Holder>,
    /// Names of the built-in starter sets that have been merged into this library (see
    /// [`ToolLibrary::seed_starter_sets`]). Libraries saved before this field existed all began
    /// from the metric set, so they load as having that one.
    #[serde(default = "legacy_seeded")]
    pub seeded: Vec<String>,
}

fn legacy_seeded() -> Vec<String> {
    vec![STARTER_METRIC.into()]
}

/// Name of the metric starter set in [`ToolLibrary::seeded`].
pub const STARTER_METRIC: &str = "metric";
/// Name of the imperial starter set in [`ToolLibrary::seeded`].
pub const STARTER_IMPERIAL: &str = "imperial";

impl ToolLibrary {
    pub fn load(path: &Path) -> anyhow::Result<Self> {
        Ok(serde_json::from_str(&std::fs::read_to_string(path)?)?)
    }

    pub fn save(&self, path: &Path) -> anyhow::Result<()> {
        if let Some(dir) = path.parent() {
            std::fs::create_dir_all(dir)?;
        }
        let tmp = path.with_extension("tmp");
        std::fs::write(&tmp, serde_json::to_string_pretty(self)?)?;
        std::fs::rename(tmp, path)?;
        Ok(())
    }

    /// Default library location: `%APPDATA%\WarpNine\tools.json` on Windows,
    /// `~/.config/warpnine/tools.json` elsewhere.
    pub fn default_path() -> std::path::PathBuf {
        if let Some(appdata) = std::env::var_os("APPDATA") {
            return std::path::PathBuf::from(appdata).join("WarpNine").join("tools.json");
        }
        let home = std::env::var_os("HOME").map(std::path::PathBuf::from).unwrap_or_default();
        home.join(".config").join("warpnine").join("tools.json")
    }

    pub fn load_or_default() -> Self {
        Self::load(&Self::default_path()).unwrap_or_else(|_| Self::starter())
    }

    /// Merge every built-in starter set this library has not seen yet and record it, so a set
    /// added in a later version appears once in an existing library and tools the user has since
    /// deleted or changed are not re-added. Returns the numbers of the tools added.
    pub fn seed_starter_sets(&mut self) -> Vec<u32> {
        let mut added = Vec::new();
        for (name, set) in [(STARTER_METRIC, Self::starter_metric as fn() -> ToolLibrary), (STARTER_IMPERIAL, Self::starter_imperial)] {
            if !self.seeded.iter().any(|s| s == name) {
                added.extend(self.merge(set()));
                self.seeded.push(name.into());
            }
        }
        added
    }

    pub fn tool(&self, number: u32) -> Option<&Tool> {
        self.tools.iter().find(|t| t.number == number)
    }

    pub fn holder(&self, id: &str) -> Option<&Holder> {
        self.holders.iter().find(|h| h.id == id)
    }

    pub fn next_number(&self) -> u32 {
        self.tools.iter().map(|t| t.number).max().unwrap_or(0) + 1
    }

    /// Tools of one division, in number order.
    pub fn in_units(&self, units: ToolUnits) -> impl Iterator<Item = &Tool> {
        self.tools.iter().filter(move |t| t.units == units)
    }

    /// Add every tool and holder of `other`. A tool whose number is already taken by a different
    /// tool is renumbered to the next free number, so nothing in either library is lost; a tool
    /// identical to an existing one is skipped. Returns the numbers of the tools added.
    pub fn merge(&mut self, other: ToolLibrary) -> Vec<u32> {
        let mut added = Vec::new();
        for mut t in other.tools {
            match self.tool(t.number) {
                Some(existing) if *existing == t => continue,
                Some(_) => t.number = self.next_number(),
                None => {}
            }
            added.push(t.number);
            self.upsert(t);
        }
        for h in other.holders {
            if self.holder(&h.id).is_none() {
                self.upsert_holder(h);
            }
        }
        added
    }

    /// Insert or replace a tool by number.
    pub fn upsert(&mut self, tool: Tool) {
        match self.tools.iter_mut().find(|t| t.number == tool.number) {
            Some(t) => *t = tool,
            None => {
                self.tools.push(tool);
                self.tools.sort_by_key(|t| t.number);
            }
        }
    }

    pub fn upsert_holder(&mut self, holder: Holder) {
        match self.holders.iter_mut().find(|h| h.id == holder.id) {
            Some(h) => *h = holder,
            None => self.holders.push(holder),
        }
    }

    /// A small starter library so a fresh install has something to cut with: the metric set
    /// (T1-T9) and the imperial set (T101-T114).
    pub fn starter() -> Self {
        let mut lib = ToolLibrary::default();
        lib.seed_starter_sets();
        lib
    }

    /// The metric starter set, T1-T9, with an ER20 holder.
    pub fn starter_metric() -> Self {
        let mut lib = ToolLibrary { seeded: vec![STARTER_METRIC.into()], ..Default::default() };
        let mut add = |n, name: &str, shape, d: f64, rpm, feed, plunge, sd, so| {
            let mut t = Tool::new(n, name, shape, d);
            t.cutting.spindle_rpm = rpm;
            t.cutting.feed_rate = feed;
            t.cutting.plunge_rate = plunge;
            t.cutting.ramp_rate = feed * 0.5;
            t.cutting.stepdown = sd;
            t.cutting.stepover = so;
            t.holder = Some("er20".into());
            lib.tools.push(t);
        };
        add(1, "6mm 2F flat", ToolShape::FlatEndMill, 6.0, 18000.0, 1500.0, 400.0, 2.0, 0.4);
        add(2, "3mm 2F flat", ToolShape::FlatEndMill, 3.0, 20000.0, 900.0, 250.0, 1.0, 0.4);
        add(3, "6mm ball", ToolShape::BallEndMill, 6.0, 18000.0, 1500.0, 400.0, 1.0, 0.1);
        add(4, "3mm ball", ToolShape::BallEndMill, 3.0, 20000.0, 1000.0, 300.0, 0.5, 0.08);
        add(5, "90° V-bit", ToolShape::VBit { included_angle: 90.0, tip_diameter: 0.0 }, 12.7, 16000.0, 1200.0, 300.0, 1.5, 0.2);
        add(6, "60° V-bit", ToolShape::VBit { included_angle: 60.0, tip_diameter: 0.0 }, 12.7, 16000.0, 1200.0, 300.0, 1.5, 0.2);
        add(7, "3mm drill", ToolShape::Drill { point_angle: 118.0 }, 3.0, 3000.0, 200.0, 150.0, 1.0, 0.0);
        add(8, "6mm bull 1mm R", ToolShape::BullNose { corner_radius: 1.0 }, 6.0, 18000.0, 1500.0, 400.0, 1.5, 0.3);
        add(9, "25mm surfacing", ToolShape::FaceMill, 25.0, 14000.0, 2500.0, 300.0, 0.5, 0.7);
        lib.holders.push(Holder {
            id: "er20".into(),
            name: "ER20 collet chuck".into(),
            kind: HolderKind::Er20,
            segments: vec![
                HolderSegment { length: 20.0, lower_diameter: 34.0, upper_diameter: 34.0 },
                HolderSegment { length: 25.0, lower_diameter: 40.0, upper_diameter: 40.0 },
            ],
            gauge_length: 45.0,
            notes: String::new(),
        });
        lib
    }

    /// The imperial starter set, T101-T114. Every dimension is given in inches and stored in
    /// millimetres; feeds are inch-per-minute figures converted the same way.
    pub fn starter_imperial() -> Self {
        const IN: f64 = 25.4;
        let mut lib = ToolLibrary { seeded: vec![STARTER_IMPERIAL.into()], ..Default::default() };
        // (number, name, shape, diameter in, flutes, rpm, feed ipm, plunge ipm, stepdown in, stepover)
        let tool = |n, name: &str, shape, d: f64, flutes, rpm, feed: f64, plunge: f64, sd: f64, so| -> Tool {
            let mut t = Tool::imperial(n, name, shape, d);
            t.flutes = flutes;
            t.cutting.spindle_rpm = rpm;
            t.cutting.feed_rate = feed * IN;
            t.cutting.plunge_rate = plunge * IN;
            t.cutting.ramp_rate = feed * IN * 0.5;
            t.cutting.stepdown = sd * IN;
            t.cutting.stepover = so;
            t.holder = Some("er20".into());
            t
        };
        let flat = ToolShape::FlatEndMill;
        let ball = ToolShape::BallEndMill;
        lib.tools.push(tool(101, "1/8 in 2F flat", flat, 0.125, 2, 18000.0, 40.0, 15.0, 0.0625, 0.4));
        lib.tools.push(tool(102, "1/4 in 2F flat", flat, 0.25, 2, 18000.0, 80.0, 25.0, 0.125, 0.4));
        lib.tools.push(tool(103, "3/8 in 2F flat", flat, 0.375, 2, 16000.0, 100.0, 30.0, 0.1875, 0.4));
        lib.tools.push(tool(104, "1/2 in 2F flat", flat, 0.5, 2, 14000.0, 120.0, 30.0, 0.25, 0.4));
        lib.tools.push(tool(105, "1/8 in ball", ball, 0.125, 2, 18000.0, 40.0, 15.0, 0.03125, 0.1));
        lib.tools.push(tool(106, "1/4 in ball", ball, 0.25, 2, 18000.0, 80.0, 25.0, 0.0625, 0.1));
        lib.tools.push(tool(107, "3/8 in ball", ball, 0.375, 2, 16000.0, 100.0, 30.0, 0.09375, 0.1));
        lib.tools.push(tool(108, "1/2 in ball", ball, 0.5, 2, 14000.0, 120.0, 30.0, 0.125, 0.1));
        let mut t = tool(109, "1/4 in 3F rougher", flat, 0.25, 3, 18000.0, 100.0, 25.0, 0.25, 0.5);
        t.notes = "Corncob roughing end mill".into();
        lib.tools.push(t);
        let mut t = tool(110, "3/8 in 3F rougher", flat, 0.375, 3, 16000.0, 120.0, 30.0, 0.375, 0.5);
        t.notes = "Corncob roughing end mill".into();
        lib.tools.push(t);
        lib.tools.push(tool(111, "90 deg V-mill 1/2 in", ToolShape::VBit { included_angle: 90.0, tip_diameter: 0.0 }, 0.5, 2, 16000.0, 60.0, 15.0, 0.0625, 0.2));
        // 1/4 in shank tapered ball: 0.075 in tip, 5 deg per side reaches 1/4 in at a 1 in flute.
        let mut t = tool(112, "1/4 to 0.075 in tapered carving", ToolShape::TaperedBall { taper_angle: 5.0, tip_radius: 0.0375 * IN }, 0.25, 2, 18000.0, 60.0, 15.0, 0.1, 0.1);
        t.flute_length = t.profile_height(t.radius()).unwrap(); // about 1.04 in: where the taper reaches 1/4 in
        t.overall_length = 2.5 * IN;
        t.stickout = 1.5 * IN;
        lib.tools.push(t);
        let mut t = tool(113, "2 in 3F surfacing", ToolShape::FaceMill, 2.0, 3, 12000.0, 150.0, 10.0, 0.03, 0.7);
        t.shank_diameter = 0.5 * IN;
        t.flute_length = 0.25 * IN;
        t.overall_length = 2.0 * IN;
        t.stickout = 1.25 * IN;
        lib.tools.push(t);
        // Router-style T-slot cutter: 1/4 in shank, 1/2 in cutting diameter, 1/8 in slot width.
        let mut t = tool(114, "1/4 in T-slot (1/2 x 1/8)", ToolShape::SlotCutter { thickness: 0.125 * IN }, 0.5, 3, 10000.0, 20.0, 5.0, 0.125, 0.3);
        t.shank_diameter = 0.25 * IN;
        t.flute_length = 0.125 * IN;
        t.overall_length = 2.0 * IN;
        t.stickout = 1.0 * IN;
        lib.tools.push(t);
        lib.holders.push(Holder {
            id: "er20".into(),
            name: "ER20 collet chuck".into(),
            kind: HolderKind::Er20,
            segments: vec![
                HolderSegment { length: 20.0, lower_diameter: 34.0, upper_diameter: 34.0 },
                HolderSegment { length: 25.0, lower_diameter: 40.0, upper_diameter: 40.0 },
            ],
            gauge_length: 45.0,
            notes: String::new(),
        });
        lib
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn profiles() {
        let ball = Tool::new(1, "b", ToolShape::BallEndMill, 6.0);
        assert_eq!(ball.profile_height(0.0), Some(0.0));
        assert!((ball.profile_height(3.0).unwrap() - 3.0).abs() < 1e-9);
        assert_eq!(ball.profile_height(3.1), None);

        let v = Tool::new(2, "v", ToolShape::VBit { included_angle: 90.0, tip_diameter: 0.0 }, 10.0);
        assert!((v.profile_height(2.0).unwrap() - 2.0).abs() < 1e-9);

        let tb = Tool::new(3, "tb", ToolShape::TaperedBall { taper_angle: 10.0, tip_radius: 0.5 }, 6.0);
        // continuous at the tangent point
        let rt = 0.5 * 10f64.to_radians().cos();
        let a = tb.profile_height(rt - 1e-7).unwrap();
        let b = tb.profile_height(rt + 1e-7).unwrap();
        assert!((a - b).abs() < 1e-5);
    }

    #[test]
    fn roundtrip_json() {
        let lib = ToolLibrary::starter();
        let s = serde_json::to_string(&lib).unwrap();
        let back: ToolLibrary = serde_json::from_str(&s).unwrap();
        assert_eq!(back.tools.len(), lib.tools.len());
        assert_eq!(back.next_number(), 115);
        assert_eq!(ToolLibrary::starter_metric().next_number(), 10);
        assert!(back.tools.iter().all(|t| lib.tool(t.number) == Some(t)));
    }

    #[test]
    fn libraries_without_a_units_field_load_as_metric() {
        let mut lib = ToolLibrary::starter_metric();
        lib.tools.truncate(1);
        let mut v: serde_json::Value = serde_json::to_value(&lib).unwrap();
        let t = &mut v["tools"][0];
        assert!(t.as_object_mut().unwrap().remove("units").is_some());
        let back: ToolLibrary = serde_json::from_value(v).unwrap();
        assert_eq!(back.tools[0].units, ToolUnits::Metric);
        assert_eq!(back.tools[0], lib.tools[0]);
    }

    #[test]
    fn imperial_starter_set_matches_its_inch_specification() {
        const IN: f64 = 25.4;
        let lib = ToolLibrary::starter_imperial();
        assert_eq!(lib.tools.len(), 14);
        assert!(lib.tools.iter().all(|t| t.units == ToolUnits::Imperial));
        let mut numbers: Vec<u32> = lib.tools.iter().map(|t| t.number).collect();
        numbers.dedup();
        assert_eq!(numbers.len(), 14, "tool numbers are unique");
        let near = |a: f64, b: f64| (a - b).abs() < 1e-9;
        // Flat and ball mills in the four fractional sizes.
        for (d, flat_n, ball_n) in [(0.125, 101, 105), (0.25, 102, 106), (0.375, 103, 107), (0.5, 104, 108)] {
            let f = lib.tool(flat_n).unwrap();
            let b = lib.tool(ball_n).unwrap();
            assert_eq!(f.shape, ToolShape::FlatEndMill);
            assert_eq!(b.shape, ToolShape::BallEndMill);
            assert!(near(f.diameter, d * IN) && near(b.diameter, d * IN), "{d} in");
            assert_eq!((f.flutes, b.flutes), (2, 2));
            // The ball profile reaches its radius at the edge, like any sphere of that diameter.
            assert!(near(b.profile_height(b.radius()).unwrap(), d * IN * 0.5));
        }
        for (d, n) in [(0.25, 109), (0.375, 110)] {
            let t = lib.tool(n).unwrap();
            assert_eq!((t.shape, t.flutes), (ToolShape::FlatEndMill, 3));
            assert!(near(t.diameter, d * IN));
        }
        // 90 degree V: depth equals half the width.
        let v = lib.tool(111).unwrap();
        assert!(near(v.diameter, 0.5 * IN));
        assert!(near(v.depth_for_width(0.2 * IN).unwrap(), 0.1 * IN));
        // Tapered carving mill: 0.075 in tip ball, 1/4 in at the top of a 1 in flute.
        let c = lib.tool(112).unwrap();
        let ToolShape::TaperedBall { taper_angle, tip_radius } = c.shape else { panic!("{:?}", c.shape) };
        assert!(near(tip_radius * 2.0, 0.075 * IN));
        assert!(near(c.diameter, 0.25 * IN));
        let top = c.profile_height(c.radius()).unwrap();
        assert!((top - 1.0 * IN).abs() < 0.05 * IN, "taper {taper_angle} deg reaches full diameter at {} in", top / IN);
        assert!(near(c.flute_length, top), "the flute ends where the taper reaches full diameter");
        // Surfacing cutter: 2 in, three inserts, 1/2 in shank.
        let s = lib.tool(113).unwrap();
        assert_eq!((s.shape, s.flutes), (ToolShape::FaceMill, 3));
        assert!(near(s.diameter, 2.0 * IN) && near(s.shank_diameter, 0.5 * IN));
        // T-slot cutter: 1/4 in shank under a wider disc.
        let ts = lib.tool(114).unwrap();
        let ToolShape::SlotCutter { thickness } = ts.shape else { panic!("{:?}", ts.shape) };
        assert!(near(ts.shank_diameter, 0.25 * IN));
        assert!(ts.diameter > ts.shank_diameter);
        assert!(near(thickness, 0.125 * IN) && near(ts.flute_length, thickness));
        // Feeds were specified in inches per minute.
        let f = lib.tool(102).unwrap();
        assert!(near(f.cutting.feed_rate, 80.0 * IN) && near(f.cutting.stepdown, 0.125 * IN));
    }

    #[test]
    fn existing_libraries_receive_the_imperial_set_once() {
        // A library saved before divisions existed: metric tools, one of them edited, one deleted,
        // one the user added on T10, and no `seeded` field.
        let mut old = ToolLibrary::starter_metric();
        old.tools.retain(|t| t.number != 7);
        old.tools.iter_mut().find(|t| t.number == 3).unwrap().diameter = 6.1;
        old.tools.push(Tool::new(10, "18mm 2F flat", ToolShape::FlatEndMill, 18.0));
        let mut v = serde_json::to_value(&old).unwrap();
        v.as_object_mut().unwrap().remove("seeded");
        let mut lib: ToolLibrary = serde_json::from_value(v).unwrap();
        assert_eq!(lib.seeded, vec![STARTER_METRIC.to_string()]);

        let added = lib.seed_starter_sets();
        assert_eq!(added, (101..=114).collect::<Vec<u32>>());
        assert_eq!(lib.seeded, vec![STARTER_METRIC.to_string(), STARTER_IMPERIAL.to_string()]);
        // The user's metric changes survive: nothing re-added, nothing reverted.
        assert!(lib.tool(7).is_none());
        assert_eq!(lib.tool(3).unwrap().diameter, 6.1);
        assert_eq!(lib.in_units(ToolUnits::Metric).count(), old.tools.len());
        // Seeding again is a no-op, even after the user deletes an imperial tool.
        lib.tools.retain(|t| t.number != 111);
        assert!(lib.seed_starter_sets().is_empty());
        assert!(lib.tool(111).is_none());
        // A fresh library gets both sets.
        let fresh = ToolLibrary::starter();
        assert_eq!(fresh.in_units(ToolUnits::Metric).count(), 9);
        assert_eq!(fresh.in_units(ToolUnits::Imperial).count(), 14);
        assert_eq!(fresh.holders.len(), 1);
    }

    #[test]
    fn merge_keeps_both_libraries_and_renumbers_collisions() {
        let mut lib = ToolLibrary::starter_metric();
        let count = lib.tools.len();
        // Merging the same set again changes nothing.
        assert!(lib.merge(ToolLibrary::starter_metric()).is_empty());
        assert_eq!(lib.tools.len(), count);
        // A different tool on a taken number is kept under a free number.
        let mut other = ToolLibrary::default();
        other.tools.push(Tool::imperial(1, "1/4 in flat", ToolShape::FlatEndMill, 0.25));
        other.tools.push(Tool::imperial(200, "1/2 in flat", ToolShape::FlatEndMill, 0.5));
        let added = lib.merge(other);
        assert_eq!(added, vec![10, 200]);
        assert_eq!(lib.tool(1).unwrap().name, "6mm 2F flat");
        assert_eq!(lib.tool(10).unwrap().name, "1/4 in flat");
        assert_eq!(lib.in_units(ToolUnits::Imperial).count(), 2);
        assert_eq!(lib.in_units(ToolUnits::Metric).count(), count);
    }
}
