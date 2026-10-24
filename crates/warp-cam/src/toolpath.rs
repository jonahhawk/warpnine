//! Toolpath representation and a builder that handles safe rapids, plunges and links.

use glam::{DVec2, DVec3};
use serde::{Deserialize, Serialize};
use warp_tools::Coolant;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Default)]
pub enum DrillCycle {
    /// G81
    #[default]
    Simple,
    /// G82 (dwell at bottom)
    Dwell,
    /// G83 full-retract peck
    Peck,
    /// G73 chip-break peck
    ChipBreak,
    /// G85 bore, feed out
    BoreFeedOut,
    /// G89 bore, dwell, feed out
    BoreDwellFeedOut,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum Move {
    Rapid(DVec3),
    /// Cutting move at the given feed (mm/min).
    Linear {
        to: DVec3,
        feed: f64,
    },
    /// XY-plane arc or helix; `center` is absolute (its Z is ignored).
    Arc {
        to: DVec3,
        center: DVec2,
        cw: bool,
        feed: f64,
    },
    Drill {
        at: DVec2,
        /// Retract (R) plane.
        r_plane: f64,
        bottom: f64,
        /// Height to return to after the hole (initial Z for G98).
        clearance: f64,
        peck: f64,
        dwell: f64,
        cycle: DrillCycle,
        feed: f64,
    },
    Comment(String),
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Toolpath {
    #[serde(default = "default_wcs")]
    pub wcs: u8,
    pub name: String,
    pub tool: u32,
    pub rpm: f64,
    pub clockwise: bool,
    pub coolant: Coolant,
    pub moves: Vec<Move>,
    /// Checks the project relaxed that this path trips (see `Project::checks`). The path is
    /// valid to post; the operator is told what was waived.
    #[serde(default)]
    pub warnings: Vec<String>,
}
fn default_wcs() -> u8 {
    54
}

impl Toolpath {
    /// Expand canned drill cycles and arcs into straight segments: (start, end, is_rapid, feed).
    pub fn segments(&self, start: DVec3, tol: f64) -> Vec<(DVec3, DVec3, bool, f64)> {
        let mut out = Vec::new();
        let mut cur = start;
        let mut push = |to: DVec3, rapid: bool, feed: f64, cur: &mut DVec3| {
            out.push((*cur, to, rapid, feed));
            *cur = to;
        };
        for m in &self.moves {
            match *m {
                Move::Rapid(p) => push(p, true, 0.0, &mut cur),
                Move::Linear { to, feed } => push(to, false, feed, &mut cur),
                Move::Arc {
                    to,
                    center,
                    cw,
                    feed,
                } => {
                    for p in arc_points(cur, to, center, cw, tol) {
                        push(p, false, feed, &mut cur);
                    }
                }
                Move::Drill {
                    at,
                    r_plane,
                    bottom,
                    clearance,
                    peck,
                    cycle,
                    feed,
                    ..
                } => {
                    let p = |z| DVec3::new(at.x, at.y, z);
                    push(p(cur.z.max(r_plane)), true, 0.0, &mut cur);
                    push(p(r_plane), true, 0.0, &mut cur);
                    if peck > 0.0 && matches!(cycle, DrillCycle::Peck | DrillCycle::ChipBreak) {
                        let mut z = r_plane;
                        while z > bottom + 1e-9 {
                            z = (z - peck).max(bottom);
                            push(p(z), false, feed, &mut cur);
                            if z > bottom + 1e-9 {
                                if cycle == DrillCycle::Peck {
                                    push(p(r_plane), true, 0.0, &mut cur);
                                }
                                push(p(z + peck.min(0.5)), true, 0.0, &mut cur);
                            }
                        }
                    } else {
                        push(p(bottom), false, feed, &mut cur);
                    }
                    let out_rapid = !matches!(
                        cycle,
                        DrillCycle::BoreFeedOut | DrillCycle::BoreDwellFeedOut
                    );
                    push(p(clearance), out_rapid, feed, &mut cur);
                }
                Move::Comment(_) => {}
            }
        }
        out
    }

    pub fn cut_length(&self) -> f64 {
        self.segments(DVec3::ZERO, 0.01)
            .iter()
            .filter(|s| !s.2)
            .map(|s| s.0.distance(s.1))
            .sum()
    }

    pub fn end_position(&self) -> Option<DVec3> {
        self.moves.iter().rev().find_map(|m| match *m {
            Move::Rapid(p) => Some(p),
            Move::Linear { to, .. } | Move::Arc { to, .. } => Some(to),
            Move::Drill { at, clearance, .. } => Some(DVec3::new(at.x, at.y, clearance)),
            Move::Comment(_) => None,
        })
    }
}

/// Points along an XY arc/helix from `from` to `to` (excluding `from`).
pub fn arc_points(from: DVec3, to: DVec3, center: DVec2, cw: bool, tol: f64) -> Vec<DVec3> {
    let r = (from.truncate() - center).length();
    let a0 = (from.truncate() - center).to_angle();
    let mut a1 = (to.truncate() - center).to_angle();
    if cw {
        while a1 >= a0 - 1e-9 {
            a1 -= std::f64::consts::TAU;
        }
    } else {
        while a1 <= a0 + 1e-9 {
            a1 += std::f64::consts::TAU;
        }
    }
    let n = warp_core::geom::arc_segments(r, a1 - a0, tol).max(2);
    (1..=n)
        .map(|i| {
            let t = i as f64 / n as f64;
            if i == n {
                to
            } else {
                let p = center + DVec2::from_angle(a0 + (a1 - a0) * t) * r;
                p.extend(from.z + (to.z - from.z) * t)
            }
        })
        .collect()
}

/// Feeds and heights a builder needs to link cuts safely.
#[derive(Debug, Clone, Copy)]
pub struct Links {
    /// Rapid height used between disconnected cuts within an operation.
    pub safe_z: f64,
    /// Height above which rapid moves are allowed before plunging (feed from here down).
    pub feed_z: f64,
    pub feed: f64,
    pub plunge: f64,
    /// Max distance for linking two cuts at depth with a feed move instead of retracting.
    pub link_distance: f64,
}

pub struct PathBuilder {
    pub moves: Vec<Move>,
    pub pos: Option<DVec3>,
    pub links: Links,
    pub policy: crate::planner::LinkPolicy,
    pub ramp_feed: f64,
    pub helix_entry: Option<crate::ops::HelixEntry>,
    error: Option<String>,
}

impl PathBuilder {
    pub fn new(links: Links) -> Self {
        PathBuilder {
            moves: Vec::new(),
            pos: None,
            policy: Default::default(),
            helix_entry: None,
            error: None,
            ramp_feed: links.feed.min(links.plunge.max(links.feed * 0.5)),
            links,
        }
    }

    pub fn comment(&mut self, s: impl Into<String>) {
        self.moves.push(Move::Comment(s.into()));
    }

    pub fn rapid(&mut self, p: DVec3) {
        if self.pos != Some(p) {
            self.moves.push(Move::Rapid(p));
            self.pos = Some(p);
        }
    }

    pub fn feed(&mut self, p: DVec3) {
        self.feed_at(p, self.links.feed);
    }

    pub fn feed_at(&mut self, p: DVec3, f: f64) {
        if self.pos.is_some_and(|c| c.distance(p) < 1e-7) {
            return;
        }
        self.moves.push(Move::Linear { to: p, feed: f });
        self.pos = Some(p);
    }

    pub fn arc(&mut self, to: DVec3, center: DVec2, cw: bool) {
        self.moves.push(Move::Arc {
            to,
            center,
            cw,
            feed: self.links.feed,
        });
        self.pos = Some(to);
    }

    pub fn retract(&mut self) {
        if let Some(p) = self.pos {
            if p.z < self.links.safe_z {
                // Withdraw through the potentially engaged part of the stock at feed.
                let exit = self.links.feed_z.min(self.links.safe_z);
                if p.z < exit {
                    self.feed_at(DVec3::new(p.x, p.y, exit), self.links.plunge);
                }
                self.rapid(DVec3::new(p.x, p.y, self.links.safe_z));
            }
        }
    }

    /// Move to `p` (which is at cutting depth) from wherever we are: link directly if close
    /// and allowed, otherwise retract, rapid over, rapid down to feed height, and plunge.
    pub fn go_to(&mut self, p: DVec3, allow_link: bool) {
        match self.pos {
            Some(c)
                if allow_link
                    && self.policy.permits(c, p)
                    && c.truncate().distance(p.truncate()) <= self.links.link_distance
                    && (c.z - p.z).abs() < 1e-6 =>
            {
                self.feed(p);
            }
            _ => {
                self.retract();
                let s = self.links.safe_z;
                self.rapid(DVec3::new(p.x, p.y, s));
                let fz = self.links.feed_z.max(p.z);
                self.rapid(DVec3::new(p.x, p.y, fz));
                let f = self.links.plunge;
                self.feed_at(p, f);
            }
        }
    }

    /// Follow a 3D polyline, entering at its first point.
    pub fn polyline(&mut self, pts: &[DVec3], allow_link: bool) {
        let Some(&first) = pts.first() else { return };
        self.go_to(first, allow_link);
        for &p in &pts[1..] {
            self.feed(p);
        }
    }

    /// Follow a closed 2D loop at depth `z`, ramping down from `z_from` over the loop at
    /// `ramp_angle_deg` (0 = straight plunge), and re-cutting the ramp section at full depth.
    pub fn ramp_loop(
        &mut self,
        lp: &[DVec2],
        z_from: f64,
        z: f64,
        ramp_angle_deg: f64,
        allow_link: bool,
    ) {
        if lp.len() < 2 {
            return;
        }
        if self.error.is_some() {
            return;
        }
        if let Some(entry) = self.helix_entry.filter(|_| z_from > z + 1e-9) {
            if let Err(e) = self.enter_helix(lp, z_from, z, entry) {
                self.error = Some(format!("{e:#}"));
                return;
            }
            for p in lp[1..].iter().chain(std::iter::once(&lp[0])) {
                self.feed(p.extend(z));
            }
            return;
        }
        let dz = z_from - z;
        if ramp_angle_deg <= 0.0 || dz <= 1e-9 {
            self.go_to(lp[0].extend(z), allow_link);
            for p in lp[1..].iter().chain(std::iter::once(&lp[0])) {
                self.feed(p.extend(z));
            }
            return;
        }
        let perim: f64 = lp
            .iter()
            .zip(lp.iter().cycle().skip(1))
            .map(|(a, b)| a.distance(*b))
            .sum();
        if perim < 1e-6 {
            return;
        }
        let ramp_len = dz / ramp_angle_deg.to_radians().tan();
        // Go around as many times as needed to descend.
        self.go_to(lp[0].extend(z_from), allow_link);
        let n = lp.len();
        let mut travelled = 0.0;
        let mut i = 0;
        let mut zc = z_from;
        while zc > z + 1e-9 {
            let a = lp[i % n];
            let b = lp[(i + 1) % n];
            let l = a.distance(b);
            travelled += l;
            zc = (z_from - dz * travelled / ramp_len).max(z);
            let f = self.ramp_feed;
            self.feed_at(b.extend(zc), f);
            i += 1;
        }
        // Full loop at depth from where the ramp ended.
        for k in 0..=n {
            self.feed(lp[(i + k) % n].extend(z));
        }
    }

    /// Ramp back and forth along an open chain, then follow its selected direction at depth.
    pub fn ramp_open(&mut self, points: &[DVec2], top: f64, bottom: f64, angle: f64) {
        if points.len() < 2 {
            return;
        }
        if angle <= 0. || top <= bottom {
            self.polyline(
                &points.iter().map(|p| p.extend(bottom)).collect::<Vec<_>>(),
                false,
            );
            return;
        }
        self.go_to(points[0].extend(top), false);
        let mut z = top;
        let mut reverse = false;
        while z > bottom + 1e-9 {
            if let Err(e) = crate::workflow::check_cancel() {
                self.error = Some(e.to_string());
                return;
            }
            let ordered: Vec<_> = if reverse {
                points.iter().rev().copied().collect()
            } else {
                points.to_vec()
            };
            for pair in ordered.windows(2) {
                z = (z - pair[0].distance(pair[1]) * angle.to_radians().tan()).max(bottom);
                self.feed_at(pair[1].extend(z), self.ramp_feed);
            }
            reverse = !reverse;
        }
        self.polyline(
            &points.iter().map(|p| p.extend(bottom)).collect::<Vec<_>>(),
            false,
        );
    }

    fn enter_helix(
        &mut self,
        lp: &[DVec2],
        top: f64,
        bottom: f64,
        entry: crate::ops::HelixEntry,
    ) -> anyhow::Result<()> {
        let area = self.policy.centre_region.as_ref().ok_or_else(|| {
            anyhow::anyhow!("Helical entry requires a bounded pocket tool-centre region")
        })?;
        let inset = crate::region::offset(area, -entry.radius - 1e-6);
        let mut candidates: Vec<DVec2> = crate::region::loops(&inset, true)
            .into_iter()
            .flatten()
            .collect();
        candidates.sort_by(|a, b| {
            a.distance_squared(lp[0])
                .total_cmp(&b.distance_squared(lp[0]))
        });
        let (centre,entry_start)=candidates.into_iter().find_map(|c|{
            let start=c+(lp[0]-c).normalize_or_zero()*entry.radius;
            crate::region::covers_segment(area,start,lp[0]).then_some((c,start))
        }).ok_or_else(||anyhow::anyhow!("Requested entry helix does not fit this pocket; reduce radius or choose ramp/plunge"))?;
        self.go_to(entry_start.extend(top), false);
        let start = (entry_start - centre).to_angle();
        let mut angle = start;
        let mut z = top;
        let feed = self.links.feed;
        self.links.feed = self.ramp_feed;
        while z > bottom + 1e-9 {
            crate::workflow::check_cancel()?;
            let dz = (z - bottom).min(entry.pitch * 0.5);
            z -= dz;
            angle += std::f64::consts::TAU * dz / entry.pitch;
            self.arc(
                (centre + DVec2::from_angle(angle) * entry.radius).extend(z),
                centre,
                false,
            );
        }
        if self.pos.unwrap().truncate().distance(entry_start) > 1e-8 {
            self.arc(entry_start.extend(bottom), centre, false);
        }
        self.feed(lp[0].extend(bottom));
        self.links.feed = feed;
        Ok(())
    }
    pub fn finish(mut self) -> anyhow::Result<Vec<Move>> {
        if let Some(e) = self.error {
            anyhow::bail!(e);
        }
        self.retract();
        Ok(self.moves)
    }
}

/// Douglas–Peucker simplification of a 3D polyline.
pub fn simplify(pts: &[DVec3], tol: f64) -> Vec<DVec3> {
    if pts.len() < 3 {
        return pts.to_vec();
    }
    let mut keep = vec![false; pts.len()];
    keep[0] = true;
    keep[pts.len() - 1] = true;
    let mut stack = vec![(0usize, pts.len() - 1)];
    while let Some((s, e)) = stack.pop() {
        let (a, b) = (pts[s], pts[e]);
        let ab = b - a;
        let l2 = ab.length_squared();
        let mut best = (0.0, 0);
        for (i, &p) in pts.iter().enumerate().take(e).skip(s + 1) {
            let t = if l2 > 0.0 {
                ((p - a).dot(ab) / l2).clamp(0.0, 1.0)
            } else {
                0.0
            };
            let d = p.distance(a + ab * t);
            if d > best.0 {
                best = (d, i);
            }
        }
        if best.0 > tol {
            keep[best.1] = true;
            stack.push((s, best.1));
            stack.push((best.1, e));
        }
    }
    pts.iter()
        .zip(keep)
        .filter(|(_, k)| *k)
        .map(|(p, _)| *p)
        .collect()
}
