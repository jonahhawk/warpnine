//! Shared transition policy. A short distance is never proof of a safe low link.
use crate::region::{self, Region};
use glam::DVec3;
#[derive(Default)]
pub struct LinkPolicy {
    pub centre_region: Option<Region>,
}
impl LinkPolicy {
    pub fn permits(&self, from: DVec3, to: DVec3) -> bool {
        from.distance(to) < 1e-8
            || self.centre_region.as_ref().is_some_and(|r| {
                (from.z - to.z).abs() < 1e-8
                    && region::covers_segment(r, from.truncate(), to.truncate())
            })
    }
}
