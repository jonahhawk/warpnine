//! Persistent selection intent. Empty selected sets never mean "everything".
use crate::scene::{InstanceId, ObjectId};
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct GeometryRef {
    pub object: ObjectId,
    #[serde(default)]
    pub instances: Vec<InstanceId>,
    #[serde(default)]
    pub reversed: bool,
    #[serde(default)]
    pub start_vertex: usize,
}
impl GeometryRef {
    pub fn object(id: ObjectId) -> Self {
        Self {
            object: id,
            instances: vec![],
            reversed: false,
            start_vertex: 0,
        }
    }
    pub fn same_object(&self, other: &Self) -> bool {
        self.object == other.object && self.instances == other.instances
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum SelectionMode {
    Selected,
    AllEligible,
    StockBoundary,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct Selection {
    pub mode: SelectionMode,
    pub references: Vec<GeometryRef>,
}
impl Default for Selection {
    fn default() -> Self {
        Self {
            mode: SelectionMode::Selected,
            references: vec![],
        }
    }
}
impl Selection {
    pub fn all() -> Self {
        Self {
            mode: SelectionMode::AllEligible,
            ..Self::default()
        }
    }
    pub fn stock() -> Self {
        Self {
            mode: SelectionMode::StockBoundary,
            ..Self::default()
        }
    }
    pub fn toggle(&mut self, reference: GeometryRef) {
        self.mode = SelectionMode::Selected;
        if let Some(i) = self
            .references
            .iter()
            .position(|r| r.same_object(&reference))
        {
            self.references.remove(i);
        } else {
            self.references.push(reference);
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Default)]
pub enum GeometryRole {
    #[default]
    Regions,
    Containment,
    Avoid,
    Drive,
    PartSurfaces,
    CheckSurfaces,
    Holes,
    Starts,
}
impl GeometryRole {
    pub const ALL: [Self; 8] = [
        Self::Regions,
        Self::Containment,
        Self::Avoid,
        Self::Drive,
        Self::PartSurfaces,
        Self::CheckSurfaces,
        Self::Holes,
        Self::Starts,
    ];
    pub fn label(self) -> &'static str {
        match self {
            Self::Regions => "Machining regions",
            Self::Containment => "Containment",
            Self::Avoid => "Avoid regions",
            Self::Drive => "Drive curves",
            Self::PartSurfaces => "Part surfaces",
            Self::CheckSurfaces => "Check surfaces",
            Self::Holes => "Hole locations",
            Self::Starts => "Start points",
        }
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ControlGeometry {
    pub regions: Selection,
    pub containment: Selection,
    pub avoid: Selection,
    pub drive: Selection,
    pub part_surfaces: Selection,
    pub check_surfaces: Selection,
    pub holes: Selection,
    pub starts: Selection,
    /// Ambiguous legacy intent must be explicitly reviewed, never broadened.
    pub review: Option<String>,
}
impl Default for ControlGeometry {
    fn default() -> Self {
        Self {
            regions: Selection::default(),
            containment: Selection::stock(),
            avoid: Selection::default(),
            drive: Selection::default(),
            part_surfaces: Selection::all(),
            check_surfaces: Selection::default(),
            holes: Selection::default(),
            starts: Selection::default(),
            review: None,
        }
    }
}
impl ControlGeometry {
    pub fn get(&self, role: GeometryRole) -> &Selection {
        match role {
            GeometryRole::Regions => &self.regions,
            GeometryRole::Containment => &self.containment,
            GeometryRole::Avoid => &self.avoid,
            GeometryRole::Drive => &self.drive,
            GeometryRole::PartSurfaces => &self.part_surfaces,
            GeometryRole::CheckSurfaces => &self.check_surfaces,
            GeometryRole::Holes => &self.holes,
            GeometryRole::Starts => &self.starts,
        }
    }
    pub fn get_mut(&mut self, role: GeometryRole) -> &mut Selection {
        match role {
            GeometryRole::Regions => &mut self.regions,
            GeometryRole::Containment => &mut self.containment,
            GeometryRole::Avoid => &mut self.avoid,
            GeometryRole::Drive => &mut self.drive,
            GeometryRole::PartSurfaces => &mut self.part_surfaces,
            GeometryRole::CheckSurfaces => &mut self.check_surfaces,
            GeometryRole::Holes => &mut self.holes,
            GeometryRole::Starts => &mut self.starts,
        }
    }
}
