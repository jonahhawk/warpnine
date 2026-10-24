//! Source geometry and instance-aware identity, independent of CAM array order.
use crate::{selection::GeometryRef, Mesh, Polyline2, Units};
use anyhow::{bail, ensure, Context, Result};
use glam::{DMat4, DVec3};
use serde::{Deserialize, Serialize};

macro_rules! id {
    ($name:ident) => {
        #[derive(Debug, Clone, Default, PartialEq, Eq, Hash, Serialize, Deserialize)]
        #[serde(transparent)]
        pub struct $name(pub String);
    };
}
id!(ObjectId);
id!(LayerId);
id!(DefinitionId);
id!(InstanceId);
id!(OperationId);
id!(SetupId);

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Source {
    pub file: String,
    pub uuid: Option<String>,
    pub units: Units,
    #[serde(default)]
    pub original_units: Option<String>,
    pub tolerance_mm: Option<f64>,
    #[serde(default)]
    pub approximation_tolerance_mm: Option<f64>,
}
/// Retained exact source data; machining uses separately supplied approximation.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum AnalyticCurve {
    Line {
        from: DVec3,
        to: DVec3,
    },
    Arc {
        center: DVec3,
        x_axis: DVec3,
        y_axis: DVec3,
        radius: f64,
        start: f64,
        end: f64,
    },
    Nurbs {
        degree: usize,
        knots: Vec<f64>,
        controls: Vec<DVec3>,
        weights: Vec<f64>,
    },
}
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum Geometry {
    /// Retains identity and an actionable error for geometry we cannot machine directly.
    /// `represented_by` names a qualified machining mesh imported for this object, so
    /// selecting the solid machines that mesh and the file does not count as incomplete.
    Unavailable {
        reason: String,
        #[serde(default, skip_serializing_if = "Option::is_none")]
        represented_by: Option<ObjectId>,
    },
    Curve {
        points: Vec<DVec3>,
        closed: bool,
        analytic: Option<AnalyticCurve>,
    },
    Mesh(Mesh),
    Point(DVec3),
    Instance {
        definition: DefinitionId,
        transform: DMat4,
    },
}
impl Geometry {
    pub fn polyline(c: &Polyline2) -> Self {
        Self::Curve {
            points: c.pts.iter().map(|p| p.extend(0.0)).collect(),
            closed: c.closed,
            analytic: None,
        }
    }
}
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SceneObject {
    pub id: ObjectId,
    pub name: String,
    pub layer: LayerId,
    pub visible: bool,
    pub source: Option<Source>,
    pub geometry: Geometry,
}
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Layer {
    pub id: LayerId,
    pub name: String,
    pub visible: bool,
}
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Definition {
    pub id: DefinitionId,
    pub name: String,
    pub objects: Vec<SceneObject>,
}
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Group {
    pub name: String,
    pub members: Vec<GeometryRef>,
}
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Scene {
    pub objects: Vec<SceneObject>,
    pub layers: Vec<Layer>,
    pub definitions: Vec<Definition>,
    pub groups: Vec<Group>,
    pub next_id: u64,
}
impl Default for Scene {
    fn default() -> Self {
        Self {
            objects: vec![],
            layers: vec![Layer {
                id: LayerId("default".into()),
                name: "Default".into(),
                visible: true,
            }],
            definitions: vec![],
            groups: vec![],
            next_id: 1,
        }
    }
}
#[derive(Debug, Clone)]
pub struct ResolvedObject {
    pub reference: GeometryRef,
    pub name: String,
    pub layer: LayerId,
    pub visible: bool,
    pub geometry: Geometry,
    pub transform: DMat4,
}
impl Scene {
    pub fn add(&mut self, name: String, geometry: Geometry) -> ObjectId {
        let id = loop {
            let id = ObjectId(format!("object/{}", self.next_id));
            self.next_id += 1;
            if !self
                .objects
                .iter()
                .chain(self.definitions.iter().flat_map(|d| &d.objects))
                .any(|o| o.id == id)
            {
                break id;
            }
        };
        self.objects.push(SceneObject {
            id: id.clone(),
            name,
            layer: LayerId("default".into()),
            visible: true,
            source: None,
            geometry,
        });
        id
    }
    pub fn validate(&self) -> Result<()> {
        let mut ids = std::collections::HashSet::new();
        for o in self
            .objects
            .iter()
            .chain(self.definitions.iter().flat_map(|d| &d.objects))
        {
            ensure!(
                !o.id.0.is_empty() && ids.insert(&o.id),
                "duplicate or empty object ID: {}",
                o.id.0
            );
            ensure!(
                self.layers.iter().any(|l| l.id == o.layer),
                "missing layer: {}",
                o.layer.0
            );
            match &o.geometry {
                Geometry::Unavailable { .. } => {}
                Geometry::Curve { points, .. } => ensure!(
                    points.len() >= 2 && points.iter().all(|p| p.is_finite()),
                    "invalid curve: {}",
                    o.name
                ),
                Geometry::Mesh(m) => ensure!(
                    m.tris.iter().flatten().all(|p| p.is_finite()),
                    "invalid mesh: {}",
                    o.name
                ),
                Geometry::Point(p) => ensure!(p.is_finite(), "invalid point"),
                Geometry::Instance { transform, .. } => ensure!(
                    transform.is_finite()
                        && transform.determinant().abs() > 1e-12
                        && transform.row(3).abs_diff_eq(glam::DVec4::W, 1e-9),
                    "invalid instance transform"
                ),
            }
        }
        let mut definitions = std::collections::HashSet::new();
        for d in &self.definitions {
            ensure!(definitions.insert(&d.id), "duplicate definition ID");
        }
        let mut layers = std::collections::HashSet::new();
        for l in &self.layers {
            ensure!(layers.insert(&l.id), "duplicate layer ID");
        }
        self.resolved()?;
        // Unreferenced definitions can also contain a cycle.
        for d in &self.definitions {
            self.walk(
                &d.objects,
                DMat4::IDENTITY,
                true,
                &mut vec![],
                &mut vec![d.id.clone()],
                &mut vec![],
            )?;
        }
        Ok(())
    }
    pub fn resolved(&self) -> Result<Vec<ResolvedObject>> {
        let mut out = vec![];
        self.walk(
            &self.objects,
            DMat4::IDENTITY,
            true,
            &mut vec![],
            &mut vec![],
            &mut out,
        )?;
        Ok(out)
    }
    fn walk(
        &self,
        objects: &[SceneObject],
        transform: DMat4,
        visible: bool,
        path: &mut Vec<InstanceId>,
        stack: &mut Vec<DefinitionId>,
        out: &mut Vec<ResolvedObject>,
    ) -> Result<()> {
        ensure!(
            stack.len() < 64 && out.len() < 1_000_000,
            "instance expansion limit exceeded"
        );
        for o in objects {
            let shown = visible
                && o.visible
                && self
                    .layers
                    .iter()
                    .find(|l| l.id == o.layer)
                    .is_some_and(|l| l.visible);
            if let Geometry::Instance {
                definition,
                transform: local,
            } = &o.geometry
            {
                if stack.contains(definition) {
                    bail!("cyclic instance definition: {}", definition.0);
                }
                let d = self
                    .definitions
                    .iter()
                    .find(|d| &d.id == definition)
                    .context("missing instance definition")?;
                stack.push(definition.clone());
                path.push(InstanceId(o.id.0.clone()));
                self.walk(&d.objects, transform * *local, shown, path, stack, out)?;
                stack.pop();
                path.pop();
            } else {
                out.push(ResolvedObject {
                    reference: GeometryRef {
                        object: o.id.clone(),
                        instances: path.clone(),
                        reversed: false,
                        start_vertex: 0,
                    },
                    name: o.name.clone(),
                    layer: o.layer.clone(),
                    visible: shown,
                    geometry: o.geometry.clone(),
                    transform,
                });
            }
        }
        Ok(())
    }
}
