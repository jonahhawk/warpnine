//! Versioned document, legacy migration, and a compatibility adapter for generators.
use crate::{OpKind, Operation, Project, Stock};
use anyhow::{bail, ensure, Context, Result};
use glam::{DMat4, DQuat, DVec3, EulerRot};
use serde::{Deserialize, Serialize};
use warp_core::{scene::*, selection::*, Mesh, Polyline2};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Setup {
    pub id: SetupId,
    pub name: String,
    pub origin: DVec3,
    pub rotation_deg: DVec3,
    pub wcs: u8,
    pub stock: Stock,
    pub fixtures: Selection,
}
impl Setup {
    pub fn transform(&self) -> DMat4 {
        let a = self.rotation_deg * (std::f64::consts::PI / 180.0);
        DMat4::from_quat(DQuat::from_euler(EulerRot::XYZ, a.x, a.y, a.z).conjugate())
            * DMat4::from_translation(-self.origin)
    }
}
pub fn primary_role(kind: &OpKind) -> GeometryRole {
    match kind {
        OpKind::ProjectCurves { .. } | OpKind::Engrave | OpKind::Slot => GeometryRole::Drive,
        OpKind::Drill { .. }
        | OpKind::HelicalBore { .. }
        | OpKind::ThreadMill { .. }
        | OpKind::HolePocket { .. }
        | OpKind::HoleProfile { .. } => GeometryRole::Holes,
        k if k.is_3d() => GeometryRole::Containment,
        _ => GeometryRole::Regions,
    }
}
impl Project {
    pub fn from_json(text: &str) -> Result<Self> {
        let value: serde_json::Value = serde_json::from_str(text)?;
        let mut p: Project = if let Some(version) = value.get("schema_version") {
            ensure!(
                version.as_u64() == Some(1),
                "unsupported project schema_version: {version}"
            );
            let document = value
                .get("document")
                .context("versioned project is missing document")?;
            ensure!(
                document.get("scene").is_some_and(|s| s.is_object())
                    && document
                        .get("document_id")
                        .and_then(|v| v.as_str())
                        .is_some_and(|s| !s.is_empty()),
                "invalid versioned project identity/scene"
            );
            let p: Project = serde_json::from_value(document.clone())?;
            ensure!(!p.setups.is_empty(), "versioned project has no setups");
            p
        } else {
            for key in [
                "name",
                "display_units",
                "stock",
                "curves",
                "mesh",
                "operations",
                "post",
            ] {
                ensure!(value.get(key).is_some(), "legacy project missing {key}");
            }
            let mut p: Project = serde_json::from_value(value)?;
            ensure!(p.scene.is_none(), "versioned scene requires schema_version");
            p.ensure_document()?;
            p
        };
        p.validate_document()?;
        p.stock = p.current_setup()?.stock;
        p.refresh_view()?;
        Ok(p)
    }
    pub fn save_versioned(&self, path: &std::path::Path) -> Result<()> {
        use std::io::Write;
        let mut p = self.clone();
        p.ensure_document()?;
        p.commit_stock();
        p.validate_document()?;
        let text = p.document_json()?;
        Self::from_json(&text)?;
        // Do not overwrite an unsupported/corrupt file. Backup before replacement.
        if path.exists() {
            Self::load(path).context("refusing to overwrite an unreadable project")?;
        }
        let temp = path.with_extension(format!("warp.{}.tmp", std::process::id()));
        let mut file = std::fs::OpenOptions::new()
            .write(true)
            .create_new(true)
            .open(&temp)?;
        let result = (|| -> Result<()> {
            file.write_all(text.as_bytes())?;
            file.sync_all()?;
            drop(file);
            if path.exists() {
                let backup = path.with_extension("warp.bak");
                std::fs::copy(path, &backup).context("cannot create project backup")?;
                #[cfg(windows)]
                {
                    use std::os::windows::ffi::OsStrExt;
                    #[link(name = "kernel32")]
                    unsafe extern "system" {
                        fn MoveFileExW(from: *const u16, to: *const u16, flags: u32) -> i32;
                    }
                    let from: Vec<u16> = temp.as_os_str().encode_wide().chain(Some(0)).collect();
                    let to: Vec<u16> = path.as_os_str().encode_wide().chain(Some(0)).collect();
                    if unsafe { MoveFileExW(from.as_ptr(), to.as_ptr(), 1 | 8) } == 0 {
                        return Err(std::io::Error::last_os_error().into());
                    }
                }
                #[cfg(not(windows))]
                std::fs::rename(&temp, path)?;
            } else {
                std::fs::rename(&temp, path)?;
            }
            Ok(())
        })();
        if result.is_err() {
            let _ = std::fs::remove_file(temp);
        }
        result
    }
    pub fn document_json(&self) -> Result<String> {
        let mut value = serde_json::to_value(self)?;
        for key in ["curves", "mesh", "stock"] {
            value.as_object_mut().unwrap().remove(key);
        }
        Ok(serde_json::to_string(
            &serde_json::json!({"schema_version":1,"document":value}),
        )?)
    }
    pub fn current_setup(&self) -> Result<&Setup> {
        self.setups
            .iter()
            .find(|s| s.id == self.active_setup)
            .context("active setup is missing")
    }
    pub fn commit_stock(&mut self) {
        if let Some(s) = self.setups.iter_mut().find(|s| s.id == self.active_setup) {
            s.stock = self.stock;
        }
    }
    pub fn switch_setup(&mut self, id: SetupId) -> Result<()> {
        self.commit_stock();
        ensure!(self.setups.iter().any(|s| s.id == id), "missing setup");
        self.active_setup = id;
        self.stock = self.current_setup()?.stock;
        self.refresh_view()
    }
    pub fn fresh_id(&mut self, kind: &str) -> String {
        loop {
            self.next_id += 1;
            let id = format!("{kind}/{}", self.next_id);
            if !self.operations.iter().any(|o| o.common.id.0 == id)
                && !self.setups.iter().any(|s| s.id.0 == id)
            {
                return id;
            }
        }
    }
    pub fn ensure_document(&mut self) -> Result<()> {
        if self.scene.is_none() {
            // Deterministic identity for repeated migration of the same legacy
            // document. This is a namespace marker, not a cryptographic digest.
            let legacy_bytes = serde_json::to_vec(&(
                self.name.as_str(),
                self.display_units,
                self.stock,
                &self.curves,
                &self.mesh,
                &self.operations,
                &self.post,
            ))?;
            let hash = legacy_bytes.iter().fold(0xcbf29ce484222325u64, |h, b| {
                (h ^ (*b as u64)).wrapping_mul(0x100000001b3)
            });
            let mut scene = Scene::default();
            for (i, c) in self.curves.iter().enumerate() {
                scene.objects.push(SceneObject {
                    id: ObjectId(format!("legacy/curve/{i}")),
                    name: format!("Curve {i}"),
                    layer: LayerId("default".into()),
                    visible: true,
                    source: None,
                    geometry: Geometry::polyline(c),
                });
            }
            if let Some(m) = &self.mesh {
                scene.objects.push(SceneObject {
                    id: ObjectId("legacy/mesh/0".into()),
                    name: "Model".into(),
                    layer: LayerId("default".into()),
                    visible: true,
                    source: None,
                    geometry: Geometry::Mesh(m.clone()),
                });
            }
            self.scene = Some(scene);
            self.document_id = format!("legacy-{hash:016x}");
            self.active_setup = SetupId("legacy/setup/0".into());
            self.setups = vec![Setup {
                id: self.active_setup.clone(),
                name: "Setup 1".into(),
                origin: DVec3::ZERO,
                rotation_deg: DVec3::ZERO,
                wcs: 54,
                stock: self.stock,
                fixtures: Selection::default(),
            }];
            for (i, op) in self.operations.iter_mut().enumerate() {
                op.common.id = OperationId(format!("legacy/operation/{i}"));
                op.common.setup = self.active_setup.clone();
                let mut control = ControlGeometry::default();
                let role = primary_role(&op.kind);
                let selection = if op.common.curves.is_empty() {
                    if op.kind.is_3d() {
                        Selection::stock()
                    } else {
                        Selection::all()
                    }
                } else {
                    Selection {
                        mode: SelectionMode::Selected,
                        references: op
                            .common
                            .curves
                            .iter()
                            .map(|i| GeometryRef::object(ObjectId(format!("legacy/curve/{i}"))))
                            .collect(),
                    }
                };
                if matches!(op.kind, OpKind::ProjectCurves { .. }) {
                    control.drive = selection.clone();
                    control.containment = selection;
                    if op.common.curves.is_empty()
                        || op
                            .common
                            .curves
                            .iter()
                            .any(|i| self.curves.get(*i).is_none_or(|c| !c.closed))
                    {
                        control.drive.mode = SelectionMode::Selected;
                        control.containment = Selection::stock();
                        control.review = Some("Legacy projection: boundary curves must be closed in the old workflow. Review separate drive and containment roles, then acknowledge.".into());
                    }
                } else {
                    *control.get_mut(role) = selection;
                }
                op.common.geometry = Some(control);
            }
        }
        for i in 0..self.operations.len() {
            if self.operations[i].common.id.0.is_empty() {
                self.operations[i].common.id = OperationId(self.fresh_id("operation"));
            }
            if self.operations[i].common.setup.0.is_empty() {
                self.operations[i].common.setup = self.active_setup.clone();
            }
            if self.operations[i].common.geometry.is_none() {
                self.operations[i].common.geometry = Some(ControlGeometry::default());
            }
        }
        Ok(())
    }
    pub fn validate_document(&self) -> Result<()> {
        self.scene.as_ref().context("missing scene")?.validate()?;
        self.current_setup()?;
        let mut ids = std::collections::HashSet::new();
        for s in &self.setups {
            ensure!(
                !s.id.0.is_empty() && ids.insert(&s.id),
                "duplicate/empty setup ID"
            );
            ensure!(
                s.origin.is_finite() && s.rotation_deg.is_finite() && (54..=59).contains(&s.wcs),
                "invalid setup transform/WCS"
            );
            ensure!(
                s.stock.min.is_finite()
                    && s.stock.max.is_finite()
                    && s.stock.min.cmple(s.stock.max).all(),
                "invalid stock"
            );
        }
        let mut ids = std::collections::HashSet::new();
        for op in &self.operations {
            ensure!(
                !op.common.id.0.is_empty() && ids.insert(&op.common.id),
                "duplicate/empty operation ID"
            );
            ensure!(
                self.setups.iter().any(|s| s.id == op.common.setup),
                "operation has missing setup"
            );
            ensure!(
                op.common.geometry.is_some(),
                "operation is missing control geometry"
            );
        }
        if let Some(nesting) = &self.nesting {
            nesting
                .validate_against(self.scene.as_ref().context("missing scene")?)
                .context("nesting job")?;
        }
        Ok(())
    }
    pub fn refresh_view(&mut self) -> Result<()> {
        let Some(scene) = &self.scene else {
            return Ok(());
        };
        let transform = self.current_setup()?.transform();
        self.curves.clear();
        self.curve_refs.clear();
        self.view_points.clear();
        self.view_visible.clear();
        let mut mesh = Mesh::default();
        for obj in scene.resolved()? {
            let m = transform * obj.transform;
            match obj.geometry {
                Geometry::Curve { points, closed, .. } => {
                    let points: Vec<_> = points.iter().map(|p| m.transform_point3(*p)).collect();
                    self.curves.push(Polyline2::new(
                        points.iter().map(|p| p.truncate()).collect(),
                        closed,
                    ));
                    self.view_points.push(points);
                    self.curve_refs.push(obj.reference);
                    self.view_visible.push(obj.visible);
                }
                Geometry::Mesh(model) if obj.visible => {
                    mesh.tris.extend(model.transformed(&m).tris)
                }
                _ => {}
            }
        }
        self.mesh = if mesh.tris.is_empty() {
            None
        } else {
            Some(mesh)
        };
        Ok(())
    }
    /// Resolve stable intent into a temporary, setup-local legacy generator input.
    pub fn resolve_operation(&self, op: &Operation) -> Result<(Project, Operation)> {
        let scene = self.scene.as_ref().context("missing scene")?;
        let objects = scene.resolved()?;
        let setup = self
            .setups
            .iter()
            .find(|s| s.id == op.common.setup)
            .context("missing operation setup")?;
        let control = op
            .common
            .geometry
            .as_ref()
            .context("missing geometry roles")?;
        if let Some(review) = &control.review {
            bail!("{review}");
        }
        let mut p = Project {
            stock: if setup.id == self.active_setup {
                self.stock
            } else {
                setup.stock
            },
            execution_resolved: true,
            checks: self.checks,
            ..Default::default()
        };
        ensure!(
            setup.fixtures.mode == SelectionMode::Selected,
            "Fixtures require an explicit selection"
        );
        for reference in &setup.fixtures.references {
            let obj = objects
                .iter()
                .find(|o| o.reference.same_object(reference))
                .context("Unresolved fixture reference; repair the setup")?;
            let Geometry::Mesh(mesh) = &obj.geometry else {
                bail!(
                    "Fixtures must be mesh objects; their conservative bounding boxes are checked"
                );
            };
            let matrix = setup.transform() * obj.transform;
            let mut bounds = warp_core::Aabb3::EMPTY;
            for point in mesh.tris.iter().flatten() {
                bounds.add(matrix.transform_point3(*point));
            }
            ensure!(!bounds.is_empty(), "Empty fixture mesh");
            p.execution_fixtures.push(Stock {
                min: bounds.min,
                max: bounds.max,
            });
        }
        let mut adapted = op.clone();
        adapted.common.geometry = None;
        let select = |selection: &Selection,
                      mesh: bool|
         -> Result<Vec<(ResolvedObject, GeometryRef)>> {
            if selection.mode == SelectionMode::AllEligible {
                // A solid represented by a qualified machining mesh does not make the
                // file incomplete; its mesh is eligible in its place.
                if let Some(obj) = objects.iter().find(|o| {
                    matches!(
                        o.geometry,
                        Geometry::Unavailable {
                            represented_by: None,
                            ..
                        }
                    )
                }) {
                    bail!("{} is unavailable. Repair the import or explicitly select the intended supported geometry before generating.", obj.name);
                }
                return Ok(objects
                    .iter()
                    .filter(|o| {
                        if mesh {
                            matches!(o.geometry, Geometry::Mesh(_))
                        } else {
                            matches!(o.geometry, Geometry::Curve { .. } | Geometry::Point(_))
                        }
                    })
                    .map(|o| (o.clone(), o.reference.clone()))
                    .collect());
            }
            ensure!(
                selection.mode == SelectionMode::Selected,
                "Stock boundary is not valid for this geometry role"
            );
            selection
                .references
                .iter()
                .map(|r| {
                    Ok((
                        objects
                            .iter()
                            .find(|o| o.reference.same_object(r))
                            .with_context(|| {
                                format!(
                                    "Unresolved geometry reference {}. Repair the selection.",
                                    r.object.0
                                )
                            })?
                            .clone(),
                        r.clone(),
                    ))
                })
                .collect()
        };
        let curves = |s: &Selection, closed_only: bool, project: bool| -> Result<Vec<Polyline2>> {
            let mut out = vec![];
            for (obj, r) in select(s, false)? {
                let m = setup.transform() * obj.transform;
                let (mut points, closed) = match obj.geometry {
                    Geometry::Curve { points, closed, .. } => (
                        points
                            .into_iter()
                            .map(|p| m.transform_point3(p))
                            .collect::<Vec<_>>(),
                        closed,
                    ),
                    Geometry::Point(p)
                        if matches!(
                            op.kind,
                            OpKind::Drill {
                                source: crate::ops::DrillSource::CurveStarts,
                                ..
                            }
                        ) && !closed_only =>
                    {
                        (vec![m.transform_point3(p); 2], false)
                    }
                    _ => bail!("{} requires curves", obj.name),
                };
                ensure!(
                    !closed_only || closed,
                    "boundary curves must be closed: {}",
                    obj.name
                );
                ensure!(
                    project
                        || points
                            .iter()
                            .all(|p| (p.z - points[0].z).abs() <= op.common.tolerance),
                    "{} is not planar in this setup; explicit projection is required",
                    obj.name
                );
                ensure!(
                    r.start_vertex < points.len(),
                    "start vertex no longer exists: {}",
                    obj.name
                );
                if closed {
                    points.rotate_left(r.start_vertex);
                }
                if r.reversed {
                    if closed && points.len() > 1 {
                        points[1..].reverse()
                    } else {
                        points.reverse()
                    }
                }
                out.push(Polyline2 {
                    pts: points.into_iter().map(|p| p.truncate()).collect(),
                    closed,
                });
            }
            Ok(out)
        };
        // Resolve *every* explicit reference, even optional roles, before generation.
        for role in GeometryRole::ALL {
            for r in &control.get(role).references {
                ensure!(
                    objects.iter().any(|o| o.reference.same_object(r)),
                    "Unresolved {} reference {}",
                    role.label(),
                    r.object.0
                );
            }
        }
        ensure!(
            control.check_surfaces.references.is_empty()
                && control.check_surfaces.mode == SelectionMode::Selected,
            "check-surface collision checking is not yet supported; generation blocked"
        );
        ensure!(
            control.starts.references.is_empty() && control.starts.mode == SelectionMode::Selected,
            "point-controlled entry is not yet supported; use curve start vertices"
        );
        let role = primary_role(&op.kind);
        if !op.kind.is_3d() || matches!(op.kind, OpKind::ProjectCurves { .. }) {
            let s = control.get(role);
            if s.mode == SelectionMode::StockBoundary {
                ensure!(
                    matches!(op.kind, OpKind::Facing { .. }),
                    "Stock boundary is only valid for facing or 3D containment"
                );
                p.curves.push(Polyline2::rect(
                    p.stock.min.truncate(),
                    p.stock.max.truncate(),
                ));
            } else {
                p.curves = curves(
                    s,
                    matches!(
                        op.kind,
                        OpKind::Pocket { .. }
                            | OpKind::Facing { .. }
                            | OpKind::FaceTop { .. }
                            | OpKind::Roughing2D { .. }
                            | OpKind::VCarve { .. }
                    ),
                    matches!(op.kind, OpKind::ProjectCurves { .. }),
                )?;
            }
            ensure!(!p.curves.is_empty(), "No {} selected", role.label());
        }
        adapted.common.curves = (0..p.curves.len()).collect();
        if op.kind.is_3d() {
            let mut mesh = Mesh::default();
            for (mut obj, _) in select(&control.part_surfaces, true)? {
                // Selecting a solid machines the qualified mesh that represents it.
                if let Geometry::Unavailable {
                    represented_by: Some(id),
                    ..
                } = &obj.geometry
                {
                    obj = objects
                        .iter()
                        .find(|o| {
                            o.reference.object == *id
                                && o.reference.instances == obj.reference.instances
                        })
                        .with_context(|| {
                            format!("{} is represented by a mesh that is missing", obj.name)
                        })?
                        .clone();
                }
                let Geometry::Mesh(model) = &obj.geometry else {
                    bail!("{} is not a mesh; part surfaces must be meshes or solids represented by qualified meshes", obj.name)
                };
                if let Some(deviation) = model.provenance.as_ref().and_then(|p| p.deviation_mm()) {
                    ensure!(
                        deviation <= op.common.tolerance,
                        "{}: measured mesh deviation {:.4} mm exceeds the operation tolerance {:.4} mm; re-import or re-mesh the source at a tighter tolerance, or relax the operation tolerance",
                        obj.name,
                        deviation,
                        op.common.tolerance
                    );
                }
                let m = setup.transform() * obj.transform;
                mesh.tris.extend(model.transformed(&m).tris);
            }
            ensure!(!mesh.tris.is_empty(), "No part surfaces selected");
            p.mesh = Some(mesh);
            p.execution_boundary = if control.containment.mode == SelectionMode::StockBoundary {
                None
            } else {
                let c = curves(&control.containment, true, false)?;
                ensure!(
                    !c.is_empty(),
                    "No containment selected; choose Stock boundary explicitly"
                );
                Some(c)
            };
            p.execution_avoid = curves(&control.avoid, true, false)?;
        } else {
            ensure!(control.avoid.references.is_empty() && control.containment.mode == SelectionMode::StockBoundary, "separate containment/avoid roles are currently supported for 3D operations; use region islands for 2D");
        }
        Ok((p, adapted))
    }
}
