//! Common CAD import contract. Importing is separate from document mutation.
//!
//! Native readers (openNURBS, Open CASCADE) return a [`NativeScene`] whose object IDs are
//! the source file's own identifiers; [`finish_native_import`] namespaces them, attaches
//! source provenance and applies the one machining-mesh qualification policy shared by
//! every format.
use crate::scene::*;
use anyhow::{ensure, Context, Result};
use serde::{Deserialize, Serialize};
use std::{collections::HashMap, path::Path};
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ImportDiagnostic {
    pub object: String,
    pub code: String,
    pub message: String,
}
#[derive(Debug, Clone)]
pub struct ImportOptions {
    /// Maximum curve approximation error in millimetres, including block scale. For
    /// B-rep formats that WarpNine meshes itself this is also the requested mesh deviation.
    pub tolerance_mm: f64,
    /// Memory bound: the import fails before exceeding this many mesh triangles in total.
    pub max_triangles: usize,
    /// Cooperative cancellation; the reader polls it between objects and inside long loops.
    pub cancel: Option<std::sync::Arc<std::sync::atomic::AtomicBool>>,
}
impl Default for ImportOptions {
    fn default() -> Self {
        Self {
            tolerance_mm: 0.01,
            max_triangles: 2_000_000,
            cancel: None,
        }
    }
}
impl ImportOptions {
    pub fn validate(&self) -> Result<()> {
        ensure!(
            self.tolerance_mm.is_finite() && self.tolerance_mm >= 1e-6 && self.tolerance_mm <= 10.0,
            "Import tolerance must be between 0.000001 and 10 mm"
        );
        ensure!(self.max_triangles >= 1, "Mesh limit must be at least one triangle");
        Ok(())
    }
    /// Raw pointer for a native reader to poll; null when cancellation is not requested.
    pub fn cancel_flag(&self) -> *const u8 {
        self.cancel
            .as_ref()
            .map(|c| c.as_ptr() as *const u8)
            .unwrap_or(std::ptr::null())
    }
}
#[derive(Debug, Clone)]
pub struct ImportResult {
    pub scene: Scene,
    pub diagnostics: Vec<ImportDiagnostic>,
    pub source_file: String,
    pub namespace: String,
    /// Human-readable file format, e.g. "Rhino 8" or "STEP AP214".
    pub format: String,
    pub file_version: u32,
    pub unit_scale_mm: f64,
    pub source_units: String,
    pub source_tolerance_mm: f64,
    pub approximation_tolerance_mm: f64,
}
pub trait GeometryImporter {
    fn read(&self, path: &Path, options: &ImportOptions) -> Result<ImportResult>;
}
impl ImportResult {
    /// A source scope is replaced transactionally. Stable source/type IDs survive;
    /// removed or structurally changed objects leave CAM references unresolved.
    pub fn merge_into(&self, scene: &mut Scene) -> Result<()> {
        let mut merged = scene.clone();
        merged
            .objects
            .retain(|o| !o.id.0.starts_with(&self.namespace));
        merged
            .definitions
            .retain(|d| !d.id.0.starts_with(&self.namespace));
        merged
            .layers
            .retain(|l| !l.id.0.starts_with(&self.namespace));
        merged.objects.extend(self.scene.objects.clone());
        merged.definitions.extend(self.scene.definitions.clone());
        merged.layers.extend(self.scene.layers.clone());
        merged.validate()?;
        *scene = merged;
        Ok(())
    }
    pub fn summary(&self) -> String {
        format!(
            "{}: {} objects, {} block definitions, {} layers; {} to mm ×{}; {} diagnostics",
            self.format,
            self.scene.objects.len(),
            self.scene.definitions.len(),
            self.scene.layers.len(),
            self.source_units,
            self.unit_scale_mm,
            self.diagnostics.len()
        )
    }
}

/// What a native reader returns before namespacing and qualification. Object, layer and
/// definition IDs are the file's own identifiers; mesh objects that represent a solid
/// carry `provenance`, and the solid is `Unavailable { represented_by: Some(mesh id) }`.
#[derive(Debug, Clone, Deserialize)]
pub struct NativeScene {
    pub scene: Scene,
    pub diagnostics: Vec<ImportDiagnostic>,
    pub file_version: u32,
    pub unit_scale_mm: f64,
    pub source_units: String,
    pub source_tolerance_mm: f64,
}

/// Format-specific wording for the shared policy messages.
#[derive(Debug, Clone)]
pub struct ImportFormat {
    /// Namespace prefix for object IDs, e.g. "3dm" or "step".
    pub key: &'static str,
    /// Display name, e.g. "Rhino 8".
    pub name: String,
    /// What the user can do about a rejected machining mesh, e.g. "re-mesh the source in
    /// Rhino with a tighter tolerance".
    pub remesh_hint: String,
}

pub fn fnv_hash(bytes: &[u8]) -> u64 {
    bytes.iter().fold(0xcbf29ce484222325, |h, b| {
        (h ^ u64::from(*b)).wrapping_mul(0x100000001b3)
    })
}
fn topology(g: &Geometry) -> String {
    match g {
        Geometry::Curve {
            points,
            closed,
            analytic,
        } => match analytic {
            Some(AnalyticCurve::Nurbs {
                degree,
                knots,
                controls,
                ..
            }) => format!(
                "curve/{closed}/{degree}/{}/{}/{}",
                knots.len(),
                controls.len(),
                points.len()
            ),
            _ => format!("curve/{closed}/{}", points.len()),
        },
        Geometry::Mesh(_) => "mesh".into(),
        Geometry::Point(_) => "point".into(),
        Geometry::Instance { definition, .. } => format!("instance/{}", definition.0),
        Geometry::Unavailable { .. } => "unavailable".into(),
    }
}
/// Verdict on a machining mesh that claims to represent a solid.
enum Verdict {
    Qualified(f64),
    Claimed,
    Rejected(String),
}

/// Namespaces a native scene, records provenance and applies the machining-mesh policy:
/// a claim is honoured only when the measured deviation meets it within the source model
/// tolerance (`solid_represented`); otherwise the solid stays unavailable with the measured
/// reason (`mesh_unqualified`). Unmeasurable sources stay claimed-only (`mesh_unmeasured`).
/// Topology problems are reported for every mesh (`mesh_topology`).
pub fn finish_native_import(
    mut raw: NativeScene,
    options: &ImportOptions,
    source_file: String,
    format: ImportFormat,
) -> Result<ImportResult> {
    let key = if cfg!(windows) {
        source_file.to_lowercase()
    } else {
        source_file.clone()
    };
    let namespace = format!("{}/{:016x}/", format.key, fnv_hash(key.as_bytes()));
    for l in &mut raw.scene.layers {
        l.id.0 = format!("{namespace}layer/{}", l.id.0);
    }
    let mut by_uuid: HashMap<String, ObjectId> = HashMap::new();
    for o in raw.scene.objects.iter_mut().chain(
        raw.scene
            .definitions
            .iter_mut()
            .flat_map(|d| &mut d.objects),
    ) {
        let uuid = o.id.0.clone();
        o.id.0 = format!(
            "{namespace}object/{uuid}/{:016x}",
            fnv_hash(topology(&o.geometry).as_bytes())
        );
        by_uuid.insert(uuid.clone(), o.id.clone());
        o.layer.0 = format!("{namespace}layer/{}", o.layer.0);
        if o.name.is_empty() {
            o.name = uuid.clone();
        }
        o.source = Some(Source {
            file: source_file.clone(),
            uuid: Some(uuid),
            units: crate::Units::Millimeters,
            original_units: Some(raw.source_units.clone()),
            tolerance_mm: Some(raw.source_tolerance_mm),
            approximation_tolerance_mm: Some(options.tolerance_mm),
        });
        if let Geometry::Instance { definition, .. } = &mut o.geometry {
            definition.0 = format!("{namespace}definition/{}", definition.0);
        }
    }
    for d in &mut raw.scene.definitions {
        d.id.0 = format!("{namespace}definition/{}", d.id.0);
    }
    let slack = raw.source_tolerance_mm.max(0.0);
    let mut verdicts: HashMap<String, (String, Verdict)> = HashMap::new();
    let mut rejected_sources: std::collections::HashSet<String> = Default::default();
    let weld = (raw.source_tolerance_mm * 0.1).clamp(1e-6, 1e-3);
    for o in raw.scene.objects.iter().chain(
        raw.scene
            .definitions
            .iter()
            .flat_map(|d| &d.objects),
    ) {
        let Geometry::Mesh(mesh) = &o.geometry else {
            continue;
        };
        let uuid = o.source.as_ref().and_then(|s| s.uuid.clone()).unwrap_or_default();
        let quality = mesh.analyze(weld);
        if quality.nonmanifold_edges > 0 || quality.inconsistent_edges > 0 {
            raw.diagnostics.push(ImportDiagnostic {
                object: uuid.clone(),
                code: "mesh_topology".into(),
                message: format!(
                    "Mesh '{}' is not an oriented manifold: {}",
                    o.name,
                    quality.describe()
                ),
            });
        }
        let Some(p) = &mesh.provenance else {
            continue;
        };
        let verdict = match p.meets_claim(slack) {
            Some(true) => Verdict::Qualified(p.deviation_mm().unwrap_or(0.0)),
            Some(false) => {
                let m = p.measured.as_ref().unwrap();
                let mut why = format!(
                    "deviates {:.4} mm from its source (requested {:.4} mm)",
                    p.deviation_mm().unwrap_or(0.0),
                    p.requested_tolerance_mm.unwrap_or(0.0)
                );
                if m.outside_trim_samples > 0 {
                    why += &format!(
                        "; {} of {} mesh samples lie outside the source trims",
                        m.outside_trim_samples, m.mesh_samples
                    );
                }
                Verdict::Rejected(why)
            }
            None if p.measured.is_some() => Verdict::Rejected(
                "has a measured deviation but no requested tolerance".into(),
            ),
            None => Verdict::Claimed,
        };
        match &verdict {
            Verdict::Rejected(why) => {
                rejected_sources.insert(p.source.clone());
                raw.diagnostics.push(ImportDiagnostic {
                    object: uuid.clone(),
                    code: "mesh_unqualified".into(),
                    message: format!("Machining mesh '{}' {why}; {}", o.name, format.remesh_hint),
                })
            }
            Verdict::Claimed => raw.diagnostics.push(ImportDiagnostic {
                object: uuid.clone(),
                code: "mesh_unmeasured".into(),
                message: format!(
                    "Machining mesh '{}' accuracy is claimed by the exporter, not measured: {}",
                    o.name,
                    p.note.clone().unwrap_or_default()
                ),
            }),
            Verdict::Qualified(_) => {}
        }
        verdicts.entry(uuid.clone()).or_insert((o.name.clone(), verdict));
    }
    raw.diagnostics
        .retain(|d| !(d.code == "solid_represented" && rejected_sources.contains(&d.object)));
    for o in raw.scene.objects.iter_mut().chain(
        raw.scene
            .definitions
            .iter_mut()
            .flat_map(|d| &mut d.objects),
    ) {
        let (reason, mesh_uuid) = match &o.geometry {
            Geometry::Unavailable {
                reason,
                represented_by: Some(m),
            } => (reason.clone(), m.0.clone()),
            _ => continue,
        };
        let unavailable = |reason: String| Geometry::Unavailable {
            reason,
            represented_by: None,
        };
        o.geometry = match verdicts.get(&mesh_uuid) {
            None => unavailable(reason),
            Some((claim_name, Verdict::Qualified(dev))) => Geometry::Unavailable {
                reason: format!(
                    "Represented by qualified machining mesh '{claim_name}' (measured deviation {dev:.4} mm)"
                ),
                represented_by: Some(by_uuid[&mesh_uuid].clone()),
            },
            Some((claim_name, Verdict::Claimed)) => Geometry::Unavailable {
                reason: format!(
                    "Represented by machining mesh '{claim_name}' (accuracy claimed by the exporter, not measured)"
                ),
                represented_by: Some(by_uuid[&mesh_uuid].clone()),
            },
            Some((claim_name, Verdict::Rejected(why))) => unavailable(format!(
                "Machining mesh '{claim_name}' {why}. {}.",
                capitalize(&format.remesh_hint)
            )),
        };
    }
    raw.scene
        .validate()
        .context("Imported scene failed validation")?;
    Ok(ImportResult {
        scene: raw.scene,
        diagnostics: raw.diagnostics,
        source_file,
        namespace,
        format: format.name,
        file_version: raw.file_version,
        unit_scale_mm: raw.unit_scale_mm,
        source_units: raw.source_units,
        source_tolerance_mm: raw.source_tolerance_mm,
        approximation_tolerance_mm: options.tolerance_mm,
    })
}
fn capitalize(s: &str) -> String {
    let mut c = s.chars();
    match c.next() {
        Some(f) => f.to_uppercase().collect::<String>() + c.as_str(),
        None => String::new(),
    }
}
