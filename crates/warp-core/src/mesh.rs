use crate::geom::Aabb3;
use glam::{DMat4, DVec3};
use serde::{Deserialize, Serialize};
use std::collections::HashMap;

/// Triangle soup used for 3D machining.
///
/// Winding is counter-clockwise seen from outside (right-hand normal points out of the
/// solid). [`Mesh::transformed`] preserves that convention under mirroring transforms.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct Mesh {
    pub tris: Vec<[DVec3; 3]>,
    /// How this mesh relates to the solid it approximates, when an exporter recorded it.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub provenance: Option<MeshProvenance>,
}

/// Exporter claims and importer measurements for a machining mesh.
///
/// Claims come from the host exporter (Rhino) as object user strings; measurements are
/// made by the importer against the source geometry when it can evaluate that geometry.
/// A requested tolerance is never treated as a measured accuracy.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct MeshProvenance {
    /// Identity of the source solid in the originating file (UUID string).
    pub source: String,
    /// `brep`, `extrusion`, `surface`, `subd` or `unknown`.
    pub source_type: String,
    /// Maximum deviation the exporter asked its mesher for, in millimetres.
    #[serde(default)]
    pub requested_tolerance_mm: Option<f64>,
    #[serde(default)]
    pub exporter: Option<String>,
    /// Exporter-recorded revision of the source object (opaque string).
    #[serde(default)]
    pub revision: Option<String>,
    /// Which source face produced which triangles (half-open ranges).
    #[serde(default)]
    pub face_ranges: Vec<FaceRange>,
    /// Importer measurement; `None` when the source could not be evaluated.
    #[serde(default)]
    pub measured: Option<MeasuredDeviation>,
    /// Why the measurement is missing or approximate.
    #[serde(default)]
    pub note: Option<String>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct FaceRange {
    pub face: usize,
    pub start: usize,
    pub end: usize,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct MeasuredDeviation {
    /// Largest distance from a sampled source-surface point to the mesh.
    pub source_to_mesh_mm: f64,
    /// Largest distance from a sampled mesh point to the trimmed source.
    pub mesh_to_source_mm: f64,
    pub source_samples: usize,
    pub mesh_samples: usize,
    /// Mesh samples whose closest surface point fell outside the source trims
    /// (filled holes, overshoot past a trim edge).
    pub outside_trim_samples: usize,
    /// Per source face maximum deviation, indexed like `face_ranges[i].face`.
    #[serde(default)]
    pub per_face_mm: Vec<f64>,
}

impl MeshProvenance {
    /// Measured deviation in either direction, when measured.
    pub fn deviation_mm(&self) -> Option<f64> {
        self.measured
            .as_ref()
            .map(|m| m.source_to_mesh_mm.max(m.mesh_to_source_mm))
    }
    /// Whether the measurement honours the exporter's claim within `slack_mm`
    /// (normally the source model's absolute tolerance). `None` when either side is missing.
    pub fn meets_claim(&self, slack_mm: f64) -> Option<bool> {
        Some(self.deviation_mm()? <= self.requested_tolerance_mm? + slack_mm.max(0.0))
    }
    pub fn summary(&self) -> String {
        let claim = match self.requested_tolerance_mm {
            Some(t) => format!("claimed {t:.4} mm"),
            None => "no claimed tolerance".into(),
        };
        match &self.measured {
            Some(m) => format!(
                "{} {}: measured {:.4} mm ({})",
                self.source_type,
                short(&self.source),
                m.source_to_mesh_mm.max(m.mesh_to_source_mm),
                claim
            ),
            None => format!(
                "{} {}: unmeasured ({}){}",
                self.source_type,
                short(&self.source),
                claim,
                self.note
                    .as_ref()
                    .map(|n| format!("; {n}"))
                    .unwrap_or_default()
            ),
        }
    }
}

fn short(uuid: &str) -> String {
    uuid.chars().take(8).collect()
}

/// Topology and orientation summary of a triangle soup.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct MeshQuality {
    pub triangles: usize,
    /// Triangles with repeated vertices or (near) zero area after welding.
    pub degenerate: usize,
    /// Edges used by exactly one triangle (open boundary or crack).
    pub boundary_edges: usize,
    /// Edges used by more than two triangles.
    pub nonmanifold_edges: usize,
    /// Edges shared by two triangles that wind the same way (orientation flip).
    pub inconsistent_edges: usize,
    /// Divergence-theorem volume; positive for outward winding of a closed mesh.
    pub signed_volume: f64,
}

impl MeshQuality {
    pub fn closed(&self) -> bool {
        self.triangles > self.degenerate && self.boundary_edges == 0 && self.nonmanifold_edges == 0
    }
    pub fn oriented(&self) -> bool {
        self.inconsistent_edges == 0 && self.nonmanifold_edges == 0
    }
    pub fn describe(&self) -> String {
        let mut s = format!("{} triangles", self.triangles);
        if self.degenerate > 0 {
            s += &format!(", {} degenerate", self.degenerate);
        }
        s += if self.closed() {
            ", closed"
        } else {
            ", open"
        };
        if self.boundary_edges > 0 {
            s += &format!(" ({} boundary edges)", self.boundary_edges);
        }
        if self.nonmanifold_edges > 0 {
            s += &format!(", {} non-manifold edges", self.nonmanifold_edges);
        }
        if self.inconsistent_edges > 0 {
            s += &format!(", {} flipped edges", self.inconsistent_edges);
        } else if self.closed() && self.signed_volume < 0.0 {
            s += ", inward facing";
        }
        s
    }
}

impl Mesh {
    pub fn bbox(&self) -> Aabb3 {
        let mut b = Aabb3::EMPTY;
        for t in &self.tris {
            for v in t {
                b.add(*v);
            }
        }
        b
    }

    pub fn translate(&mut self, d: DVec3) {
        for t in &mut self.tris {
            for v in t {
                *v += d;
            }
        }
    }

    pub fn scale(&mut self, s: f64) {
        for t in &mut self.tris {
            for v in t {
                *v *= s;
            }
        }
    }

    /// Apply an affine transform. A mirroring transform (negative determinant) reverses the
    /// triangle winding so outward normals stay outward.
    pub fn transformed(&self, m: &DMat4) -> Mesh {
        let flip = m.determinant() < 0.0;
        Mesh {
            tris: self
                .tris
                .iter()
                .map(|t| {
                    let p = t.map(|v| m.transform_point3(v));
                    if flip {
                        [p[0], p[2], p[1]]
                    } else {
                        p
                    }
                })
                .collect(),
            provenance: self.provenance.clone(),
        }
    }

    /// Volume by the divergence theorem; positive when triangles wind outward.
    pub fn signed_volume(&self) -> f64 {
        self.tris
            .iter()
            .map(|t| t[0].dot(t[1].cross(t[2])) / 6.0)
            .sum()
    }

    /// Weld vertices within `weld_tol` and classify edges. Independent of the triangle order.
    pub fn analyze(&self, weld_tol: f64) -> MeshQuality {
        let q = weld_tol.max(1e-9);
        let mut ids: HashMap<[i64; 3], usize> = HashMap::new();
        let mut id = |p: DVec3| {
            let key = [
                (p.x / q).round() as i64,
                (p.y / q).round() as i64,
                (p.z / q).round() as i64,
            ];
            let n = ids.len();
            *ids.entry(key).or_insert(n)
        };
        // edge key -> (uses, forward uses)
        let mut edges: HashMap<(usize, usize), (u32, u32)> = HashMap::new();
        let mut degenerate = 0;
        for t in &self.tris {
            let v = [id(t[0]), id(t[1]), id(t[2])];
            let area2 = (t[1] - t[0]).cross(t[2] - t[0]).length();
            if v[0] == v[1] || v[1] == v[2] || v[0] == v[2] || area2 < 2.0 * q * q {
                degenerate += 1;
                continue;
            }
            for k in 0..3 {
                let (a, b) = (v[k], v[(k + 1) % 3]);
                let e = edges.entry((a.min(b), a.max(b))).or_insert((0, 0));
                e.0 += 1;
                if a < b {
                    e.1 += 1;
                }
            }
        }
        let mut out = MeshQuality {
            triangles: self.tris.len(),
            degenerate,
            boundary_edges: 0,
            nonmanifold_edges: 0,
            inconsistent_edges: 0,
            signed_volume: self.signed_volume(),
        };
        for (uses, forward) in edges.values() {
            match uses {
                1 => out.boundary_edges += 1,
                2 => {
                    if *forward != 1 {
                        out.inconsistent_edges += 1;
                    }
                }
                _ => out.nonmanifold_edges += 1,
            }
        }
        out
    }

    /// Move the model so its XY minimum is at the origin and its top is at Z=0
    /// (the common "top of stock is zero" convention).
    pub fn place_top_at_zero(&mut self) {
        let b = self.bbox();
        if !b.is_empty() {
            self.translate(DVec3::new(-b.min.x, -b.min.y, -b.max.z));
        }
    }

    pub fn load_stl(path: &std::path::Path) -> anyhow::Result<Mesh> {
        let mut f = std::fs::File::open(path)?;
        let m = stl_io::read_stl(&mut f)?;
        let v = |i: usize| {
            let p = m.vertices[i];
            DVec3::new(p[0] as f64, p[1] as f64, p[2] as f64)
        };
        let tris = m
            .faces
            .iter()
            .map(|f| [v(f.vertices[0]), v(f.vertices[1]), v(f.vertices[2])])
            .collect();
        Ok(Mesh {
            tris,
            provenance: None,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Axis-aligned box with outward (counter-clockwise from outside) winding.
    pub fn boxed(min: DVec3, max: DVec3) -> Mesh {
        let p = |x: f64, y: f64, z: f64| {
            DVec3::new(
                if x > 0.0 { max.x } else { min.x },
                if y > 0.0 { max.y } else { min.y },
                if z > 0.0 { max.z } else { min.z },
            )
        };
        let quad = |a: DVec3, b: DVec3, c: DVec3, d: DVec3| [[a, b, c], [a, c, d]];
        let mut tris = vec![];
        tris.extend(quad(p(0., 0., 0.), p(0., 1., 0.), p(1., 1., 0.), p(1., 0., 0.))); // bottom
        tris.extend(quad(p(0., 0., 1.), p(1., 0., 1.), p(1., 1., 1.), p(0., 1., 1.))); // top
        tris.extend(quad(p(0., 0., 0.), p(1., 0., 0.), p(1., 0., 1.), p(0., 0., 1.))); // front
        tris.extend(quad(p(1., 0., 0.), p(1., 1., 0.), p(1., 1., 1.), p(1., 0., 1.))); // right
        tris.extend(quad(p(1., 1., 0.), p(0., 1., 0.), p(0., 1., 1.), p(1., 1., 1.))); // back
        tris.extend(quad(p(0., 1., 0.), p(0., 0., 0.), p(0., 0., 1.), p(0., 1., 1.))); // left
        Mesh {
            tris,
            provenance: None,
        }
    }

    #[test]
    fn closed_box_is_oriented_and_mirroring_keeps_it_outward() {
        let m = boxed(DVec3::ZERO, DVec3::new(2.0, 3.0, 4.0));
        let q = m.analyze(1e-6);
        assert!(q.closed() && q.oriented(), "{q:?}");
        assert_eq!(q.boundary_edges, 0);
        assert!((q.signed_volume - 24.0).abs() < 1e-9);
        let mirrored = m.transformed(&DMat4::from_scale(DVec3::new(-1.0, 1.0, 1.0)));
        assert!((mirrored.signed_volume() - 24.0).abs() < 1e-9);
        assert!(mirrored.analyze(1e-6).oriented());
        // A plain vertex map without the winding fix would invert the volume.
        let naive: f64 = m
            .tris
            .iter()
            .map(|t| {
                let p = t.map(|v| DVec3::new(-v.x, v.y, v.z));
                p[0].dot(p[1].cross(p[2])) / 6.0
            })
            .sum();
        assert!((naive + 24.0).abs() < 1e-9);
    }

    #[test]
    fn open_flipped_degenerate_and_nonmanifold_meshes_are_reported() {
        let mut m = boxed(DVec3::ZERO, DVec3::ONE);
        m.tris.pop();
        let q = m.analyze(1e-6);
        assert_eq!(q.boundary_edges, 3);
        assert!(!q.closed());

        let mut m = boxed(DVec3::ZERO, DVec3::ONE);
        m.tris[0].swap(1, 2);
        let q = m.analyze(1e-6);
        assert_eq!(q.inconsistent_edges, 3);
        assert!(!q.oriented());

        let mut m = boxed(DVec3::ZERO, DVec3::ONE);
        m.tris.push([DVec3::ZERO, DVec3::X, DVec3::X * 0.5]);
        m.tris.push([DVec3::ZERO, DVec3::ZERO, DVec3::Y]);
        assert_eq!(m.analyze(1e-6).degenerate, 2);

        let mut m = boxed(DVec3::ZERO, DVec3::ONE);
        // A fin hanging off the bottom front edge makes that edge non-manifold.
        m.tris.push([DVec3::ZERO, DVec3::X, DVec3::new(0.5, -1.0, -1.0)]);
        let q = m.analyze(1e-6);
        assert_eq!(q.nonmanifold_edges, 1);
        assert_eq!(q.boundary_edges, 2);
    }

    #[test]
    fn provenance_claim_comparison_uses_both_directions_and_slack() {
        let mut p = MeshProvenance {
            source: "abc".into(),
            source_type: "brep".into(),
            requested_tolerance_mm: Some(0.01),
            exporter: None,
            revision: None,
            face_ranges: vec![],
            measured: None,
            note: None,
        };
        assert_eq!(p.meets_claim(0.0), None);
        p.measured = Some(MeasuredDeviation {
            source_to_mesh_mm: 0.004,
            mesh_to_source_mm: 0.0105,
            source_samples: 1,
            mesh_samples: 1,
            outside_trim_samples: 0,
            per_face_mm: vec![],
        });
        assert_eq!(p.deviation_mm(), Some(0.0105));
        assert_eq!(p.meets_claim(0.0), Some(false));
        assert_eq!(p.meets_claim(0.001), Some(true));
        let json = serde_json::to_string(&Mesh {
            tris: vec![],
            provenance: Some(p),
        })
        .unwrap();
        let back: Mesh = serde_json::from_str(&json).unwrap();
        assert!(back.provenance.unwrap().measured.is_some());
        let legacy: Mesh = serde_json::from_str(r#"{"tris":[]}"#).unwrap();
        assert!(legacy.provenance.is_none());
    }
}
