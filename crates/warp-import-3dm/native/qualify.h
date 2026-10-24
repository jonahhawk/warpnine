// Mesh-against-source accuracy measurement for machining meshes exported from Rhino.
// All lengths are in the model's own units; the bridge scales results to millimetres.
#pragma once
#include "opennurbs.h"
#include <string>
#include <vector>

struct QTri {
  ON_3dPoint a, b, c;
};

struct QualifyResult {
  bool measured = false;
  double source_to_mesh = 0; // max distance sampled source point -> mesh
  double mesh_to_source = 0; // max distance sampled mesh point -> trimmed source
  size_t source_samples = 0, mesh_samples = 0, outside_trim_samples = 0;
  std::vector<double> per_face; // max deviation per source face
  std::string note;             // why the result is missing or approximate
};

// Measures `tris` against `source` (ON_Brep, ON_Extrusion, ON_Surface or
// ON_SubD). Throws std::runtime_error when `cancel` becomes non-zero.
QualifyResult qualify_mesh(const ON_Geometry *source,
                           const std::vector<QTri> &tris,
                           const volatile unsigned char *cancel);

// Dense approximation of a SubD limit surface: every face of the control net
// subdivided `levels` times, with vertices moved to their limit points.
// Returns false when the SDK cannot subdivide this SubD.
bool subd_limit_triangles(const ON_SubD &subd, unsigned levels,
                          std::vector<QTri> &out);
