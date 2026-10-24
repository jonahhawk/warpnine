// WarpNine STEP reader on Open CASCADE Technology (static, pinned in vendor/occt-pin.txt).
//
// Reads every solid, free shell and free face of a STEP file, meshes it with
// BRepMesh at the requested deviation and then measures the mesh against the exact
// B-rep in both directions (the same sampled measurement the openNURBS bridge applies
// to Rhino-exported meshes). The mesher tolerance is halved until the measurement meets
// the request or the triangle budget is reached. Results are emitted as the scene JSON
// shared with the openNURBS bridge: a solid object that is `Unavailable` but
// `represented_by` its machining mesh, plus the mesh object carrying `provenance`.
//
// All output lengths are millimetres; the STEP reader converts file units itself.
#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepAlgoAPI_Cut.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepPrimAPI_MakeSphere.hxx>
#include <BRepTools.hxx>
#include <BRepTopAdaptor_FClass2d.hxx>
#include <BRep_Builder.hxx>
#include <BRep_Tool.hxx>
#include <GeomAPI_PointsToBSplineSurface.hxx>
#include <Geom_BSplineSurface.hxx>
#include <IFSelect_ReturnStatus.hxx>
#include <IMeshTools_Parameters.hxx>
#include <Interface_InterfaceModel.hxx>
#include <Interface_Static.hxx>
#include <Message.hxx>
#include <Message_Messenger.hxx>
#include <Message_PrinterOStream.hxx>
#include <Poly_Triangulation.hxx>
#include <STEPControl_Reader.hxx>
#include <STEPControl_Writer.hxx>
#include <ShapeAnalysis_ShapeTolerance.hxx>
#include <ShapeAnalysis_Surface.hxx>
#include <Standard_Failure.hxx>
#include <StepBasic_Product.hxx>
#include <StepRepr_RepresentationItem.hxx>
#include <TColStd_SequenceOfAsciiString.hxx>
#include <TColgp_Array2OfPnt.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopLoc_Location.hxx>
#include <TopTools_IndexedDataMapOfShapeListOfShape.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <TopTools_ListOfShape.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Compound.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Shape.hxx>
#include <TransferBRep.hxx>
#include <Transfer_TransientProcess.hxx>
#include <XSControl_TransferReader.hxx>
#include <XSControl_WorkSession.hxx>
#include <gp_Pnt.hxx>
#include <gp_Pnt2d.hxx>
#include <gp_Trsf.hxx>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <limits>
#include <locale>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
constexpr const char *EXPORTER = "warpnine-occt/7.9.3";
constexpr size_t SOURCE_BUDGET = 40000;
constexpr size_t MESH_BUDGET = 20000;
constexpr int EDGE_SAMPLES = 256;
constexpr int MESH_ATTEMPTS = 4;

std::string quote(const std::string &s) {
  std::ostringstream o;
  o << '"';
  for (unsigned char c : s) {
    if (c == '"' || c == '\\')
      o << '\\' << c;
    else if (c < 32)
      o << "\\u" << std::hex << std::setw(4) << std::setfill('0') << (unsigned)c << std::dec;
    else
      o << c;
  }
  o << '"';
  return o.str();
}
struct Json : std::ostringstream {
  Json() {
    imbue(std::locale::classic());
    *this << std::setprecision(17);
  }
};
void trace(const std::string &what) {
  static const bool on = std::getenv("WARPSTEP_TRACE") != nullptr;
  if (on)
    std::fprintf(stderr, "[warpstep] %s\n", what.c_str());
}
void check(const volatile unsigned char *cancel) {
  if (cancel && *cancel)
    throw std::runtime_error("Import cancelled");
}

struct V3 {
  double x = 0, y = 0, z = 0;
  V3() = default;
  V3(double x, double y, double z) : x(x), y(y), z(z) {}
  V3(const gp_Pnt &p) : x(p.X()), y(p.Y()), z(p.Z()) {}
  double operator[](int k) const { return k == 0 ? x : k == 1 ? y : z; }
  V3 operator-(const V3 &o) const { return {x - o.x, y - o.y, z - o.z}; }
  V3 operator+(const V3 &o) const { return {x + o.x, y + o.y, z + o.z}; }
  V3 operator*(double s) const { return {x * s, y * s, z * s}; }
  double dot(const V3 &o) const { return x * o.x + y * o.y + z * o.z; }
  double len2() const { return dot(*this); }
  double dist(const V3 &o) const { return std::sqrt((*this - o).len2()); }
  bool finite() const { return std::isfinite(x) && std::isfinite(y) && std::isfinite(z); }
};
struct Tri {
  V3 a, b, c;
  int face = 0;
  gp_Pnt2d uv_a, uv_centroid; // surface parameters of `a` and of the centroid seed
  bool has_uv = false;
};

V3 closest_on_segment(const V3 &a, const V3 &b, const V3 &p) {
  V3 ab = b - a;
  double len2 = ab.len2();
  if (len2 <= 0)
    return a;
  double t = std::clamp((p - a).dot(ab) / len2, 0.0, 1.0);
  return a + ab * t;
}
// Ericson, Real-Time Collision Detection, 5.1.5.
V3 closest_on_tri(const Tri &t, const V3 &p) {
  V3 ab = t.b - t.a, ac = t.c - t.a, ap = p - t.a;
  double d1 = ab.dot(ap), d2 = ac.dot(ap);
  if (d1 <= 0 && d2 <= 0)
    return t.a;
  V3 bp = p - t.b;
  double d3 = ab.dot(bp), d4 = ac.dot(bp);
  if (d3 >= 0 && d4 <= d3)
    return t.b;
  double vc = d1 * d4 - d3 * d2;
  if (vc <= 0 && d1 >= 0 && d3 <= 0)
    return t.a + ab * (d1 / (d1 - d3));
  V3 cp = p - t.c;
  double d5 = ab.dot(cp), d6 = ac.dot(cp);
  if (d6 >= 0 && d5 <= d6)
    return t.c;
  double vb = d5 * d2 - d1 * d6;
  if (vb <= 0 && d2 >= 0 && d6 <= 0)
    return t.a + ac * (d2 / (d2 - d6));
  double va = d3 * d6 - d5 * d4;
  if (va <= 0 && (d4 - d3) >= 0 && (d5 - d6) >= 0)
    return t.b + (t.c - t.b) * ((d4 - d3) / ((d4 - d3) + (d5 - d6)));
  double sum = va + vb + vc;
  if (!(std::abs(sum) > 1e-300)) {
    V3 best = closest_on_segment(t.a, t.b, p);
    for (auto q : {closest_on_segment(t.b, t.c, p), closest_on_segment(t.c, t.a, p)})
      if (q.dist(p) < best.dist(p))
        best = q;
    return best;
  }
  double denom = 1.0 / sum;
  return t.a + ab * (vb * denom) + ac * (vc * denom);
}
template <class Fn> void visit_shell(const int c[3], const int n[3], int r, Fn fn) {
  auto ok = [&](int i, int j, int k) {
    return i >= 0 && j >= 0 && k >= 0 && i < n[0] && j < n[1] && k < n[2];
  };
  if (r == 0) {
    if (ok(c[0], c[1], c[2]))
      fn(c[0], c[1], c[2]);
    return;
  }
  for (int di = -r; di <= r; ++di)
    for (int dj = -r; dj <= r; ++dj)
      for (int dk : {-r, r})
        if (ok(c[0] + di, c[1] + dj, c[2] + dk))
          fn(c[0] + di, c[1] + dj, c[2] + dk);
  for (int dk = -r + 1; dk <= r - 1; ++dk) {
    for (int dj = -r; dj <= r; ++dj)
      for (int di : {-r, r})
        if (ok(c[0] + di, c[1] + dj, c[2] + dk))
          fn(c[0] + di, c[1] + dj, c[2] + dk);
    for (int di = -r + 1; di <= r - 1; ++di)
      for (int dj : {-r, r})
        if (ok(c[0] + di, c[1] + dj, c[2] + dk))
          fn(c[0] + di, c[1] + dj, c[2] + dk);
  }
}
// Uniform grid over the triangles; nearest-triangle search by expanding Chebyshev shells.
struct TriGrid {
  V3 lo;
  double cell = 1;
  int n[3] = {1, 1, 1};
  std::vector<std::vector<int>> cells;
  const std::vector<Tri> *tris = nullptr;
  int index(int i, int j, int k) const { return (k * n[1] + j) * n[0] + i; }
  void coords(const V3 &p, int c[3]) const {
    for (int k = 0; k < 3; ++k)
      c[k] = std::clamp(int(std::floor((p[k] - lo[k]) / cell)), 0, n[k] - 1);
  }
  void build(const std::vector<Tri> &t) {
    tris = &t;
    V3 mn(1e300, 1e300, 1e300), mx(-1e300, -1e300, -1e300);
    for (auto &tri : t)
      for (auto &p : {tri.a, tri.b, tri.c}) {
        mn = {std::min(mn.x, p.x), std::min(mn.y, p.y), std::min(mn.z, p.z)};
        mx = {std::max(mx.x, p.x), std::max(mx.y, p.y), std::max(mx.z, p.z)};
      }
    if (t.empty())
      mn = mx = V3();
    V3 size = mx - mn;
    double maxd = std::max({size.x, size.y, size.z, 1e-9});
    double target = std::cbrt(std::max<double>(1.0, t.size() / 2.0));
    cell = maxd / std::clamp(target, 1.0, 96.0);
    for (int k = 0; k < 3; ++k)
      n[k] = std::clamp(int(size[k] / cell) + 1, 1, 256);
    lo = mn;
    cells.assign(size_t(n[0]) * n[1] * n[2], {});
    for (size_t i = 0; i < t.size(); ++i) {
      V3 bmin = t[i].a, bmax = t[i].a;
      for (auto &p : {t[i].b, t[i].c}) {
        bmin = {std::min(bmin.x, p.x), std::min(bmin.y, p.y), std::min(bmin.z, p.z)};
        bmax = {std::max(bmax.x, p.x), std::max(bmax.y, p.y), std::max(bmax.z, p.z)};
      }
      int c0[3], c1[3];
      coords(bmin, c0);
      coords(bmax, c1);
      for (int k = c0[2]; k <= c1[2]; ++k)
        for (int j = c0[1]; j <= c1[1]; ++j)
          for (int ii = c0[0]; ii <= c1[0]; ++ii)
            cells[index(ii, j, k)].push_back(int(i));
    }
  }
  double distance(const V3 &p) const {
    int c[3];
    coords(p, c);
    double best2 = std::numeric_limits<double>::infinity();
    int rmax = 0;
    for (int k = 0; k < 3; ++k)
      rmax = std::max({rmax, c[k], n[k] - 1 - c[k]});
    for (int r = 0; r <= rmax; ++r) {
      if (r > 1) {
        double lb = (r - 1) * cell;
        if (lb * lb >= best2)
          break;
      }
      visit_shell(c, n, r, [&](int i, int j, int k) {
        for (int ti : cells[index(i, j, k)]) {
          double d = closest_on_tri((*tris)[ti], p).dist(p);
          if (d * d < best2)
            best2 = d * d;
        }
      });
    }
    return std::sqrt(best2);
  }
};

struct Meshed {
  std::vector<Tri> tris;
  std::vector<size_t> face_start; // faces + 1 entries
  double deflection = 0;
};
struct Measurement {
  double source_to_mesh = 0, mesh_to_source = 0;
  size_t source_samples = 0, mesh_samples = 0, outside_trim = 0;
  std::vector<double> per_face;
  double worst() const { return std::max(source_to_mesh, mesh_to_source); }
};

// Meshes `shape` with BRepMesh at `deflection` (mm) and collects outward-wound triangles.
Meshed mesh_shape(const TopoDS_Shape &shape, const TopTools_IndexedMapOfShape &faces,
                  double deflection, const volatile unsigned char *cancel) {
  BRepTools::Clean(shape);
  IMeshTools_Parameters p;
  p.Deflection = deflection;
  p.DeflectionInterior = deflection;
  p.Angle = 0.3;
  p.AngleInterior = 0.3;
  p.Relative = false;
  p.InParallel = true;
  p.ControlSurfaceDeflection = true;
  BRepMesh_IncrementalMesh mesher(shape, p);
  Meshed out;
  out.deflection = deflection;
  out.face_start.assign(size_t(faces.Extent()) + 1, 0);
  for (int fi = 1; fi <= faces.Extent(); ++fi) {
    check(cancel);
    out.face_start[fi - 1] = out.tris.size();
    const TopoDS_Face &face = TopoDS::Face(faces(fi));
    TopLoc_Location loc;
    Handle(Poly_Triangulation) tri = BRep_Tool::Triangulation(face, loc);
    if (tri.IsNull() || tri->NbTriangles() == 0)
      throw std::runtime_error("face " + std::to_string(fi) + " could not be meshed");
    const gp_Trsf trsf = loc.Transformation();
    const bool identity = loc.IsIdentity();
    const bool flip = (face.Orientation() == TopAbs_REVERSED) != (!identity && trsf.IsNegative());
    const bool uv = tri->HasUVNodes();
    for (int ti = 1; ti <= tri->NbTriangles(); ++ti) {
      if ((ti & 4095) == 0)
        check(cancel);
      int n1, n2, n3;
      tri->Triangle(ti).Get(n1, n2, n3);
      if (flip)
        std::swap(n2, n3);
      gp_Pnt a = tri->Node(n1), b = tri->Node(n2), c = tri->Node(n3);
      if (!identity) {
        a.Transform(trsf);
        b.Transform(trsf);
        c.Transform(trsf);
      }
      Tri t;
      t.a = a;
      t.b = b;
      t.c = c;
      t.face = fi - 1;
      if (!t.a.finite() || !t.b.finite() || !t.c.finite())
        throw std::runtime_error("mesh produced non-finite coordinates");
      if (t.a.dist(t.b) <= 0 || t.b.dist(t.c) <= 0 || t.c.dist(t.a) <= 0)
        continue; // collapsed triangle
      if (uv) {
        gp_Pnt2d ua = tri->UVNode(n1), ub = tri->UVNode(n2), uc = tri->UVNode(n3);
        t.uv_a = ua;
        t.uv_centroid = gp_Pnt2d((ua.X() + ub.X() + uc.X()) / 3, (ua.Y() + ub.Y() + uc.Y()) / 3);
        t.has_uv = true;
      }
      out.tris.push_back(t);
    }
  }
  out.face_start[faces.Extent()] = out.tris.size();
  return out;
}

// Samples the exact B-rep against the mesh and the mesh against the exact B-rep.
Measurement measure(const TopoDS_Shape &shape, const TopTools_IndexedMapOfShape &faces,
                    const Meshed &mesh, double tol, const volatile unsigned char *cancel) {
  Measurement m;
  const int nf = faces.Extent();
  m.per_face.assign(size_t(nf), 0.0);
  TriGrid grid;
  grid.build(mesh.tris);
  TopTools_IndexedDataMapOfShapeListOfShape edge_faces;
  TopExp::MapShapesAndAncestors(shape, TopAbs_EDGE, TopAbs_FACE, edge_faces);
  std::vector<std::vector<std::vector<V3>>> edge_polylines(static_cast<size_t>(nf));
  size_t counter = 0;
  auto tick = [&] {
    if ((++counter & 1023) == 0)
      check(cancel);
  };
  // Source -> mesh: trimmed surface samples.
  const int n = std::clamp(int(std::sqrt(double(SOURCE_BUDGET) / std::max(1, nf))), 6, 64);
  // Strict classification for source samples; a band of `tol` (3D) for mesh samples, whose
  // boundary vertices lie on the trims by construction.
  std::vector<std::unique_ptr<BRepTopAdaptor_FClass2d>> classifiers(static_cast<size_t>(nf)), lenient(static_cast<size_t>(nf));
  for (int fi = 1; fi <= nf; ++fi) {
    const TopoDS_Face &face = TopoDS::Face(faces(fi));
    BRepAdaptor_Surface surf(face, true);
    double u0, u1, v0, v1;
    BRepTools::UVBounds(face, u0, u1, v0, v1);
    classifiers[fi - 1] = std::make_unique<BRepTopAdaptor_FClass2d>(face, BRep_Tool::Tolerance(face));
    lenient[fi - 1] = std::make_unique<BRepTopAdaptor_FClass2d>(face, std::max(BRep_Tool::Tolerance(face), tol));
    for (int j = 0; j < n; ++j)
      for (int i = 0; i < n; ++i) {
        tick();
        gp_Pnt2d uv(u0 + (u1 - u0) * (i + 0.5) / n, v0 + (v1 - v0) * (j + 0.5) / n);
        if (classifiers[fi - 1]->Perform(uv) != TopAbs_IN)
          continue;
        double d = grid.distance(V3(surf.Value(uv.X(), uv.Y())));
        m.source_to_mesh = std::max(m.source_to_mesh, d);
        m.per_face[fi - 1] = std::max(m.per_face[fi - 1], d);
        ++m.source_samples;
      }
  }
  // Source -> mesh: 3D edge samples (bound the chord error along trims and seams).
  for (int ei = 1; ei <= edge_faces.Extent(); ++ei) {
    const TopoDS_Edge &edge = TopoDS::Edge(edge_faces.FindKey(ei));
    if (BRep_Tool::Degenerated(edge))
      continue;
    BRepAdaptor_Curve curve(edge);
    const double t0 = curve.FirstParameter(), t1 = curve.LastParameter();
    if (!(t1 > t0))
      continue;
    std::vector<int> owners;
    for (TopTools_ListIteratorOfListOfShape it(edge_faces.FindFromIndex(ei)); it.More(); it.Next())
      owners.push_back(faces.FindIndex(it.Value()) - 1);
    std::vector<V3> polyline;
    polyline.reserve(EDGE_SAMPLES + 1);
    for (int k = 0; k <= EDGE_SAMPLES; ++k) {
      tick();
      V3 p(curve.Value(t0 + (t1 - t0) * k / EDGE_SAMPLES));
      polyline.push_back(p);
      double d = grid.distance(p);
      m.source_to_mesh = std::max(m.source_to_mesh, d);
      ++m.source_samples;
      for (int f : owners)
        if (f >= 0)
          m.per_face[size_t(f)] = std::max(m.per_face[size_t(f)], d);
    }
    for (int f : owners)
      if (f >= 0)
        edge_polylines[size_t(f)].push_back(polyline);
  }
  // Mesh -> source: vertices and centroids projected onto their own face.
  std::vector<std::unique_ptr<ShapeAnalysis_Surface>> analyzers(static_cast<size_t>(nf));
  const size_t stride = std::max<size_t>(1, mesh.tris.size() * 2 / MESH_BUDGET);
  for (size_t ti = 0; ti < mesh.tris.size(); ti += stride) {
    tick();
    const Tri &t = mesh.tris[ti];
    auto &sas = analyzers[size_t(t.face)];
    if (!sas)
      sas = std::make_unique<ShapeAnalysis_Surface>(BRep_Tool::Surface(TopoDS::Face(faces(t.face + 1))));
    const V3 centroid = (t.a + t.b + t.c) * (1.0 / 3.0);
    const std::pair<V3, gp_Pnt2d> samples[2] = {{t.a, t.uv_a}, {centroid, t.uv_centroid}};
    for (auto &[p, seed] : samples) {
      gp_Pnt gp(p.x, p.y, p.z);
      gp_Pnt2d uv = t.has_uv ? sas->NextValueOfUV(seed, gp, tol) : sas->ValueOfUV(gp, tol);
      double d = gp.Distance(sas->Value(uv));
      if (lenient[size_t(t.face)]->Perform(uv) == TopAbs_OUT) {
        ++m.outside_trim;
        double e = std::numeric_limits<double>::infinity();
        for (auto &poly : edge_polylines[size_t(t.face)])
          for (size_t i = 1; i < poly.size(); ++i)
            e = std::min(e, closest_on_segment(poly[i - 1], poly[i], p).dist(p));
        if (std::isfinite(e))
          d = std::max(d, e);
      }
      m.mesh_to_source = std::max(m.mesh_to_source, d);
      m.per_face[size_t(t.face)] = std::max(m.per_face[size_t(t.face)], d);
      ++m.mesh_samples;
    }
  }
  return m;
}

double unit_scale(const std::string &name) {
  std::string u;
  for (char c : name)
    u += char(std::toupper((unsigned char)c));
  if (u.find("MILLI") != std::string::npos)
    return 1.0;
  if (u.find("CENTI") != std::string::npos)
    return 10.0;
  if (u.find("MICRO") != std::string::npos)
    return 0.001;
  if (u.find("INCH") != std::string::npos)
    return 25.4;
  if (u.find("FOOT") != std::string::npos || u.find("FEET") != std::string::npos)
    return 304.8;
  if (u.find("METRE") != std::string::npos || u.find("METER") != std::string::npos)
    return 1000.0;
  return 1.0;
}
// Schema from the FILE_SCHEMA header: 203, 214, 242 or 0.
unsigned schema_of(const std::string &path) {
  std::ifstream in(path, std::ios::binary);
  std::string head(8192, '\0');
  in.read(&head[0], std::streamsize(head.size()));
  head.resize(size_t(in.gcount()));
  for (auto &c : head)
    c = char(std::toupper((unsigned char)c));
  auto pos = head.find("FILE_SCHEMA");
  if (pos == std::string::npos)
    return 0;
  std::string s = head.substr(pos, 200);
  if (s.find("AP242") != std::string::npos)
    return 242;
  if (s.find("AUTOMOTIVE_DESIGN") != std::string::npos || s.find("AP214") != std::string::npos)
    return 214;
  if (s.find("CONFIG_CONTROL_DESIGN") != std::string::npos || s.find("AP203") != std::string::npos)
    return 203;
  return 0;
}

struct Reader {
  std::string path;
  double tolerance, max_triangles;
  const volatile unsigned char *cancel;
  double triangles = 0;
  std::vector<std::string> diagnostics;
  void warn(const std::string &id, const std::string &code, const std::string &message) {
    diagnostics.push_back("{\"object\":" + quote(id) + ",\"code\":" + quote(code) +
                          ",\"message\":" + quote(message) + "}");
  }
  static std::string entity_name(const Handle(Standard_Transient) & ent) {
    Handle(StepRepr_RepresentationItem) item = Handle(StepRepr_RepresentationItem)::DownCast(ent);
    if (item.IsNull() || item->Name().IsNull())
      return "";
    return item->Name()->ToCString();
  }
  // Result shape -> STEP entity name, collected once from the transfer process.
  std::vector<std::pair<TopoDS_Shape, std::string>> names;
  void collect_names(STEPControl_Reader &reader) {
    Handle(Transfer_TransientProcess) tp = reader.WS()->TransferReader()->TransientProcess();
    if (tp.IsNull())
      return;
    for (int i = 1; i <= tp->NbMapped(); ++i) {
      Handle(Standard_Transient) ent = tp->Mapped(i);
      std::string n = entity_name(ent);
      if (n.empty())
        continue;
      TopoDS_Shape sh = TransferBRep::ShapeResult(tp, ent);
      if (!sh.IsNull())
        names.push_back({sh, n});
    }
  }
  std::string name_of(const TopoDS_Shape &shape) {
    for (auto &[s, n] : names)
      if (s.IsSame(shape))
        return n;
    // A named entity whose result contains this shape (e.g. a shell-based surface model).
    for (auto &[s, n] : names)
      for (TopExp_Explorer ex(s, shape.ShapeType()); ex.More(); ex.Next())
        if (ex.Current().IsSame(shape))
          return n;
    // A named sub-shape (a shell named instead of its solid).
    for (auto &[s, n] : names)
      for (TopExp_Explorer ex(shape, s.ShapeType()); ex.More(); ex.Next())
        if (ex.Current().IsSame(s))
          return n;
    return "";
  }
  void point(Json &o, const V3 &p) { o << '[' << p.x << ',' << p.y << ',' << p.z << ']'; }
  // Emits the solid/surface object and its machining mesh; returns both JSON objects.
  std::pair<std::string, std::string> object(const TopoDS_Shape &shape, const std::string &kind,
                                             const std::string &name, int k) {
    const std::string sid = "s" + std::to_string(k), mid = "m" + std::to_string(k);
    TopTools_IndexedMapOfShape faces;
    TopExp::MapShapes(shape, TopAbs_FACE, faces);
    if (faces.Extent() == 0)
      throw std::runtime_error(kind + " has no faces");
    Meshed best;
    Measurement best_m;
    bool have = false, met = false;
    std::string note;
    double deflection = tolerance;
    for (int attempt = 0; attempt < MESH_ATTEMPTS; ++attempt) {
      check(cancel);
      trace("mesh " + sid + " at " + std::to_string(deflection));
      Meshed meshed = mesh_shape(shape, faces, deflection, cancel);
      trace("meshed " + std::to_string(meshed.tris.size()) + " tris");
      if (triangles + double(meshed.tris.size()) > max_triangles) {
        if (!have)
          throw std::runtime_error("Import exceeds the mesh memory limit of " +
                                   std::to_string(size_t(max_triangles)) +
                                   " triangles; raise the limit or relax the import tolerance");
        note = "mesh limit reached before the requested deviation was met";
        break;
      }
      Measurement m = measure(shape, faces, meshed, tolerance, cancel);
      {
        size_t worst_face = 0;
        for (size_t i = 0; i < m.per_face.size(); ++i)
          if (m.per_face[i] > m.per_face[worst_face])
            worst_face = i;
        trace("measured s2m " + std::to_string(m.source_to_mesh) + " m2s " + std::to_string(m.mesh_to_source) +
              " outside " + std::to_string(m.outside_trim) + " samples " + std::to_string(m.source_samples) + "/" +
              std::to_string(m.mesh_samples) + " worst face " + std::to_string(worst_face) + " of " +
              std::to_string(m.per_face.size()));
      }
      if (!have || m.worst() < best_m.worst()) {
        best = std::move(meshed);
        best_m = std::move(m);
        have = true;
      }
      if (best_m.worst() <= tolerance) {
        met = true;
        break;
      }
      deflection *= 0.5;
    }
    if (have && !met && note.empty())
      note = "requested deviation not met after " + std::to_string(MESH_ATTEMPTS) + " refinements";
    triangles += double(best.tris.size());
    Json mesh;
    mesh << "{\"id\":" << quote(mid) << ",\"name\":" << quote(name + " machining mesh")
         << ",\"layer\":\"model\",\"visible\":true,\"source\":null,\"geometry\":{\"Mesh\":{\"tris\":[";
    for (size_t i = 0; i < best.tris.size(); ++i) {
      if (i)
        mesh << ',';
      mesh << '[';
      point(mesh, best.tris[i].a);
      mesh << ',';
      point(mesh, best.tris[i].b);
      mesh << ',';
      point(mesh, best.tris[i].c);
      mesh << ']';
    }
    mesh << "],\"provenance\":{\"source\":" << quote(sid) << ",\"source_type\":" << quote(kind)
         << ",\"requested_tolerance_mm\":" << tolerance << ",\"exporter\":" << quote(EXPORTER)
         << ",\"revision\":null,\"face_ranges\":[";
    for (int fi = 0; fi < faces.Extent(); ++fi) {
      if (fi)
        mesh << ',';
      mesh << "{\"face\":" << fi << ",\"start\":" << best.face_start[size_t(fi)]
           << ",\"end\":" << best.face_start[size_t(fi) + 1] << '}';
    }
    mesh << "],\"measured\":{\"source_to_mesh_mm\":" << best_m.source_to_mesh
         << ",\"mesh_to_source_mm\":" << best_m.mesh_to_source
         << ",\"source_samples\":" << best_m.source_samples
         << ",\"mesh_samples\":" << best_m.mesh_samples
         << ",\"outside_trim_samples\":" << best_m.outside_trim << ",\"per_face_mm\":[";
    for (size_t i = 0; i < best_m.per_face.size(); ++i) {
      if (i)
        mesh << ',';
      mesh << best_m.per_face[i];
    }
    mesh << "]},\"note\":" << (note.empty() ? std::string("null") : quote(note)) << "}}}}";
    std::ostringstream reason;
    reason << kind << " with " << faces.Extent() << " faces, meshed by WarpNine at "
           << std::setprecision(4) << best.deflection << " mm deflection";
    warn(sid, "solid_represented", "Represented by machining mesh " + mid);
    Json solid;
    solid << "{\"id\":" << quote(sid) << ",\"name\":" << quote(name)
          << ",\"layer\":\"model\",\"visible\":true,\"source\":null,\"geometry\":{\"Unavailable\":{\"reason\":"
          << quote(reason.str()) << ",\"represented_by\":" << quote(mid) << "}}}";
    return {solid.str(), mesh.str()};
  }
  std::string run() {
    STEPControl_Reader reader;
    Interface_Static::SetCVal("xstep.cascade.unit", "MM");
    Interface_Static::SetIVal("read.step.product.mode", 1);
    Interface_Static::SetIVal("read.surfacecurve.mode", 3);
    if (reader.ReadFile(path.c_str()) != IFSelect_RetDone)
      throw std::runtime_error("Not a readable STEP file: " + path);
    check(cancel);
    trace("read file");
    reader.TransferRoots();
    trace("transferred " + std::to_string(reader.NbShapes()) + " roots");
    collect_names(reader);
    trace("collected " + std::to_string(names.size()) + " names");
    TColStd_SequenceOfAsciiString length_units, angle_units, solid_angle_units;
    reader.FileUnits(length_units, angle_units, solid_angle_units);
    std::string units = length_units.Length() > 0 ? length_units(1).ToCString() : "MILLIMETRE";
    for (auto &c : units)
      c = char(std::toupper((unsigned char)c));
    double scale = unit_scale(units);
    unsigned schema = schema_of(path);
    std::vector<std::pair<std::string, std::string>> objects;
    double source_tolerance = 0;
    int k = 0;
    std::vector<std::string> product_names;
    for (int i = 1; i <= reader.Model()->NbEntities(); ++i) {
      Handle(StepBasic_Product) product = Handle(StepBasic_Product)::DownCast(reader.Model()->Value(i));
      if (!product.IsNull() && !product->Name().IsNull() && product->Name()->Length() > 0)
        product_names.push_back(product->Name()->ToCString());
    }
    auto add = [&](const TopoDS_Shape &shape, const std::string &kind) {
      ++k;
      trace("object " + std::to_string(k) + " " + kind);
      std::string name = name_of(shape);
      trace("named '" + name + "'");
      if (name.empty() && product_names.size() == 1)
        name = product_names.front() + (k > 1 ? " " + std::to_string(k) : "");
      if (name.empty())
        name = (kind == "brep" ? "Solid " : "Surface ") + std::to_string(k);
      source_tolerance = std::max(source_tolerance, ShapeAnalysis_ShapeTolerance().Tolerance(shape, 1));
      trace("tolerance " + std::to_string(source_tolerance));
      const std::string sid = "s" + std::to_string(k);
      try {
        objects.push_back(object(shape, kind, name, k));
      } catch (const Standard_Failure &e) {
        std::string what = e.GetMessageString() ? e.GetMessageString() : "Open CASCADE failure";
        warn(sid, "unavailable_geometry", "Could not mesh '" + name + "': " + what);
        objects.push_back({"{\"id\":" + quote(sid) + ",\"name\":" + quote(name) +
                               ",\"layer\":\"model\",\"visible\":true,\"source\":null,\"geometry\":{\"Unavailable\":{\"reason\":" +
                               quote("Could not mesh this " + kind + ": " + what) + "}}}",
                           ""});
      } catch (const std::runtime_error &e) {
        std::string what = e.what();
        if (what.find("cancelled") != std::string::npos || what.find("memory limit") != std::string::npos)
          throw;
        warn(sid, "unavailable_geometry", "Could not mesh '" + name + "': " + what);
        objects.push_back({"{\"id\":" + quote(sid) + ",\"name\":" + quote(name) +
                               ",\"layer\":\"model\",\"visible\":true,\"source\":null,\"geometry\":{\"Unavailable\":{\"reason\":" +
                               quote("Could not mesh this " + kind + ": " + what) + "}}}",
                           ""});
      }
    };
    for (int r = 1; r <= reader.NbShapes(); ++r) {
      TopoDS_Shape root = reader.Shape(r);
      for (TopExp_Explorer ex(root, TopAbs_SOLID); ex.More(); ex.Next())
        add(ex.Current(), "brep");
      for (TopExp_Explorer ex(root, TopAbs_SHELL, TopAbs_SOLID); ex.More(); ex.Next())
        add(ex.Current(), "surface");
      for (TopExp_Explorer ex(root, TopAbs_FACE, TopAbs_SHELL); ex.More(); ex.Next())
        add(ex.Current(), "surface");
    }
    if (objects.empty())
      throw std::runtime_error("The STEP file contains no solids, shells or faces");
    Json o;
    o << "{\"file_version\":" << schema << ",\"unit_scale_mm\":" << scale
      << ",\"source_units\":" << quote(units) << ",\"source_tolerance_mm\":" << source_tolerance
      << ",\"scene\":{\"next_id\":1,\"groups\":[],\"layers\":[{\"id\":\"model\",\"name\":\"STEP model\",\"visible\":true}],"
         "\"definitions\":[],\"objects\":[";
    bool first = true;
    for (auto &[solid, mesh] : objects) {
      for (auto *s : {&solid, &mesh}) {
        if (s->empty())
          continue;
        if (!first)
          o << ',';
        first = false;
        o << *s;
      }
    }
    o << "]},\"diagnostics\":[";
    for (size_t i = 0; i < diagnostics.size(); ++i) {
      if (i)
        o << ',';
      o << diagnostics[i];
    }
    o << "]}";
    return o.str();
  }
};

char *dup(const std::string &s) {
  char *p = static_cast<char *>(std::malloc(s.size() + 1));
  if (p)
    std::memcpy(p, s.c_str(), s.size() + 1);
  return p;
}
void quiet() {
  static bool done = false;
  if (done)
    return;
  done = true;
  Message::DefaultMessenger()->RemovePrinters(STANDARD_TYPE(Message_PrinterOStream));
}

TopoDS_Shape wavy_surface(double x0, double y0, double w, double h, double amp) {
  const int nu = 6, nv = 5;
  TColgp_Array2OfPnt pts(1, nu, 1, nv);
  for (int i = 1; i <= nu; ++i)
    for (int j = 1; j <= nv; ++j) {
      double x = x0 + w * (i - 1) / (nu - 1), y = y0 + h * (j - 1) / (nv - 1);
      double z = amp * std::sin(3.0 * (i - 1) / (nu - 1) * 3.14159265) * std::cos(2.0 * (j - 1) / (nv - 1) * 3.14159265);
      pts.SetValue(i, j, gp_Pnt(x, y, z));
    }
  Handle(Geom_BSplineSurface) surf = GeomAPI_PointsToBSplineSurface(pts, 3, 3, GeomAbs_C2, 1e-6).Surface();
  return BRepBuilderAPI_MakeFace(surf, 1e-6).Face();
}
} // namespace

extern "C" char *warpstep_read(const char *path, double tolerance, double max_triangles,
                               const unsigned char *cancel, int *ok) {
  quiet();
  *ok = 0;
  try {
    Reader r{path, tolerance, max_triangles, cancel};
    std::string out = r.run();
    *ok = 1;
    return dup(out);
  } catch (const Standard_Failure &e) {
    return dup(std::string("Open CASCADE failure: ") + (e.GetMessageString() ? e.GetMessageString() : "unknown"));
  } catch (const std::exception &e) {
    return dup(e.what());
  } catch (...) {
    return dup("Unknown STEP reader failure");
  }
}
extern "C" void warpstep_free(char *p) { std::free(p); }

// Synthetic fixtures for tests. Variant 0 (mm): sphere r10 at (10,10,10); 20x30x15 box at
// x 40..60 with a vertical r4 through hole at its centre; an open wavy B-spline face over
// x 70..130, y 0..40, amplitude 3. Variant 1 (inch): a 1 x 2 x 0.5 inch box at the origin.
extern "C" int warpstep_write_fixture(const char *path, int variant) {
  quiet();
  try {
    STEPControl_Writer writer; // constructing it registers the STEP parameters
    BRep_Builder builder;
    TopoDS_Compound compound;
    builder.MakeCompound(compound);
    if (!Interface_Static::SetCVal("write.step.unit", variant == 1 ? "INCH" : "MM"))
      return 0;
    Interface_Static::SetCVal("write.step.product.name", "WarpNine fixture");
    if (variant == 0) {
      builder.Add(compound, BRepPrimAPI_MakeSphere(gp_Pnt(10, 10, 10), 10.0).Shape());
      TopoDS_Shape box = BRepPrimAPI_MakeBox(gp_Pnt(40, 0, 0), 20.0, 30.0, 15.0).Shape();
      TopoDS_Shape hole = BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(50, 15, -5), gp_Dir(0, 0, 1)), 4.0, 25.0).Shape();
      builder.Add(compound, BRepAlgoAPI_Cut(box, hole).Shape());
      builder.Add(compound, wavy_surface(70, 0, 60, 40, 3.0));
    } else if (variant == 1) {
      builder.Add(compound, BRepPrimAPI_MakeBox(25.4, 50.8, 12.7).Shape());
    } else {
      return 0;
    }
    if (writer.Transfer(compound, STEPControl_AsIs) != IFSelect_RetDone)
      return 0;
    int ok = writer.Write(path) == IFSelect_RetDone;
    Interface_Static::SetCVal("write.step.unit", "MM");
    return ok;
  } catch (...) {
    return 0;
  }
}
