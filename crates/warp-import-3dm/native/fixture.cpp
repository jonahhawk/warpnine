#include "opennurbs.h"
#include "qualify.h"
#include <cmath>
#include <memory>
#include <stdexcept>
#include <vector>
static ON_UUID fixture_id(unsigned n) {
  ON_UUID id = ON_nil_uuid;
  id.Data1 = n;
  id.Data2 = 0x1234;
  id.Data3 = 0x5678;
  id.Data4[0] = 0x90;
  return id;
}
namespace {
constexpr double PI = 3.14159265358979323846;
ON_Mesh from_tris(const std::vector<QTri> &tris) {
  ON_Mesh mesh;
  int v = 0;
  for (size_t i = 0; i < tris.size(); ++i) {
    mesh.SetVertex(v, tris[i].a);
    mesh.SetVertex(v + 1, tris[i].b);
    mesh.SetVertex(v + 2, tris[i].c);
    mesh.SetTriangle(int(i), v, v + 1, v + 2);
    v += 3;
  }
  mesh.ComputeVertexNormals();
  return mesh;
}
// Latitude/longitude sphere, outward winding.
std::vector<QTri> sphere_tris(ON_3dPoint c, double r, int nu, int nv) {
  auto P = [&](int i, int j) {
    double th = 2 * PI * i / nu, ph = -PI / 2 + PI * j / nv;
    return c + ON_3dVector(r * std::cos(ph) * std::cos(th), r * std::cos(ph) * std::sin(th),
                           r * std::sin(ph));
  };
  std::vector<QTri> out;
  for (int j = 0; j < nv; ++j)
    for (int i = 0; i < nu; ++i) {
      ON_3dPoint a = P(i, j), b = P(i + 1, j), cc = P(i + 1, j + 1), d = P(i, j + 1);
      if (j > 0)
        out.push_back({a, b, cc});
      if (j < nv - 1)
        out.push_back({a, cc, d});
    }
  return out;
}
std::vector<QTri> annulus_tris(ON_3dPoint c, double rin, double rout, int n) {
  std::vector<QTri> out;
  for (int i = 0; i < n; ++i) {
    double t0 = 2 * PI * i / n, t1 = 2 * PI * (i + 1) / n;
    ON_3dPoint a = c + ON_3dVector(rin * std::cos(t0), rin * std::sin(t0), 0);
    ON_3dPoint b = c + ON_3dVector(rout * std::cos(t0), rout * std::sin(t0), 0);
    ON_3dPoint cc = c + ON_3dVector(rout * std::cos(t1), rout * std::sin(t1), 0);
    ON_3dPoint d = c + ON_3dVector(rin * std::cos(t1), rin * std::sin(t1), 0);
    out.push_back({a, b, cc});
    out.push_back({a, cc, d});
  }
  return out;
}
std::vector<QTri> disc_tris(ON_3dPoint c, double r, int n, bool up) {
  std::vector<QTri> out;
  for (int i = 0; i < n; ++i) {
    double t0 = 2 * PI * i / n, t1 = 2 * PI * (i + 1) / n;
    ON_3dPoint a = c + ON_3dVector(r * std::cos(t0), r * std::sin(t0), 0);
    ON_3dPoint b = c + ON_3dVector(r * std::cos(t1), r * std::sin(t1), 0);
    out.push_back(up ? QTri{c, a, b} : QTri{c, b, a});
  }
  return out;
}
std::vector<QTri> cylinder_tris(ON_3dPoint c, double r, double z0, double z1, int n) {
  std::vector<QTri> out = disc_tris(ON_3dPoint(c.x, c.y, z1), r, n, true);
  auto bottom = disc_tris(ON_3dPoint(c.x, c.y, z0), r, n, false);
  out.insert(out.end(), bottom.begin(), bottom.end());
  for (int i = 0; i < n; ++i) {
    double t0 = 2 * PI * i / n, t1 = 2 * PI * (i + 1) / n;
    ON_3dPoint a(c.x + r * std::cos(t0), c.y + r * std::sin(t0), z0);
    ON_3dPoint b(c.x + r * std::cos(t1), c.y + r * std::sin(t1), z0);
    ON_3dPoint cc(c.x + r * std::cos(t1), c.y + r * std::sin(t1), z1);
    ON_3dPoint d(c.x + r * std::cos(t0), c.y + r * std::sin(t0), z1);
    out.push_back({a, b, cc});
    out.push_back({a, cc, d});
  }
  return out;
}
std::vector<QTri> box_tris(ON_3dPoint lo, ON_3dPoint hi) {
  auto p = [&](int x, int y, int z) {
    return ON_3dPoint(x ? hi.x : lo.x, y ? hi.y : lo.y, z ? hi.z : lo.z);
  };
  std::vector<QTri> out;
  auto quad = [&](ON_3dPoint a, ON_3dPoint b, ON_3dPoint c, ON_3dPoint d) {
    out.push_back({a, b, c});
    out.push_back({a, c, d});
  };
  quad(p(0, 0, 0), p(0, 1, 0), p(1, 1, 0), p(1, 0, 0));
  quad(p(0, 0, 1), p(1, 0, 1), p(1, 1, 1), p(0, 1, 1));
  quad(p(0, 0, 0), p(1, 0, 0), p(1, 0, 1), p(0, 0, 1));
  quad(p(1, 0, 0), p(1, 1, 0), p(1, 1, 1), p(1, 0, 1));
  quad(p(1, 1, 0), p(0, 1, 0), p(0, 1, 1), p(1, 1, 1));
  quad(p(0, 1, 0), p(0, 0, 0), p(0, 0, 1), p(0, 1, 1));
  return out;
}
std::unique_ptr<ON_Brep> annulus_face(ON_3dPoint c) {
  ON_Plane plane(c, ON_zaxis);
  ON_ArcCurve outer(ON_Circle(plane, 10));
  std::unique_ptr<ON_Brep> trimmed(ON_BrepTrimmedPlane(plane, outer));
  ON_SimpleArray<ON_Curve *> hole;
  auto inner = new ON_ArcCurve(ON_Circle(plane, 3));
  inner->Reverse();
  hole.Append(inner);
  if (!trimmed || !trimmed->NewPlanarFaceLoop(0, ON_BrepLoop::inner, hole, false))
    throw std::runtime_error("Trimmed fixture failed");
  return trimmed;
}
} // namespace
bool warp3dm_write_fixture_impl(const char *path, int variant, int version) {
  ONX_Model m;
  const bool inch = variant == 1;
  double s = inch ? 1.0 / 25.4 : 1.0;
  m.m_settings.m_ModelUnitsAndTolerances.m_unit_system = ON_UnitSystem(
      inch ? ON::LengthUnitSystem::Inches : ON::LengthUnitSystem::Millimeters);
  m.m_settings.m_ModelUnitsAndTolerances.m_absolute_tolerance = 0.001 * s;
  ON_Layer layer;
  layer.SetId(fixture_id(900));
  layer.SetName(L"Parts");
  layer.SetIndex(0);
  m.AddModelComponent(layer, false);
  ON_Layer detail;
  detail.SetId(fixture_id(901));
  detail.SetName(L"Details");
  detail.SetIndex(1);
  m.AddModelComponent(detail, false);
  auto add = [&](const ON_Geometry &geom, unsigned id, const wchar_t *name,
                 bool member = false, int li = 0) {
    std::unique_ptr<ON_Geometry> g(geom.Duplicate());
    if (s != 1)
      g->Transform(ON_Xform::ScaleTransformation(ON_3dPoint::Origin, s));
    ON_3dmObjectAttributes a;
    a.m_uuid = fixture_id(id);
    a.m_name = name;
    a.m_layer_index = li;
    if (member)
      a.SetMode(ON::idef_object);
    if (m.AddModelGeometryComponent(g.get(), &a, false).IsEmpty())
      throw std::runtime_error("Fixture geometry insertion failed");
  };
  auto instance = [&](unsigned id, unsigned def, const wchar_t *name,
                      ON_Xform x, bool member = false) {
    ON_InstanceRef r;
    r.m_instance_definition_uuid = fixture_id(def);
    r.m_xform = x;
    ON_3dmObjectAttributes a;
    a.m_uuid = fixture_id(id);
    a.m_name = name;
    a.m_layer_index = 0;
    if (member)
      a.SetMode(ON::idef_object);
    for (int i = 0; i < 3; ++i)
      r.m_xform[i][3] *= s;
    if (m.AddModelGeometryComponent(&r, &a, false).IsEmpty())
      throw std::runtime_error("Fixture instance failed");
  };
  ON_Polyline poly;
  poly.Append(ON_3dPoint(0, 0, 0));
  poly.Append(ON_3dPoint(10 + (variant == 2 ? 2 : 0), 0, 0));
  poly.Append(ON_3dPoint(10, 5, 0));
  poly.Append(ON_3dPoint(0, 5, 0));
  if (variant != 8)
    poly.Append(ON_3dPoint(0, 0, 0));
  if (variant != 3)
    add(ON_PolylineCurve(poly), 10, L"Block perimeter", true);
  add(ON_ArcCurve(ON_Circle(ON_Plane(ON_3dPoint(5, 2.5, 0), ON_zaxis), 0.8)),
      11, L"Block hole", true);
  ON_InstanceDefinition def;
  def.SetId(fixture_id(100));
  def.SetName(L"Part");
  ON_SimpleArray<ON_UUID> ids;
  if (variant != 3)
    ids.Append(fixture_id(10));
  ids.Append(fixture_id(11));
  def.SetInstanceGeometryIdList(ids);
  m.AddModelComponent(def, false);
  instance(20, 100, L"Part A", ON_Xform::TranslationTransformation(0, 0, 0));
  instance(21, 100, L"Part B", ON_Xform::TranslationTransformation(30, 0, 0));
  ON_Xform mirror = ON_Xform::IdentityTransformation;
  mirror[0][0] = -2;
  mirror[1][1] = 3;
  mirror[0][3] = 80;
  mirror[1][3] = 20;
  instance(22, 100, L"Mirrored scaled part", mirror, true);
  ON_InstanceDefinition nested;
  nested.SetId(fixture_id(101));
  nested.SetName(L"Nested");
  ON_SimpleArray<ON_UUID> children;
  children.Append(fixture_id(22));
  nested.SetInstanceGeometryIdList(children);
  m.AddModelComponent(nested, false);
  instance(23, 101, L"Nested part",
           ON_Xform::TranslationTransformation(0, 10, 0));
  add(ON_LineCurve(ON_3dPoint(0, 20, 0), ON_3dPoint(15, 20, 0)), 30,
      L"Loose drive", false, 1);
  add(ON_Point(ON_3dPoint(7, 8, 0)), 31, L"Drill point", false, 1);
  ON_ArcCurve circle(ON_Circle(ON_Plane(ON_3dPoint(50, 50, 0), ON_zaxis), 4));
  add(circle, 32, L"Circle", false, 1);
  ON_NurbsCurve rat(3, true, 3, 3);
  rat.SetCV(0, ON_4dPoint(10, 0, 0, 1));
  double w = std::sqrt(0.5);
  rat.SetCV(1, ON_4dPoint(10 * w, 10 * w, 0, w));
  rat.SetCV(2, ON_4dPoint(0, 10, 0, 1));
  rat.SetKnot(0, 0);
  rat.SetKnot(1, 0);
  rat.SetKnot(2, 1);
  rat.SetKnot(3, 1);
  add(rat, 33, L"Rational quarter circle", false, 1);
  ON_3dPoint cvs[6] = {ON_3dPoint(0, 0, 3),  ON_3dPoint(5, 0, 5),
                       ON_3dPoint(10, 5, 3), ON_3dPoint(5, 10, 2),
                       ON_3dPoint(0, 10, 1), ON_3dPoint(-5, 5, 2)};
  ON_NurbsCurve periodic;
  periodic.CreatePeriodicUniformNurbs(3, 4, 6, cvs);
  add(periodic, 34, L"Periodic nonplanar", false, 1);
  ON_PolyCurve pc;
  pc.Append(new ON_LineCurve(ON_3dPoint(0, 30, 0), ON_3dPoint(10, 30, 0)));
  pc.Append(new ON_ArcCurve(ON_Arc(
      ON_3dPoint(10, 30, 0), ON_3dPoint(13.5355339059327, 31.4644660940673, 0),
      ON_3dPoint(15, 35, 0))));
  add(pc, 35, L"Mixed polycurve", false, 1);
  ON_Mesh mesh;
  mesh.SetVertex(0, ON_3dPoint(0, 0, 0));
  mesh.SetVertex(1, ON_3dPoint(100, 0, 0));
  mesh.SetVertex(2, ON_3dPoint(100, 100, 0));
  mesh.SetVertex(3, ON_3dPoint(0, 100, 0));
  mesh.SetQuad(0, 0, 1, 2, 3);
  mesh.ComputeVertexNormals();
  add(mesh, 40, L"Explicit planar mesh");
  if (variant == 4) {
    ON_ArcCurve outer(ON_Circle(ON_xy_plane, 10));
    std::unique_ptr<ON_Brep> trimmed(ON_BrepTrimmedPlane(ON_xy_plane, outer));
    ON_SimpleArray<ON_Curve *> hole;
    auto inner = new ON_ArcCurve(ON_Circle(ON_xy_plane, 3));
    inner->Reverse();
    hole.Append(inner);
    if (!trimmed ||
        !trimmed->NewPlanarFaceLoop(0, ON_BrepLoop::inner, hole, false))
      throw std::runtime_error("Trimmed fixture failed");
    auto cached = new ON_Mesh();
    cached->SetVertex(0, ON_3dPoint(-10, -10, 0));
    cached->SetVertex(1, ON_3dPoint(10, -10, 0));
    cached->SetVertex(2, ON_3dPoint(10, 10, 0));
    cached->SetVertex(3, ON_3dPoint(-10, 10, 0));
    cached->SetQuad(0, 0, 1, 2, 3);
    cached->ComputeVertexNormals();
    if (!trimmed->m_F[0].SetMesh(ON::render_mesh, cached)) {
      delete cached;
      throw std::runtime_error("Fixture cache assignment failed");
    }
    add(*trimmed, 50, L"Trimmed face with hole");
    std::unique_ptr<ON_Brep> sphere(
        ON_BrepSphere(ON_Sphere(ON_3dPoint(30, 30, 10), 10)));
    add(*sphere, 51, L"Curved sphere with seam");
    std::unique_ptr<ON_Extrusion> extrusion(
        ON_Extrusion::CreateFrom3dCurve(outer, nullptr, 5, true));
    add(*extrusion, 52, L"Capped extrusion");
  }
  if (variant == 9 || variant == 10) {
    // M5/D01: machining meshes exported with provenance claims against their solids.
    auto claimed = [&](const std::vector<QTri> &tris, unsigned id, const wchar_t *name,
                       unsigned source, const wchar_t *type, double tol, bool member = false) {
      ON_Mesh mesh = from_tris(tris);
      ON_3dmObjectAttributes a;
      a.m_uuid = fixture_id(id);
      a.m_name = name;
      a.m_layer_index = 0;
      if (member)
        a.SetMode(ON::idef_object);
      if (source) {
        ON_wString su;
        ON_UuidToString(fixture_id(source), su);
        a.SetUserString(L"warpnine.source", su.Array());
        a.SetUserString(L"warpnine.source_type", type);
        ON_wString st;
        st.Format(L"%g", tol);
        a.SetUserString(L"warpnine.tolerance", st.Array());
        a.SetUserString(L"warpnine.exporter", L"fixture/1");
        a.SetUserString(L"warpnine.revision", L"fixture-rev-1");
        ON_wString ranges;
        ranges.Format(L"0:0-%d", mesh.m_F.Count());
        a.SetUserString(L"warpnine.face_ranges", ranges.Array());
      }
      if (m.AddModelGeometryComponent(&mesh, &a, false).IsEmpty())
        throw std::runtime_error("Fixture mesh insertion failed");
    };
    ON_3dPoint sc(30, 30, 10);
    std::unique_ptr<ON_Brep> sphere(ON_BrepSphere(ON_Sphere(sc, 10)));
    add(*sphere, 60, L"Qualified sphere");
    auto annulus = annulus_face(ON_3dPoint(0, 0, 0));
    add(*annulus, 62, L"Qualified annulus");
    ON_Circle circle(ON_Plane(ON_3dPoint(60, 60, 0), ON_zaxis), 10);
    ON_ArcCurve rim(circle);
    std::unique_ptr<ON_Extrusion> extrusion(
        ON_Extrusion::CreateFrom3dCurve(rim, nullptr, 5, true));
    if (!extrusion)
      throw std::runtime_error("Extrusion fixture failed");
    add(*extrusion, 64, L"Qualified extrusion");
    ON_3dPoint corners[8] = {ON_3dPoint(80, 0, 0),  ON_3dPoint(100, 0, 0),  ON_3dPoint(100, 20, 0),
                             ON_3dPoint(80, 20, 0), ON_3dPoint(80, 0, 20),  ON_3dPoint(100, 0, 20),
                             ON_3dPoint(100, 20, 20), ON_3dPoint(80, 20, 20)};
    std::unique_ptr<ON_SubD> subd(
        ON_SubD::CreateSubDBox(corners, ON_SubDEdgeTag::Smooth, 1, 1, 1, nullptr));
    if (!subd)
      throw std::runtime_error("SubD fixture failed");
    add(*subd, 66, L"Qualified SubD");
    if (variant == 9) {
      claimed(sphere_tris(sc, 10, 128, 64), 61, L"Sphere mesh", 60, L"brep", 0.01);
      claimed(annulus_tris(ON_3dPoint(0, 0, 0), 3, 10, 128), 63, L"Annulus mesh", 62, L"brep", 0.01);
      ON_BoundingBox eb = extrusion->BoundingBox();
      claimed(cylinder_tris(ON_3dPoint(60, 60, 0), 10, eb.m_min.z, eb.m_max.z, 128), 65,
              L"Extrusion mesh", 64, L"extrusion", 0.01);
      std::vector<QTri> limit;
      if (!subd_limit_triangles(*subd, 2, limit))
        throw std::runtime_error("SubD limit fixture failed");
      claimed(limit, 67, L"SubD mesh", 66, L"subd", 0.5);
      // A closed box mesh inside a mirrored block instance.
      claimed(box_tris(ON_3dPoint(0, 0, 0), ON_3dPoint(4, 4, 4)), 68, L"Box mesh", 0, L"", 0, true);
      ON_InstanceDefinition meshes;
      meshes.SetId(fixture_id(103));
      meshes.SetName(L"Mesh block");
      ON_SimpleArray<ON_UUID> ids;
      ids.Append(fixture_id(68));
      meshes.SetInstanceGeometryIdList(ids);
      m.AddModelComponent(meshes, false);
      ON_Xform mirror = ON_Xform::IdentityTransformation;
      mirror[0][0] = -1;
      mirror[0][3] = 120;
      instance(69, 103, L"Mirrored mesh block", mirror);
    } else {
      claimed(sphere_tris(sc, 10, 12, 6), 71, L"Coarse sphere mesh", 60, L"brep", 0.01);
      claimed(disc_tris(ON_3dPoint(0, 0, 0), 10, 128, true), 72, L"Hole-filling disc", 62, L"brep", 0.01);
      claimed(box_tris(ON_3dPoint(0, 40, 0), ON_3dPoint(4, 44, 4)), 73, L"Orphan mesh", 999, L"brep", 0.01);
      std::vector<QTri> flipped = {{ON_3dPoint(0, 60, 0), ON_3dPoint(10, 60, 0), ON_3dPoint(10, 70, 0)},
                                   {ON_3dPoint(0, 60, 0), ON_3dPoint(0, 70, 0), ON_3dPoint(10, 70, 0)}};
      claimed(flipped, 74, L"Flipped mesh", 0, L"", 0);
      claimed(box_tris(corners[0], corners[6]), 75, L"Control net mesh", 66, L"subd", 0.5);
    }
  }
  if (variant == 5)
    instance(80, 999, L"Missing linked definition",
             ON_Xform::IdentityTransformation);
  if (variant == 6) {
    ON_InstanceDefinition cycle;
    cycle.SetId(fixture_id(102));
    cycle.SetName(L"Cycle");
    ON_SimpleArray<ON_UUID> cyc;
    cyc.Append(fixture_id(81));
    cycle.SetInstanceGeometryIdList(cyc);
    m.AddModelComponent(cycle, false);
    instance(81, 102, L"Cyclic member", ON_Xform::IdentityTransformation, true);
    instance(82, 102, L"Cyclic root", ON_Xform::IdentityTransformation);
  }
  if (variant == 7)
    m.m_settings.m_ModelUnitsAndTolerances.m_unit_system = ON_UnitSystem::None;
  return m.Write(ON_wString(path).Array(), version, nullptr);
}
