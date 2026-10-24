#include "opennurbs.h"
#undef min
#undef max
#include "qualify.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <map>
#include <mutex>
#include <set>
#include <sstream>
#include <stdexcept>
#include <vector>

namespace {
std::mutex gate;
void init() {
  static bool started = (ON::Begin(), true);
  (void)started;
}
std::string uuid(ON_UUID id) {
  char b[40]{};
  ON_UuidToString(id, b);
  return b;
}
std::string utf8(const ON_wString &s) {
  const ON_String encoded(s);
  return encoded.Array() ? std::string(encoded.Array()) : std::string();
}
std::string quote(const std::string &s) {
  std::ostringstream o;
  o << '"';
  for (unsigned char c : s) {
    if (c == '"' || c == '\\')
      o << '\\' << c;
    else if (c < 32)
      o << "\\u" << std::hex << std::setw(4) << std::setfill('0') << (unsigned)c
        << std::dec;
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
void point(Json &o, ON_3dPoint p, double scale = 1) {
  if (!p.IsValid())
    throw std::runtime_error("Invalid curve coordinates");
  o << '[' << p.x * scale << ',' << p.y * scale << ',' << p.z * scale << ']';
}
void sample(const ON_BezierCurve &b, double tol, int depth,
            std::vector<ON_3dPoint> &pts) {
  ON_3dPoint a, z;
  b.GetCV(0, a);
  b.GetCV(b.CVCount() - 1, z);
  ON_3dVector d = z - a;
  double dev = 0;
  const double len = d.LengthSquared();
  for (int i = 0; i < b.CVCount(); ++i) {
    ON_4dPoint h;
    b.GetCV(i, h);
    if (!(h.w > 0))
      throw std::runtime_error("Non-positive rational weight is unsupported");
    ON_3dPoint p;
    b.GetCV(i, p);
    double t = len > 0 ? std::clamp(((p - a) * d) / len, 0.0, 1.0) : 0;
    dev = std::max(dev, p.DistanceTo(a + t * d));
  }
  if (dev <= tol) {
    if (pts.empty())
      pts.push_back(a);
    pts.push_back(z);
    return;
  }
  if (depth >= 32 || pts.size() > 1000000)
    throw std::runtime_error(
        "Curve approximation limit exceeded; increase import tolerance");
  ON_BezierCurve l, r;
  if (!b.Split(0.5, l, r))
    throw std::runtime_error("Bezier split failed");
  sample(l, tol, depth + 1, pts);
  sample(r, tol, depth + 1, pts);
}
std::string curve(const ON_Curve &c, double scale, double tol) {
  ON_NurbsCurve n;
  if (!c.GetNurbForm(n, 0.0) || !n.IsValid())
    throw std::runtime_error("Curve cannot be represented as valid NURBS");
  std::vector<ON_3dPoint> pts;
  for (int i = 0; i <= n.CVCount() - n.Order(); ++i) {
    if (n.Knot(i + n.Order() - 2) >= n.Knot(i + n.Order() - 1))
      continue;
    ON_BezierCurve b;
    if (!n.ConvertSpanToBezier(i, b))
      throw std::runtime_error("NURBS span conversion failed");
    sample(b, tol, 0, pts);
  }
  bool closed = c.IsClosed();
  if (closed && pts.size() > 2 && pts.front().DistanceTo(pts.back()) < tol)
    pts.pop_back();
  if (pts.size() < 2)
    throw std::runtime_error("Degenerate curve");
  Json o;
  o << "{\"Curve\":{\"closed\":" << (closed ? "true" : "false")
    << ",\"points\":[";
  for (size_t i = 0; i < pts.size(); ++i) {
    if (i)
      o << ',';
    point(o, pts[i], scale);
  }
  o << "],\"analytic\":{\"Nurbs\":{\"degree\":" << n.Degree() << ",\"knots\":[";
  // openNURBS omits the two unused end knots; restore the full conventional
  // vector.
  o << n.Knot(0);
  for (int i = 0; i < n.KnotCount(); ++i)
    o << ',' << n.Knot(i);
  o << ',' << n.Knot(n.KnotCount() - 1) << "],\"controls\":[";
  for (int i = 0; i < n.CVCount(); ++i) {
    if (i)
      o << ',';
    ON_3dPoint p;
    n.GetCV(i, p);
    point(o, p, scale);
  }
  o << "],\"weights\":[";
  for (int i = 0; i < n.CVCount(); ++i) {
    if (i)
      o << ',';
    o << n.Weight(i);
  }
  o << "]}}}}";
  return o.str();
}
// Exporter claims carried as object user strings (see docs/decisions/3dm-meshing.md).
struct Claim {
  bool present = false;
  std::string source, type, exporter, revision;
  double tolerance = -1; // model units; < 0 when absent
  std::vector<std::array<int, 3>> face_ranges; // face, first mesh face, end mesh face
};
Claim read_claim(const ON_3dmObjectAttributes *a) {
  Claim c;
  if (!a)
    return c;
  ON_wString v;
  if (!a->GetUserString(L"warpnine.source", v))
    return c;
  c.present = true;
  c.source = utf8(v);
  if (a->GetUserString(L"warpnine.source_type", v))
    c.type = utf8(v);
  if (a->GetUserString(L"warpnine.tolerance", v)) {
    try {
      c.tolerance = std::stod(utf8(v));
    } catch (...) {
      c.tolerance = -1;
    }
    if (!std::isfinite(c.tolerance))
      c.tolerance = -1;
  }
  if (a->GetUserString(L"warpnine.exporter", v))
    c.exporter = utf8(v);
  if (a->GetUserString(L"warpnine.revision", v))
    c.revision = utf8(v);
  if (a->GetUserString(L"warpnine.face_ranges", v)) {
    std::istringstream in(utf8(v));
    std::string item;
    while (std::getline(in, item, ';')) {
      int f, s, e;
      char c1, c2;
      std::istringstream x(item);
      if (x >> f >> c1 >> s >> c2 >> e && c1 == ':' && c2 == '-')
        c.face_ranges.push_back({f, s, e});
    }
  }
  return c;
}
std::string type_name(const ON_Geometry *g) {
  if (ON_Brep::Cast(g))
    return "brep";
  if (ON_Extrusion::Cast(g))
    return "extrusion";
  if (ON_SubD::Cast(g))
    return "subd";
  if (ON_Surface::Cast(g))
    return "surface";
  return "unknown";
}
struct Reader {
  ONX_Model model;
  double scale = 1, tolerance = 0.01, gain = 1;
  double max_triangles = 2e6;
  size_t triangles = 0;
  const volatile unsigned char *cancel = nullptr;
  std::map<std::string, const ON_ModelGeometryComponent *> objects;
  std::map<std::string, const ON_InstanceDefinition *> defs;
  std::map<int, std::string> layers;
  std::map<std::string, Claim> claims;                   // mesh id -> claim
  std::map<std::string, std::vector<std::string>> claimants; // source id -> mesh ids
  std::vector<std::string> diagnostics;
  void check() const {
    if (cancel && *cancel)
      throw std::runtime_error("Import cancelled");
  }
  void warn(std::string id, std::string code, std::string msg) {
    diagnostics.push_back("{\"object\":" + quote(id) + ",\"code\":" +
                          quote(code) + ",\"message\":" + quote(msg) + "}");
  }
  double growth(const ON_Geometry *g, std::set<std::string> stack) {
    auto r = ON_InstanceRef::Cast(g);
    if (!r)
      return 1;
    std::string id = uuid(r->m_instance_definition_uuid);
    if (stack.size() >= 64 || !stack.insert(id).second)
      throw std::runtime_error(
          "Cyclic or excessively nested block definitions");
    auto it = defs.find(id);
    if (it == defs.end())
      return 1;
    // sqrt(||A||1*||A||inf) bounds the maximum singular value, including shear.
    double one = 0, inf = 0;
    for (int i = 0; i < 3; ++i) {
      double row = 0, col = 0;
      for (int j = 0; j < 3; ++j) {
        row += std::abs(r->m_xform[i][j]);
        col += std::abs(r->m_xform[j][i]);
      }
      one = std::max(one, col);
      inf = std::max(inf, row);
    }
    double child = 1;
    for (int k = 0; k < it->second->InstanceGeometryIdList().Count(); ++k) {
      auto gid = it->second->InstanceGeometryIdList()[k];
      auto obj = objects.find(uuid(gid));
      if (obj != objects.end())
        child = std::max(child, growth(obj->second->Geometry(nullptr), stack));
    }
    return std::sqrt(one * inf) * child;
  }
  std::string mesh(const ON_Mesh &m, const std::string &id) {
    if (!m.IsValid())
      throw std::runtime_error("Invalid mesh");
    std::vector<QTri> tris;
    std::vector<size_t> tri_start(m.m_F.Count() + 1, 0);
    for (int i = 0; i < m.m_F.Count(); ++i) {
      if ((i & 4095) == 0)
        check();
      const auto &f = m.m_F[i];
      tri_start[i] = tris.size();
      const int count = f.IsTriangle() ? 1 : 2;
      for (int t = 0; t < count; ++t)
        tris.push_back({m.Vertex(f.vi[0]), m.Vertex(f.vi[t == 0 ? 1 : 2]),
                        m.Vertex(f.vi[t == 0 ? 2 : 3])});
    }
    tri_start[m.m_F.Count()] = tris.size();
    if (double(triangles + tris.size()) > max_triangles)
      throw std::runtime_error(
          "Import exceeds the mesh memory limit of " +
          std::to_string(size_t(max_triangles)) +
          " triangles; raise the limit or simplify meshes in Rhino");
    triangles += tris.size();
    for (auto &t : tris)
      if (!t.a.IsValid() || !t.b.IsValid() || !t.c.IsValid())
        throw std::runtime_error("Invalid mesh coordinates");
    Json o;
    o << "{\"Mesh\":{\"tris\":[";
    for (size_t i = 0; i < tris.size(); ++i) {
      if (i)
        o << ',';
      o << '[';
      point(o, tris[i].a, scale);
      o << ',';
      point(o, tris[i].b, scale);
      o << ',';
      point(o, tris[i].c, scale);
      o << ']';
    }
    o << ']';
    auto claim = claims.find(id);
    if (claim != claims.end()) {
      const Claim &c = claim->second;
      auto src = objects.find(c.source);
      const ON_Geometry *source = src == objects.end() ? nullptr : src->second->Geometry(nullptr);
      std::string type = source ? type_name(source) : (c.type.empty() ? "unknown" : c.type);
      QualifyResult q;
      std::string note;
      if (!source) {
        note = "source object " + c.source + " is not in the file";
        warn(id, "mesh_source_missing", "Machining mesh claims source " + c.source + ", which is not in this file");
      } else if (type == "unknown") {
        note = "source is not a solid, surface or SubD";
        warn(id, "mesh_source_unsupported", "Machining mesh claims a source that is not a B-rep, extrusion, surface or SubD");
      } else {
        q = qualify_mesh(source, tris, cancel);
        note = q.note;
        if (!c.type.empty() && c.type != type)
          note += (note.empty() ? "" : "; ") + ("exporter recorded type '" + c.type + "', file has " + type);
      }
      o << ",\"provenance\":{\"source\":" << quote(c.source) << ",\"source_type\":" << quote(type)
        << ",\"requested_tolerance_mm\":";
      if (c.tolerance >= 0)
        o << c.tolerance * scale;
      else
        o << "null";
      o << ",\"exporter\":" << (c.exporter.empty() ? "null" : quote(c.exporter))
        << ",\"revision\":" << (c.revision.empty() ? "null" : quote(c.revision)) << ",\"face_ranges\":[";
      bool first = true;
      for (auto &r : c.face_ranges) {
        int s = std::clamp(r[1], 0, m.m_F.Count());
        int e = std::clamp(r[2], s, m.m_F.Count());
        if (!first)
          o << ',';
        first = false;
        o << "{\"face\":" << r[0] << ",\"start\":" << tri_start[s] << ",\"end\":" << tri_start[e] << '}';
      }
      o << "],\"measured\":";
      if (q.measured) {
        o << "{\"source_to_mesh_mm\":" << q.source_to_mesh * scale
          << ",\"mesh_to_source_mm\":" << q.mesh_to_source * scale
          << ",\"source_samples\":" << q.source_samples << ",\"mesh_samples\":" << q.mesh_samples
          << ",\"outside_trim_samples\":" << q.outside_trim_samples << ",\"per_face_mm\":[";
        for (size_t i = 0; i < q.per_face.size(); ++i) {
          if (i)
            o << ',';
          o << q.per_face[i] * scale;
        }
        o << "]}";
      } else
        o << "null";
      o << ",\"note\":" << (note.empty() ? "null" : quote(note)) << '}';
    }
    o << "}}";
    return o.str();
  }
  std::string solid(const std::string &id, const std::string &reason) {
    auto it = claimants.find(id);
    if (it == claimants.end()) {
      warn(id, "unavailable_geometry", reason);
      return "{\"Unavailable\":{\"reason\":" + quote(reason) + "}}";
    }
    if (it->second.size() > 1)
      warn(id, "multiple_meshes", std::to_string(it->second.size()) +
                                      " machining meshes claim this source; the first is used");
    warn(id, "solid_represented", "Represented by machining mesh " + it->second.front());
    return "{\"Unavailable\":{\"reason\":" + quote(reason) +
           ",\"represented_by\":" + quote(it->second.front()) + "}}";
  }
  std::string geometry(const ON_Geometry *g, std::string id) {
    try {
      check();
      if (auto p = ON_Point::Cast(g)) {
        Json o;
        o << "{\"Point\":";
        point(o, p->point, scale);
        o << '}';
        return o.str();
      }
      if (auto c = ON_Curve::Cast(g))
        return curve(*c, scale, tolerance / (scale * gain));
      if (auto m = ON_Mesh::Cast(g))
        return mesh(*m, id);
      if (auto r = ON_InstanceRef::Cast(g)) {
        std::string def = uuid(r->m_instance_definition_uuid);
        if (!defs.count(def) || (defs.at(def)->IsLinkedType() &&
                                defs.at(def)->InstanceGeometryIdList().Count() == 0))
          throw std::runtime_error("Missing linked block definition; embed it "
                                   "in Rhino and save again");
        Json o;
        o << "{\"Instance\":{\"definition\":" << quote(def)
          << ",\"transform\":[";
        for (int col = 0; col < 4; ++col)
          for (int row = 0; row < 4; ++row) {
            if (col || row)
              o << ',';
            o << r->m_xform[row][col] * ((col == 3 && row < 3) ? scale : 1);
          }
        o << "]}}";
        return o.str();
      }
      if (auto b = ON_Brep::Cast(g)) {
        ON_SimpleArray<const ON_Mesh *> cached;
        b->GetMesh(ON::render_mesh, cached);
        return solid(id, "B-rep has " + std::to_string(b->m_F.Count()) + " faces, " +
                             std::to_string(b->m_L.Count()) + " trim loops and " +
                             std::to_string(cached.Count()) +
                             " cached render meshes. Create an explicit Mesh in Rhino with a "
                             "controlled tolerance; cached render meshes are not qualified "
                             "machining geometry.");
      }
      if (ON_Extrusion::Cast(g) || ON_Surface::Cast(g) || ON_SubD::Cast(g))
        return solid(id, "Solid/surface has no qualified machining mesh. Create an explicit "
                         "Mesh in Rhino using a controlled tolerance, then reimport. Cached "
                         "render meshes are not machining geometry.");
      throw std::runtime_error("Unsupported Rhino object type; convert it to a "
                               "curve or mesh in Rhino");
    } catch (const std::exception &e) {
      std::string what = e.what();
      if (what == "Import cancelled" || what.rfind("Import exceeds", 0) == 0)
        throw;
      warn(id, "unavailable_geometry", what);
      return "{\"Unavailable\":{\"reason\":" + quote(what) + "}}";
    }
  }
  std::string object(std::string id) {
    auto it = objects.find(id);
    if (it == objects.end()) {
      warn(id, "missing_block_member",
           "Missing block member; embed linked geometry in Rhino");
      return "{\"id\":" + quote(id) +
             ",\"name\":\"Missing block "
             "member\",\"layer\":\"default\",\"visible\":true,\"source\":null,"
             "\"geometry\":{\"Unavailable\":{\"reason\":\"Missing block "
             "member\"}}}";
    }
    auto obj = it->second;
    auto a = obj->Attributes(nullptr);
    Json o;
    o << "{\"id\":" << quote(id)
      << ",\"name\":" << quote(a ? utf8(a->m_name) : id) << ",\"layer\":"
      << quote(a && layers.count(a->m_layer_index) ? layers[a->m_layer_index]
                                                   : "default")
      << ",\"visible\":" << (!a || a->IsVisible() ? "true" : "false")
      << ",\"source\":null,\"geometry\":"
      << geometry(obj->Geometry(nullptr), id) << '}';
    return o.str();
  }
  std::string read(const char *path, double tol) {
    tolerance = tol;
    ON_wString file(path);
    ON_wString log;
    ON_TextLog errors(log);
    if (!model.Read(file.Array(), &errors))
      throw std::runtime_error("Cannot read .3dm file (corrupt, unsupported "
                               "version, or inaccessible)");
    check();
    if (model.m_3dm_file_version > 80)
      throw std::runtime_error(
          "Rhino file is newer than supported version 8; save as Rhino 8");
    const auto &units = model.m_settings.m_ModelUnitsAndTolerances;
    auto us = units.m_unit_system.UnitSystem();
    if (us == ON::LengthUnitSystem::None || us == ON::LengthUnitSystem::Unset)
      throw std::runtime_error("Rhino document has no units; set model units "
                               "in Rhino before import");
    scale = ON::UnitScale(units.m_unit_system, ON_UnitSystem::Millimeters);
    if (!(scale > 0) || !std::isfinite(scale))
      throw std::runtime_error("Invalid model unit scale");
    ONX_ModelComponentIterator oi(model,
                                  ON_ModelComponent::Type::ModelGeometry);
    for (auto c = oi.FirstComponent(); c; c = oi.NextComponent())
      if (auto obj = ON_ModelGeometryComponent::Cast(c))
        objects[uuid(c->Id())] = obj;
    ONX_ModelComponentIterator di(model,
                                  ON_ModelComponent::Type::InstanceDefinition);
    for (auto c = di.FirstComponent(); c; c = di.NextComponent())
      if (auto d = ON_InstanceDefinition::Cast(c))
        defs[uuid(c->Id())] = d;
    for (auto [id, obj] : objects)
      gain = std::max(gain, growth(obj->Geometry(nullptr), {}));
    if (!std::isfinite(gain) || gain > 1e12)
      throw std::runtime_error(
          "Block scale exceeds supported approximation range");
    // Exporter claims must be known before any solid is emitted.
    for (auto [id, obj] : objects) {
      if (!ON_Mesh::Cast(obj->Geometry(nullptr)))
        continue;
      Claim c = read_claim(obj->Attributes(nullptr));
      if (c.present) {
        claimants[c.source].push_back(id);
        claims[id] = std::move(c);
      }
    }
    Json o;
    o << "{\"file_version\":" << model.m_3dm_file_version
      << ",\"unit_scale_mm\":" << scale
      << ",\"source_units\":" << quote(utf8(units.m_unit_system.ToString()))
      << ",\"source_tolerance_mm\":" << units.m_absolute_tolerance * scale
      << ",\"scene\":{\"next_id\":1,\"groups\":[],\"layers\":[{\"id\":"
         "\"default\",\"name\":\"Default\",\"visible\":true}";
    ONX_ModelComponentIterator li(model, ON_ModelComponent::Type::Layer);
    for (auto c = li.FirstComponent(); c; c = li.NextComponent())
      if (auto l = ON_Layer::Cast(c)) {
        layers[l->Index()] = uuid(l->Id());
        o << ",{\"id\":" << quote(uuid(l->Id()))
          << ",\"name\":" << quote(utf8(l->Name()))
          << ",\"visible\":" << (l->IsVisible() ? "true" : "false") << '}';
      }
    o << "],\"definitions\":[";
    bool first = true;
    std::set<std::string> members;
    for (auto [id, d] : defs) {
      if (!first)
        o << ',';
      first = false;
      o << "{\"id\":" << quote(id) << ",\"name\":" << quote(utf8(d->Name()))
        << ",\"objects\":[";
      bool f = true;
      if (d->IsLinkedType())
        warn(id, "linked_definition",
             "Using embedded linked-block content only; external files are not "
             "loaded");
      for (int k = 0; k < d->InstanceGeometryIdList().Count(); ++k) {
        auto member = d->InstanceGeometryIdList()[k];
        auto s = uuid(member);
        members.insert(s);
        if (!f)
          o << ',';
        f = false;
        o << object(s);
      }
      o << "]}";
    }
    o << "],\"objects\":[";
    first = true;
    for (auto [id, obj] : objects) {
      if (members.count(id))
        continue;
      if (!first)
        o << ',';
      first = false;
      o << object(id);
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
char *copy(const std::string &s) {
  char *p = new char[s.size() + 1];
  std::memcpy(p, s.c_str(), s.size() + 1);
  return p;
}
} // namespace
// One allocation crosses this boundary. Caller must always release via
// warp3dm_free. `cancel` may be null; a non-zero byte aborts the read.
extern "C" char *warp3dm_read(const char *path, double tolerance,
                              double max_triangles, const unsigned char *cancel,
                              int *ok) noexcept {
  std::lock_guard<std::mutex> lock(gate);
  *ok = 0;
  try {
    init();
    Reader r;
    r.max_triangles = max_triangles;
    r.cancel = cancel;
    auto data = r.read(path, tolerance);
    auto result = copy(data);
    *ok = 1;
    return result;
  } catch (const std::exception &e) {
    try {
      return copy(e.what());
    } catch (...) {
      return nullptr;
    }
  } catch (...) {
    try {
      return copy("Unknown native import failure");
    } catch (...) {
      return nullptr;
    }
  }
}
extern "C" void warp3dm_free(char *p) noexcept { delete[] p; }
extern bool warp3dm_write_fixture_impl(const char *, int, int);
extern "C" int warp3dm_write_fixture(const char *path, int variant,
                                     int version) noexcept {
  std::lock_guard<std::mutex> lock(gate);
  try {
    init();
    return warp3dm_write_fixture_impl(path, variant, version) ? 1 : 0;
  } catch (...) {
    return 0;
  }
}
