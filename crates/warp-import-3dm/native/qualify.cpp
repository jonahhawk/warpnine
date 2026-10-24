// Measures an explicit machining mesh against the source solid it claims to represent.
//
// Source -> mesh: trimmed surface samples and edge samples are projected onto the mesh
// through a uniform grid of triangles. Mesh -> source: mesh vertices and centroids are
// projected onto the closest surfaces by Newton iteration seeded from a grid of surface
// points; a projection that lands outside the face trims (a filled hole, overshoot past
// an edge) falls back to the distance to the 3D trim edges and is counted separately.
// Both directions are sampled, so the results are measured maxima, not proven bounds.
#include "qualify.h"
#undef min
#undef max
#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <memory>
#include <stdexcept>

namespace {
constexpr size_t SOURCE_BUDGET = 40000;
constexpr size_t MESH_BUDGET = 20000;
constexpr int TRIM_SAMPLES = 96;
constexpr int EDGE_SAMPLES = 256;
constexpr int SEEDS_PER_SIDE = 12;
constexpr int SEED_CANDIDATES = 4;

void check(const volatile unsigned char *cancel) {
  if (cancel && *cancel)
    throw std::runtime_error("Import cancelled");
}

ON_3dPoint closest_on_segment(const ON_3dPoint &a, const ON_3dPoint &b,
                              const ON_3dPoint &p) {
  ON_3dVector ab = b - a;
  double len2 = ab.LengthSquared();
  if (len2 <= 0)
    return a;
  double t = std::clamp(((p - a) * ab) / len2, 0.0, 1.0);
  return a + t * ab;
}

// Ericson, Real-Time Collision Detection, 5.1.5.
ON_3dPoint closest_on_tri(const QTri &t, const ON_3dPoint &p) {
  ON_3dVector ab = t.b - t.a, ac = t.c - t.a, ap = p - t.a;
  double d1 = ab * ap, d2 = ac * ap;
  if (d1 <= 0 && d2 <= 0)
    return t.a;
  ON_3dVector bp = p - t.b;
  double d3 = ab * bp, d4 = ac * bp;
  if (d3 >= 0 && d4 <= d3)
    return t.b;
  double vc = d1 * d4 - d3 * d2;
  if (vc <= 0 && d1 >= 0 && d3 <= 0)
    return t.a + (d1 / (d1 - d3)) * ab;
  ON_3dVector cp = p - t.c;
  double d5 = ab * cp, d6 = ac * cp;
  if (d6 >= 0 && d5 <= d6)
    return t.c;
  double vb = d5 * d2 - d1 * d6;
  if (vb <= 0 && d2 >= 0 && d6 <= 0)
    return t.a + (d2 / (d2 - d6)) * ac;
  double va = d3 * d6 - d5 * d4;
  if (va <= 0 && (d4 - d3) >= 0 && (d5 - d6) >= 0)
    return t.b + ((d4 - d3) / ((d4 - d3) + (d5 - d6))) * (t.c - t.b);
  double sum = va + vb + vc;
  if (!(std::abs(sum) > 1e-300)) {
    // Degenerate (collinear) triangle: closest point on its edges.
    ON_3dPoint best = closest_on_segment(t.a, t.b, p);
    for (auto q : {closest_on_segment(t.b, t.c, p), closest_on_segment(t.c, t.a, p)})
      if (q.DistanceTo(p) < best.DistanceTo(p))
        best = q;
    return best;
  }
  double denom = 1.0 / sum;
  return t.a + ab * (vb * denom) + ac * (vc * denom);
}

// Visit every cell on the Chebyshev shell at distance r around c, inside [0,n).
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

struct GridBase {
  ON_3dPoint lo;
  double cell = 1;
  int n[3] = {1, 1, 1};
  std::vector<std::vector<int>> cells;
  void frame(const ON_BoundingBox &bb, size_t items) {
    ON_3dVector size = bb.IsValid() ? bb.Diagonal() : ON_3dVector(1, 1, 1);
    double maxd = std::max({size.x, size.y, size.z, 1e-9});
    double target = std::cbrt(std::max<double>(1.0, items / 2.0));
    cell = maxd / std::clamp(target, 1.0, 96.0);
    for (int k = 0; k < 3; ++k)
      n[k] = std::clamp(int(size[k] / cell) + 1, 1, 256);
    lo = bb.IsValid() ? bb.m_min : ON_3dPoint::Origin;
    cells.assign(size_t(n[0]) * n[1] * n[2], {});
  }
  int index(int i, int j, int k) const { return (k * n[1] + j) * n[0] + i; }
  void coords(const ON_3dPoint &p, int c[3]) const {
    for (int k = 0; k < 3; ++k)
      c[k] = std::clamp(int(std::floor((p[k] - lo[k]) / cell)), 0, n[k] - 1);
  }
  int max_ring(const int c[3]) const {
    int r = 0;
    for (int k = 0; k < 3; ++k)
      r = std::max({r, c[k], n[k] - 1 - c[k]});
    return r;
  }
};

struct TriGrid : GridBase {
  const std::vector<QTri> *tris = nullptr;
  void build(const std::vector<QTri> &t) {
    tris = &t;
    ON_BoundingBox bb;
    for (auto &tri : t) {
      bb.Set(tri.a, true);
      bb.Set(tri.b, true);
      bb.Set(tri.c, true);
    }
    frame(bb, t.size());
    for (size_t i = 0; i < t.size(); ++i) {
      ON_BoundingBox b;
      b.Set(t[i].a, true);
      b.Set(t[i].b, true);
      b.Set(t[i].c, true);
      int c0[3], c1[3];
      coords(b.m_min, c0);
      coords(b.m_max, c1);
      for (int k = c0[2]; k <= c1[2]; ++k)
        for (int j = c0[1]; j <= c1[1]; ++j)
          for (int ii = c0[0]; ii <= c1[0]; ++ii)
            cells[index(ii, j, k)].push_back(int(i));
    }
  }
  double distance(const ON_3dPoint &p) const {
    int c[3];
    coords(p, c);
    double best2 = std::numeric_limits<double>::infinity();
    const int rmax = max_ring(c);
    for (int r = 0; r <= rmax; ++r) {
      if (r > 1) {
        double lb = (r - 1) * cell;
        if (lb * lb >= best2)
          break;
      }
      visit_shell(c, n, r, [&](int i, int j, int k) {
        for (int ti : cells[index(i, j, k)]) {
          double d2 = closest_on_tri((*tris)[ti], p).DistanceTo(p);
          d2 *= d2;
          if (d2 < best2)
            best2 = d2;
        }
      });
    }
    return std::sqrt(best2);
  }
};

struct Seed {
  int face;
  double u, v;
  ON_3dPoint p;
};

struct SeedGrid : GridBase {
  std::vector<Seed> seeds;
  void build(std::vector<Seed> s) {
    seeds = std::move(s);
    ON_BoundingBox bb;
    for (auto &x : seeds)
      bb.Set(x.p, true);
    frame(bb, seeds.size());
    for (size_t i = 0; i < seeds.size(); ++i) {
      int c[3];
      coords(seeds[i].p, c);
      cells[index(c[0], c[1], c[2])].push_back(int(i));
    }
  }
  // The best seed on each of the `count` nearest faces, closest first. Seeds are dense,
  // so the nearest few seeds usually share one face; the true closest face can be a
  // neighbour whose seeds are slightly farther, and every candidate face is projected.
  std::vector<const Seed *> nearest(const ON_3dPoint &p, size_t count) const {
    std::map<int, std::pair<double, const Seed *>> best; // face -> (d2, seed)
    int c[3];
    coords(p, c);
    const int rmax = max_ring(c);
    for (int r = 0; r <= rmax; ++r) {
      if (r > 1 && best.size() >= count) {
        std::vector<double> d2s;
        for (auto &b : best)
          d2s.push_back(b.second.first);
        std::sort(d2s.begin(), d2s.end());
        double lb = (r - 1) * cell;
        if (lb * lb >= d2s[count - 1])
          break;
      }
      visit_shell(c, n, r, [&](int i, int j, int k) {
        for (int si : cells[index(i, j, k)]) {
          double d = seeds[si].p.DistanceTo(p);
          auto it = best.find(seeds[si].face);
          if (it == best.end() || d * d < it->second.first)
            best[seeds[si].face] = {d * d, &seeds[si]};
        }
      });
    }
    std::vector<std::pair<double, const Seed *>> found;
    for (auto &b : best)
      found.push_back(b.second);
    std::sort(found.begin(), found.end());
    std::vector<const Seed *> out;
    for (size_t i = 0; i < found.size() && out.size() < count; ++i)
      out.push_back(found[i].second);
    return out;
  }
};

struct Poly {
  std::vector<ON_2dPoint> pts;
};

bool point_in_poly(const Poly &poly, ON_2dPoint q) {
  bool in = false;
  const auto &v = poly.pts;
  for (size_t i = 0, j = v.size() - 1; i < v.size(); j = i++) {
    if ((v[i].y > q.y) != (v[j].y > q.y)) {
      double x = v[j].x + (q.y - v[j].y) * (v[i].x - v[j].x) / (v[i].y - v[j].y);
      if (q.x < x)
        in = !in;
    }
  }
  return in;
}

double dist_to_poly(const Poly &poly, ON_2dPoint q) {
  double best = std::numeric_limits<double>::infinity();
  const auto &v = poly.pts;
  for (size_t i = 0, j = v.size() - 1; i < v.size(); j = i++) {
    ON_2dVector ab = v[i] - v[j];
    double len2 = ab.LengthSquared();
    double t = len2 > 0 ? std::clamp(((q - v[j]) * ab) / len2, 0.0, 1.0) : 0.0;
    ON_2dPoint c = v[j] + t * ab;
    best = std::min(best, c.DistanceTo(q));
  }
  return best;
}

struct FaceInfo {
  const ON_Surface *srf = nullptr;
  ON_Interval du, dv;
  std::vector<Poly> outer, inner;
  double band = 0; // uv tolerance absorbing the trim polygon chord error
  // `lenient` widens the trims by `band`; used when classifying mesh projections so a
  // point on a trim edge is not mistaken for a point outside the face. Source sampling
  // is strict so no sample lies outside the true face.
  bool inside(ON_2dPoint uv, bool lenient = true) const {
    const double band = lenient ? this->band : 0.0;
    if (uv.x < du.Min() - band || uv.x > du.Max() + band || uv.y < dv.Min() - band ||
        uv.y > dv.Max() + band)
      return false;
    if (outer.empty())
      return true;
    bool in = false;
    for (auto &o : outer)
      if (point_in_poly(o, uv) || dist_to_poly(o, uv) <= band) {
        in = true;
        break;
      }
    if (!in)
      return false;
    for (auto &h : inner)
      if (point_in_poly(h, uv) && dist_to_poly(h, uv) > band)
        return false;
    return true;
  }
};

FaceInfo face_info(const ON_Brep &brep, const ON_BrepFace &face) {
  FaceInfo f;
  f.srf = face.SurfaceOf();
  if (!f.srf)
    throw std::runtime_error("B-rep face has no surface");
  f.du = f.srf->Domain(0);
  f.dv = f.srf->Domain(1);
  double chord = 0;
  for (int li = 0; li < face.m_li.Count(); ++li) {
    const ON_BrepLoop &loop = brep.m_L[face.m_li[li]];
    if (loop.m_type != ON_BrepLoop::outer && loop.m_type != ON_BrepLoop::inner)
      continue;
    Poly poly;
    for (int k = 0; k < loop.m_ti.Count(); ++k) {
      const ON_BrepTrim &trim = brep.m_T[loop.m_ti[k]];
      ON_Interval d = trim.Domain();
      for (int s = 0; s < TRIM_SAMPLES; ++s) {
        ON_3dPoint a = trim.PointAt(d.ParameterAt(double(s) / TRIM_SAMPLES));
        ON_3dPoint b = trim.PointAt(d.ParameterAt(double(s + 1) / TRIM_SAMPLES));
        ON_3dPoint m = trim.PointAt(d.ParameterAt((s + 0.5) / TRIM_SAMPLES));
        chord = std::max(chord, closest_on_segment(a, b, m).DistanceTo(m));
        poly.pts.emplace_back(a.x, a.y);
      }
    }
    if (poly.pts.size() >= 3)
      (loop.m_type == ON_BrepLoop::outer ? f.outer : f.inner).push_back(std::move(poly));
  }
  // Chord error of the sampled trims, plus a sliver of the domain for roundoff.
  f.band = 2.0 * chord + 1e-6 * std::max(f.du.Length(), f.dv.Length());
  return f;
}

// Newton descent on |S(u,v) - p|^2 from (u, v). Returns the distance and final (u, v).
double project_to_surface(const FaceInfo &f, const ON_3dPoint &p, double &u, double &v) {
  ON_3dPoint S;
  ON_3dVector Su, Sv, Suu, Suv, Svv;
  double lu = f.du.Length(), lv = f.dv.Length();
  if (!f.srf->Ev2Der(u, v, S, Su, Sv, Suu, Suv, Svv))
    return std::numeric_limits<double>::infinity();
  double dist = S.DistanceTo(p);
  for (int it = 0; it < 20; ++it) {
    ON_3dVector d = S - p;
    double g0 = Su * d, g1 = Sv * d;
    double h00 = Su * Su + Suu * d, h01 = Su * Sv + Suv * d, h11 = Sv * Sv + Svv * d;
    double det = h00 * h11 - h01 * h01;
    double du, dv;
    if (det > 1e-18 && h00 > 0) {
      du = -(h11 * g0 - h01 * g1) / det;
      dv = -(-h01 * g0 + h00 * g1) / det;
    } else {
      double su2 = Su * Su, sv2 = Sv * Sv;
      du = su2 > 1e-18 ? -g0 / su2 : 0;
      dv = sv2 > 1e-18 ? -g1 / sv2 : 0;
    }
    du = std::clamp(du, -0.25 * lu, 0.25 * lu);
    dv = std::clamp(dv, -0.25 * lv, 0.25 * lv);
    if (std::abs(du) <= 1e-12 * lu && std::abs(dv) <= 1e-12 * lv)
      break;
    // Backtrack until the step improves the distance.
    bool improved = false;
    for (int bt = 0; bt < 6; ++bt) {
      double nu = std::clamp(u + du, f.du.Min(), f.du.Max());
      double nv = std::clamp(v + dv, f.dv.Min(), f.dv.Max());
      ON_3dPoint S2;
      ON_3dVector a, b, c, e, g;
      if (f.srf->Ev2Der(nu, nv, S2, a, b, c, e, g) && S2.DistanceTo(p) < dist) {
        u = nu;
        v = nv;
        S = S2;
        Su = a;
        Sv = b;
        Suu = c;
        Suv = e;
        Svv = g;
        dist = S.DistanceTo(p);
        improved = true;
        break;
      }
      du *= 0.5;
      dv *= 0.5;
    }
    if (!improved)
      break;
  }
  return dist;
}

QualifyResult qualify_brep(const ON_Brep &brep, const std::vector<QTri> &tris,
                           const volatile unsigned char *cancel) {
  QualifyResult r;
  const int F = brep.m_F.Count();
  if (F == 0) {
    r.note = "source has no faces";
    return r;
  }
  std::vector<FaceInfo> faces;
  faces.reserve(F);
  for (int fi = 0; fi < F; ++fi)
    faces.push_back(face_info(brep, brep.m_F[fi]));
  std::vector<std::vector<ON_3dPoint>> edges;
  for (int ei = 0; ei < brep.m_E.Count(); ++ei) {
    const ON_BrepEdge &e = brep.m_E[ei];
    ON_Interval d = e.Domain();
    std::vector<ON_3dPoint> pl;
    for (int s = 0; s <= EDGE_SAMPLES; ++s)
      pl.push_back(e.PointAt(d.ParameterAt(double(s) / EDGE_SAMPLES)));
    edges.push_back(std::move(pl));
  }
  auto edge_distance = [&](const ON_3dPoint &p) {
    double best = std::numeric_limits<double>::infinity();
    for (auto &pl : edges)
      for (size_t i = 1; i < pl.size(); ++i)
        best = std::min(best, closest_on_segment(pl[i - 1], pl[i], p).DistanceTo(p));
    return best;
  };
  TriGrid grid;
  grid.build(tris);
  r.per_face.assign(F, 0.0);

  // Source -> mesh.
  const int n = std::clamp(int(std::lround(std::sqrt(double(SOURCE_BUDGET) / F))), 8, 128);
  for (int fi = 0; fi < F; ++fi) {
    const FaceInfo &f = faces[fi];
    for (int j = 0; j < n; ++j) {
      check(cancel);
      for (int i = 0; i < n; ++i) {
        ON_2dPoint uv(f.du.ParameterAt((i + 0.5) / n), f.dv.ParameterAt((j + 0.5) / n));
        if (!f.inside(uv, false))
          continue;
        ON_3dPoint P;
        if (!f.srf->EvPoint(uv.x, uv.y, P))
          continue;
        double d = grid.distance(P);
        r.source_to_mesh = std::max(r.source_to_mesh, d);
        r.per_face[fi] = std::max(r.per_face[fi], d);
        ++r.source_samples;
      }
    }
  }
  for (auto &pl : edges) {
    check(cancel);
    for (auto &p : pl) {
      r.source_to_mesh = std::max(r.source_to_mesh, grid.distance(p));
      ++r.source_samples;
    }
  }

  // Seeds for mesh -> source projection.
  std::vector<Seed> seeds;
  for (int fi = 0; fi < F; ++fi) {
    const FaceInfo &f = faces[fi];
    for (int j = 0; j < SEEDS_PER_SIDE; ++j)
      for (int i = 0; i < SEEDS_PER_SIDE; ++i) {
        double u = f.du.ParameterAt((i + 0.5) / SEEDS_PER_SIDE);
        double v = f.dv.ParameterAt((j + 0.5) / SEEDS_PER_SIDE);
        ON_3dPoint P;
        if (f.srf->EvPoint(u, v, P))
          seeds.push_back({fi, u, v, P});
      }
  }
  if (seeds.empty()) {
    r.note = "source surfaces could not be evaluated";
    return r;
  }
  SeedGrid sg;
  sg.build(std::move(seeds));

  // Mesh -> source: vertices and centroids, strided to the budget.
  const size_t total = tris.size() * 4;
  const size_t stride = std::max<size_t>(1, (total + MESH_BUDGET - 1) / MESH_BUDGET);
  for (size_t s = 0; s < total; s += stride) {
    if (s % 1024 == 0)
      check(cancel);
    const QTri &t = tris[s / 4];
    ON_3dPoint p;
    switch (s % 4) {
    case 0:
      p = t.a;
      break;
    case 1:
      p = t.b;
      break;
    case 2:
      p = t.c;
      break;
    default:
      p = ON_3dPoint((t.a.x + t.b.x + t.c.x) / 3, (t.a.y + t.b.y + t.c.y) / 3,
                     (t.a.z + t.b.z + t.c.z) / 3);
    }
    double best = std::numeric_limits<double>::infinity();
    int best_face = -1;
    for (const Seed *seed : sg.nearest(p, SEED_CANDIDATES)) {
      double u = seed->u, v = seed->v;
      const FaceInfo &f = faces[seed->face];
      double d = project_to_surface(f, p, u, v);
      if (d < best && f.inside(ON_2dPoint(u, v))) {
        best = d;
        best_face = seed->face;
      }
    }
    if (best_face < 0) {
      best = edge_distance(p);
      ++r.outside_trim_samples;
    } else {
      r.per_face[best_face] = std::max(r.per_face[best_face], best);
    }
    r.mesh_to_source = std::max(r.mesh_to_source, best);
    ++r.mesh_samples;
  }
  r.measured = true;
  return r;
}

QualifyResult qualify_subd(const ON_SubD &subd, const std::vector<QTri> &tris,
                           const volatile unsigned char *cancel) {
  QualifyResult r;
  unsigned levels = 3;
  while (levels > 0 && double(subd.FaceCount()) * std::pow(4.0, levels) > 2e6)
    --levels;
  std::vector<QTri> ref;
  if (!subd_limit_triangles(subd, levels, ref)) {
    r.note = "SubD could not be subdivided for measurement";
    return r;
  }
  TriGrid mesh_grid, ref_grid;
  mesh_grid.build(tris);
  ref_grid.build(ref);
  const size_t stride_s = std::max<size_t>(1, (ref.size() * 3 + SOURCE_BUDGET - 1) / SOURCE_BUDGET);
  for (size_t s = 0; s < ref.size() * 3; s += stride_s) {
    if (s % 1024 == 0)
      check(cancel);
    const QTri &t = ref[s / 3];
    const ON_3dPoint &p = s % 3 == 0 ? t.a : (s % 3 == 1 ? t.b : t.c);
    r.source_to_mesh = std::max(r.source_to_mesh, mesh_grid.distance(p));
    ++r.source_samples;
  }
  const size_t total = tris.size() * 4;
  const size_t stride = std::max<size_t>(1, (total + MESH_BUDGET - 1) / MESH_BUDGET);
  for (size_t s = 0; s < total; s += stride) {
    if (s % 1024 == 0)
      check(cancel);
    const QTri &t = tris[s / 4];
    ON_3dPoint p = s % 4 == 0   ? t.a
                   : s % 4 == 1 ? t.b
                   : s % 4 == 2 ? t.c
                                : ON_3dPoint((t.a.x + t.b.x + t.c.x) / 3, (t.a.y + t.b.y + t.c.y) / 3,
                                             (t.a.z + t.b.z + t.c.z) / 3);
    r.mesh_to_source = std::max(r.mesh_to_source, ref_grid.distance(p));
    ++r.mesh_samples;
  }
  r.measured = true;
  r.note = "SubD reference is the level-" + std::to_string(levels) +
           " subdivided limit polyhedron; its own deviation from the limit surface is not bounded";
  return r;
}
} // namespace

bool subd_limit_triangles(const ON_SubD &subd, unsigned levels, std::vector<QTri> &out) {
  std::unique_ptr<ON_SubD> copy(ON_SubD::Cast(subd.Duplicate()));
  if (!copy)
    return false;
  if (levels > 0 && !copy->GlobalSubdivide(levels))
    return false;
  ON_SubDFaceIterator fit = copy->FaceIterator();
  for (const ON_SubDFace *f = fit.FirstFace(); f; f = fit.NextFace()) {
    unsigned n = f->EdgeCount();
    if (n < 3)
      continue;
    std::vector<ON_3dPoint> pts(n);
    ON_3dPoint c(0, 0, 0);
    for (unsigned i = 0; i < n; ++i) {
      const ON_SubDVertex *v = f->Vertex(i);
      if (!v)
        return false;
      pts[i] = v->SurfacePoint();
      if (!pts[i].IsValid())
        return false;
      c += pts[i];
    }
    c /= double(n);
    if (n == 3)
      out.push_back({pts[0], pts[1], pts[2]});
    else if (n == 4) {
      out.push_back({pts[0], pts[1], pts[2]});
      out.push_back({pts[0], pts[2], pts[3]});
    } else
      for (unsigned i = 0; i < n; ++i)
        out.push_back({c, pts[i], pts[(i + 1) % n]});
  }
  return !out.empty();
}

QualifyResult qualify_mesh(const ON_Geometry *source, const std::vector<QTri> &tris,
                           const volatile unsigned char *cancel) {
  if (tris.empty()) {
    QualifyResult r;
    r.note = "mesh has no triangles";
    return r;
  }
  if (auto b = ON_Brep::Cast(source))
    return qualify_brep(*b, tris, cancel);
  if (auto e = ON_Extrusion::Cast(source)) {
    std::unique_ptr<ON_Brep> b(e->BrepForm(nullptr));
    if (!b) {
      QualifyResult r;
      r.note = "extrusion has no B-rep form";
      return r;
    }
    return qualify_brep(*b, tris, cancel);
  }
  if (auto s = ON_Surface::Cast(source)) {
    std::unique_ptr<ON_Brep> b(s->BrepForm(nullptr));
    if (!b) {
      QualifyResult r;
      r.note = "surface has no B-rep form";
      return r;
    }
    return qualify_brep(*b, tris, cancel);
  }
  if (auto sd = ON_SubD::Cast(source))
    return qualify_subd(*sd, tris, cancel);
  QualifyResult r;
  r.note = "source type cannot be measured";
  return r;
}
