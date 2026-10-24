"""Rebuild synthetic M0 fixtures, using only Python's standard library.

These inputs are deliberately independent of the CAM implementation and starter
tool library. Changing fixtures changes the benchmark; review the JSON diff.
"""
import json
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1] / "fixtures" / "cam-baseline"
ROOT.mkdir(parents=True, exist_ok=True)


def write_json(name, value):
    (ROOT / name).write_text(json.dumps(value, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def curve(points, closed=False):
    return {"pts": points, "closed": closed}


def rect(x0, y0, x1, y1):
    return curve([[x0, y0], [x1, y0], [x1, y1], [x0, y1]], True)


def circle(x, y, radius=2):
    return curve([[x + radius * math.cos(i * math.tau / 64),
                   y + radius * math.sin(i * math.tau / 64)] for i in range(64)], True)


cutting = {"spindle_rpm": 10000.0, "feed_rate": 600.0, "plunge_rate": 120.0,
           "ramp_rate": 240.0, "stepdown": 1.0, "stepover": 0.4,
           "coolant": "Off", "clockwise": True}
tools = []
for number, shape, diameter in [(1, "FlatEndMill", 3.0), (2, "BallEndMill", 2.0),
                                 (3, {"Drill": {"point_angle": 118.0}}, 4.0)]:
    tools.append({"number": number, "name": f"M0 tool {number}", "shape": shape,
                  "diameter": diameter, "shank_diameter": diameter, "flute_length": 15.0,
                  "overall_length": 50.0, "stickout": 20.0, "flutes": 2,
                  "material": "Carbide", "coating": "", "manufacturer": "Synthetic",
                  "part_number": "", "holder": None, "length_offset": number,
                  "diameter_offset": number, "cutting": cutting, "notes": "M0 fixture only"})
write_json("tools.json", {"tools": tools, "holders": []})


def project(name, curves, kind, selected, tool=1, mesh=None):
    return {"name": name, "display_units": "Millimeters",
            "stock": {"min": [0.0, 0.0, -5.0], "max": [40.0, 30.0, 0.0]},
            "curves": curves, "mesh": mesh,
            "operations": [{"name": name, "enabled": True, "kind": kind,
                            "common": {"tool": tool,
                                       "heights": {"clearance": 10.0, "retract": 5.0,
                                                   "feed_height": 1.0, "top": 0.0, "bottom": -2.0},
                                       "feeds": {"spindle_rpm": 10000.0, "clockwise": True,
                                                 "feed": 600.0, "plunge": 120.0,
                                                 "ramp": 240.0, "coolant": "Off"},
                                       "stepdown": 1.0, "stock_xy": 0.0, "stock_z": 0.0,
                                       "climb": True, "entry": "Plunge", "tolerance": 0.01,
                                       "curves": selected, "resolution": 0.25}}],
            "post": "Mach3 / Mach4"}


cases = {}
cases["pocket-island"] = project("pocket-island", [rect(2, 2, 38, 28), rect(15, 10, 25, 20)],
    {"Pocket": {"pattern": "Offset", "stepover": 1.0, "finish_pass": True,
                "inside_out": True, "order": "LayerFirst"}}, [0, 1])
open_curve = curve([[5, 5], [20, 5], [20, 15]])
cases["open-profile"] = project("open-profile", [open_curve],
    {"Profile": {"side": "On", "finish_passes": 0, "finish_stepover": 0.2,
                 "spring_pass": False, "lead_radius": 0.0, "tabs": None}}, [0])
cases["drill-pattern"] = project("drill-pattern", [circle(x, y) for x, y in [(8, 8), (32, 8), (8, 22), (32, 22)]],
    {"Drill": {"cycle": "Peck", "peck": 1.0, "dwell": 0.0,
               "source": {"CircleCenters": {"min_diameter": 3.9, "max_diameter": 4.1}},
               "tip_compensation": False, "breakthrough": 0.0}}, [0, 1, 2, 3], tool=3)

# Analytic paraboloid, sampled at 2 mm. Tessellation is the input, not an
# accuracy-certified approximation of an arbitrary CAD surface.
def vertex(i, j):
    x, y = i * 2.0, j * 2.0
    return [x, y, -0.002 * ((x - 20.0) ** 2 + (y - 15.0) ** 2)]


tris = []
for j in range(15):
    for i in range(20):
        a, b, c, d = vertex(i, j), vertex(i + 1, j), vertex(i + 1, j + 1), vertex(i, j + 1)
        tris.extend([[a, b, c], [a, c, d]])
mesh = {"tris": tris}
cases["sculpted-parallel"] = project("sculpted-parallel", [],
    {"Parallel": {"stepover": 1.0, "angle": 0.0, "zigzag": True}}, [], tool=2, mesh=mesh)
cases["projected-open-curve"] = project("projected-open-curve", [open_curve],
    {"ProjectCurves": {"depth": 0.0}}, [0], tool=2, mesh=mesh)
for name, value in cases.items():
    write_json(name + ".warp", value)

# ASCII DXF with exact rectangles, an open polyline and true circle entities.
tags = [(0, "SECTION"), (2, "HEADER"), (9, "$ACADVER"), (1, "AC1015"),
        (9, "$INSUNITS"), (70, 4), (0, "ENDSEC"), (0, "SECTION"), (2, "ENTITIES")]
for c in [rect(2, 2, 38, 28), rect(15, 10, 25, 20), open_curve]:
    tags += [(0, "LWPOLYLINE"), (100, "AcDbEntity"), (8, "0"),
             (100, "AcDbPolyline"), (90, len(c["pts"])), (70, int(c["closed"]))]
    for x, y in c["pts"]:
        tags += [(10, x), (20, y)]
for x, y in [(8, 8), (32, 8), (8, 22), (32, 22)]:
    tags += [(0, "CIRCLE"), (100, "AcDbEntity"), (8, "0"), (100, "AcDbCircle"),
             (10, x), (20, y), (30, 0), (40, 2)]
tags += [(0, "ENDSEC"), (0, "EOF")]
(ROOT / "planar-mm.dxf").write_text("".join(f"{code}\n{value}\n" for code, value in tags), encoding="ascii")

lines = ["solid m0_paraboloid"]
for tri in tris:
    a, b, c = tri
    u, v = [b[k] - a[k] for k in range(3)], [c[k] - a[k] for k in range(3)]
    n = [u[1]*v[2]-u[2]*v[1], u[2]*v[0]-u[0]*v[2], u[0]*v[1]-u[1]*v[0]]
    length = math.sqrt(sum(k*k for k in n))
    lines += ["facet normal " + " ".join(str(k/length) for k in n), "outer loop"]
    lines += ["vertex " + " ".join(map(str, p)) for p in tri]
    lines += ["endloop", "endfacet"]
lines.append("endsolid m0_paraboloid")
(ROOT / "sculpted-mm.stl").write_text("\n".join(lines) + "\n", encoding="ascii")
print(f"Wrote {len(cases)} projects, tools, DXF and STL to {ROOT}")
