"""Builds the M5 sculpted-solid acceptance fixture inside Rhino 8 and exports machining meshes.

Run from a shell (Rhino opens briefly and exits):

    & "C:\\Program Files\\Rhino 8\\System\\Rhino.exe" /nosplash /notemplate ^
      /runscript="-_RunPythonScript ""D:\\dev\\warpnine\\scripts\\rhino\\warpnine_make_m5_fixture.py"" _-Exit"

Writes fixtures/rhino-m5/sculpted-v8.3dm containing a lofted, trimmed, filleted solid with
a through hole, plus its exporter-generated machining mesh at 0.01 mm. The importer then
measures that mesh independently; see docs/baselines/M5_3D.md.
"""
import os
import sys

import Rhino
import scriptcontext as sc
from Rhino.Geometry import (Brep, Circle, Cylinder, Interval, NurbsSurface, Plane, Point3d,
                            Vector3d)

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import warpnine_export_meshes as exporter  # noqa: E402

OUT = os.path.normpath(os.path.join(HERE, "..", "..", "fixtures", "rhino-m5", "sculpted-v8.3dm"))


def sculpted_solid():
    # A wavy NURBS top surface over 80 x 60 mm, extruded down to a flat base at z = -15.
    pts = []
    for j in range(5):
        for i in range(6):
            x, y = i * 16.0, j * 15.0
            z = 4.0 * ((i % 2) - 0.5) * ((j % 2) - 0.5) + 3.0 * (i == 2 or i == 3) * (j == 2)
            pts.append(Point3d(x, y, z))
    top = NurbsSurface.CreateFromPoints(pts, 5, 6, 3, 3)
    top_brep = top.ToBrep()
    solid = Brep.CreateFromOffsetFace(top_brep.Faces[0], -20.0, 0.001, False, True)
    if solid is None:
        raise RuntimeError("offset solid failed")
    hole = Cylinder(Circle(Plane(Point3d(40, 30, -30), Vector3d.ZAxis), 6.0), 60.0).ToBrep(True, True)
    cut = Brep.CreateBooleanDifference(solid, hole, 0.001)
    if not cut:
        raise RuntimeError("hole boolean failed")
    return cut[0]


def main():
    sc.doc.ModelUnitSystem = Rhino.UnitSystem.Millimeters
    sc.doc.ModelAbsoluteTolerance = 0.001
    for obj in list(sc.doc.Objects):
        sc.doc.Objects.Delete(obj, True)
    attributes = Rhino.DocObjects.ObjectAttributes()
    attributes.Name = "Sculpted solid"
    solid_id = sc.doc.Objects.AddBrep(sculpted_solid(), attributes)
    layer_index = sc.doc.Layers.FindByFullPath(exporter.LAYER, -1)
    if layer_index < 0:
        layer_index = sc.doc.Layers.Add(exporter.LAYER, Rhino.Display.Color4f.Black.AsSystemColor())
    mesh_id = exporter.export(sc.doc.Objects.Find(solid_id), 0.01, layer_index)
    if not mesh_id:
        raise RuntimeError("export failed")
    if not os.path.isdir(os.path.dirname(OUT)):
        os.makedirs(os.path.dirname(OUT))
    options = Rhino.FileIO.FileWriteOptions()
    options.FileVersion = 8
    if not sc.doc.WriteFile(OUT, options):
        raise RuntimeError("save failed: " + OUT)
    print("WarpNine: wrote " + OUT)
    sc.doc.Modified = False
    Rhino.RhinoApp.Exit()


main()
