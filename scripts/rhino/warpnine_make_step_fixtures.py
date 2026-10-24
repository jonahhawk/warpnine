"""Exports the M5 sculpted-solid fixture to STEP from Rhino 8 (solid and its top face).

Run from a shell with the 3dm fixture as the document (Rhino opens briefly and exits):

    & "C:\\Program Files\\Rhino 8\\System\\Rhino.exe" "D:\\dev\\warpnine\\fixtures\\rhino-m5\\sculpted-v8.3dm" ^
      /nosplash /runscript="-_RunPythonScript ""D:\\dev\\warpnine\\scripts\\rhino\\warpnine_make_step_fixtures.py"" _-Exit"

Writes fixtures/rhino-m5/sculpted-v8.step (the named solid, AP214) and
fixtures/rhino-m5/sculpted-top-surface.step (the wavy top face alone, an open surface).
WarpNine meshes and measures these itself on import; see docs/baselines/M5_3D.md.
"""
import os

import Rhino
import rhinoscriptsyntax as rs
import scriptcontext as sc

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.normpath(os.path.join(HERE, "..", "..", "fixtures", "rhino-m5"))


def export_selected(path):
    if os.path.exists(path):
        os.remove(path)
    ok = rs.Command('-_Export "%s" _Enter' % path, False)
    if not ok or not os.path.exists(path):
        raise RuntimeError("STEP export failed: " + path)
    print("WarpNine: wrote " + path)


def main():
    solids = [o for o in sc.doc.Objects if isinstance(o.Geometry, Rhino.Geometry.Brep) and o.Name == "Sculpted solid"]
    if len(solids) != 1:
        raise RuntimeError("expected one 'Sculpted solid' brep, found %d" % len(solids))
    solid = solids[0]
    rs.UnselectAllObjects()
    rs.SelectObject(solid.Id)
    export_selected(os.path.join(OUT, "sculpted-v8.step"))

    brep = solid.Geometry
    top = max(brep.Faces, key=lambda f: f.GetBoundingBox(True).Center.Z)
    face_brep = top.DuplicateFace(False)
    attributes = Rhino.DocObjects.ObjectAttributes()
    attributes.Name = "Sculpted top"
    face_id = sc.doc.Objects.AddBrep(face_brep, attributes)
    rs.UnselectAllObjects()
    rs.SelectObject(face_id)
    export_selected(os.path.join(OUT, "sculpted-top-surface.step"))
    sc.doc.Objects.Delete(face_id, True)
    sc.doc.Modified = False
    Rhino.RhinoApp.Exit()


main()
