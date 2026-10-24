"""WarpNine machining-mesh exporter for Rhino 8 (run inside Rhino: ScriptEditor or RunPythonScript).

Meshes every selected (or, with nothing selected, every) B-rep, extrusion, surface and SubD
object with controlled meshing parameters and records the exporter claims that WarpNine's
importer verifies:

    warpnine.source       UUID of the source object
    warpnine.source_type  brep | extrusion | surface | subd
    warpnine.tolerance    requested maximum deviation, in model units
    warpnine.exporter     exporter name/version
    warpnine.revision     SHA-1 of the source geometry's JSON at export time
    warpnine.face_ranges  "face:firstMeshFace-endMeshFace;..." (B-rep faces -> mesh faces)
    warpnine.mesher_tolerance  the Rhino meshing tolerance that finally met the request
    warpnine.exporter_measured the exporter's own sampled deviation (informational)

The mesh objects are added on the layer "WarpNine machining meshes" and the file still has
to be saved (Rhino 6-8 format). Previous exports for the same source are replaced.
WarpNine measures the mesh against the source on import; the recorded tolerance is a
request, not a measured accuracy.
"""
import hashlib

import Rhino
import rhinoscriptsyntax as rs
import scriptcontext as sc
from Rhino.Geometry import Mesh, MeshingParameters, Point3d, PointFaceRelation

EXPORTER = "warpnine-rhino-export/1"
LAYER = "WarpNine machining meshes"
TOLERANCE_PROMPT = "Maximum mesh deviation (model units)"


def source_type(geometry):
    if isinstance(geometry, Rhino.Geometry.Extrusion):
        return "extrusion"
    if isinstance(geometry, Rhino.Geometry.Brep):
        return "brep"
    if isinstance(geometry, Rhino.Geometry.SubD):
        return "subd"
    if isinstance(geometry, Rhino.Geometry.Surface):
        return "surface"
    return None


def revision_of(geometry):
    text = geometry.ToJSON(Rhino.FileIO.SerializationOptions())
    return hashlib.sha1(text.encode("utf-8")).hexdigest()


def measure_brep(brep, mesh, budget=20000):
    """Exporter-side check in both directions (sampled maxima, like the importer's)."""
    worst = 0.0
    count = mesh.Vertices.Count
    stride = max(1, count // budget)
    for i in range(0, count, stride):
        pt = Point3d(mesh.Vertices[i])
        worst = max(worst, pt.DistanceTo(brep.ClosestPoint(pt)))
    for fi in range(mesh.Faces.Count):
        if fi % stride:
            continue
        pt = mesh.Faces.GetFaceCenter(fi)
        worst = max(worst, pt.DistanceTo(brep.ClosestPoint(pt)))
    n = max(8, min(64, int((budget / max(brep.Faces.Count, 1)) ** 0.5)))
    for face in brep.Faces:
        du, dv = face.Domain(0), face.Domain(1)
        for j in range(n):
            for i in range(n):
                u = du.ParameterAt((i + 0.5) / n)
                v = dv.ParameterAt((j + 0.5) / n)
                if face.IsPointOnFace(u, v) != PointFaceRelation.Interior:
                    continue
                pt = face.PointAt(u, v)
                worst = max(worst, pt.DistanceTo(mesh.ClosestPoint(pt)))
    for edge in brep.Edges:
        params = edge.DivideByCount(128, True)
        if not params:
            continue
        for t in params:
            pt = edge.PointAt(t)
            worst = max(worst, pt.DistanceTo(mesh.ClosestPoint(pt)))
    return worst


def mesh_brep(brep, tolerance):
    """Mesh with a tightening tolerance until the exporter's own measurement meets the request.

    Rhino's Tolerance parameter does not bound the chord error along curved trim edges, so
    a single pass can miss the request by 2-3x; halving the mesher tolerance converges.
    """
    factor = 1.0
    for _attempt in range(6):
        mp = MeshingParameters.Default
        mp.Tolerance = tolerance * factor
        mp.RelativeTolerance = 0.0
        mp.RefineGrid = True
        mp.JaggedSeams = False
        mp.SimplePlanes = False
        mp.MinimumEdgeLength = tolerance * factor * 0.5
        pieces = Mesh.CreateFromBrep(brep, mp)
        if not pieces:
            return None, None, None
        joined = Mesh()
        ranges = []
        start = 0
        for face_index, piece in enumerate(pieces):
            piece.Faces.ConvertQuadsToTriangles()
            joined.Append(piece)
            ranges.append("%d:%d-%d" % (face_index, start, start + piece.Faces.Count))
            start += piece.Faces.Count
        measured = measure_brep(brep, joined)
        if measured <= tolerance:
            return joined, ";".join(ranges), (mp.Tolerance, measured)
        factor *= 0.5
    return joined, ";".join(ranges), (mp.Tolerance, measured)


def mesh_subd(subd, tolerance):
    # SubD density is a subdivision level, not a tolerance; pick the level whose limit
    # polyhedron edge length is below the requested deviation scale, capped at 5.
    bbox = subd.GetBoundingBox(True)
    size = bbox.Diagonal.Length
    density = 1
    while density < 5 and size / (2.0 ** density) / max(subd.Faces.Count, 1) ** 0.5 > tolerance * 20:
        density += 1
    mesh = Mesh.CreateFromSubD(subd, density)
    if mesh:
        mesh.Faces.ConvertQuadsToTriangles()
    return mesh, None, None


def export(obj, tolerance, layer_index):
    geometry = obj.Geometry
    kind = source_type(geometry)
    if kind is None:
        return None
    if kind == "surface":
        geometry = geometry.ToBrep()
    if kind == "extrusion":
        brep = geometry.ToBrep(True)
    elif kind == "brep":
        brep = geometry
    else:
        brep = None
    if kind == "subd":
        mesh, ranges, check = mesh_subd(geometry, tolerance)
    else:
        mesh, ranges, check = mesh_brep(brep, tolerance)
    if mesh is None or mesh.Faces.Count == 0:
        return None
    if check and check[1] > tolerance:
        print("WarpNine: %s still deviates %.4f after refinement; exported anyway, the importer will reject it"
              % (obj.Name or str(obj.Id), check[1]))
    mesh.Weld(3.14159)
    mesh.Normals.ComputeNormals()
    mesh.Compact()
    attributes = Rhino.DocObjects.ObjectAttributes()
    attributes.LayerIndex = layer_index
    attributes.Name = "%s machining mesh" % (obj.Name or str(obj.Id)[:8])
    attributes.SetUserString("warpnine.source", str(obj.Id))
    attributes.SetUserString("warpnine.source_type", kind)
    attributes.SetUserString("warpnine.tolerance", repr(tolerance))
    attributes.SetUserString("warpnine.exporter", EXPORTER)
    attributes.SetUserString("warpnine.revision", revision_of(obj.Geometry))
    if ranges:
        attributes.SetUserString("warpnine.face_ranges", ranges)
    if check:
        attributes.SetUserString("warpnine.mesher_tolerance", repr(check[0]))
        attributes.SetUserString("warpnine.exporter_measured", repr(check[1]))
    return sc.doc.Objects.AddMesh(mesh, attributes)


def remove_previous(source_id):
    for existing in list(sc.doc.Objects):
        if existing.Attributes.GetUserString("warpnine.source") == str(source_id):
            sc.doc.Objects.Delete(existing, True)


def main():
    tolerance = rs.GetReal(TOLERANCE_PROMPT, sc.doc.ModelAbsoluteTolerance, 1e-6, 10.0)
    if tolerance is None:
        return
    ids = rs.GetObjects("Solids, surfaces or SubDs to mesh (Enter for all)", preselect=True)
    objects = [sc.doc.Objects.Find(i) for i in ids] if ids else list(sc.doc.Objects)
    layer_index = sc.doc.Layers.FindByFullPath(LAYER, -1)
    if layer_index < 0:
        layer_index = sc.doc.Layers.Add(LAYER, Rhino.Display.Color4f.Black.AsSystemColor())
    exported = 0
    skipped = []
    for obj in objects:
        if obj is None or obj.Attributes.GetUserString("warpnine.source"):
            continue
        if source_type(obj.Geometry) is None:
            continue
        remove_previous(obj.Id)
        if export(obj, tolerance, layer_index):
            exported += 1
        else:
            skipped.append(obj.Name or str(obj.Id))
    sc.doc.Views.Redraw()
    print("WarpNine: exported %d machining mesh(es) at %g model units; save the file as Rhino 8."
          % (exported, tolerance))
    if skipped:
        print("Skipped (meshing failed): %s" % ", ".join(skipped))


if __name__ == "__main__":
    main()
