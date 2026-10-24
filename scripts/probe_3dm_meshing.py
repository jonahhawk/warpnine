"""I05 analytic meshing experiment; not a general B-rep mesher.
Run: python scripts/probe_3dm_meshing.py
"""
import json, math, platform, time
from collections import Counter
from pathlib import Path

def sphere(n):
    verts=[(0.,0.,10.)]
    for j in range(1,n//2):
        p=math.pi*j/(n//2)
        verts += [(10*math.sin(p)*math.cos(2*math.pi*i/n),10*math.sin(p)*math.sin(2*math.pi*i/n),10*math.cos(p)) for i in range(n)]
    end=len(verts);verts.append((0.,0.,-10.))
    faces=[]
    for i in range(n):faces.append((0,1+i,1+(i+1)%n))
    for j in range(n//2-2):
        for i in range(n):
            a=1+j*n+i;b=1+j*n+(i+1)%n;c=a+n;d=b+n
            faces += [(a,c,b),(b,c,d)]
    for i in range(n):faces.append((end,end-n+(i+1)%n,end-n+i))
    return verts,faces

def annulus(n,extrude):
    verts=[(r*math.cos(2*math.pi*i/n),r*math.sin(2*math.pi*i/n),z) for z in ([0.,5.] if extrude else [0.]) for r in [3.,10.] for i in range(n)]
    faces=[]
    def quad(a,b,c,d):faces.extend([(a,b,c),(a,c,d)])
    for i in range(n):
        j=(i+1)%n;quad(i,j,n+j,n+i)
        if extrude:
            quad(2*n+i,3*n+i,3*n+j,2*n+j)
            quad(i,2*n+i,2*n+j,j)
            quad(n+i,n+j,3*n+j,3*n+i)
    return verts,faces

def measure(kind,n):
    start=time.perf_counter()
    verts,faces=sphere(n) if kind=='sphere_seam' else annulus(n,kind=='extruded_annulus')
    elapsed=(time.perf_counter()-start)*1000
    edges=Counter(tuple(sorted(e)) for f in faces for e in [(f[0],f[1]),(f[1],f[2]),(f[2],f[0])])
    error=0.
    if kind=='sphere_seam':
        for face in faces:
            p=[sum(verts[v][a] for v in face)/3 for a in range(3)]
            error=max(error,10-math.sqrt(sum(x*x for x in p)))
    else:error=10*(1-math.cos(math.pi/n))
    return dict(shape=kind,segments=n,vertices=len(verts),triangles=len(faces),build_ms=round(elapsed,3),boundary_edges=sum(v==1 for v in edges.values()),nonmanifold_edges=sum(v>2 for v in edges.values()),watertight=all(v==2 for v in edges.values()),sampled_surface_or_exact_boundary_error_mm=error,hole_boundary_intrusion_mm=0 if kind=='sphere_seam' else 3*(1-math.cos(math.pi/n)))

report={'purpose':'Analytic-only candidates for I05; no arbitrary B-rep conversion or machining qualification','environment':platform.platform(),'target_tolerance_mm':0.01,'results':[measure(k,n) for n in [16,128] for k in ['sphere_seam','trimmed_annulus','extruded_annulus']]}
for row in report['results']:
    if row['shape']!='trimmed_annulus':assert row['watertight']
    assert row['nonmanifold_edges']==0
    if row['segments']==128:assert row['sampled_surface_or_exact_boundary_error_mm']<0.01
path=Path(__file__).resolve().parents[1]/'docs/decisions/3dm-meshing-measurements.json'
path.write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report,indent=2))
