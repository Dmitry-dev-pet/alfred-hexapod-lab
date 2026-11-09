"""Restore the closed tire directly from the pinned SLDPRT Parasolid torus.

Requires OCP and the pure-Python sw2fc reader from
djbarrow/freecad_solidworks_plugin@72d8a1ccde195725da159fc1266729d936f52884.
Only the specific one-face, one-periodic-seam tire topology is accepted.
The native source is read-only; its assembly placements are not changed.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import struct
import sys
from collections import Counter

from OCP.BRepPrimAPI import BRepPrimAPI_MakeTorus
from OCP.BRepCheck import BRepCheck_Analyzer
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps
from OCP.StlAPI import StlAPI_Writer
from OCP.gp import gp_Ax2, gp_Pnt, gp_Dir


def mesh_audit(data):
    count = struct.unpack_from('<I', data, 80)[0]
    assert len(data) == 84 + count * 50
    edges, sectors = Counter(), Counter()
    volume = 0.
    for n in range(count):
        values = struct.unpack_from('<12fH', data, 84 + n*50)
        points = [tuple(round(v, 5) for v in values[k:k+3]) for k in (3,6,9)]
        for i in range(3):
            edges[tuple(sorted([points[i], points[(i+1)%3]]))] += 1
        x = sum(p[0] for p in points)/3
        z = sum(p[2] for p in points)/3
        sectors[int((math.degrees(math.atan2(z,x))+360)%360)//10] += 1
        a,b,c=points
        volume += (a[0]*(b[1]*c[2]-b[2]*c[1])+a[1]*(b[2]*c[0]-b[0]*c[2])+a[2]*(b[0]*c[1]-b[1]*c[0]))/6
    return {'triangles':count, 'boundary_edges':sum(n==1 for n in edges.values()),
            'nonmanifold_edges':sum(n>2 for n in edges.values()),
            'angular_coverage_degrees':len(sectors)*10, 'signed_volume_mm3':volume}


def main():
    p=argparse.ArgumentParser()
    p.add_argument('source', type=Path)
    p.add_argument('converter', type=Path)
    p.add_argument('output', type=Path)
    args=p.parse_args()
    sys.path.insert(0,str(args.converter))
    from sw2fc.loader import load_transmit
    native=args.source/'original/Model/轮胎1.SLDPRT'
    original=native.read_bytes()
    source_sha=hashlib.sha1(b'blob '+str(len(original)).encode()+b'\0'+original).hexdigest()
    assert source_sha=='c86528aaebaf7663c8b9fb2e91e063b7fd0678aa'
    transmit,_=load_transmit(str(native))
    faces=transmit.of_type(14);toruses=transmit.of_type(54)
    fins=transmit.of_type(17);edges=transmit.of_type(16);loops=transmit.of_type(15)
    assert len(faces)==len(toruses)==len(edges)==1
    assert len(fins)==len(loops)==2 and not transmit.of_type(18)
    assert {f['sense'] for f in fins}=={'+','-'}
    assert all(f['edge']==edges[0].index for f in fins)
    assert all(f['other']==next(x.index for x in fins if x.index!=f.index) for f in fins)
    assert all(f['forward']==f.index==f['backward'] for f in fins)
    assert faces[0]['surface']==toruses[0].index
    assert any(r['type']=='S' for r in transmit.of_type(19))
    torus=toruses[0]
    major=torus['major_radius']*1000;minor=torus['minor_radius']*1000
    assert abs(major-25)<1e-9 and abs(minor-2.5)<1e-9
    assert torus['centre']==[0.,0.,0.] and torus['axis']==[0.,1.,0.]
    origin=gp_Pnt(*(x*1000 for x in torus['centre']))
    axes=gp_Ax2(origin,gp_Dir(*torus['axis']),gp_Dir(*torus['x_axis']))
    shape=BRepPrimAPI_MakeTorus(axes,major,minor).Shape()
    assert BRepCheck_Analyzer(shape).IsValid()
    properties=GProp_GProps();BRepGProp.VolumeProperties_s(shape,properties)
    expected=2*math.pi**2*major*minor**2
    assert abs(properties.Mass()/expected-1)<1e-10
    BRepMesh_IncrementalMesh(shape,.08,False,.2,True).Perform()
    args.output.mkdir(parents=True,exist_ok=True)
    path=args.output/'part-09.stl'
    writer=StlAPI_Writer();writer.ASCIIMode=False
    assert writer.Write(shape,str(path))
    data=path.read_bytes();after=mesh_audit(data)
    assert after['boundary_edges']==after['nonmanifold_edges']==0
    assert after['angular_coverage_degrees']==360
    assert abs(abs(after['signed_volume_mm3'])/expected-1)<.005
    report={'schema':'alfred-bb8-tire-source-repair/v1',
        'source_repo':'HWeigo/BB8_Robot','source_commit':'7318411cfbfbffdbd68f75e3734a50c3933969e5',
        'source_file':'Model/轮胎1.SLDPRT','source_git_blob_sha1':source_sha,
        'reader_repo':'djbarrow/freecad_solidworks_plugin',
        'reader_commit':'72d8a1ccde195725da159fc1266729d936f52884',
        'source_topology':'one torus face; one closed seam edge used twice with opposite FIN senses; solid region; no cut end faces',
        'source_parameters':{'major_radius_mm':major,'tube_radius_mm':minor,'axis':torus['axis'],'centre_m':torus['centre'],'outer_diameter_mm':2*(major+minor),'cross_section_diameter_mm':2*minor},
        'before':mesh_audit((args.source/'meshes/part-09.stl').read_bytes()),
        'after':after,'analytical_volume_mm3':expected,'occt_volume_mm3':properties.Mass(),
        'replacement_stl_sha256':hashlib.sha256(data).hexdigest(),
        'replacement_stl_bytes':len(data),'affected_instances':['c12','c13','c46','c47'],
        'placements_unchanged':True,'recorded_poses_unchanged':True,
        'scope':'source-derived visual mesh repair only; no inner-wheel contact or motor dynamics calibration'}
    (args.output/'tire-repair.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(report,ensure_ascii=False,indent=2))


if __name__=='__main__':main()
