"""Tessellate pinned EN01 STEP into original assembly STL parts and viewer mesh data.

No dimension changes, guessed part geometry or dynamics calculation. Visual parts
follow the existing four recorded joint coordinates on the diagnostic rigid rig.
"""
from pathlib import Path
import hashlib,json,base64,zipfile,sys
import numpy as np
from inspect_step import read_parts

ROOT=Path(__file__).parent
MOBILE='--mobile' in sys.argv
LINEAR=.15 if MOBILE else .06
ANGULAR=.45 if MOBILE else .2
SOURCE=ROOT/'source/OriginalRobotModel.stp'
RAW=SOURCE.read_bytes()
assert len(RAW)==12358579
assert hashlib.sha1(b'blob '+str(len(RAW)).encode()+b'\0'+RAW).hexdigest()=='9156683a0bb694c84dc541a3e26b5f6b08db1a41'
PARTS=read_parts(SOURCE)
EXPECTED=json.loads((ROOT/'assembly-inspection.json').read_text())
assert [p['name'] for p in PARTS]==[p['name'] for p in EXPECTED]

# CAD X forward, Y vertical, Z lateral -> proper rotation X,-Z,Y; mm -> m.
ORIGIN=np.array([-4.793581489766,30.5,32.0])
def mapped(v):
    a=(np.asarray(v,dtype=np.float64)-ORIGIN)*.001
    return np.stack([a[...,0],-a[...,2],a[...,1]],axis=-1)

BODY=set(range(18))|{36,38,40,41,42,43,44,45,46,47,49,51,52,53,54,55,56,59,60,61,66,67,68,70,72}
WHEEL={19,21,64,65}
stls=ROOT/('STL_mobile' if MOBILE else 'STL');stls.mkdir(exist_ok=True)
packed=[];total=0
dtype=np.dtype([('normal','<f4',3),('v','<f4',(3,3)),('attr','<u2')])
for p in PARTS:
    sid=p['id'];stl=stls/f'part-{sid:02d}.stl'
    assert p['shape'].exportStl(str(stl),tolerance=LINEAR,angularTolerance=ANGULAR,relative=False)
    raw=stl.read_bytes();count=int.from_bytes(raw[80:84],'little');assert len(raw)==84+count*50
    triangles=np.frombuffer(raw,dtype=dtype,count=count,offset=84)['v']
    assert count>0 and np.isfinite(triangles).all()
    verts,indices=np.unique(triangles.reshape(-1,3),axis=0,return_inverse=True)
    positions=mapped(verts).astype('<f4');indices=indices.astype('<u4')
    # Rigid transform must preserve source dimensions to float32 accuracy.
    source_extent=np.ptp(verts.astype(float),axis=0)*.001
    assert np.max(np.abs(np.ptp(positions.astype(float),axis=0)-source_extent[[0,2,1]]))<1e-7
    side='left' if p['center'][2]<32 else 'right'
    group='body' if sid in BODY else f'{side}_wheel' if sid in WHEEL else f'{side}_leg'
    packed.append({'id':sid,'name':p['name'],'source_path':p['path'],
        'color':p['color'] or [.65,.68,.7,1], 'group':group,
        'vertices_f32le':base64.b64encode(positions.tobytes()).decode(),
        'indices_u32le':base64.b64encode(indices.tobytes()).decode(),
        'vertex_count':len(verts),'triangles':count,
        'stl_sha256':hashlib.sha256(raw).hexdigest(),
        'cad_bounds_mm':p['bounds']})
    total+=count
    print(sid,p['name'],group,count,flush=True)

doc={'schema':'en01-original-step-mesh/v1','source':{
    'repository':'fuwei007/Navbot-EN01','commit':'3139f2fb32b38d10611feb905e16f8aa012f2f2e',
    'path':'Hardware/RebotModel/OriginalRobotModel.stp','git_blob_sha1':'9156683a0bb694c84dc541a3e26b5f6b08db1a41',
    'raw_sha256':hashlib.sha256(RAW).hexdigest(),'bytes':len(RAW)},
    'coordinate_transform':{'origin_cad_mm':ORIGIN.tolist(),'rotation_xyz':['x','-z','y'],'scale':.001},
    'tessellation':{'linear_deflection_mm':LINEAR,'angular_deflection_rad':ANGULAR,'rescaled_parts':False},
    'rig':{'left_hip':mapped([-4.793581489766,15.5,2.4]).tolist(),
        'right_hip':mapped([-4.793581489766,15.5,61.6]).tolist(),
        'left_wheel':mapped([-2.890291062,-20.914475612,-5.15]).tolist(),
        'right_wheel':mapped([-2.890291062,-20.914475612,69.15]).tolist(),
        'type':'diagnostic rigid leg groups; original four-bar articulation NOT reconstructed',
        'contact_geometry':'Original CAD is a visual shell only; contacts and forces are from the unchanged surrogate traces'},
    'part_count':len(packed),'solid_count':sum(p['solids'] for p in PARTS),'triangle_count':total,'parts':packed}
out=ROOT/('original-meshes-mobile.json' if MOBILE else 'original-meshes.json');out.write_text(json.dumps(doc,ensure_ascii=False,separators=(',',':'))+'\n')
if MOBILE:
    print(json.dumps({'parts':len(packed),'triangles':total,'mesh_bytes':out.stat().st_size}),flush=True)
    sys.exit(0)
manifest={**doc,'parts':[{k:v for k,v in p.items() if k not in ['vertices_f32le','indices_u32le']} for p in packed]}
(ROOT/'STL_MANIFEST.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2)+'\n')
readme='EN01 original geometry\nSource: fuwei007/Navbot-EN01 at 3139f2fb32b38d10611feb905e16f8aa012f2f2e\nOriginalRobotModel.stp git blob: 9156683a0bb694c84dc541a3e26b5f6b08db1a41\n73 STEP component occurrences exported into binary STL in original global assembly coordinates (millimetres). No scaling or invented geometry. STEP->STL tessellation uses 0.06 mm deflection. See STL_MANIFEST.json for names, original assembly paths and checksums.\nThis is geometry, not validated dynamics or a print-ready kit; keep original assembly placements.\n'
(ROOT/'README_STL.txt').write_text(readme)
archive=ROOT/'EN01_original_CAD_STL_20261109.zip'
with zipfile.ZipFile(archive,'w',zipfile.ZIP_DEFLATED) as z:
    for p in sorted(stls.glob('*.stl')):z.write(p,'STL/'+p.name)
    z.write(ROOT/'STL_MANIFEST.json','STL_MANIFEST.json');z.write(ROOT/'README_STL.txt','README.txt')
print(json.dumps({'parts':len(packed),'solids':doc['solid_count'],'triangles':total,'mesh_bytes':out.stat().st_size,'STL_archive_bytes':archive.stat().st_size}),flush=True)
