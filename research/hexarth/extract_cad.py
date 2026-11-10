#!/usr/bin/env python3
"""Extract OEM rigid geometry and measured joint axes, without redistributing STEP.

Input: verified Waveshare STEP plus inspect_cad.py hierarchy JSON. Output is a
private local asset cache. Coordinates retain the OEM CAD axes (mm, Z up).
"""
import argparse
import hashlib
import json
from pathlib import Path
import re

import cadquery as cq
import numpy as np
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.TDocStd import TDocStd_Document
from OCP.TCollection import TCollection_ExtendedString
from OCP.XCAFDoc import XCAFDoc_DocumentTool
from OCP.TDF import TDF_LabelSequence, TDF_Label
from OCP.TDataStd import TDataStd_Name
from OCP.TopLoc import TopLoc_Location


def name(label):
    attr = TDataStd_Name()
    return attr.Get().ToExtString() if label.FindAttribute(TDataStd_Name.GetID_s(), attr) else '?'


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def extract(step, hierarchy, output):
    rows = json.loads(hierarchy.read_text())
    bypath = {r['path']: r for r in rows}
    tops = [r for r in rows if r['depth'] == 1]
    # Output axis inferred independently from the two coaxial OEM metal horns.
    servos = []
    for row in tops:
        if not row['name'].startswith('ST3215'):
            continue
        kids = [r for r in rows if r['path'].startswith(row['path']+'[')
                and r['depth'] == 2 and '金属舵盘' in r['name']]
        assert len(kids) == 2
        a, b = [np.array(r['center']) for r in kids]
        axis = (a-b)/np.linalg.norm(a-b)
        servos.append(dict(path=row['path'], pivot=(a+b)/2, axis=axis))
    assert len(servos) == 18
    bases = [s for s in servos if abs(s['axis'][2]) > .99]
    others = [s for s in servos if abs(s['axis'][2]) < .01]
    assert len(bases) == 6 and len(others) == 12
    bases.sort(key=lambda s: np.arctan2(s['pivot'][1], s['pivot'][0]))
    legs = []
    for i, base in enumerate(bases):
        hip = min(others, key=lambda s: np.linalg.norm((s['pivot']-base['pivot'])[:2]))
        # The knee axis is parallel, next-nearest radially outward.
        choices = [s for s in others if s is not hip
                   and abs(s['axis']@hip['axis']) > .99
                   and np.linalg.norm(s['pivot'][:2]) > np.linalg.norm(hip['pivot'][:2])+20]
        knee = min(choices, key=lambda s: np.linalg.norm(s['pivot']-hip['pivot']))
        shoes = [r for r in tops if r['name'] == 'RUBER-SHOE v1']
        shoe = min(shoes, key=lambda r: np.linalg.norm(np.array(r['center'])-knee['pivot']))
        # A yaw axis is a line: place its origin at shoulder height.
        p0 = base['pivot'].copy(); p0[2] = hip['pivot'][2]
        p1 = hip['pivot']; p2 = knee['pivot']
        radial = p1-p0; radial[2] = 0; radial /= np.linalg.norm(radial)
        pitch_axis = np.cross([0, 0, 1], radial)
        # Positive pitch rotates a radial outward vector downwards.
        legs.append(dict(index=i, base_path=base['path'], hip_path=hip['path'],
                         knee_path=knee['path'], shoe_path=shoe['path'],
                         pivots_mm=[p0.tolist(), p1.tolist(), p2.tolist()],
                         axes=[[0,0,1],pitch_axis.tolist(),pitch_axis.tolist()],
                         foot_center_mm=shoe['center'],
                         hip_offset_mm=float(np.linalg.norm(p1-p0)),
                         femur_axis_distance_mm=float(np.linalg.norm(p2-p1)),
                         knee_to_shoe_center_mm=float(np.linalg.norm(np.array(shoe['center'])-p2))))
    assert len({x['hip_path'] for x in legs}) == len({x['knee_path'] for x in legs}) == 6
    assigned = {}
    # Principal structure has a semantic mapping; only hardware uses proximity.
    structural = {'大臂基础':'coxa', '六足大臂43 v2':'femur', '小臂基础':'tibia',
                  'LEG-HOLDER v1':'tibia', 'CARBON_16X70 v2':'tibia', 'RUBER-SHOE v1':'tibia'}
    anchors = []
    for r in tops:
        if r['name'] not in structural:
            continue
        c = np.array(r['center'])
        leg = min(legs, key=lambda l: np.linalg.norm(c-np.array(l['pivots_mm'][2])))
        group = f"leg{leg['index']}_{structural[r['name']]}"
        assigned[r['path']] = group
        anchors.append((c,group))
    servo_mapping = {}
    for leg in legs:
        i = leg['index']
        for path, parent, child in ((leg['base_path'],'base',f'leg{i}_coxa'),
                                    (leg['hip_path'],f'leg{i}_coxa',f'leg{i}_femur'),
                                    (leg['knee_path'],f'leg{i}_femur',f'leg{i}_tibia')):
            servo_mapping[path] = (parent,child)
            pieces=[r for r in rows if r['path'].startswith(path+'[') and r['depth']==2]
            centers=[np.array(r['center']) for r in pieces if 'center' in r and '金属舵盘' not in r['name']]
            anchors.append((np.mean(centers,axis=0),parent))
    # Fixed case/horn split. All visual electronics remain on the chassis.
    bolts = re.compile('螺丝')
    for r in tops:
        if r['path'] in assigned or r['path'] in servo_mapping:
            continue
        if bolts.search(r['name']) and 'center' in r:
            c = np.array(r['center'])
            group = min(anchors,key=lambda a:np.linalg.norm(a[0]-c))[1]
            assigned[r['path']] = group
        else:
            assigned[r['path']] = 'base'

    doc=TDocStd_Document(TCollection_ExtendedString('HexArth'))
    reader=STEPCAFControl_Reader(); reader.SetNameMode(True)
    assert int(reader.ReadFile(str(step))) == 1 and reader.Transfer(doc)
    tool=XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
    roots=TDF_LabelSequence();tool.GetFreeShapes(roots)
    assert roots.Length() == 1
    root=roots.Value(1)
    parts=TDF_LabelSequence();tool.GetComponents_s(root,parts,False)
    output.mkdir(parents=True,exist_ok=True)
    (output/'meshes').mkdir(exist_ok=True)
    records=[]
    grouped={g:[] for g in ['base']+[f'leg{i}_{j}' for i in range(6) for j in ('coxa','femur','tibia')]}

    def export(label,path,parent=TopLoc_Location(),forced_group=None):
        ref=TDF_Label(); target=label; loc=parent
        if tool.IsReference_s(label):
            tool.GetReferredShape_s(label,ref); target=ref
            loc=parent.Multiplied(tool.GetLocation_s(label))
        nm=name(target)
        shape=cq.Shape.cast(tool.GetShape_s(target)).moved(cq.Location(loc))
        if path in servo_mapping:
            children=TDF_LabelSequence();tool.GetComponents_s(target,children,False)
            for j in range(1,children.Length()+1):
                ch=children.Value(j); rr=TDF_Label();tool.GetReferredShape_s(ch,rr)
                cn=name(rr)
                par,child=servo_mapping[path]
                export(ch,path+f'[{j}]/'+cn,loc,child if '金属舵盘' in cn else par)
            return
        group=forced_group or assigned.get(path,'base')
        if not shape.Solids():
            return
        grouped[group].append(shape)
        # Collision uses CAD convex hulls of each exterior part, not one hull
        # spanning a whole articulated leg. Tiny fasteners/electronics excluded.
        is_case=forced_group is not None and '金属舵盘' not in nm
        collides=(nm in structural or nm in ('躯干上盖板','躯干下盖板','前盖板 3D打印','后盖板 3D打印')
                  or (is_case and nm in ('ZK_122','SG-ZIJI_15','XG-ZIJI_16')))
        row=bypath.get(path,{})
        rec={'path':path,'name':nm,'body':group,'collision':collides,
             'volume_mm3':row.get('volume'), 'center_mm':row.get('center')}
        if collides:
            filename=f'collision_{len(records):04d}.stl'
            cq.exporters.export(shape,str(output/'meshes'/filename),tolerance=.35,angularTolerance=.18)
            rec['collision_file']=filename
        records.append(rec)
    rootname=name(root)
    for j in range(1,parts.Length()+1):
        label=parts.Value(j); ref=TDF_Label();tool.GetReferredShape_s(label,ref)
        export(label,'/'+rootname+f'[{j}]/'+name(ref))
        if j%40==0: print('exported top-level components',j,flush=True)
    for group,shapes in grouped.items():
        assert shapes,group
        compound=cq.Compound.makeCompound(shapes)
        filename=group+'.stl'
        cq.exporters.export(compound,str(output/'meshes'/filename),tolerance=.35,angularTolerance=.18)
        print(group,'parts',len(shapes),flush=True)
    result={'schema':'hexarth-oem-cad-extraction/v1','step_sha256':sha(step),
            'units':'mm','mesh_tolerance_mm':.35,'angular_tolerance_rad':.18,
            'legs':legs,'parts':records,
            'visual_meshes':{g:g+'.stl' for g in grouped},
            'limitations':['Fasteners assigned to the nearest principal CAD component center; mechanical review pending.',
                           'Axis origins are inferred from OEM horn centers; electrical zero and firmware tibia convention are not calibrated.']}
    result['mesh_sha256']={p.name:sha(p) for p in (output/'meshes').glob('*.stl')}
    (output/'cad-manifest.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')


if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('--step',type=Path,required=True)
    ap.add_argument('--hierarchy',type=Path,required=True);ap.add_argument('--output',type=Path,required=True)
    a=ap.parse_args();extract(a.step,a.hierarchy,a.output)
