"""Audit cached SolidWorks mass metadata; do not adopt it as measured mass.

Uses the pinned LGPL sw2fc container reader supplied separately. No native CAD
or third-party converter implementation is distributed by this script.
"""
import argparse,hashlib,json,sys,xml.etree.ElementTree as E
from pathlib import Path

def properties(raw):
    root=E.fromstring(raw)
    return {x.get('name'):''.join(x.itertext()).strip() for x in root.iter() if x.get('name')}

def inspect(source,converter):
    sys.path.insert(0,str(converter));from sw2fc.container import read_streams
    scene=json.loads((source/'visual/scene.json').read_text());rows={}
    for key,m in scene['meshes'].items():
        path=source/'original/Model'/m['source_file'];raw=path.read_bytes()
        blob=hashlib.sha1(b'blob '+str(len(raw)).encode()+b'\0'+raw).hexdigest()
        if blob!=m['source_git_blob_sha']:raise ValueError('Native CAD hash mismatch')
        _,streams=read_streams(raw)
        info=properties(streams['docProps/ISolidWorksInformation.xml']) if streams.get('docProps/ISolidWorksInformation.xml') else {}
        cache=info.get('SW-MassProp-Config-0');vals=[float(x) for x in cache.split(',')] if cache else None
        mat=streams.get('swXmlContents/MATERIALTREE');materials=[]
        if mat:
            materials=[x.get('name') for x in E.fromstring(mat.decode('utf-16')).iter() if x.tag.split('}')[-1]=='material']
        customs=properties(streams['docProps/custom.xml']) if streams.get('docProps/custom.xml') else {}
        flags=['NOT_HARDWARE_MEASURED']
        if not materials:flags.append('MATERIAL_NOT_ASSIGNED')
        if vals:
            # Cache serializes volume to 6 decimals m^3: ±500 mm^3 rounding.
            delta=vals[3]*1e9-m['source_shape_volume_mm3']
            if abs(delta)>500+.01*abs(m['source_shape_volume_mm3']):flags.append('CACHED_VOLUME_INCONSISTENT_WITH_CURRENT_GEOMETRY')
        else:flags.append('NO_CACHED_MASS')
        if any('玻璃' in x or '陶瓷' in x for x in materials):flags.append('DISPLAY_OR_SUSPECT_MATERIAL')
        rows[key]={'source_file':m['source_file'],'source_git_blob_sha1':blob,'material_names_in_native_CAD':materials,
                   'current_shape_volume_mm3':m['source_shape_volume_mm3'],
                   'cached_mass_kg_UNVERIFIED':vals[5] if vals else None,
                   'cached_volume_m3':vals[3] if vals else None,'cached_mass_properties_raw':cache,
                   'custom_weight_text_UNVERIFIED':customs.get('Weight'),
                   'mass_used_for_next_physical_model_kg':None,'mass_evidence':None,'flags':flags}
    components=[]
    for item in scene['instances']:
        components.append({'id':item['id'],'source_file':scene['meshes'][item['part_key']]['source_file'],
                           'source_position_mm':item['position_mm'],'source_rotation_xyzw':item['rotation_xyzw'],
                           'mass_kg':None,'center_of_mass_local_m':None,'inertia_at_COM_kg_m2':None,
                           'evidence':None,'status':'UNRESOLVED'})
    return {'schema':'alfred-bb8-mass-audit/v2','source_commit':scene['source_commit'],
            'converter_commit':'72d8a1ccde195725da159fc1266729d936f52884',
            'physical_mass_model_ready':False,'unique_parts':rows,'components':components,
            'previous_diagnostic_only':{'shell_mass_kg':.2,'lumped_chassis_kg':.38,'each_wheel_kg':.025,'total_without_head_kg':.63,
                                        'battery_individually_modelled':False,'chassis_COM_m':[-.003,.000735,.008],
                                        'status':'ASSUMED_NOT_VALIDATED_DO_NOT_INFER_REAL_ROBOT_BEHAVIOR'},
            'required_next':['Identify battery model, chemistry, configuration, capacity and mass',
                             'Identify exact motor/gearbox, hub, servo and board masses',
                             'Select actual plate, shell and spacer materials and shell wall thickness',
                             'Assign component masses with evidence; recompute group COM and full inertia tensor',
                             'Use measured mass ranges/sensitivity for unknown wiring, fasteners and magnets before behavior claims']}

def require_complete_mass_model(manifest):
    unresolved=[c['id'] for c in manifest['components'] if c['status'] not in {'verified','excluded_with_reason'}
                or (c['status']=='verified' and (not isinstance(c['mass_kg'],(int,float)) or c['mass_kg']<=0 or not c['evidence']))]
    if unresolved:raise ValueError('Unverified component masses: '+', '.join(unresolved))
    if not manifest.get('physical_mass_model_ready'):raise ValueError('Physical mass model is not declared ready')
    return True

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--source',type=Path,required=True);p.add_argument('--converter',type=Path,required=True);p.add_argument('--out',type=Path,required=True)
    a=p.parse_args();r=inspect(a.source,a.converter);a.out.write_text(json.dumps(r,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps({'physical_mass_model_ready':r['physical_mass_model_ready'],'parts':len(r['unique_parts']),'components':len(r['components'])}))
