"""Explicit nonphysical ablation: does removing servo-plate contact solve the issue?

No modified CAD or successful robot is claimed. Keep this separate from validation.
"""
import argparse,hashlib,json
from pathlib import Path
import xml.etree.ElementTree as E
import numpy as np
import mujoco
from simulate import make,simulate,zero_angle,load_parts,build

def main():
    p=argparse.ArgumentParser();p.add_argument('--source',type=Path,required=True);p.add_argument('--tire',type=Path,required=True)
    p.add_argument('--fit',type=Path,required=True);p.add_argument('--out',type=Path,required=True);p.add_argument('--allow-estimated-mass',action='store_true')
    a=p.parse_args()
    if not a.allow_estimated_mass:p.error('--allow-estimated-mass required')
    a.out.mkdir(parents=True,exist_ok=True);parts,_=load_parts(a.source,a.tire);profile=build(a.source,a.tire,'nominal')
    fit=np.array(json.loads(a.fit.read_text())['fit']['rigid_translation_from_source_pose_mm'])*.001
    zero,_=zero_angle(profile,fit);xml,meta=make(profile,parts,fit,level=4,peak=.2,collision='facets')
    root=E.fromstring(xml);g=root.find('.//geom[@name="cad_c21"]');g.set('contype','0');g.set('conaffinity','0')
    xml=E.tostring(root,encoding='unicode');meta.update(xml_sha256=hashlib.sha256(xml.encode()).hexdigest(),
        ablation='servo plate c21 collision disabled; visual geometry and mass unchanged',eligible_for_physical_validation=False)
    (a.out/'plate-contact-off.xml').write_text(xml);m=mujoco.MjModel.from_xml_string(xml)
    for kv,ki in [(0.,0.),(1.2,.3)]:
        gains={'kp':.3,'kd':.04,'kv':kv,'ki':ki}
        r=simulate(m,meta,fit,profile,zero,'hold',4.,gains,peak=.2)
        r['eligible_for_physical_validation']=False;r['controller_gains']=gains
        (a.out/f'plate-contact-off-kv{kv}.json').write_text(json.dumps(r,separators=(',',':'))+'\n')
        print(json.dumps({'gains':gains,'summary':r['summary']}),flush=True)

if __name__=='__main__':main()
