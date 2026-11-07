#!/usr/bin/env python3
"""Geometric support polygon schedule ONLY; not actual contact/force proof."""
from __future__ import annotations
import argparse,csv,hashlib,json,math
from pathlib import Path
import numpy as np
import mujoco as mj
from hexlab.stand_geometry_v04 import load_legs,tip_world
from hexlab.simulation import support_margin
from reachability_sweep_v6 import DIRECTIONS,direction_offsets,foot_curve

GAIT_SWING_FRACTION={'wave':1/6,'ripple':1/3,'tripod':.5}

def sample_gait(m,k,gait,stride,direction,samples=96):
 d=mj.MjData(m);d.qpos[:3]=k['root_position'];d.qpos[3:7]=k['root_quat_wxyz'];d.qpos[7:]=k['q_stand'];mj.mj_forward(m,d)
 root=np.asarray(d.xpos[m.body('base_link_2').id]).copy()
 f=np.asarray(k['forward_world'],float);l=np.asarray(k['left_world'],float)
 legs=load_legs(m,d,mj)
 feet={z['suffix']:tip_world(m,d,z).copy() for z in legs}
 anchors={name:np.array([(p-root)@f,(p-root)@l]) for name,p in feet.items()}
 radius=float(np.median([np.linalg.norm(a) for a in anchors.values()]))
 offsets=direction_offsets(direction,stride,anchors,radius)
 com=(np.asarray(d.xipos)*np.asarray(m.body_mass)[:,None]).sum(axis=0)/float(m.body_mass.sum())
 ratios=[];support=[];mask_trace=[]
 for j in range(samples):
  ph=j/samples;contact_feet=[];swing_names=[]
  for i,name in enumerate(k['leg_order']):
   legphase=(ph+(.5*(i%2) if gait=='tripod' else i/6))%1
   duty=GAIT_SWING_FRACTION[gait]
   swing=legphase<duty-1e-10
   if swing:
    swing_names.append(name)
   else:
    along,_=foot_curve(min(1.,max(0.,(legphase-duty)/(1-duty))),False)
    move=along*(f*offsets[name][0]+l*offsets[name][1])
    contact_feet.append(feet[name][:2]+move[:2])
  mrg=support_margin(tuple(com[:2]),[tuple(p) for p in contact_feet])
  ratios.append(mrg);support.append(len(contact_feet));mask_trace.append(swing_names)
 return {'gait':gait,'stride_mm':stride,'direction':direction,'min_schedule_supports':min(support),'max_schedule_supports':max(support),
   'min_geometric_com_margin_mm':round(min(ratios)*1000,4),'support_polygon_positive_at_all_phases':all(v>=0 for v in ratios),
   'worst_phase_index':int(np.argmin(ratios)),'com_projection_xy_m':[round(float(x),7) for x in com[:2]]}

def main():
 a=argparse.ArgumentParser();a.add_argument('--scene',type=Path,required=True);a.add_argument('--kit',type=Path,required=True);a.add_argument('--outdir',type=Path,required=True);o=a.parse_args()
 m=mj.MjModel.from_xml_path(str(o.scene));k=json.loads(o.kit.read_text())
 rows=[sample_gait(m,k,gait,stride,dir) for gait in GAIT_SWING_FRACTION for stride in [15,25,35,45,55,70,90] for dir in DIRECTIONS]
 obj={'schema':'cad30-geometric-support-schedule-v6','status':'COM_PROJECTION_PROXY_ONLY','source_model_sha256':hashlib.sha256(o.scene.read_bytes()).hexdigest(),
  'frame_caveat':'Precomputed support feet at each gait phase with original body COM XY held fixed; no actual contact/Fz, no body displacement or dynamic stability',
  'gait_phase_source':'Same phase scheduling as V4: wave 1/6 swing, ripple 1/3, tripod 1/2',
  'samples_per_cycle':96,'results':rows}
 o.outdir.mkdir(parents=True,exist_ok=True)
 (o.outdir/'gait_support_proxy_v6.json').write_text(json.dumps(obj,ensure_ascii=False,separators=(',',':'))+'\n')
 with (o.outdir/'gait_support_proxy_v6.csv').open('w',newline='') as file:
  cols=list(rows[0]);writer=csv.DictWriter(file,fieldnames=cols);writer.writeheader();writer.writerows(rows)
 print(json.dumps({'cases':len(rows),'each_gait':{g:{'min_margin_mm':min(z['min_geometric_com_margin_mm'] for z in rows if z['gait']==g),'positive_cases':sum(z['support_polygon_positive_at_all_phases'] for z in rows if z['gait']==g)} for g in GAIT_SWING_FRACTION}}))
if __name__=='__main__':main()
