from __future__ import annotations
import argparse,json,copy,sys
from pathlib import Path
import numpy as np
HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE/'math'))
from math_first_four_crawl_v1 import ModelStudy, WALK

def run(source,target,root,millimeters):
    d=json.loads(Path(source).read_text());nodes=d['nodes']
    model=ModelStudy(Path(root)/'source/mjcf/scene_local30_nominal.xml',nodes[0]['qpos'])
    diagnostics=[];changed=0
    for i in range(107,132):
        orig=nodes[i]
        model.set_pose(np.array(orig['qpos']))
        foot_targets={k:model.toes()[k] for k in WALK}
        alpha=min(1,(i-106)/11)
        offset_xy=np.array([.319,-.948])*(millimeters/1000)*alpha
        adjusted_root=np.array(orig['root_xy_m'])+offset_xy
        newq,errors=model.ik(adjusted_root,foot_targets,np.array(orig['qpos']),anchored=WALK)
        model.set_pose(newq)
        grounded=tuple(orig['grounded'])
        mar=model.margin(foot_targets,grounded)
        wrench=model.wrench(grounded)
        node=nodes[i]
        node['qpos']=newq.tolist();node['root_xy_m']=newq[:2].tolist();node['COM_xyz_m']=model.com().tolist()
        node['margin_mm']=mar
        node['min_peak_tau_Nm']=wrench.get('min_peak_tau_Nm')
        diagnostics.append({'i':i,'phase':node['phase'],'offset_mm':float(np.linalg.norm(offset_xy)*1000),'margin_mm':mar,'peak_tau_Nm':wrench.get('min_peak_tau_Nm'),'max_joint_rad':float(np.max(np.abs(newq[7:25]))),'max_foot_error_mm':max(errors.values())})
        changed+=1
    d['modification']={'name':'pre_ml_contact_margin_rebalance','root_displacement_xy_mm':(np.array([.319,-.948])*millimeters).tolist(),'modified_nodes':[107,131], 'status':'math_only', 'diagnostics':diagnostics}
    Path(target).write_text(json.dumps(d,ensure_ascii=False,indent=2)+'\n')
    return diagnostics

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--src',required=True);p.add_argument('--out',required=True);p.add_argument('--cad-root',required=True);p.add_argument('--mm',type=float,default=10.)
    a=p.parse_args();r=run(a.src,a.out,a.cad_root,a.mm)
    print(json.dumps({'changed':len(r),'min_margin_mm':min(x['margin_mm'] for x in r),'max_tau':max(x['peak_tau_Nm'] for x in r),'max_q':max(x['max_joint_rad'] for x in r),'max_foot_error_mm':max(x['max_foot_error_mm'] for x in r),'last_5':r[-5:]},indent=2))
