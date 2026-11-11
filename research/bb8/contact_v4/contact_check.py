"""Recompute tangential wheel/shell slip at saved contact points."""
from pathlib import Path
import argparse,json,hashlib
import numpy as np
import mujoco

def main():
    p=argparse.ArgumentParser();p.add_argument('--model',type=Path,required=True);p.add_argument('--replay',type=Path,required=True);p.add_argument('--out',type=Path,required=True);a=p.parse_args()
    r=json.loads(a.replay.read_text());assert hashlib.sha256(a.model.read_bytes()).hexdigest()==r['model_sha256']
    m=mujoco.MjModel.from_xml_path(str(a.model));d=mujoco.MjData(m);names=[m.geom(i).name for i in range(m.ngeom)]
    ja=np.zeros((3,m.nv));jb=np.zeros_like(ja);jr=np.zeros_like(ja);force=np.zeros(6)
    samples=[];weighted=0.;weight=0.
    for f in r['frames'][::10]:
        if not 3<=f['t']<=9:continue
        d.qpos[:]=f['qpos'];d.qvel[:]=f['qvel'];d.ctrl[:]=f['ctrl'];mujoco.mj_forward(m,d)
        normal=0.;slip2=0.;n=0
        for j,c in enumerate(d.contact):
            pair=[names[c.geom1],names[c.geom2]]
            if not any(x.startswith(('left_ring','right_ring')) for x in pair) or not any(x.startswith('cavity') for x in pair):continue
            mujoco.mj_jac(m,d,ja,jr,c.pos,int(m.geom_bodyid[c.geom1]));mujoco.mj_jac(m,d,jb,jr,c.pos,int(m.geom_bodyid[c.geom2]))
            relative=(ja-jb)@d.qvel;axis=c.frame[:3];tangent=relative-axis*float(relative@axis)
            mujoco.mj_contactForce(m,d,j,force);load=max(0.,float(force[0]))
            normal+=load;slip2+=load*float(tangent@tangent);n+=1
        weighted+=slip2;weight+=normal
        samples.append({'t':f['t'],'contacts':n,'total_wheel_normal_N':normal,'force_weighted_tangential_slip_rms_m_s':float(np.sqrt(slip2/max(normal,1e-15)))})
    result={'schema':'alfred-bb8-wheel-slip-check/v4','replay_sha256':hashlib.sha256(a.replay.read_bytes()).hexdigest(),
            'force_weighted_wheel_slip_rms_m_s':float(np.sqrt(weighted/max(weight,1e-15))),'samples':samples,
            'method':'mj_jac at each wheel/shell contact, both point velocities in world coordinates, tangential projection weighted by normal force; sampled every 0.2 s from 3 to 9 s'}
    a.out.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps({k:v for k,v in result.items() if k!='samples'}))

if __name__=='__main__':main()
