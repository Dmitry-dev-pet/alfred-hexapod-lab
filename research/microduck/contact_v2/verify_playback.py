#!/usr/bin/env python3
"""Audit the player's actual linear/slerp interpolation, not just physics keyframes."""
from pathlib import Path
import contextlib,io,json,math
import numpy as np,mujoco as mj
from collision_model import setup,terrain,WORK
HERE=Path(__file__).resolve().parent

def main():
    root=WORK/'microduck-publish/site/dist';index=json.loads((root/'hexapod/microduck-wall-v1/replays.json').read_text());ip=terrain.bench.upstream_inference(terrain.RL);out=[];worst=0.;count=0
    for c in index['clips']:
        if not c['height_m']:continue
        with contextlib.redirect_stdout(io.StringIO()):m,d,*_=setup(ip,c['height_m'],c['edge_x_m'],'back')
        visible=[g for g in range(m.ngeom) if m.geom_group[g]==2];parts=[(j,g) for j,g in enumerate(visible) if m.body_weldid[m.geom_bodyid[g]]==1];step=mj.mj_name2id(m,mj.mjtObj.mjOBJ_GEOM,'step_top')
        data=np.frombuffer((root/c['url'].lstrip('/')).read_bytes(),dtype='<f4',offset=16).reshape(c['frames'],498);peak=0.
        for a,b in zip(data,data[1:]):
            for j,g in parts:
                k=8+j*7;p0=a[k:k+3].astype(float);p1=b[k:k+3].astype(float);q0=a[k+3:k+7].astype(float);q1=b[k+3:k+7].astype(float);q0/=np.linalg.norm(q0);q1/=np.linalg.norm(q1);dot=float(q0@q1)
                if dot<0:q1=-q1;dot=-dot
                theta=math.acos(np.clip(dot,-1,1))
                for t in [.25,.5,.75]:
                    q=(1-t)*q0+t*q1 if dot>.9995 else (math.sin((1-t)*theta)*q0+math.sin(t*theta)*q1)/math.sin(theta);q/=np.linalg.norm(q)
                    d.geom_xpos[g]=(1-t)*p0+t*p1;mj.mju_quat2Mat(d.geom_xmat[g],q);peak=max(peak,-float(mj.mj_geomDistance(m,d,g,step,.2,None)));count+=1
        worst=max(worst,peak);out.append({'clip':c['id'],'max_interpolated_body_step_overlap_m':peak});print(json.dumps(out[-1]),flush=True)
    assert worst<.001
    r={'samples':count,'fractions_per_interval':[.25,.5,.75],'maximum_body_step_overlap_m':worst,'episodes':out};(HERE/'playback-verification.json').write_text(json.dumps(r,indent=2)+'\n');print(json.dumps({'max_m':worst,'samples':count}))
if __name__=='__main__':main()
