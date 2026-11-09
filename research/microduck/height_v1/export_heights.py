#!/usr/bin/env python3
"""Export all saved step trials, without regenerating their motion."""
import contextlib,hashlib,io,json,struct,sys
from pathlib import Path
import mujoco
import numpy as np
from height_cpu import e as experiment

PACK='e1a968eb41538d4f6760e480babf21d7fc3d4d9b4ae208297b01fe4c2a669e6b'
ROOT=experiment.WORK

def digest(b):return hashlib.sha256(b).hexdigest()

def main():
    source=Path(__file__).parent/'traces';dist=ROOT/'microduck-publish/site/dist'
    route=dist/'hexapod/microduck-heights-v1';route.mkdir(parents=True,exist_ok=True)
    dest=dist/'assets/robot-replays/microduck-heights-v1';dest.mkdir(parents=True,exist_ok=True)
    pb=(dist/'assets/robot-models'/f'microduck-original-{PACK}.json').read_bytes();assert digest(pb)==PACK
    manifest=json.loads(pb)['manifest'];visible=manifest['visible_geoms'];assert len(visible)==70
    summary_bytes=(source/'summary.json').read_bytes();summary=json.loads(summary_bytes)
    ip=experiment.bench.upstream_inference(experiment.RL)
    with contextlib.redirect_stdout(io.StringIO()):base,_,_,_=experiment.terrain.build(ip,0)
    index={'schema':'microduck-roll-heights-replays/v1','model_commit':experiment.bench.UPSTREAM_COMMIT,'model_package_sha256':PACK,
           'source_summary_sha256':digest(summary_bytes),'roulade_policy_sha256':experiment.ROLL_SHA,'policy_set_commit':experiment.HF_COMMIT,'frame_stride':498,'visible_mesh_count':70,'clips':[]}
    maxerr=0.;totalframes=0
    for entry in summary['episodes']:
        raw=(source/entry['trace_file']).read_bytes();assert digest(raw)==entry['trace_sha256'];tr=json.loads(raw)
        experiment.terrain.EDGE=tr['terrain']['edge_x_m']
        with contextlib.redirect_stdout(io.StringIO()):m,d,_,_=experiment.terrain.build(ip,tr['terrain']['height_m'],tr['dt'],tr['contact_profile']=='firm_1ms')
        assert m.nq==35 and m.nu==14
        # Geometry and physical robot parameters remain byte-identical.
        assert np.array_equal(m.body_mass[:base.nbody],base.body_mass)
        assert np.array_equal(m.body_inertia[:base.nbody],base.body_inertia)
        assert np.array_equal(m.jnt_range,base.jnt_range)
        assert np.array_equal(m.mesh_vert,base.mesh_vert)
        found=[g for g in range(m.ngeom) if m.geom_type[g]==mujoco.mjtGeom.mjGEOM_MESH and m.geom_group[g]==2]
        assert found==[g['gid'] for g in visible]
        for g in visible:
            assert mujoco.mj_id2name(m,mujoco.mjtObj.mjOBJ_MESH,int(m.geom_dataid[g['gid']]))==g['mesh']
        rows=[]
        for frame in tr['frames']:
            d.qpos[:]=frame['qpos'];mujoco.mj_forward(m,d);row=[frame['t'],*d.qpos[:7]]
            for g in visible:
                q=np.empty(4);mujoco.mju_mat2Quat(q,d.geom_xmat[g['gid']]);row.extend(d.geom_xpos[g['gid']]);row.extend(q)
            rows.append(row)
        exact=np.asarray(rows);arr=np.asarray(rows,dtype='<f4');err=float(np.max(np.abs(exact-arr)));maxerr=max(maxerr,err)
        assert err<1e-6 and arr.shape==(entry['frames'],498)
        blob=struct.pack('<8sII',b'MDHGHT1\0',len(rows),498)+arr.tobytes();h=digest(blob)
        cid=Path(entry['trace_file']).stem;name=cid+'-'+h[:12]+'.bin';(dest/name).write_bytes(blob)
        phases=[]
        for f in tr['frames']:
            if not phases or phases[-1]['name']!=f['phase']:phases.append({'t':f['t'],'name':f['phase']})
        index['clips'].append({**entry,'id':cid,'url':'/assets/robot-replays/microduck-heights-v1/'+name,'size':len(blob),'sha256':h,
                               'terrain':tr['terrain'],'phase_events':phases})
        totalframes+=len(rows)
    index['max_float32_absolute_error']=maxerr
    (route/'replays.json').write_text(json.dumps(index,separators=(',',':'),ensure_ascii=False)+'\n')
    print(json.dumps({'clips':len(index['clips']),'frames':totalframes,'max_float32_absolute_error':maxerr,'index_sha256':digest((route/'replays.json').read_bytes())}))

if __name__=='__main__':main()
