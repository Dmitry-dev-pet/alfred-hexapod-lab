#!/usr/bin/env python3
"""Export all saved step trials, without regenerating their motion."""
import contextlib,hashlib,io,json,struct,sys
from pathlib import Path
import mujoco
import numpy as np
import contact_cpu as experiment
from collision_model import build as corrected_build
experiment.bench=experiment.e.bench
experiment.RL=experiment.e.RL
experiment.terrain=experiment.e

PACK='e1a968eb41538d4f6760e480babf21d7fc3d4d9b4ae208297b01fe4c2a669e6b'
ROOT=experiment.WORK

def digest(b):return hashlib.sha256(b).hexdigest()

def main():
    source=Path(__file__).parent/'traces';dist=ROOT/'microduck-publish/site/dist'
    route=dist/'hexapod/microduck-wall-v1';route.mkdir(parents=True,exist_ok=True)
    dest=dist/'assets/robot-replays/microduck-contact-v2';dest.mkdir(parents=True,exist_ok=True)
    pb=(dist/'assets/robot-models'/f'microduck-original-{PACK}.json').read_bytes();assert digest(pb)==PACK
    manifest=json.loads(pb)['manifest'];visible=manifest['visible_geoms'];assert len(visible)==70
    summary_bytes=(source/'summary.json').read_bytes();summary=json.loads(summary_bytes)
    ip=experiment.bench.upstream_inference(experiment.RL)
    with contextlib.redirect_stdout(io.StringIO()):base,_,_,_=corrected_build(ip,0)
    index={'schema':'microduck-body-terrain-replays/v2','model_commit':experiment.bench.UPSTREAM_COMMIT,'model_package_sha256':PACK,
           'collision_model_sha256':digest((Path(__file__).parent/'collision_model.py').read_bytes()),'environment_mesh_count':60,'source_summary_sha256':digest(summary_bytes),'stand_policy_sha256':experiment.sha(experiment.e.POLICY),'frame_stride':498,'visible_mesh_count':70,'clips':[]}
    maxerr=0.;totalframes=0
    for entry in summary['episodes']:
        raw=(source/entry['trace_file']).read_bytes();assert digest(raw)==entry['trace_sha256'];tr=json.loads(raw)
        experiment.terrain.EDGE=tr['terrain']['edge_x_m']
        with contextlib.redirect_stdout(io.StringIO()):m,d,_,_=corrected_build(ip,tr['terrain']['height_m'],tr['dt'],tr['contact_profile']=='firm_1ms')
        assert m.nq==35 and m.nu==14
        # Geometry and physical robot parameters remain byte-identical.
        assert np.array_equal(m.body_mass[:base.nbody],base.body_mass)
        assert np.array_equal(m.body_inertia[:base.nbody],base.body_inertia)
        assert np.array_equal(m.jnt_range,base.jnt_range)
        assert np.array_equal(m.mesh_vert,base.mesh_vert)
        found=[g for g in range(m.ngeom) if m.geom_type[g]==mujoco.mjtGeom.mjGEOM_MESH and m.geom_group[g]==2]
        assert len(found)==len(visible)==70
        for g,actual_gid in zip(visible,found):
            assert mujoco.mj_id2name(m,mujoco.mjtObj.mjOBJ_MESH,int(m.geom_dataid[actual_gid]))==g['mesh']
        rows=[]
        for frame in tr['frames']:
            d.qpos[:]=frame['qpos'];mujoco.mj_forward(m,d);row=[frame['t'],*d.qpos[:7]]
            for actual_gid in found:
                q=np.empty(4);mujoco.mju_mat2Quat(q,d.geom_xmat[actual_gid]);row.extend(d.geom_xpos[actual_gid]);row.extend(q)
            rows.append(row)
        exact=np.asarray(rows);arr=np.asarray(rows,dtype='<f4');err=float(np.max(np.abs(exact-arr)));maxerr=max(maxerr,err)
        assert err<1e-6 and arr.shape==(entry['frames'],498)
        blob=struct.pack('<8sII',b'MDWALL2\0',len(rows),498)+arr.tobytes();h=digest(blob)
        cid=Path(entry['trace_file']).stem;name=cid+'-'+h[:12]+'.bin';(dest/name).write_bytes(blob)
        phases=[]
        for f in tr['frames']:
            if not phases or phases[-1]['name']!=f['phase']:phases.append({'t':f['t'],'name':f['phase']})
        edge_stance=bool(entry['height_m'] and entry['standing_final_1s'] and all(all(v>.05 for v in fr['step_loads_n'].values()) for fr in tr['frames'][-50:]))
        overhang=max(0.,max(entry['edge_x_m']-b['min'][0] for fr in tr['frames'][-50:] for b in fr['foot_bounds'].values()))
        index['clips'].append({**entry,'supported_top_stance':edge_stance,'max_sole_overhang_m':overhang,'contact_profile':tr['contact_profile'],'id':cid,'url':'/assets/robot-replays/microduck-contact-v2/'+name,'size':len(blob),'sha256':h,
                               'terrain':tr['terrain'],'phase_events':phases})
        totalframes+=len(rows)
    index['max_float32_absolute_error']=maxerr
    (route/'replays.json').write_text(json.dumps(index,separators=(',',':'),ensure_ascii=False)+'\n')
    print(json.dumps({'clips':len(index['clips']),'frames':totalframes,'max_float32_absolute_error':maxerr,'index_sha256':digest((route/'replays.json').read_bytes())}))

if __name__=='__main__':main()
