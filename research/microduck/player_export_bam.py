#!/usr/bin/env python3
"""Export saved, source-verified MicroDuck states to compact visual FK tracks.

No physics is regenerated. Original mesh geometry is reused from the immutable
hosted robot package. Every active 50 Hz frame is retained in little-endian
Float32 with its exact source-trace SHA-256 and an independent payload digest.
"""
import argparse
import hashlib
import importlib.util
import json
from pathlib import Path
import struct

import mujoco
import numpy as np

PIN='bae39f5900b318f7e87fc485c30a31213cfa14eb'
PACK_SHA='e1a968eb41538d4f6760e480babf21d7fc3d4d9b4ae208297b01fe4c2a669e6b'
MODES=('xml_pd','bam','bam_backlash')
CASES=('stand','sit','fold','roll180','pitch180','side90','push')
SEEDS=(20261109,20261110,20261111)


def sha(b):return hashlib.sha256(b).hexdigest()


def build(root, traces, site):
    pack_path=site/'assets/robot-models'/f'microduck-original-{PACK_SHA}.json'
    pack_bytes=pack_path.read_bytes()
    assert sha(pack_bytes)==PACK_SHA
    pack=json.loads(pack_bytes);manifest=pack['manifest'];visible=manifest['visible_geoms']
    assert manifest['revision']==PIN and len(visible)==70
    source_bytes=(traces/'summary.json').read_bytes();summary=json.loads(source_bytes)
    assert summary['rl_commit']==PIN and len(summary['episodes'])==63
    sp=importlib.util.spec_from_file_location('microduck_ip',root/'scripts/infer_policy.py')
    ip=importlib.util.module_from_spec(sp);sp.loader.exec_module(ip)
    models={}
    base=root/'src/mjlab_microduck/robot/microduck'
    for mode in MODES:
        scene=base/('scene_backlash.xml' if mode=='bam_backlash' else 'scene.xml')
        if mode=='xml_pd':m=mujoco.MjModel.from_xml_path(str(scene))
        else:
            bam=ip.load_bam_model(ip.BAM_KP_FW,7.4,ip.BAM_MAX_CURRENT)
            m,_,_,_=ip.load_mujoco_with_bam(str(scene),bam,.005,.1,ip.BAM_VIN_MIN)
        # Passive backlash bodies must preserve each original visual instance
        # and MuJoCo's raw-STL normalization; no hand-made geometry mapping.
        found=[]
        for gid in range(m.ngeom):
            if m.geom_type[gid]==mujoco.mjtGeom.mjGEOM_MESH and m.geom_group[gid]==2:
                found.append(gid)
        assert found==[g['gid'] for g in visible]
        for g in visible:
            mid=int(m.geom_dataid[g['gid']]);name=mujoco.mj_id2name(m,mujoco.mjtObj.mjOBJ_MESH,mid)
            assert name==g['mesh']
        models[mode]=(m,mujoco.MjData(m),sha(scene.read_bytes()))
    out=site/'assets/robot-replays/microduck-bam-v2';out.mkdir(parents=True,exist_ok=True)
    route=site/'hexapod/microduck-bam-v2';route.mkdir(parents=True,exist_ok=True)
    index={'schema':'microduck-bam-replays/v2','model_commit':PIN,'model_package_sha256':PACK_SHA,
           'source_summary_sha256':sha(source_bytes),'duration_s':6,'frame_count':300,
           'frame_stride':8+7*len(visible),'visible_mesh_count':len(visible),'clips':[]}
    max_error=0.0
    for entry in summary['episodes']:
        raw=(traces/entry['trace_file']).read_bytes();assert sha(raw)==entry['trace_sha256']
        tr=json.loads(raw);mode=tr['mode'];m,d,scene_sha=models[mode]
        assert mode in MODES and tr['case'] in CASES and tr['seed'] in SEEDS
        assert len(tr['frames'])==300 and tr['physical_steps']>=1200
        rows=[]
        for f in tr['frames']:
            assert (len(f['qpos']),len(f['qvel']),len(f['ctrl']))==(m.nq,m.nv,m.nu)
            d.qpos[:]=f['qpos'];d.qvel[:]=f['qvel'];d.ctrl[:]=f['ctrl'];mujoco.mj_forward(m,d)
            row=[f['t'],*d.qpos[:7]]
            for g in visible:
                q=np.zeros(4);mujoco.mju_mat2Quat(q,d.geom_xmat[g['gid']])
                row.extend(d.geom_xpos[g['gid']]);row.extend(q)
            rows.append(row)
        arr=np.asarray(rows,dtype='<f4');exact=np.asarray(rows,dtype=np.float64)
        assert arr.shape==(300,index['frame_stride']) and np.isfinite(arr).all()
        err=float(np.max(np.abs(exact-arr)));max_error=max(max_error,err)
        assert err<1e-6 and np.all(np.diff(arr[:,0])>0) and arr[-1,0]==6
        data=struct.pack('<8sII',b'MDPOSE2\0',300,index['frame_stride'])+arr.tobytes()
        digest=sha(data);name=f"{mode}-{tr['case']}-{tr['seed']}-{digest[:12]}.bin"
        (out/name).write_bytes(data)
        index['clips'].append({**entry,'url':'/assets/robot-replays/microduck-bam-v2/'+name,
                               'size':len(data),'sha256':digest,'scene_sha256':scene_sha,
                               'source_model_nq':m.nq})
    assert len({(c['mode'],c['case'],c['seed']) for c in index['clips']})==63
    index['max_float32_absolute_error']=max_error
    (route/'replays.json').write_text(json.dumps(index,separators=(',',':'))+'\n')
    print(json.dumps({'clips':63,'frames':18900,'instances':len(visible),'bytes':sum(c['size'] for c in index['clips']),
                      'max_float32_absolute_error':max_error,'index_sha256':sha((route/'replays.json').read_bytes())}))


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--rl-root',type=Path,required=True)
    p.add_argument('--traces',type=Path,required=True);p.add_argument('--site-dist',type=Path,required=True)
    a=p.parse_args();build(a.rl_root,a.traces,a.site_dist)
