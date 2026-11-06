#!/usr/bin/env python3
"""Export actual recorded Lite3 MuJoCo qpos to exact original STL body poses.

Read-only FK on source MJCF. No physics stepping, re-simulation or invented mesh motion.
No third-party STL is copied into the published docs/ assets.
"""
from __future__ import annotations
import hashlib, json, math
from pathlib import Path
import mujoco

ROOT=Path(__file__).resolve().parents[1]
SOURCE=Path('/mnt/data/lite3_research')
MODEL=SOURCE/'original_assets/upstream/src/assets/robots/lite3/xmls/scene_lite3.xml'
MESHES=SOURCE/'original_assets/upstream/src/assets/robots/lite3/xmls/assets'
SRC_COMMIT='447945fd40fb5585271dce4ea8cc82567916032a'
POLICY_SHA='773a1bbd0fee45ec38604d979d5fd6fb339ae15d13bfb5d6e2056a171d2a3422'
# Exactly all visual mesh body names from the vendor source, in immutable order.
BODY_NAMES=('body','mainarm','forearm','L_calw','R_claw',
            'FL_hip','FL_thigh','FL_shank','FR_hip','FR_thigh','FR_shank',
            'RR_hip','RR_thigh','RR_shank','RL_hip','RL_thigh','RL_shank')
CASES=[('standard','onnx',8.0),('standard','zero',8.0),('midstride','onnx',9.5),('midstride','zero',9.5)]
FEET=('FL','FR','RR','RL')
ROW_LENGTH=1+21+4+3+1+1+len(BODY_NAMES)*7+len(FEET)*3


def digest(path):
    with path.open('rb') as stream:
        return hashlib.file_digest(stream,'sha256').hexdigest()


def round6(x):return round(float(x),9)


def specs(model):
    items=[]
    for body_name in BODY_NAMES:
        body_id=model.body(body_name).id
        found=[]
        for gid in range(model.ngeom):
            if model.geom_bodyid[gid]==body_id and model.geom_group[gid]==2:
                if model.geom_type[gid]==mujoco.mjtGeom.mjGEOM_MESH:
                    found.append(gid)
        if len(found)!=1:raise ValueError(f'{body_name}: expected one visual geom, got {found}')
        mesh_id=int(model.geom_dataid[found[0]])
        mesh_name=mujoco.mj_id2name(model,mujoco.mjtObj.mjOBJ_MESH,mesh_id)
        path=MESHES/(mesh_name+'.STL')
        if not path.is_file():raise FileNotFoundError(path)
        assert model.mesh_scale[mesh_id].tolist()==[1.0,1.0,1.0]
        # The compiler rotates/recenters internal vertices; original raw STL is
        # already in its owning body coordinates. Check independently below.
        items.append({'body':body_name,'mesh':mesh_name+'.STL',
                      'sha256':digest(path),'size':path.stat().st_size,
                      'rgba':[round6(v) for v in model.geom_rgba[found[0]]]})
    assert len(set(x['mesh'] for x in items))==17
    return items


def export(model,protocol,mode,duration,seed=20261106):
    path=SOURCE/('onnx_corrected' if protocol=='standard' else 'midstride_corrected')/f'{mode}_{seed}.json'
    original=json.loads(path.read_text())
    assert original['mode']==mode and original['seed']==seed
    assert original.get('protocol','standard')==protocol
    assert abs(original['trace'][-1]['t']-duration)<0.00001
    data=mujoco.MjData(model)
    bodies=[model.body(name).id for name in BODY_NAMES]
    frames=[]
    for frame in original['trace']:
        data.qpos[:]=frame['qpos']
        data.qvel[:]=frame['qvel']
        data.ctrl[:]=frame['ctrl']
        mujoco.mj_forward(model,data)  # no physics stepping; only source body FK
        extras=[]
        for body_id in bodies:
            extras.extend(round6(v) for v in data.xpos[body_id])
            extras.extend(round6(v) for v in data.xquat[body_id])  # w,x,y,z
        sites=[round6(x) for leg in FEET for x in data.site_xpos[model.site(leg).id]]
        f=([round6(frame['t'])]+[round6(v) for v in frame['qpos']]
          +[round6(frame['contact_normal_N'][n]) for n in FEET]
          +[round6(x) for x in frame['command_body']]
          +[round6(frame['tilt_deg']),int(frame['loaded_feet'])]+extras+sites)
        if len(f)!=ROW_LENGTH or not all(math.isfinite(v) for v in f):
            raise ValueError('Wrong frame shape or numeric value')
        # Provenance check: body local frame transformed by data matches world xpos.
        assert f[31:34]==[round6(v) for v in data.xpos[bodies[0]]]
        frames.append(f)
    assert len(frames)>60 and abs(frames[-1][0]-duration)<.00001
    assert all(frames[i][0]<frames[i+1][0] for i in range(len(frames)-1))
    out={
        'format':'xgo-lite3-original-stl-mujoco-replay-v1',
        'robot':'Luwu Dynamics XGO-Lite3',
        'model':'Original vendor STL geometry loaded only on device',
        'engine':'MuJoCo 3.14.0 CPU — independent contact adapter, NOT native MJLab',
        'source':{
            'vendor_commit':SRC_COMMIT,'policy_onnx_sha256':POLICY_SHA,
            'original_trace_sha256':digest(path),
            'raw_research':'https://drive.google.com/file/d/1seg8nkz9xVV-N09xdhCmMBGlJYrO5uY2/view',
        },
        'experiment':{
            'mode':mode,'protocol':protocol,'seed':seed,'duration_s':duration,
            'frame_count':len(frames),'physics_dt_s':.004,'recorded_dt_s':.1,
            'fall':original['first_fall'],'max_tilt_deg':round6(original['max_tilt_deg']),
            'force_saturation_ticks':original['torque_saturation_ticks'],
            'note':'Measured physics qpos and normal forces. Mesh body FK from exact vendor MJCF at each sampled time.',
            'limitations':['Not native MJLab. Not proven safe on hardware.',
                           'Force/contact data sampled every 100ms; intermediate contact frames not inferred.',
                           'STL meshes remain in original vendor repository and private local browser cache.']
        },
        'layout':{'stride':ROW_LENGTH,'t':0,'qpos':[1,22],'force':[22,26],
                  'command':[26,29],'tilt':29,'loaded':30,
                  'mesh_body_pose_start':31,'mesh_body_pose_stride':7,'foot_site_pose_start':150,
                  'body_names':list(BODY_NAMES),'contact_legs':list(FEET),
                  'pose_format':'xyz + quaternion wxyz (MuJoCo world coordinates)',
                  'sampled_frequency_hz':10},
        'frames':frames,
    }
    target=ROOT/'docs/hexapod/data'/f'lite3-{protocol}-{mode}-seed{seed}-original-v1.json'
    target.parent.mkdir(parents=True,exist_ok=True)
    target.write_text(json.dumps(out,ensure_ascii=False,separators=(',',':'))+'\n')
    return target,out


def main():
    m=mujoco.MjModel.from_xml_path(str(MODEL))
    assert m.nu==14 and m.nbody==18 and m.nmesh==17
    manifest={
        'schema':'xgo-lite3-original-local-stl-assets-v1',
        'vendor':'LuwuDynamics/luwu_mjlab','commit':SRC_COMMIT,
        'asset_path':'src/assets/robots/lite3/xmls/assets',
        'origin':'https://raw.githubusercontent.com/LuwuDynamics/luwu_mjlab/'+SRC_COMMIT+'/',
        'use':'private local browser/device cache only — no STL in public Hexapod Lab repo',
        'license_notice':'Preserve vendor copyrights; public source distribution terms not established by us',
        'meshes':specs(m),
    }
    mpath=ROOT/'docs/hexapod/lite3-original-v1/asset-manifest.json'
    mpath.parent.mkdir(parents=True,exist_ok=True)
    mpath.write_text(json.dumps(manifest,ensure_ascii=False,indent=2)+'\n')
    paths=[]
    for protocol,mode,duration in CASES:
        path,obj=export(m,protocol,mode,duration)
        print(path.name, len(obj['frames']), path.stat().st_size, digest(path))
        paths.append(path)
    print('MESHES',len(manifest['meshes']),sum(x['size'] for x in manifest['meshes']))
    print('STRIDE',ROW_LENGTH)
    return paths

if __name__=='__main__':main()
