"""Export exactly recorded S2 MuJoCo qpos and per-foot Fz to existing Lite3 3D player schema.

No physics integration or trajectory synthesis: mj_forward at actual recorded qpos for FK only.
No original STL or policy weights in public outputs.
"""
from __future__ import annotations

import hashlib
import json
import os
import zipfile
from pathlib import Path
import mujoco

ZIP=Path(os.environ.get('LITE3_S2_ARCHIVE', './Lite3_S2_Reproducible_60Runs_2026-11-06.zip'))
MODEL=Path(os.environ.get('LITE3_MJCF', './original_assets/upstream/src/assets/robots/lite3/xmls/scene_lite3.xml'))
OUT=Path(os.environ.get('LITE3_S2_VIEWER_OUT', './lite3_s2_player_assets'))
VENDOR='447945fd40fb5585271dce4ea8cc82567916032a'
BODY_NAMES=['body','mainarm','forearm','L_calw','R_claw','FL_hip','FL_thigh','FL_shank','FR_hip','FR_thigh','FR_shank','RR_hip','RR_thigh','RR_shank','RL_hip','RL_thigh','RL_shank']
FOOT_NAMES=['FL','FR','RR','RL']
VARIATIONS=['original_baseline','friction_035','friction_020','uniform_mass_0610','actuator_cap_02256','combined_stress']


def write_one(archive:zipfile.ZipFile, m:mujoco.MjModel, variation:str, seed:int) -> dict:
    name=f'raw/{variation}_{seed}.json'
    member=archive.read(name)
    src=json.loads(member)
    run, traces=src['run'],src['trace']
    assert len(traces)==96 and run['seed']==seed and run['variation']==variation
    assert all(len(x['qpos'])==m.nq and len(x['qvel'])==m.nv for x in traces)
    d=mujoco.MjData(m)
    body_ids=[mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_BODY,n) for n in BODY_NAMES]
    site_ids=[mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_SITE,n) for n in FOOT_NAMES]
    rows=[]
    for tr in traces:
        d.qpos[:]=tr['qpos'];d.qvel[:]=tr['qvel'];mujoco.mj_forward(m,d)
        # 0: time; 1-21: MuJoCo free-base+14-joint qpos; 22-25: recorded real normal Fz;
        # 26-28: command (body velocities);29: recorded tilt;30: loaded count;
        # 31-149: original MJCF FK (17 xyz + wxyz poses);150-161: foot sites xyz.
        row=[tr['t'],*tr['qpos'],*[tr['contact_normal_N'][name] for name in FOOT_NAMES],
             *tr['command_body'],tr['tilt_deg'],tr['loaded_feet']]
        for b in body_ids:
            row.extend(map(float,d.xpos[b]));row.extend(map(float,d.xquat[b]))
        for s in site_ids:row.extend(map(float,d.site_xpos[s]))
        assert len(row)==162
        rows.append([round(float(v),9) for v in row])
    assert abs(rows[-1][0]-9.5)<1e-6
    output={'format':'xgo-lite3-original-stl-mujoco-replay-v1',
            'robot':'Luwu Dynamics XGO-Lite3',
            'model':'Original vendor STL geometry loaded only on device',
            'engine':'MuJoCo 3.14.0 CPU S2 — independent adapter; NOT native MJLab',
            'source':{
              'vendor_commit':VENDOR,
              'policy_onnx_sha256':'773a1bbd0fee45ec38604d979d5fd6fb339ae15d13bfb5d6e2056a171d2a3422',
              'original_trace_sha256':hashlib.sha256(member).hexdigest(),
              'raw_research':'https://drive.google.com/file/d/13eyLt-BdtKfQR76ld5xQyzacSeHSIkNc/view?usp=drivesdk',
              'research_commit':'https://github.com/Dmitry-dev-pet/alfred-hexapod-lab/pull/69'},
            'experiment':{'mode':'onnx','protocol':'midstride','seed':seed,'variation':variation,
              'duration_s':9.5,'frame_count':len(rows),'physics_dt_s':0.004,'recorded_dt_s':0.1,
              'fall':run.get('first_fall'),
              'max_tilt_deg':run['max_tilt_deg'],
              'force_saturation_ticks':run['torque_saturation_ticks'],
              'note':'Original saved 250Hz MuJoCo qpos and per-foot force samples with offline mj_forward FK only. No physics in browser, no hardware, not full 60-replay viewer.'},
            'layout':{'stride':162,'t':0,'qpos':[1,22],'force':[22,26],
              'command':[26,29],'tilt':29,'loaded':30,'mesh_body_pose_start':31,
              'mesh_body_pose_stride':7,'foot_site_pose_start':150,'body_names':BODY_NAMES},
            'frames':rows}
    fn=f'lite3-s2-{variation}-seed{seed}-v1.json'
    p=OUT/'docs/hexapod/data'/fn
    p.parent.mkdir(parents=True,exist_ok=True)
    p.write_text(json.dumps(output,ensure_ascii=False,separators=(',',':'))+'\n')
    return {'variation':variation,'seed':seed,'file':p.name,'size':p.stat().st_size,
            'sha256':hashlib.sha256(p.read_bytes()).hexdigest(),'rows':len(rows),
            'original_sha256':hashlib.sha256(member).hexdigest()}


def main():
    m=mujoco.MjModel.from_xml_path(str(MODEL))
    assert m.nq==21 and m.nbody==18
    with zipfile.ZipFile(ZIP) as z:
        entries=[write_one(z,m,v,20261106) for v in VARIATIONS]
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT/'exports.json').write_text(json.dumps(entries,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(entries,ensure_ascii=False,indent=2))

if __name__=='__main__':main()