#!/usr/bin/env python3
"""Export original upstream CAD visual placements and 60s MuJoCo rigid-body poses.

Does NOT copy CAD meshes to output. Web UI loads them by pinned upstream URLs.
The positions/quaternions are computed directly from saved free-base qpos via mj_forward.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import pathlib
import zipfile

import mujoco
import numpy as np

COMMIT = '5c9a84c16335b30036e451c691d2c544abf4bc72'
UPSTREAM = 'https://raw.githubusercontent.com/MattiaGrigoli/autonomous_hexapod/' + COMMIT + '/src/freenove_description/meshes/'
FORMAT = 'fnk0052-mujoco-cad-visual-replay-v1'


def load_recording(archive: pathlib.Path):
    with zipfile.ZipFile(archive) as z:
        src = z.read('results/cad30_gait_tripod_nominal_60s.json')
    payload = json.loads(src)
    return payload, hashlib.sha256(src).hexdigest()


def fseq(vals, digits: int = 7) -> list[float]:
    return [round(float(x), digits) for x in vals]


def export(model_path: pathlib.Path, recording: dict, qpos_sha: str):
    md = mujoco.MjModel.from_xml_path(str(model_path))
    data = mujoco.MjData(md)
    samples = recording['qpos_samples']
    assert len(samples) == 1501 and abs(float(samples[-1]['t'])-60)<1e-3
    assert md.nq == len(samples[0]['qpos']) == 27
    assert md.nmesh == 30

    visible_geoms = [i for i in range(md.ngeom) if int(md.geom_group[i]) == 2]
    b_ids = sorted({int(md.geom_bodyid[i]) for i in visible_geoms})
    m_ids = sorted({int(md.geom_dataid[i]) for i in visible_geoms if md.geom_type[i]==mujoco.mjtGeom.mjGEOM_MESH})
    assert len(b_ids) == 27 and len(m_ids) == 30

    mesh_data = []
    for m_id in m_ids:
        filename = mujoco.mj_id2name(md, mujoco.mjtObj.mjOBJ_MESH, m_id)
        assert '/' not in filename and filename.endswith('.stl')
        mesh_data.append({
            'id': m_id,
            'filename': filename,
            'url': UPSTREAM + filename,
            # MuJoCo's compiled mesh vertices satisfy
            # v_internal = inverse(mesh_quat) * (v_stl - mesh_pos).
            'mesh_pos': fseq(md.mesh_pos[m_id], 9),
            'mesh_quat_wxyz': fseq(md.mesh_quat[m_id], 9),
        })

    geom_data = []
    for g_id in visible_geoms:
        typ = mujoco.mjtGeom(md.geom_type[g_id])
        if typ not in [mujoco.mjtGeom.mjGEOM_MESH,mujoco.mjtGeom.mjGEOM_BOX,mujoco.mjtGeom.mjGEOM_CYLINDER]:
            raise RuntimeError(f'Unexpected visible geometry type: {typ}')
        mat_id = int(md.geom_matid[g_id])
        rgba = md.mat_rgba[mat_id] * md.geom_rgba[g_id] if mat_id >= 0 else md.geom_rgba[g_id]
        geom_data.append({
            'name': mujoco.mj_id2name(md,mujoco.mjtObj.mjOBJ_GEOM,g_id) or 'unnamed',
            'body': int(md.geom_bodyid[g_id]),
            'kind': {mujoco.mjtGeom.mjGEOM_MESH:'mesh',mujoco.mjtGeom.mjGEOM_BOX:'box',mujoco.mjtGeom.mjGEOM_CYLINDER:'cylinder'}[typ],
            'mesh': int(md.geom_dataid[g_id]) if typ == mujoco.mjtGeom.mjGEOM_MESH else -1,
            'pos': fseq(md.geom_pos[g_id],9),
            'quat_wxyz': fseq(md.geom_quat[g_id],9),
            'size': fseq(md.geom_size[g_id],8),
            'rgba': fseq(rgba,5)
        })

    frames = []
    for sample in samples:
        data.qpos[:] = np.asarray(sample['qpos'],dtype=float)
        data.qvel[:] = 0
        mujoco.mj_forward(md,data)
        line=[]
        for i in b_ids:
            line.extend(fseq(data.xpos[i], 7))
            line.extend(fseq(data.xquat[i], 8))
        frames.append(line)

    first_base=frames[0][b_ids.index(1)*7:b_ids.index(1)*7+7]
    last_base=frames[-1][b_ids.index(1)*7:b_ids.index(1)*7+7]
    result={
        'format':FORMAT,
        'source':{'github':'MattiaGrigoli/autonomous_hexapod','commit':COMMIT,'owner':'Freenove',
                  'third_party_notice':'https://github.com/MattiaGrigoli/autonomous_hexapod/blob/main/LICENSE-3RD-PARTY.md',
                  'asset_strategy':'reference original upstream assets by pinned commit, do not redistribute binary assets'},
        'viewer_notice':'Original CAD meshes: Freenove; linked to public source provided by Mattia Grigoli with permission. This site is not affiliated with or endorsed by Freenove.',
        'missing_meshes':['topplate.stl','bigservo1.stl'],
        'fallback_visuals':'19 MJCF boxes replace the missing source meshes; additional boxes/cylinders reflect original local simulation additions',
        'mesh_count':len(mesh_data), 'visible_geoms':len(geom_data),
        'bodies':[{'id':id,'name':mujoco.mj_id2name(md,mujoco.mjtObj.mjOBJ_BODY,id)} for id in b_ids],
        'meshes':mesh_data, 'geoms':geom_data,
        'duration_s':60,'samples':len(frames), 'sample_interval_s':0.04,
        'frame_stride':len(b_ids)*7,
        'pose_layout':'each body in bodies[]: world xyz (3), quaternion wxyz (4); every 40 ms',
        'source_qpos_sha256':qpos_sha,
        'sample_base0':first_base,'sample_base_last':last_base,
        'frames':frames
    }
    return result


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--model',default='/mnt/data/hexapod-run/model/source/mjcf/scene_local30_nominal.xml')
    ap.add_argument('--recording',default='/mnt/data/HexapodLab_Tripod_60s_2026-11-04_no_CAD.zip')
    ap.add_argument('--output',default='/mnt/data/hexapod-site/docs/hexapod/data/cad-rig-60s.json')
    args=ap.parse_args()
    recording,h=load_recording(pathlib.Path(args.recording))
    result=export(pathlib.Path(args.model),recording,h)
    out=pathlib.Path(args.output);out.parent.mkdir(parents=True,exist_ok=True)
    out.write_text(json.dumps(result,separators=(',',':'),ensure_ascii=False),encoding='utf-8')
    print(json.dumps({'output':str(out),'bytes':out.stat().st_size,'frames':len(result['frames']),'bodies':len(result['bodies']),'meshes':len(result['meshes']),'visual_geoms':len(result['geoms']),'sha256':hashlib.sha256(out.read_bytes()).hexdigest()},ensure_ascii=False))

if __name__=='__main__':main()