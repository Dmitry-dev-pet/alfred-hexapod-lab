"""Lossless data adapter for the already saved HexArth trial, with no physics rerun.

Input: directory containing rig.json, replays.json and the verified record blocks.
Output: one data-only .robotreplay package for the shared player.
No CAD or result bytes belong in this source repository.
"""
import argparse
import base64
import gzip
import hashlib
import json
import pathlib
import struct

from robot_replay import block, encoded_block, write_package


def verified(root, ref):
    path = (root/ref['path']).resolve()
    assert path.is_relative_to(root.resolve())
    raw = path.read_bytes()
    assert len(raw) == ref['size'] and hashlib.sha256(raw).hexdigest() == ref['sha256']
    return json.loads(raw)


def convert(root):
    index = json.loads((root/'replays.json').read_text())
    rig = verified(root, index['rig'])
    meshes=[]
    for part in rig['parts']:
        pv=verified(root,part['vertices_f32le_gzip_resource'])
        pi=verified(root,part['indices_u32le_gzip_resource'])
        positions=gzip.decompress(base64.b64decode(pv['data']))
        indices=gzip.decompress(base64.b64decode(pi['data']))
        # The source digest covers expanded STL vertices, not the unique table.
        assert len(positions)==part['vertex_count']*12 and len(indices)==part['triangle_count']*12
        expanded=hashlib.sha256()
        for (i,) in struct.iter_unpack('<I',indices):
            assert i<part['vertex_count']
            expanded.update(positions[i*12:i*12+12])
        assert expanded.hexdigest()==part['vertex_stream_sha256']
        body=part['body']
        color='#d4c9a7' if body=='base' else '#849ca2' if body.endswith('coxa') else '#e0b65a' if body.endswith('femur') else '#b4c9be'
        meshes.append({'node':rig['body_order'].index(body),'vertex_count':part['vertex_count'],
                       'triangle_count':part['triangle_count'],'scale':rig['unit_scale'],
                       'translation':part['local_translation_m'],
                       'material':{'color':color,'roughness':.52,'metalness':.22},
                       'positions':encoded_block(positions,'f32',part['vertex_count']*3),
                       'indices':encoded_block(indices,'u32',part['triangle_count']*3),
                       'source_stl_sha256':part['sha256']})
    clips=[]
    for c in index['clips']:
        record=verified(root,c['record']); replay=record['replay']
        assert replay['source_trace_sha256']==c['source_trace_sha256']
        frames=[[f[0]]+f[replay['layout']['visible_mesh_pose_start']:] for f in replay['frames']]
        stride=1+7*len(rig['body_order'])
        assert len(frames)==c['frame_count'] and all(len(f)==stride for f in frames)
        environment=[{'kind':'plane','size':[6,6,1],'position':[0,0,0],'color':'#2b4452'}]
        if c['terrain']['height_m']>0:
            environment.append({'kind':'box','size':[2*x for x in c['terrain']['half_size']],
                                'position':c['terrain']['center'],'color':'#617684'})
        clips.append({'id':c['id'],'title':c['label'],'duration_s':c['duration_s'],
                      'frame_count':len(frames),'frame_stride':stride,
                      'frames':block([v for f in frames for v in f]),
                      'environment':environment,'result':c['verdict'],'metrics':c['metrics'],
                      'source_trace_sha256':c['source_trace_sha256'],
                      'source_replay_sha256':record['source_replay_sha256'],
                      'contacts':{'labels':['foot'+str(i) for i in range(len(record['foot_normal_load_n'][0]))],
                                  'values':block([v for f in record['foot_normal_load_n'] for v in f])}})
    return {'schema':'robot-replay/v1','metadata':{'title':'HexArth · четыре опыта',
            'description':'Исходная CAD-геометрия и 3400 измеренных кадров MuJoCo. Массы, инерции и приводы оценочные; диапазон суставов диагностический. Ступень 2 см не пройдена. Пределы реального робота не установлены.',
            'source_commit':index['source_commit'],'source_step_sha256':rig['source_step_sha256'],
            'source':'https://github.com/Dmitry-dev-pet/alfred-hexapod-lab/pull/139'},
            'scene':{'units':'m','up_axis':'Z','background':'#142632',
                     'camera':{'follow_node':0,'offset':[.48,.34,.56]},
                     'lights':[{'kind':'hemisphere','color':'#d9f3ff','ground':'#5c6670','intensity':2.7,'position':[0,1,0]},
                               {'kind':'directional','color':'#ffffff','ground':'#000000','intensity':2,'position':[-1,2,1]}]},
            'nodes':[{'id':body} for body in rig['body_order']], 'meshes':meshes,
            'default_clip':index['default_clip'],'clips':clips}


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('input',type=pathlib.Path);p.add_argument('output',type=pathlib.Path)
    args=p.parse_args()
    print(json.dumps(write_package(convert(args.input),args.output),ensure_ascii=False))
