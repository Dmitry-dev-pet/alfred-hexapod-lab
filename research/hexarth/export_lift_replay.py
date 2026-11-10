"""Reuse exact OEM mesh blocks and package the newly measured lift comparison."""
import argparse
import gzip
import hashlib
import json
from pathlib import Path
import sys


def export(baseline_package,results,output,source_commit,writer_directory,lifts=(20,22)):
    sys.path.insert(0,str(writer_directory))
    from robot_replay import block,write_package
    raw=baseline_package.read_bytes()
    assert hashlib.sha256(raw).hexdigest()=='1abc147874c9a48891d1c8f0e9d6816cc9b431070ce56d255af9db7948442393'
    doc=json.loads(gzip.decompress(raw))
    assert len(doc['nodes'])==len(doc['meshes'])==19
    clips=[]
    for lift in lifts:
        assert lift in (12,18,20,22)
        folder='lift-sweep' if lift in (12,18) else 'fine-sweep'
        name=f'lift-{lift}mm-step-20mm'
        trace_path=results/folder/(name+'-trace.json')
        trace=json.loads(trace_path.read_text())
        replay_path=results/folder/(name+'-replay.json')
        replay=json.loads(replay_path.read_text())
        assert doc['metadata']['source_step_sha256']==trace['source_step_sha256']
        assert hashlib.sha256(trace_path.read_bytes()).hexdigest()==replay['source_trace_sha256']
        frames=[[f['t']]+f['body_poses'] for f in trace['frames']]
        assert all(len(f)==134 for f in frames)
        assert all(f==[p[0]]+p[8:] for f,p in zip(frames,replay['frames']))
        m=trace['metrics'];crossed=trace['whole_robot_on_step']
        clips.append({'id':name,'title':f'Подъём {lift} мм · '+('ступень 2 см пройдена' if crossed else 'остановка перед ступенью'),
          'duration_s':24,'frame_count':len(frames),'frame_stride':134,'frames':block([v for f in frames for v in f]),
          'environment':doc['clips'][-1]['environment'],
          'result':'Все шесть ног на ступени' if crossed else 'Ступень не пройдена; равновесие сохранено',
          'metrics':m,'source_trace_sha256':replay['source_trace_sha256'],
          'source_replay_sha256':hashlib.sha256(replay_path.read_bytes()).hexdigest(),
          'contacts':{'labels':['foot'+str(i) for i in range(6)],'values':block([v for f in trace['frames'] for v in f['foot_normal_load_n']])}})
    doc['metadata']={'title':'HexArth · ступень 2 см: подбор подъёма ног',
      'description':'Подъём 12, 18 и 20 мм: остановка; 22 мм: все шесть ног на ступени 2 см. Те же ограничения суставов и приводов. Проверено повторным расчётом; модель не откалибрована на роботе.',
      'source_commit':source_commit,'source_step_sha256':doc['metadata']['source_step_sha256'],
      'source':'https://github.com/Dmitry-dev-pet/alfred-hexapod-lab/tree/'+source_commit+'/research/hexarth'}
    doc['clips']=clips;doc['default_clip']='lift-22mm-step-20mm'
    return write_package(doc,output)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--baseline-package',type=Path,required=True)
    p.add_argument('--results',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--source-commit',required=True);p.add_argument('--writer-directory',type=Path,required=True)
    p.add_argument('--lifts',nargs='+',type=int,default=[20,22])
    a=p.parse_args();print(json.dumps(export(a.baseline_package,a.results,a.output,a.source_commit,a.writer_directory,a.lifts)))
