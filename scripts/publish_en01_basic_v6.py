"""Publish verified v6 baseline replays unchanged; physics is never recalculated here."""
from __future__ import annotations
import argparse,hashlib,json,math,shutil,zipfile
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
ROUTE='hexapod/en01-basic-v6/'
ASSETS='/assets/robot-replays/en01-basic-v6/'
STEP='9156683a0bb694c84dc541a3e26b5f6b08db1a41'
CASES={'stand-low','stand-mid','drive-forward','drive-backward','turn-left-low','turn-right-low','turn-left-mid','turn-left-high','squat-cycle','squat-turn-left','squat-turn-right','drive-turn-left'}
LABELS={'stand-low':'Низкая стойка','stand-mid':'Средняя стойка','drive-forward':'Езда вперёд','drive-backward':'Езда назад','turn-left-low':'Поворот влево · низкая стойка','turn-right-low':'Поворот вправо · низкая стойка','turn-left-mid':'Поворот · средняя стойка','turn-left-high':'Поворот · высокая стойка','squat-cycle':'Приседание и подъём','squat-turn-left':'Приседание с поворотом влево','squat-turn-right':'Приседание с поворотом вправо','drive-turn-left':'Езда по дуге'}
def sha(b):return hashlib.sha256(b).hexdigest()

def main():
 p=argparse.ArgumentParser();p.add_argument('--archive',type=Path,required=True);p.add_argument('--archive-sha256',required=True);p.add_argument('--site-dist',type=Path,required=True);a=p.parse_args()
 raw=a.archive.read_bytes()
 if sha(raw)!=a.archive_sha256:raise ValueError('Wrong source archive')
 clips=[];rig=None;frames=total=0;assets=a.site_dist/ASSETS.lstrip('/');assets.mkdir(parents=True,exist_ok=True)
 with zipfile.ZipFile(a.archive) as z:
  if len(z.namelist())!=len(set(z.namelist())):raise ValueError('Duplicate archive member')
  for name,meta in json.loads(z.read('MANIFEST.json')).items():
   b=z.read(name)
   if len(b)!=meta['bytes'] or sha(b)!=meta['sha256']:raise ValueError('Archive member mismatch')
  names={Path(n).stem for n in z.namelist() if n.startswith('replay/') and n.endswith('.json')}
  if names!=CASES:raise ValueError('Incomplete v6 battery')
  for name in sorted(names):
   b=z.read('replay/'+name+'.json');d=json.loads(b);rraw=z.read('raw/'+name+'_raw.json');r=json.loads(rraw);s=r['summary']
   if d['source']['trace_sha256']!=sha(rraw) or d['result']!=s or d['body_names']!=r['body_names']:raise ValueError('Wrong raw trace binding')
   if s['experiment']!='basic_motion_v6' or s['model_sha256']!=sha(z.read('model.xml')) or d['source']['step_git_blob_sha1']!=STEP:raise ValueError('Wrong model identity')
   check=d['basic_motion']
   if not check['geometry_ok'] or check['hardware_motion_claim_allowed'] or check['cad_surface_audit']['unexpected_intersection_frames'] or check['cad_surface_audit']['saved_frames']!=300:raise ValueError('Missing CAD validation')
   if min(s['hip_min_all_steps_rad'])<0 or max(s['hip_max_all_steps_rad'])>.65 or s['max_self_proxy_penetration_m']>1e-6:raise ValueError('Invalid folding or proxy penetration')
   candidate={k:d[k] for k in ['body_names','rest_body_positions','part_body_map','cad_asset']}
   if rig is None:rig=candidate
   if candidate!=rig or len(rig['body_names'])!=9 or len(rig['part_body_map'])!=73:raise ValueError('Changed original rig')
   if len(d['frames'])!=300 or len(r['frames'])!=300:raise ValueError('Incomplete frames')
   for f,v in zip(d['frames'],r['frames']):
    expected=[v['t'],*[x for pose in v['body_poses'] for x in pose],int(v['ground_contact']),int(v['nonwheel_ground_contact']),v['ground_normal_force_n'],v['pitch_rad'],v['roll_rad'],v['closure_error_m']]
    if f!=expected or len(f)!=70 or not all(math.isfinite(x) for x in f):raise ValueError('Changed physical frame')
    frames+=1
   if abs(d['frames'][-1][0]-(5.98+s['timestep_s']))>1e-8 or s['physics_steps']!=round(6/s['timestep_s']):raise ValueError('Incomplete trace')
   digest=sha(b);filename='en01-basic-v6-'+name+'-'+digest[:12]+'.json';(assets/filename).write_bytes(b);total+=len(b)
   clips.append({'id':'v6-'+name,'case':name,'label':LABELS[name],'mode':s['mode'],'timestep_s':s['timestep_s'],'physics_steps':s['physics_steps'],'url':ASSETS+filename,'size':len(b),'sha256':digest,'trace_sha256':sha(rraw),'model_sha256':s['model_sha256'],'status':d['status'],'max_abs_x_m':s['max_abs_x_m'],'max_abs_pitch_deg':s['max_abs_pitch_deg'],'basic_pass':s['basic_pass'],'checks':s['checks'],'metrics':s['metrics'],'geometry_ok':check['geometry_ok']})
 cad=ROOT/'docs'/rig['cad_asset'];cadb=cad.read_bytes()
 index={'schema':'en01-basic-motion-recordings/v1','duration_s':6,'frame_count':300,'frame_stride':70,'step_git_blob_sha1':STEP,'source_archive_sha256':a.archive_sha256,'cad_sha256':sha(cadb),'cad_size':len(cadb),'rig':rig,'default_clip':'v6-squat-turn-left','clips':clips}
 route=ROOT/'docs'/ROUTE;route.mkdir(parents=True,exist_ok=True);(route/'replays.json').write_text(json.dumps(index,ensure_ascii=False,separators=(',',':'))+'\n')
 files=[]
 for path in sorted(route.iterdir()):
  if path.suffix not in ('.html','.js','.css','.json'):continue
  b=path.read_bytes();files.append({'path':ROUTE+path.name,'size':len(b),'sha256':sha(b)});dest=a.site_dist/ROUTE/path.name;dest.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(path,dest)
 dest=a.site_dist/rig['cad_asset'];dest.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(cad,dest)
 catpath=ROOT/'docs/hexapod/library.json';cat=json.loads(catpath.read_bytes());old=cat['entries'][:]
 entry={'id':'en01-basic-v6','title':'EN01 · движение и приседания с поворотами · v6','detail':'12 базовых проверок · 3 пройдены · есть проблемы устойчивости и остановки · модель v5','duration_s':6,'href':'/'+ROUTE,'status':'research','files':files}
 exists=next((e for e in old if e['id']==entry['id']),None)
 if exists is None:cat['entries'].append(entry);cat['revision']+=1
 elif exists!=entry:raise ValueError('Immutable catalog registration changed')
 if any(e not in cat['entries'] for e in old):raise ValueError('Old entry changed')
 catpath.write_text(json.dumps(cat,ensure_ascii=False,indent=2)+'\n');shutil.copyfile(catpath,a.site_dist/'hexapod/library.json')
 receipt={'recordings':12,'physical_frames_exact':frames,'hosted_replay_bytes':total,'entries':len(cat['entries']),'revision':cat['revision'],'prior_entries_preserved':True,'source_archive_sha256':a.archive_sha256,'physics_rerun':False}
 (ROOT/'docs/EN01_BASIC_V6_PUBLICATION_VERIFICATION.json').write_text(json.dumps(receipt,indent=2)+'\n');print(json.dumps(receipt,indent=2))
if __name__=='__main__':main()
