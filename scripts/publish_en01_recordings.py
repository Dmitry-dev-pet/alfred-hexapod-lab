"""Verify archived physical records and package their unchanged visual replays."""
from __future__ import annotations
import argparse, hashlib, json, math, shutil, zipfile
from pathlib import Path

ARCHIVES = {
    'v2': ('EN01_v2_physics_traces_20261109.zip', '0147f6957ce97d0724f59f7006310401655aa27757a7f3c449a47c04c1bd77f1', 6),
    'v3': ('EN01_v3_position_feedback_20261109.zip', '5562ebd511ed3784e733531e70a4334848750997b6046b4f77d631360f34f550', 31),
}
ROOT = Path(__file__).resolve().parents[1]
ROUTE = 'hexapod/en01-v2-v3/'
PREFIX = '/assets/robot-replays/en01-v2-v3/'
PIN = '9156683a0bb694c84dc541a3e26b5f6b08db1a41'
MODES = {'stand':'Стойка', 'push':'Толчок', 'drive':'Езда', 'crouch':'Приседание', 'jump':'Попытка прыжка', 'fall':'Боковое падение'}

def digest(b): return hashlib.sha256(b).hexdigest()

def label(version, case, result):
    base = MODES[result['mode']]
    if version == 'v2': return base + ' · исходный контроллер'
    gain = str(result.get('position_gain_nm_per_m', 0)).replace('.', ',')
    variant = 'с отсечкой' if result.get('gated_controller') else ('сравнение' if case.startswith('paired-') else 'без отсечки')
    profile = {'mass-090':'масса −10%', 'mass-110':'масса +10%', 'friction-035':'трение 0,35'}.get(result.get('model_profile'), 'номинальная модель')
    return f'{base} · Kx {gain} · {variant} · {profile}'

def main():
    p = argparse.ArgumentParser(); p.add_argument('--archives', type=Path, required=True); p.add_argument('--site-dist', type=Path, required=True)
    args = p.parse_args(); clips=[]; rig=None; checks=0; total=0
    assets=args.site_dist / PREFIX.lstrip('/'); assets.mkdir(parents=True, exist_ok=True)
    for version, (name, sha, count) in ARCHIVES.items():
        payload=(args.archives/name).read_bytes()
        if digest(payload)!=sha: raise ValueError('Archive checksum mismatch: '+name)
        with zipfile.ZipFile(args.archives/name) as z:
            names=z.namelist()
            if len(names)!=len(set(names)): raise ValueError('Duplicate archive member')
            manifest=json.loads(z.read('MANIFEST.json'))
            for member, meta in manifest.items():
                data=z.read(member)
                if len(data)!=meta['bytes'] or digest(data)!=meta['sha256']: raise ValueError('Member checksum mismatch: '+member)
            replay_names=sorted(n for n in names if n.startswith('replay/') and n.endswith('.json'))
            if len(replay_names)!=count: raise ValueError('Wrong clip count')
            raws={digest(z.read(n)):z.read(n) for n in names if n.startswith('raw/') and n.endswith('_raw.json')}
            for member in replay_names:
                data=z.read(member); d=json.loads(data); raw=json.loads(raws[d['source']['trace_sha256']]); result=raw['summary']
                if d['result']!=result or d['body_names']!=raw['body_names']: raise ValueError('Source summary/layout mismatch')
                if d['source']['step_git_blob_sha1']!=PIN or d['source']['model_sha256']!=result['model_sha256']: raise ValueError('Wrong model source')
                if d['format']!='navbot-en01-fourbar-physical-replay-v2' or d['sample_stride']!=70 or d['frame_count']!=300 or len(d['frames'])!=300: raise ValueError('Wrong replay format')
                if len(raw['frames'])!=300 or len(d['body_names'])!=9 or len(d['part_body_map'])!=73: raise ValueError('Wrong physical layout')
                candidate={k:d[k] for k in ['body_names','rest_body_positions','part_body_map','cad_asset']}
                if rig is None: rig=candidate
                if candidate!=rig: raise ValueError('Unexpected visual rig change')
                previous=-1
                for a,b in zip(d['frames'],raw['frames']):
                    expected=[b['t'], *[v for pose in b['body_poses'] for v in pose], int(b['ground_contact']), int(b['nonwheel_ground_contact']), b['ground_normal_force_n'], b['pitch_rad'], b['roll_rad'], b['closure_error_m']]
                    if a!=expected or len(a)!=70 or not all(math.isfinite(v) for v in a) or a[0]<=previous: raise ValueError('Source-frame mismatch')
                    for i in range(9):
                        norm=math.sqrt(sum(v*v for v in a[4+7*i:8+7*i]))
                        if abs(norm-1)>1e-8: raise ValueError('Non-unit body rotation')
                    previous=a[0]; checks+=1
                if abs(previous-5.981)>1e-8 or result['physics_steps']!=6000: raise ValueError('Incomplete recording')
                expected_status='failed' if result['mode'] in ('jump','fall') or not result['upright_6s'] else 'research'
                if d['status']!=expected_status: raise ValueError('Misclassified recording')
                stem=Path(member).stem; case=stem.removeprefix('en01-fourbar-v2-' if version=='v2' else 'en01-v3-').removesuffix('-20261109')
                checksum=digest(data); filename=stem+'-'+checksum[:12]+'.json'
                (assets/filename).write_bytes(data); total+=len(data)
                clips.append({'id':version+'-'+case,'version':version,'case':case,'mode':result['mode'],'label':label(version,case,result),'url':PREFIX+filename,'size':len(data),'sha256':checksum,'trace_sha256':d['source']['trace_sha256'],'model_sha256':d['source']['model_sha256'],'status':d['status'],'max_abs_x_m':result['max_abs_x_m'],'max_abs_pitch_deg':result['max_abs_pitch_deg'],'upright_6s':result['upright_6s']})
    cad=ROOT/'docs/hexapod/en01-original-cad-v1/original-meshes.json'; cad_data=cad.read_bytes()
    index={'schema':'en01-recorded-experiments/v1','duration_s':6,'frame_count':300,'frame_stride':70,'step_git_blob_sha1':PIN,'cad_sha256':digest(cad_data),'cad_size':len(cad_data),'default_clip':'v3-gated-g30-stand','rig':rig,'archives':{v:{'filename':n,'sha256':s} for v,(n,s,c) in ARCHIVES.items()},'clips':clips}
    route=ROOT/'docs'/ROUTE; route.mkdir(parents=True, exist_ok=True)
    (route/'replays.json').write_text(json.dumps(index,ensure_ascii=False,allow_nan=False,separators=(',',':'))+'\n')
    files=[]
    for file in sorted(route.iterdir()):
        if file.suffix not in ('.html','.js','.css','.json'): continue
        b=file.read_bytes(); files.append({'path':ROUTE+file.name,'size':len(b),'sha256':digest(b)})
        target=args.site_dist/ROUTE/file.name; target.parent.mkdir(parents=True,exist_ok=True); shutil.copyfile(file,target)
    target=args.site_dist/rig['cad_asset']; target.parent.mkdir(parents=True,exist_ok=True); shutil.copyfile(cad,target)
    catalog_path=ROOT/'docs/hexapod/library.json'; catalog=json.loads(catalog_path.read_text()); old=catalog['entries'][:]
    entry={'id':'en01-v2-v3','title':'EN01 · v2 / v3 · 37 записей','detail':'6 опытов четырёхзвенной модели и 31 опыт удержания позиции · исходная CAD · приседание, прыжки и подъём пока FAIL','duration_s':6,'href':'/'+ROUTE,'status':'research','files':files}
    existing=next((e for e in old if e['id']==entry['id']),None)
    if existing is None: catalog['entries'].append(entry); catalog['revision']+=1
    elif existing!=entry: raise ValueError('Immutable registration conflict')
    for e in old:
        if e not in catalog['entries']: raise ValueError('Previous entry changed')
    catalog_path.write_text(json.dumps(catalog,ensure_ascii=False,indent=2)+'\n'); shutil.copyfile(catalog_path,args.site_dist/'hexapod/library.json')
    receipt={'clips':len(clips),'v2':6,'v3':31,'verified_source_frames':checks,'visual_pose_values':checks*63,'max_source_frame_residual':0,'hosted_replay_bytes':total,'cad_sha256':digest(cad_data),'previous_entries':len([e for e in old if e['id']!=entry['id']]),'catalog_entries':len(catalog['entries']),'catalog_revision':catalog['revision'],'physics_rerun':False}
    (ROOT/'docs/EN01_V2_V3_PUBLICATION_VERIFICATION.json').write_text(json.dumps(receipt,indent=2)+'\n')
    print(json.dumps(receipt,indent=2))

if __name__=='__main__': main()
