"""Publish unchanged archived v4 visual records; never rerun physical simulation."""
from __future__ import annotations
import argparse,hashlib,json,math,shutil,zipfile
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
ARCHIVE_SHA='43aea694c3704a59ad659674650e77268131f58ff1203e3e1e8b965a875dbd8d'
STEP='9156683a0bb694c84dc541a3e26b5f6b08db1a41'
FIRMWARE='c5c69e515d7d2a22edc3779667fc3edfe4045ae9'
ROUTE='hexapod/en01-jump-v4/'
ASSETS='/assets/robot-replays/en01-jump-v4/'
def sha(b):return hashlib.sha256(b).hexdigest()

def main():
    p=argparse.ArgumentParser();p.add_argument('--archive',type=Path,required=True);p.add_argument('--site-dist',type=Path,required=True);a=p.parse_args()
    raw=a.archive.read_bytes()
    if sha(raw)!=ARCHIVE_SHA:raise ValueError('Wrong source archive')
    clips=[];rig=None;frames=0;total=0
    assets=a.site_dist/ASSETS.lstrip('/');assets.mkdir(parents=True,exist_ok=True)
    with zipfile.ZipFile(a.archive) as z:
        if len(z.namelist())!=len(set(z.namelist())):raise ValueError('Duplicate archive member')
        for name,m in json.loads(z.read('MANIFEST.json')).items():
            b=z.read(name)
            if len(b)!=m['bytes'] or sha(b)!=m['sha256']:raise ValueError('Archive member mismatch')
        raws={sha(z.read(n)):z.read(n) for n in z.namelist() if n.startswith('raw/') and n.endswith('_raw.json')}
        names=sorted(n for n in z.namelist() if n.startswith('replay/') and n.endswith('.json'))
        if len(names)!=24 or len(raws)!=24:raise ValueError('Missing physical recording')
        for name in names:
            b=z.read(name);d=json.loads(b);r=json.loads(raws[d['source']['trace_sha256']]);s=r['summary']
            if d['result']!=s or d['body_names']!=r['body_names'] or len(d['frames'])!=300 or len(r['frames'])!=300:raise ValueError('Wrong source binding')
            if s['experiment']!='source_jump_v4' or s['source_firmware_blob']!=FIRMWARE or d['source']['step_git_blob_sha1']!=STEP or d['source']['model_sha256']!=s['model_sha256']:raise ValueError('Wrong model/firmware provenance')
            if s['jump_pass']:raise ValueError('Unexpected successful landing; review the source classification')
            candidate={k:d[k] for k in ['body_names','rest_body_positions','part_body_map','cad_asset']}
            if rig is None:rig=candidate
            if rig!=candidate or len(rig['body_names'])!=9 or len(rig['part_body_map'])!=73:raise ValueError('Changed original visual rig')
            prev=-1
            for f,v in zip(d['frames'],r['frames']):
                expected=[v['t'],*[x for p in v['body_poses'] for x in p],int(v['ground_contact']),int(v['nonwheel_ground_contact']),v['ground_normal_force_n'],v['pitch_rad'],v['roll_rad'],v['closure_error_m']]
                if f!=expected or len(f)!=70 or not all(math.isfinite(x) for x in f) or f[0]<=prev:raise ValueError('Changed physical frame')
                for j in range(9):
                    if abs(math.sqrt(sum(x*x for x in f[4+j*7:8+j*7]))-1)>1e-8:raise ValueError('Bad body rotation')
                prev=f[0];frames+=1
            if abs(prev-5.981)>1e-8 or s['physics_steps']!=6000:raise ValueError('Incomplete trace')
            stem=Path(name).stem;case=stem.removeprefix('en01-source-jump-v4-').removesuffix('-20261109');digest=sha(b);filename=stem+'-'+digest[:12]+'.json'
            (assets/filename).write_bytes(b);total+=len(b)
            kind='jump' if s['source_jump_enabled'] else 'baseline'
            expected_status='failed' if kind=='jump' or not s['upright_6s'] else 'research'
            if d['status']!=expected_status:raise ValueError('Wrong diagnostic label')
            clips.append({'id':'v4-'+case,'version':'v4','case':case,'kind':kind,'mode':s['mode'],'loop_ms':s['source_loop_period_ms_ASSUMED'],'anchor_q':s['cad_anchor_q_at_height38_rad_ASSUMED'],
                'url':ASSETS+filename,'size':len(b),'sha256':digest,'trace_sha256':d['source']['trace_sha256'],'model_sha256':d['source']['model_sha256'],'status':d['status'],
                'max_abs_x_m':s['max_abs_x_m'],'max_abs_pitch_deg':s['max_abs_pitch_deg'],'qualified_takeoff':s['qualified_flight'],'clearance_mm':1000*s['max_min_wheel_clearance_m'],
                'flight_ms':1000*max([0]+[f['duration_s'] for f in s['flights'] if f['qualified']]),'landing_s':s['stable_landing_s'],'upright_6s':s['upright_6s']})
    cad=ROOT/'docs'/rig['cad_asset'];cad_bytes=cad.read_bytes()
    index={'schema':'en01-source-jump-recordings/v1','duration_s':6,'frame_count':300,'frame_stride':70,'step_git_blob_sha1':STEP,'firmware_blob':FIRMWARE,
        'source_archive_sha256':ARCHIVE_SHA,'cad_sha256':sha(cad_bytes),'cad_size':len(cad_bytes),'rig':rig,'default_clip':'v4-a-zero-p5-jump','clips':clips}
    route=ROOT/'docs'/ROUTE;route.mkdir(parents=True,exist_ok=True);(route/'replays.json').write_text(json.dumps(index,ensure_ascii=False,separators=(',',':'),allow_nan=False)+'\n')
    files=[]
    for path in sorted(route.iterdir()):
        if path.suffix not in ('.html','.js','.css','.json'):continue
        b=path.read_bytes();files.append({'path':ROUTE+path.name,'size':len(b),'sha256':sha(b)});dest=a.site_dist/ROUTE/path.name;dest.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(path,dest)
    dest=a.site_dist/rig['cad_asset'];dest.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(cad,dest)
    catalog_path=ROOT/'docs/hexapod/library.json';catalog=json.loads(catalog_path.read_text());old=catalog['entries'][:]
    entry={'id':'en01-jump-v4','title':'EN01 · авторский прыжок · 24 записи','detail':'12 попыток и 12 контрольных стоек · есть короткий отрыв · устойчивой посадки нет · время импульса и начальная поза — предположения','duration_s':6,'href':'/'+ROUTE,'status':'research','files':files}
    exists=next((e for e in old if e['id']==entry['id']),None)
    if exists is None:catalog['entries'].append(entry);catalog['revision']+=1
    elif exists!=entry:raise ValueError('Immutable catalog registration changed')
    if any(e not in catalog['entries'] for e in old):raise ValueError('Old entry changed')
    catalog_path.write_text(json.dumps(catalog,ensure_ascii=False,indent=2)+'\n');shutil.copyfile(catalog_path,a.site_dist/'hexapod/library.json')
    receipt={'clips':len(clips),'source_frames_exact':frames,'source_frame_max_residual':0,'hosted_replay_bytes':total,'cad_sha256':sha(cad_bytes),'old_entries':len([e for e in old if e['id']!=entry['id']]),'entries':len(catalog['entries']),'revision':catalog['revision'],'physics_rerun':False}
    (ROOT/'docs/EN01_JUMP_V4_PUBLICATION_VERIFICATION.json').write_text(json.dumps(receipt,indent=2)+'\n');print(json.dumps(receipt,indent=2))

if __name__=='__main__':main()
