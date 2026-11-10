#!/usr/bin/env python3
"""Export recorded original-CAD poses to the existing GitHub library format."""
import argparse,base64,contextlib,gzip,hashlib,io,json,re,struct,sys
from pathlib import Path
import mujoco as mj
import numpy as np

ROOT=Path(__file__).resolve().parents[3]
ROUTE='hexapod/microduck-single-support-v1/'
PACK='e1a968eb41538d4f6760e480babf21d7fc3d4d9b4ae208297b01fe4c2a669e6b'
def sha(b):return hashlib.sha256(b).hexdigest()
def write(path,data):path.parent.mkdir(parents=True,exist_ok=True);path.write_bytes(data)
def jsonbytes(value):return (json.dumps(value,ensure_ascii=False,separators=(',',':'),sort_keys=True)+'\n').encode()

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--study-root',type=Path,required=True)
    ap.add_argument('--workspace',type=Path,required=True);ap.add_argument('--traces',type=Path,required=True)
    a=ap.parse_args();sys.path.insert(0,str(a.study_root.resolve()))
    import probe
    with contextlib.redirect_stdout(io.StringIO()):
        gate=probe.base.initialize(a.workspace.resolve());m,d,*_=probe.base.BUILD(probe.base.IP,0)
    source=(a.traces/'summary.json').read_bytes();summary=json.loads(source)
    verification=json.loads((a.traces/'verification.json').read_bytes())
    assert summary['code_sha256']==sha(Path(probe.__file__).read_bytes())
    assert verification['summary_sha256']==sha(source)
    visible=[g for g in range(m.ngeom) if m.geom_type[g]==mj.mjtGeom.mjGEOM_MESH and m.geom_group[g]==2]
    assert len(visible)==70
    ref=json.loads((ROOT/'docs/hexapod/microduck-step-goal-v1/replays.json').read_bytes())
    index={k:ref[k] for k in ['model_commit','model_package_sha256','stand_policy_sha256','collision_model_sha256','environment_mesh_count','frame_stride','visible_mesh_count']}
    index.update(schema='microduck-single-support-replays/v1',source_summary_sha256=sha(source),
                 source_verification_sha256=sha((a.traces/'verification.json').read_bytes()),
                 default_clip='walk030-seed20261109',clips=[])
    route=ROOT/'docs'/ROUTE;route.mkdir(parents=True,exist_ok=True)
    max_error=0.;total=0;exports=[]
    for e in summary['episodes']:
        raw=(a.traces/e['trace_file']).read_bytes();assert sha(raw)==e['trace_sha256'];tr=json.loads(raw)
        assert tr['terrain']['height_m']==0
        rows=[]
        for f in tr['frames']:
            d.qpos[:]=f['qpos'];mj.mj_forward(m,d)
            row=[f['t'],*d.qpos[:7]]
            for g in visible:
                q=np.empty(4);mj.mju_mat2Quat(q,d.geom_xmat[g]);row.extend(d.geom_xpos[g]);row.extend(q)
            rows.append(row)
        exact=np.asarray(rows);arr=np.asarray(rows,dtype='<f4')
        assert arr.shape==(len(tr['frames']),498)
        err=float(abs(exact-arr).max());max_error=max(max_error,err);assert err<1e-6
        track=struct.pack('<8sII',b'MDSSUPV1',len(rows),498)+arr.tobytes();th=sha(track)
        values=arr.view('<u4').copy();values[1:]=values[1:]^values[:-1]
        shuffled=track[:16]+values.view('u1').reshape(-1,4).T.copy().tobytes()
        encoded=gzip.compress(shuffled,compresslevel=9,mtime=0)
        record={'encoding':'gzip-xor-byte-shuffle-v1','sha256':sha(encoded),'decoded_size':len(track),'data':base64.b64encode(encoded).decode()}
        package=jsonbytes({'schema':'microduck-single-support-track-package/v1','tracks':{th:record}})
        ph=sha(package);filename='tracks-'+ph[:16]+'.json';assert len(package)<1_500_000
        write(route/filename,package)
        p=e['params'];cid=re.sub('[^a-z0-9-]','-',p['name'].lower())
        control=p.get('control','planned');group=control if control!='planned' else p['support']
        if control=='stand':label='Стойка на двух ногах · контроль'
        elif control=='walk':label='Ходьба · начальная поза '+str([20261109,20261110,20261111].index(p['seed'])+1)
        else:
            label=('Левая' if p['support']=='left' else 'Правая')+' опора · '
            label+=('обратная связь' if p.get('feedback') else 'перенос '+str(round(p['shift_fraction']*100))+'%'+(' · компенсация веса' if p.get('gravity_comp') else ''))
        phases=[]
        for f in tr['frames']:
            if not phases or phases[-1]['name']!=f['phase']:phases.append({'t':f['t'],'name':f['phase']})
        verified=next(v for v in verification['episodes'] if v['trace_file']==e['trace_file'])
        clip={k:v for k,v in e.items() if k not in ['plan','trace_file']}
        clip.update(id=cid,label=label,group=group,height_m=0.,duration_s=tr['frames'][-1]['t'],frames=len(rows),
                    url='/'+ROUTE+filename,size=len(encoded),sha256=sha(encoded),encoding=record['encoding'],
                    decoded_size=len(track),track_sha256=th,package_sha256=ph,package_size=len(package),
                    terrain={'height_m':0.,'edge_x_m':.045,'center_xy_m':[.645,0],'half_size_xy_m':[.6,.6]},
                    phase_events=phases,contact_profile='complete_env_05ms',
                    support_intervals_s=verified['longest_single_support_s'],
                    observations=[[f['tilt_deg'],f['foot_loads_n']['left'],f['foot_loads_n']['right'],
                                   f['foot_bounds']['left']['min'][2],f['foot_bounds']['right']['min'][2],
                                   int(f['single_support']),int(f['quiet_single_support'])] for f in tr['frames']])
        index['clips'].append(clip);total+=len(rows)
        exports.append({'id':cid,'frames':len(rows),'source_trace_sha256':e['trace_sha256'],'track_sha256':th,'max_float32_error':err})
    index['max_float32_absolute_error']=max_error;write(route/'replays.json',jsonbytes(index))
    files=[]
    for p in sorted(route.iterdir()):
        if p.suffix in ['.html','.js','.json','.css']:
            b=p.read_bytes();files.append({'path':ROUTE+p.name,'size':len(b),'sha256':sha(b)})
    assert len(files)==19
    entry={'id':'microduck-single-support-v1','title':'MicroDuck · равновесие на одной ноге',
           'detail':'14 записей · ходьба с короткой опорой на одну стопу · 10 неудачных попыток медленного переноса веса',
           'href':'/'+ROUTE,'duration_s':8.2,'status':'research','files':files}
    sys.path.insert(0,str(ROOT/'scripts'));from register_viewer_replays import register
    cp=ROOT/'docs/hexapod/library.json';old=json.loads(cp.read_bytes());new=register(old,[entry],ROOT/'docs')
    assert new['entries'][:len(old['entries'])]==old['entries']
    assert register(new,[entry],ROOT/'docs')==new
    write(cp,(json.dumps(new,ensure_ascii=False,indent=2)+'\n').encode())
    result={'schema':'microduck-single-support-export/v1','frames':total,'clips':len(exports),'visible_meshes':70,
            'all_source_hashes_checked':True,'physics_rerun':False,'source_gate':gate,'exports':exports,
            'max_float32_error':max_error,'catalog_revision':new['revision'],'catalog_entries':len(new['entries']),
            'previous_entries_preserved':True,'idempotent':True,'files':len(files)}
    write(Path(__file__).parent/'export-verification.json',(json.dumps(result,indent=2)+'\n').encode())
    print(json.dumps({k:v for k,v in result.items() if k not in ['exports','source_gate']}))

if __name__=='__main__':main()
