#!/usr/bin/env python3
"""Fixed-protocol higher-step sweep using the verified original rollout."""
from pathlib import Path
import contextlib,hashlib,io,json,os,sys
WORK=Path(os.environ['MICRODUCK_WORKSPACE']).resolve() if 'MICRODUCK_WORKSPACE' in os.environ else next((p for p in Path(__file__).resolve().parents if (p/'microduck-work/source-manifest.json').is_file()),Path.cwd())
sys.path.insert(0,str(WORK/'microduck-roll'))
import roll_cpu as e

def digest(b):return hashlib.sha256(b).hexdigest()

BASELINE_SHA='014e29bbd55ab5899eed6b851a02e0f85d390c8b559ece4998f1d6ca79c67758'

def baseline_bytes(out):
    for path in [out/'height5-firm-roll2s.json',WORK/'microduck-roll/traces/step12-firm-roll2s.json']:
        if path.is_file():
            raw=path.read_bytes()
            if digest(raw)!=BASELINE_SHA:raise ValueError('Unpinned baseline trace')
            return raw
    return None

def main():
    out=Path(__file__).parent/'traces';out.mkdir(exist_ok=True)
    mf=json.loads((WORK/'microduck-work/source-manifest.json').read_text());mf['_path']=str(WORK/'microduck-work/source-manifest.json');source=e.bench.source_gate(e.RL,e.BAM,mf)
    assert e.sha(e.ROLL)==e.ROLL_SHA and e.sha(e.STAND)==e.bench.POLICY_SHA256
    ip=e.bench.upstream_inference(e.RL);ip.ort.InferenceSession=e.bench.cpu_session;entries=[]
    for mm in [5,10,20,30,50,75,100]:
        if mm==5:
            raw=baseline_bytes(out)
            if raw is None:
                with contextlib.redirect_stdout(io.StringIO()):tr=e.run(ip,.005,.12,2.,True)
                raw=(json.dumps(tr,separators=(',',':'),allow_nan=False,sort_keys=True)+'\n').encode()
            else:tr=json.loads(raw)
            assert digest(raw)==BASELINE_SHA
        else:
            with contextlib.redirect_stdout(io.StringIO()):tr=e.run(ip,mm/1000,.12,2.,True)
            raw=(json.dumps(tr,separators=(',',':'),allow_nan=False,sort_keys=True)+'\n').encode()
        name=f'height{mm}-firm-roll2s.json';(out/name).write_bytes(raw)
        s=tr['summary'];entry={**s,'id':f'height{mm}-firm-roll2s','height_mm':mm,'trace_file':name,'trace_sha256':digest(raw),
            'initial_step_contact':any(82 in c['geoms'] and c['force_n']>.05 for f in tr['frames'] if f['t']<=1 for c in f['contacts'])}
        entry.update(strict_roll_status=entry['status'],status='PASS' if entry['step_goal_reached'] else 'FAIL');entries.append(entry);print(json.dumps(entry),flush=True)
    for mm in [20,30,50]:
        for cm in [8,18]:
            with contextlib.redirect_stdout(io.StringIO()):tr=e.run(ip,mm/1000,cm/100,2.,True)
            raw=(json.dumps(tr,separators=(',',':'),allow_nan=False,sort_keys=True)+'\n').encode();name=f'height{mm}-edge{cm}-firm-roll2s.json';(out/name).write_bytes(raw)
            entry={**tr['summary'],'id':Path(name).stem,'height_mm':mm,'trace_file':name,'trace_sha256':digest(raw),'initial_step_contact':any(82 in c['geoms'] and c['force_n']>.05 for f in tr['frames'] if f['t']<=1 for c in f['contacts'])};entry.update(strict_roll_status=entry['status'],status='PASS' if entry['step_goal_reached'] else 'FAIL');entries.append(entry);print(json.dumps(entry),flush=True)
    summary={'schema':'microduck-roll-heights/v1','source_gate':source,'fixed_edge_x_m':.12,'fixed_roll_timer_s':2,'fixed_contact_profile':'firm_1ms','episodes':entries}
    (out/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')

if __name__=='__main__':main()
