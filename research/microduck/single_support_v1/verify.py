#!/usr/bin/env python3
"""Independently reconstruct support from contact records and original CAD states."""
import argparse, contextlib, copy, hashlib, io, json, math
from pathlib import Path
import mujoco as mj
import numpy as np
import probe

def digest(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def longest(values,dt=.02):
    best=run=0
    for v in values:
        run=run+1 if v else 0;best=max(best,run)
    return round(best*dt,6)

def inspect_frame(m,d,f,feet,floor,mg):
    assert len(f['qpos'])==m.nq and len(f['qvel'])==m.nv
    assert np.isfinite(f['qpos']).all() and np.isfinite(f['qvel']).all()
    d.qpos[:]=f['qpos'];d.qvel[:]=f['qvel'];mj.mj_forward(m,d)
    assert np.max(abs(d.subtree_com[1]-f['com_m']))<2e-8
    loads={s:0. for s in feet};other_load=0.
    for ct in f['contacts']:
        pair=ct['geoms'];assert len(pair)==2 and all(0<=i<m.ngeom for i in pair)
        if floor not in pair:continue
        other=pair[1] if pair[0]==floor else pair[0]
        if other not in feet.values():other_load+=max(0.,ct['force_n'])
        if abs(ct['normal'][2])>.85:
            for s,g in feet.items():
                if g in pair:loads[s]+=max(0.,ct['force_n'])
    assert abs(other_load-f['nonfoot_environment_load_n'])<1e-8
    flags={};quiet={}
    for side,g in feet.items():
        swing='right' if side=='left' else 'left'
        bounds=probe.base.E.foot_bounds(m,d,g)
        for key in ['min','max']:
            assert np.max(abs(np.array(bounds[key])-f['foot_bounds'][side][key]))<2e-8
        assert abs(loads[side]-f['foot_loads_n'][side])<1e-8
        # Recomputed sole clearance is used, not the trace's claimed clearance.
        sg=feet[swing];swing_z=probe.base.E.foot_bounds(m,d,sg)['min'][2]
        flags[side]=bool(loads[side]>.8*mg and loads[swing]<.02 and swing_z>=.005 and other_load<.02)
        tilt=math.degrees(math.acos(float(np.clip(d.xmat[1].reshape(3,3)[2,2],-1,1))))
        quiet[side]=flags[side] and tilt<20 and np.linalg.norm(d.qvel[:3])<.06 and np.linalg.norm(d.qvel[3:6])<.7
    return flags,quiet

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--workspace',type=Path,required=True)
    ap.add_argument('--traces',type=Path,required=True);ap.add_argument('--repeat',type=Path)
    a=ap.parse_args()
    with contextlib.redirect_stdout(io.StringIO()):
        probe.base.initialize(a.workspace.resolve());m,d,*_=probe.base.BUILD(probe.base.IP,0)
    assert m.nq==35 and m.nu==14 and sum(m.geom_group==2)==70
    summary=json.loads((a.traces/'summary.json').read_text());assert summary['code_sha256']==digest(probe.__file__)
    feet={s:mj.mj_name2id(m,mj.mjtObj.mjOBJ_GEOM,s+'_foot_collision') for s in ['left','right']}
    floor=mj.mj_name2id(m,mj.mjtObj.mjOBJ_GEOM,'floor');mg=float(m.body_mass.sum()*9.81)
    joints=m.actuator_trnid[:,0];results=[];frames_checked=0;tamper_source=None
    for entry in summary['episodes']:
        path=a.traces/entry['trace_file'];assert digest(path)==entry['trace_sha256']
        tr=json.loads(path.read_text());assert tr['terrain']['height_m']==0
        single={s:[] for s in feet};quiet={s:[] for s in feet}
        selected=entry['params'].get('support','left')
        for f in tr['frames']:
            flags,q=inspect_frame(m,d,f,feet,floor,mg)
            assert flags[selected]==f['single_support']
            assert q[selected]==f['quiet_single_support']
            target=np.array(f['motor_target_rad'])
            assert np.all(target>=m.jnt_range[joints,0]+.07-1e-12)
            assert np.all(target<=m.jnt_range[joints,1]-.07+1e-12)
            for s in feet:single[s].append(flags[s]);quiet[s].append(q[s])
            frames_checked+=1
            if tamper_source is None and f['foot_loads_n']['left']>1:tamper_source=copy.deepcopy(f)
        dur={s:longest(single[s]) for s in feet};stable={s:longest(quiet[s]) for s in feet}
        assert dur[selected]==entry['longest_single_support_s']
        assert stable[selected]==entry['longest_quiet_single_support_s']
        assert entry['physical_steps']==len(tr['frames'])*round(tr['control_dt']/tr['dt'])
        results.append({'trace_file':path.name,'longest_single_support_s':dur,
                        'longest_quiet_single_support_s':stable,'quality_gate':entry['quality_gate'],
                        'status':entry['status'],'params':entry['params']})
    rejected=[]
    for name in ['sole_bound','foot_load']:
        bad=copy.deepcopy(tamper_source)
        if name=='sole_bound':bad['foot_bounds']['left']['min'][2]+=.02
        else:bad['foot_loads_n']['left']+=1
        try:inspect_frame(m,d,bad,feet,floor,mg)
        except AssertionError:rejected.append(name)
        else:raise AssertionError('Corrupt trace accepted: '+name)
    repeats=[]
    if a.repeat:
        r=json.loads((a.repeat/'summary.json').read_text())
        for e in r['episodes']:
            other=next(x for x in summary['episodes'] if x['trace_file']==e['trace_file'])
            assert (a.repeat/e['trace_file']).read_bytes()==(a.traces/e['trace_file']).read_bytes()
            repeats.append({'trace_file':e['trace_file'],'byte_identical':True,'sha256':other['trace_sha256']})
    out={'schema':'microduck-single-support-verification/v1','frames_checked':frames_checked,
         'sample_period_s':.02,'cad_instances':70,'mass_kg':float(m.body_mass.sum()),
         'source_code_sha256':digest(probe.__file__),'summary_sha256':digest(a.traces/'summary.json'),
         'corruption_tests_rejected':rejected,'repeats':repeats,'episodes':results,
         'measurement_limit':'Support durations are consecutive 50 Hz observations, not substep-continuous contact guarantees.',
         'hardware_tested':False}
    (a.traces/'verification.json').write_text(json.dumps(out,indent=2)+'\n')
    print(json.dumps({'episodes':len(results),'frames_checked':frames_checked,'corruption_tests':rejected,'repeats':len(repeats)}))

if __name__=='__main__':main()
