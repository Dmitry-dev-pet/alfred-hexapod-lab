#!/usr/bin/env python3
"""Independent original-CAD audit and causal handoff checks of saved rollouts."""
import argparse,contextlib,copy,hashlib,io,json,math,sys
from pathlib import Path
import numpy as np
import mujoco as mj
import probe as sim
import verify_results as legacy_audit
from verify_source import verify as verify_helpers

class CheckedDistances:
    """Bound a signed-distance result by exact mesh-vertex/box separation.

    A negative GJK result cannot imply deeper overlap than the translation
    along a world axis that completely separates the actual vertex bounds.
    This only audits saved states; the physics model and gates are untouched.
    """
    def __init__(self):self.corrections=[]
    def __getattr__(self,name):return getattr(mj,name)
    def mj_geomDistance(self,m,d,g,step,limit,segment):
        raw=float(mj.mj_geomDistance(m,d,g,step,limit,segment))
        if raw>=0:return raw
        mid=m.geom_dataid[g];start,n=m.mesh_vertadr[mid],m.mesh_vertnum[mid]
        points=m.mesh_vert[start:start+n].astype(float)@d.geom_xmat[g].reshape(3,3).T+d.geom_xpos[g]
        lower,upper=points.min(axis=0),points.max(axis=0)
        assert m.geom_type[step]==mj.mjtGeom.mjGEOM_BOX
        assert np.max(abs(d.geom_xmat[step].reshape(3,3)-np.eye(3)))<1e-12
        bmin,bmax=d.geom_xpos[step]-m.geom_size[step],d.geom_xpos[step]+m.geom_size[step]
        separation=float(np.max(np.maximum(bmin-upper,lower-bmax)))
        upper_bound=float(np.min(np.minimum(upper-bmin,bmax-lower)))
        corrected=separation if separation>=0 else -min(-raw,max(0.,upper_bound))
        if abs(corrected-raw)>1e-7:
            self.corrections.append({'raw_distance_m':raw,'bounded_distance_m':corrected,
                                     'geom_id':int(g),'vertex_lower':lower.tolist(),'vertex_upper':upper.tolist()})
        return corrected

distances=CheckedDistances()
legacy_audit.mj=distances
check_trace=legacy_audit.check_trace

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--workspace',type=Path,required=True)
    ap.add_argument('--traces',type=Path,required=True);ap.add_argument('--repeat',type=Path)
    ap.add_argument('--baseline-traces',type=Path)
    a=ap.parse_args()
    with contextlib.redirect_stdout(io.StringIO()):gate=sim.initialize(a.workspace.resolve())
    helpers=verify_helpers();summary=json.loads((a.traces/'summary.json').read_text())
    assert summary['code_sha256']==sim.sha(Path(sim.__file__))
    audits=[];corruption=[];baselines=[];count=0;cache={}
    for e in summary['episodes']:
        p=a.traces/e['trace_file'];assert sim.sha(p)==e['trace_sha256']
        tr=json.loads(p.read_text());assert tr['summary']=={k:v for k,v in e.items() if k not in ['trace_file','trace_sha256']}
        h=e['params']['height_m'];sim.E.EDGE=e['params']['edge_x_m']
        if h not in cache:
            with contextlib.redirect_stdout(io.StringIO()):cache[h]=sim.BUILD(sim.IP,h)[:2]
        m,d=cache[h];audit=check_trace(tr,e,m,d);count+=len(tr['frames'])
        assert e['physical_steps']==len(tr['frames'])*40
        if e['handoff_state'] is not None:
            hs=e['handoff_state'];assert abs(hs['t']-e['handback_s'])<1e-8
            if e['handoff_reason']=='angle':assert hs['forward_rotation_deg']>=e['params']['angle_deg']
            if e['handoff_reason']=='one_foot_top':assert any(v>.05 for v in hs['previous_step_loads_n'].values())
            before=[f for f in tr['frames'] if f['t']<hs['t']-1e-8 and f['t']>e['trigger_s']+1e-8]
            assert all(f['phase']=='roll' for f in before)
        weights=[f['blend_weight'] for f in tr['frames'] if f['phase']=='blend']
        assert all(0<=w<=1 for w in weights) and weights==sorted(weights)
        event=e['max_limit_event']
        if event:assert abs(event['overshoot_rad']-e['max_motor_limit_overshoot_rad'])<1e-12
        audit.update(handoff_reason=e['handoff_reason'],handoff_s=e['handback_s'],sampled_frames=len(tr['frames']))
        audits.append(audit)
        if a.baseline_traces and e['params']['handoff_kind']=='timer' and not e['params'].get('head_guard_margin_rad') and not e['params']['seed']:
            oldname='search-v30-g03-d20.json' if h==.01 else 'height-2cm.json'
            old=json.loads((a.baseline_traces/oldname).read_text())
            assert len(old['frames'])==len(tr['frames'])
            assert all(x['qpos']==y['qpos'] and x['qvel']==y['qvel'] and x['motor_target_rad']==y['motor_target_rad'] for x,y in zip(old['frames'],tr['frames']))
            baselines.append({'trace_file':p.name,'reference_trace_file':oldname,'all_states_and_targets_identical':True})
        if not corruption:
            for key in ['sole_geometry','illegal_target']:
                bad=copy.deepcopy(tr)
                if key=='sole_geometry':bad['frames'][0]['foot_bounds']['left']['min'][0]+=.02
                else:bad['frames'][0]['motor_target_rad'][0]+=10
                try:check_trace(bad,e,m,d)
                except AssertionError:corruption.append(key)
                else:raise AssertionError('Corrupt trace accepted')
        print(json.dumps({'trace':p.name,'goal':audit['independent_goal'],'quality':audit['quality_gate'],
                          'visible_overlap_mm':round(1000*audit['all_visible_mesh_max_step_overlap_m'],4)}),flush=True)
    repeats=[]
    if a.repeat:
        r=json.loads((a.repeat/'summary.json').read_text())
        for e in r['episodes']:
            assert (a.repeat/e['trace_file']).read_bytes()==(a.traces/e['trace_file']).read_bytes()
            repeats.append({'trace_file':e['trace_file'],'sha256':e['trace_sha256'],'byte_identical':True})
    result={'schema':'microduck-state-handoff-audit/v1','source_gate':gate,'helpers':helpers,'audits':audits,
            'frames_checked':count,'baselines':baselines,'byte_repeats':repeats,'corruption_rejected':corruption,
            'physics_steps':sum(e['physical_steps'] for e in summary['episodes']),
            'model_mass_kg':float(next(iter(cache.values()))[0].body_mass.sum()),
            'code_sha256':sim.sha(Path(sim.__file__)),'summary_sha256':sim.sha(a.traces/'summary.json'),
            'signed_distance_bound_corrections':distances.corrections,
            'hardware_tested':False,'policy_weights_changed':False}
    (a.traces/'verification.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({'verified':len(audits),'frames':count,'repeats':len(repeats),'original_baselines':len(baselines)}))

if __name__=='__main__':main()
