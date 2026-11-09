"""Export actual measured states through the established nine-body CAD renderer."""
import argparse, hashlib, json, math, tempfile
from pathlib import Path
from .linkage_export import build
from .contact_mesh_audit import audit_record


def validate_outcome(record):
    """Reject success claims contradicted by independently inspected saved data.

    Full-step success is computed in the simulator. The coarser saved stream is
    an additional rejection gate, never evidence about unsaved substeps.
    """
    s=record['summary'];frames=record['frames']
    if len(frames)!=300 or s['physics_steps']!=24000:raise ValueError('Incomplete six-second trial')
    tail=[f for f in frames if f['t']>=4.]
    valid_tail=[f for f in tail if f['tilt_rad']<=math.radians(15) and
        min(f['wheel_normal_force_n'])>=.15 and f['nonwheel_normal_force_n']<=.05 and
        f['axle_speed_m_s']<=.03 and f['body_angular_speed_rad_s']<=.3]
    if s['getup_pass'] and (len(tail)!=100 or len(valid_tail)!=100 or not all(s['checks'].values())):
        raise ValueError('Claimed success contradicts measured support/posture/motion')
    for f in frames:
        if any(abs(f['ctrl'][i])>(.14709975 if i%2==0 else .03)+1e-12 for i in range(4)):
            raise ValueError('Actuator ceiling exceeded')
    return {'saved_final_frames':len(tail),'saved_final_stable_frames':len(valid_tail)}


def export(trace, out, *, model_path, mesh_path):
    record=json.loads(Path(trace).read_bytes());s=record['summary']
    if s.get('experiment')!='getup_v7':raise ValueError('Wrong experiment')
    validate_outcome(record)
    xml=Path(model_path).read_text()
    with tempfile.TemporaryDirectory() as td:
        path=Path(td)/'replay.json';receipt=build(trace,path,model_xml=xml);doc=json.loads(path.read_bytes())
    audit=audit_record(record,mesh_path,xml)
    geometry_ok=(audit['unexpected_intersection_frames']==0 and s['checks']['hip_bounds'] and
                 s['checks']['original_cad_floor_penetration'] and s['checks']['linkage_closure'])
    passed=s['getup_pass'] and geometry_ok
    doc['getup']={'version':7,'orientation':s['orientation'],'strategy':s['strategy'],
                  'geometry_ok':geometry_ok,'passed':passed,'cad_surface_audit':audit}
    doc['status']='research' if passed else 'failed'
    doc['limitations']=[
        'All original part outer hulls contact the floor; generic floor proxies disabled; v5 self contacts retained',
        'Same 0.322 kg assumed mass, 0..0.65 rad hip operating guard, 0.14709975 Nm hip and 0.03 Nm wheel ceilings',
        'Recovery commands are our bounded experiment, not original EN01 firmware',
        'Original floor vertices and self surfaces audited at saved frames; no continuous exact mesh certificate',
        'No hardware, servo-zero, inertia, tire-friction or thermal calibration; failures do not prove impossibility']
    doc['getup_telemetry']={'fields':['t','tilt_rad','left_load_n','right_load_n','nonwheel_load_n','axle_speed_m_s','angular_speed_rad_s','cad_min_z_m','stable'],
        'frames':[[f['t'],f['tilt_rad'],*f['wheel_normal_force_n'],f['nonwheel_normal_force_n'],f['axle_speed_m_s'],f['body_angular_speed_rad_s'],f['cad_floor_minimum_z_m'],int(f['stable_gate'])] for f in record['frames']]}
    payload=json.dumps(doc,allow_nan=False,separators=(',',':'))+'\n'
    out=Path(out);out.parent.mkdir(parents=True,exist_ok=True)
    if out.exists() and out.read_text()!=payload:raise ValueError('Immutable replay conflict')
    out.write_text(payload)
    return {**receipt,'status':doc['status'],'replay_sha256':hashlib.sha256(payload.encode()).hexdigest(),
            'geometry_ok':geometry_ok,'getup_pass':passed,'cad_surface_audit':audit}


if __name__=='__main__':
    p=argparse.ArgumentParser()
    for n in ('trace','out','model','cad-mesh'):p.add_argument('--'+n,type=Path,required=True)
    a=p.parse_args();print(json.dumps(export(a.trace,a.out,model_path=a.model,mesh_path=a.cad_mesh),indent=2))
