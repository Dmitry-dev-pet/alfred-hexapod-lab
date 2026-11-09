"""Frozen manual-drive battery; preserves negative and repeat results."""
from pathlib import Path
import argparse,hashlib,json
import numpy as np
import mujoco
from direct_drive import run

def evaluate(s,effort,p):
    c=p['drive_criteria'];d=s['drive_displacement_2_to_10s_m']
    checks={'commanded_travel':bool(effort and np.sign(effort)*d[0]>=c['commanded_signed_x_at_least_m']),
            'cross_track':abs(d[1])<=c['absolute_cross_track_at_most_m'],
            'tilt':s['max_full_tilt_deg']<=c['max_full_chassis_tilt_at_most_deg'],
            'wheel_contact':min(s['wheel_contact_fraction'].values())>=c['each_wheel_contact_fraction_at_least'],
            'penetration':s['max_contact_penetration_m']<=c['max_contact_penetration_at_most_m'],
            'rolling':s['floor_rolling_residual_rms_m_s']<=c['floor_rolling_residual_rms_at_most_m_s'],
            'warnings':not any(s['warnings'])}
    stop=p['stop_criteria_separate']
    stopped=s['tail_speed_rms_m_s']<=stop['tail_speed_rms_at_most_m_s'] and max(s['tail_position_range_m'])<=stop['each_tail_position_range_at_most_m'] and s['final_speed_m_s']<=stop['final_speed_at_most_m_s']
    return {'drive_checks':checks,'first_translation_passed':all(checks.values()),'stop_passed':stopped}

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--baseline',type=Path,required=True);ap.add_argument('--out',type=Path,required=True);ap.add_argument('--allow-estimated-mass',action='store_true')
    a=ap.parse_args()
    if not a.allow_estimated_mass:ap.error('--allow-estimated-mass required')
    p=json.loads((Path(__file__).parent/'protocol.json').read_text());a.out.mkdir(parents=True,exist_ok=True)
    models={};initials={};metadata={};receipt={'schema':'alfred-bb8-first-trip-receipt/v4','hardware_calibrated':False,'protocol':p,'results':{},'model_sha256':{}}
    first_frames=None
    for profile,case,effort in p['validation_cases']:
        if profile not in models:
            xml=a.baseline/(profile+'-model.xml');models[profile]=mujoco.MjModel.from_xml_path(str(xml))
            prior=json.loads((a.baseline/('hold-'+profile+'.json')).read_text())
            initials[profile]=prior['frames'][0]['qpos'];metadata[profile]=prior['model']
            digest=hashlib.sha256(xml.read_bytes()).hexdigest();receipt['model_sha256'][profile]=digest
            (a.out/(profile+'-model.xml')).write_bytes(xml.read_bytes())
        knots=[[0,0,0],[2,0,0],[3,effort,effort],[9,effort,effort],[10,0,0],[14,0,0]]
        r=run(models[profile],initials[profile],knots)
        r.update(schema='alfred-bb8-manual-drive/v4',hardware_calibrated=False,head_in_dynamics=False,
                 profile=profile,case=case,engine_version=mujoco.__version__,knots=knots,
                 model_sha256=receipt['model_sha256'][profile],model_metadata=metadata[profile],
                 model_changes='NONE: archived v3 XML, all source-derived CAD contacts and inertias unchanged',
                 feedback='Manual prerecorded motor effort; physical motor back EMF only, no controller feedback')
        if case=='forward' and profile=='nominal':first_frames=r['frames']
        if case=='forward-repeat':
            if first_frames!=r['frames']:raise RuntimeError('Forward repeat frame stream differs')
            receipt['exact_forward_repeat']=True
        name=profile+'-'+case+'.json';text=json.dumps(r,separators=(',',':'))+'\n';(a.out/name).write_text(text)
        record={'summary':r['summary'],**evaluate(r['summary'],effort,p),'sha256':hashlib.sha256(text.encode()).hexdigest()}
        receipt['results'][name]=record
        (a.out/'receipt.json').write_text(json.dumps(receipt,indent=2)+'\n')
        print(json.dumps({'file':name,**record}),flush=True)

if __name__=='__main__':main()
