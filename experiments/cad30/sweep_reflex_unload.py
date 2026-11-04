"""Predefined contact-load reflex comparison for four-legged raised-front experiment.

All conditions chosen before running; no tuning after observing results.
No GitHub Actions and no sampled training states.
"""
import json, hashlib
from pathlib import Path
from reflex_unload_middle import experiment
HERE=Path(__file__).resolve().parent
TRIALS=[
    ('nominal',1.1,'ml','no_feedback'),
    ('nominal',1.1,'ml','press_only'),
    ('nominal',1.1,'ml','reflex'),
    ('nominal',1.1,'ml','forced_lift'),
    ('nominal',1.1,'mr','reflex'),
    ('lower',1.1,'ml','reflex'),
    ('upper',1.1,'ml','reflex'),
    ('nominal',0.6,'ml','reflex'),
    ('nominal',0.3,'ml','reflex'),
]
if __name__=='__main__':
    rows=[]
    for mass,mu,leg,mode in TRIALS:
        r=experiment(mass,mu,leg,mode,record=False)
        rows.append(r)
        print(json.dumps({k:v for k,v in r.items() if k in ('mode','mass','mu','candidate','complete','release_at_s','failure','reason','mean_candidate_load_final_1s_N','mean_oracle_margin_final_1s_m','max_press_mm','achieved_extra_forward_shift_m','saturation_fraction','min_support_contacts_after_request','front_any_contact_after_request')},ensure_ascii=False),flush=True)
    result={'experiment':'FNK0052 middle-foot load redistribution / local reflex v1',
         'date_utc':'2026-11-04','repro':'MuJoCo 3.14.0 / Python 3.13.5 / local in-session compute',
         'trial_plan':[{'mass':m,'friction_mu':fr,'candidate':leg,'mode':mode} for m,fr,leg,mode in TRIALS],
         'unmeasured_variables':['servo torque-speed,current,temperature','physical friction','actual mass and COM','physical toe pressure sensor'],
         'caveat':'Only new middle-leg unloading uses sensor feedback; initial front-pair stance is scripted. Exact COM oracle used only for scoring.',
         'results':rows}
    filename=HERE/'results'/'reflex_middle_support_sweep_2026-11-04.json'
    filename.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    print('WROTE',filename,flush=True)
