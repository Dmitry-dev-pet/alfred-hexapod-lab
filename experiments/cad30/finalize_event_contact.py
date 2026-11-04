"""Aggregate preselected held-out A/B CAD30 trials without dropping failures."""
from pathlib import Path
import json
import hashlib
import statistics
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

ROOT=Path(__file__).resolve().parents[2]
RESULTS=ROOT/'results'
SEEDS=(101,133,197)
MUS=(1.1,)

def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()

def main():
    pairs=[]
    for mu in MUS:
        for seed in SEEDS:
            stem=f'heldout_seed{seed}_mu{str(mu).replace(".","p")}_force'
            base=RESULTS/f'{stem}no.json'; force=RESULTS/f'{stem}yes.json'
            b=json.loads(base.read_text()); f=json.loads(force.read_text())
            for x in (b,f):
                assert x['seed']==seed and x['mu']==mu and x['finished']
                assert x['duration_s']==8 and x['motor_saturation_fraction']==0
            pairs.append({'seed':seed,'mu':mu,
                          'base':{k:b[k] for k in ('advance_cm','mean_target_force_abs_error_N','min_contacts_2ms','fraction_below3_2ms','fraction_negative_support_margin_sampled_50hz','peak_tau_nm','liftoffs','touchdowns')},
                          'force':{k:f[k] for k in ('advance_cm','mean_target_force_abs_error_N','min_contacts_2ms','fraction_below3_2ms','fraction_negative_support_margin_sampled_50hz','peak_tau_nm','liftoffs','touchdowns','max_abs_foot_z_offset_mm')},
                          'file_sha256':{'base':sha256(base),'force':sha256(force)}})
    metrics=('advance_cm','mean_target_force_abs_error_N','fraction_below3_2ms','fraction_negative_support_margin_sampled_50hz','peak_tau_nm','liftoffs')
    means={key:{arm:round(float(statistics.mean(pair[arm][key] for pair in pairs)),6)
                for arm in ('base','force')} for key in metrics}
    means['advance_gain_percent_of_group_means']=round(100*(means['advance_cm']['force']/means['advance_cm']['base']-1),2)
    means['force_target_mae_improvement_percent_of_group_means']=round(100*(1-means['mean_target_force_abs_error_N']['force']/means['mean_target_force_abs_error_N']['base']),2)
    report={'schema':'FNK0052_CAD30_event_math_optimizer_holdout/1',
            'date_utc':'2026-11-04','test_scope':'Three paired 8-second held-out initial seeds at assumed flat-ground mu1.1.',
            'model':'CAD30 30/32 original STL, 6 tibia convex mesh contacts; no validated real hardware',
            'physics':'MuJoCo 3.14.0, free base, 2ms physical step, ±0.7Nm assumed leg motors, nominal 1.804868kg',
            'planner':'event-based greedy contact choice, LP min-max torque, optional QP load redistribution, finite 20/35 mm IK touchdown choices; no fixed tripod/wave phase order',
            'not_mpc':True,'not_neurowalknet':True,'not_globally_optimal':True,
            'unchanged_between_arms':['6-leg CAD geometry and estimated masses','same seed','same μ','same bounds/IK','same contact scorer','same candidate enumeration','8-second duration'],
            'arm_delta':'Only whether the target-contact-normal-force error changes the bounded foot-z position setpoint.',
            'pairs':pairs,'means':means,'n_paired_seeds':len(pairs),
            'limitations':['3 paired seeds cannot support statistical generalization',
                           'Both arms show transient negative support polygon margins in some runs despite ≥3 contacts',
                           'Only mu=1.1 has completed paired force-tracking controls in this run; low-friction force-tracking outcomes are unavailable',
                           'Static LP witness does not guarantee safe dynamic rollout, touchdown or cycle recovery',
                           'Foot placement uses only two discrete IK trajectories; global optimality not shown',
                           'μ and torque curves, backlash, contact feet geometry and sensor noise are not calibrated',
                           'Motor/force controller uses simulator-perfect contact forces; no on-hardware evidence']}
    (RESULTS/'event_contact_holdout_summary_2026-11-04.json').write_text(json.dumps(report,indent=2,ensure_ascii=False)+'\n')
    lab=[str(p['seed']) for p in pairs];x=np.arange(len(pairs));width=.35
    for metric,ylabel,name in [
      ('advance_cm','Forward travel in 8s (cm)','event_contact_distance.png'),
      ('mean_target_force_abs_error_N','Mean absolute contact-force target error (N)','event_contact_force_mae.png'),
      ('fraction_below3_2ms','Fraction of physics steps with <3 contacts','event_contact_contact_loss.png')]:
        fig,ax=plt.subplots(figsize=(8.4,4.4))
        ax.bar(x-width/2,[p['base'][metric] for p in pairs],width,label='Math contact choice only')
        ax.bar(x+width/2,[p['force'][metric] for p in pairs],width,label='+ force feedback')
        ax.set_xticks(x,lab);ax.set_xlabel('Held-out initial seed (mu=1.1)')
        ax.set_ylabel(ylabel);ax.legend(loc='best')
        ax.grid(axis='y',alpha=.2);fig.tight_layout()
        fig.savefig(RESULTS/name,dpi=150);plt.close(fig)
    print(json.dumps(means,ensure_ascii=False,indent=2))

if __name__=='__main__':main()