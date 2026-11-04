"""Build a compact experiment receipt and two numerical figures from prior runs."""
from pathlib import Path
import json, math, hashlib
import numpy as np
import mujoco
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from optimize_four_leg_pose_dynamics import run, ROOT, ROOTDIR, RAW
from optimize_four_leg_middle_liftoff import run as middle_run
from front_pair_trick import SUPPORT,contact_points

OUT=ROOTDIR/'results'
def compact(r):
    return {k:v for k,v in r.items() if k not in ('trace','qpos_frames')}

def m_score(r):return 1000.*0.5*(r['final_leave_one_foot_margins_m']['ml']+r['final_leave_one_foot_margins_m']['mr'])

base=run(0,0,0,0,seconds=4)
safe=run(0,-.09,.02,0,seconds=4)
aggressive=run(0,-.095,.02,0,seconds=4)
three=middle_run(.07,-.12,.045,8,record=True)
three_frames=three.pop('qpos_frames')
three_trace=three.pop('trace')
(OUT/'optimized_body_middle_lift_physics_trace_2026-11-04.json').write_text(json.dumps({'simulation':'FNK0052 CAD30 nominal','scenario':three['params'],'trace':three_trace,'qpos_frames':three_frames},indent=2)+'\n')
friction=json.loads((OUT/'body_pose_friction_grid_2026-11-04.json').read_text())
coarse=json.loads((OUT/'body_pose_dynamics_nominal_grid_2026-11-04.json').read_text())['runs']
foc=json.loads((OUT/'body_pose_focused_grid_2026-11-04.json').read_text())['trials']
kin=json.loads((OUT/'optimized_pose_workspace_grid_2026-11-04.json').read_text())
tri=json.loads((OUT/'three_support_ik_grid_2026-11-04.json').read_text())
all_runs=coarse+foc
valid=[r for r in all_runs if r['valid_four_contact_unsaturated']]
best=max(valid,key=m_score)
strong=[r for r in valid if r['peak_tau_nm']<=.55]
best_robust=max(strong,key=m_score)
assert best['requested_body_delta_m']==[0,-.095,.02]
assert best_robust['requested_body_delta_m']==[0,-.09,.02]
assert three['hold_last_middle_mesh_min_z']>.02 and three['saturation_fraction']>.01
summary={
 'schema':'fnk0052_CAD30_optimized_four_leg_posture/1',
 'date':'2026-11-04',
 'method':'fixed-foot IK grid -> free-base physics trial -> support-three feasibility and dynamic middle-foot attempt',
 'hardware_measured':False,
 'source_CAD':'community MJCF with 30 pinned Freenove STL; private asset cache required; 2 visual proxies',
 'baseline_qpos_provenance':json.loads(Path(RAW).read_text())['source_run_sha256'],
 'models':'MuJoCo 3.14.0; nominal mass 1.804868 kg; 18 standard joint actuators plus head; PD Kp7 Kd0.32; approximate torque cap ±0.7 Nm',
 'kinematics':{'grid_poses':kin['trial_count'],'four_foot_IK_with_error_lt4mm':kin['feasible_count'],'three_support_grid_poses':tri['attempts'],'three_support_IK_feasible':tri['feasible'],'three_support_positive_margin':tri['positive_margin']},
 'physics_screen':{'coarse_test_count':len(coarse),'focused_test_count':len(foc),'nominal_friction':1.1,'valid_4_contact_count':len(valid),
    'screen_best_geometric_of_valid':compact(best),'screen_best_with_tau_le_0p55_nm':compact(best_robust)},
 'actual_reference_physics':{'baseline':compact(base),'more_rearward_bounded_4_contact':compact(safe),'more_rearward_near_cap_4_contact':compact(aggressive)},
 'friction_stress':[compact(r)|{'mu':r['mu']} for r in friction],
 'middle_lift_diagnostic':compact(three),
 'conclusion':'Support body pose reduces center-to-triangle deficit, but cannot satisfy all leave-one-foot triangles with a single static COM point; a 3-leg phase was obtained with torque saturation and cannot count as an accepted full gait',
 'gate':'no complete 4-legged gait; middle-foot step has not safely landed and repeated; no actual hardware',
 'limitations':['sampled grids not global optimum','reference high-guard snapshot with qvel zero, rather than complete walking trial','rigid body approximate CAD mass friction and noncalibrated RC motors','margin from MuJoCo solved contacts, not guaranteed static robustness under dynamic acceleration','torsion/temperature/current/PWM and mesh self-collision verification missing']
}
(OUT/'optimized_four_leg_posture_summary_2026-11-04.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
# Figure 1: all 28 physical pose tests, torque saturation emphasis and winning trade-off
fig,ax=plt.subplots(figsize=(8.2,5))
xy=[(m_score(r),r['peak_tau_nm']) for r in all_runs]
ax.scatter([x for x,y in xy],[y for x,y in xy],label='all 28 pose trials',alpha=.60)
ax.scatter([m_score(r) for r in valid],[r['peak_tau_nm'] for r in valid],label='four contacts, no torque saturation',marker='o',s=42)
ax.scatter([m_score(safe)],[safe['peak_tau_nm']],marker='*',s=180,label='selected pose: -90 mm Y, +20 mm Z')
ax.scatter([m_score(aggressive)],[aggressive['peak_tau_nm']],marker='D',s=58,label='aggressive pose: -95 mm Y')
ax.axhline(.55,linestyle='--',linewidth=1,label='chosen 0.55 Nm screening budget')
ax.axvline(0,linestyle=':',linewidth=1)
ax.set_xlabel('COM margin without one middle foot (mm; >=0 required)')
ax.set_ylabel('Maximum simulated joint moment (Nm)')
ax.set_title('Freenove CAD30: body-pose search under assumed 0.7 Nm cap')
ax.legend(loc='lower right',fontsize=7)
ax.grid(alpha=.2);fig.tight_layout()
fig.savefig(OUT/'optimized_four_leg_pose_tradeoff_2026-11-04.png',dpi=150);plt.close(fig)
# Figure 2: support footprint/center of mass
m=mujoco.MjModel.from_xml_path(ROOT);d=mujoco.MjData(m);d.qpos[:]=json.loads(Path(RAW).read_text())['qpos'];mujoco.mj_forward(m,d);pts=contact_points(m,d);cent={c:np.mean(pts[c],axis=0) for c in SUPPORT}
fig,ax=plt.subplots(figsize=(7.2,5.8))
for c in SUPPORT:
 xy=cent[c];ax.scatter([xy[0]],[xy[1]],s=65)
 ax.annotate(c,xy+np.array([.008,.009]))
for c in ['ml','mr']:
 poly=np.array([cent[z] for z in SUPPORT if z!=c]+[cent[next(z for z in SUPPORT if z!=c)]])
 ax.plot(poly[:,0],poly[:,1],linestyle='--',linewidth=1,label=f'3 supports if {c} lifts')
for name,r,marker in [('initial',base,'o'),('selected guard',safe,'s'),('saturated middle swing',{'final_COM_m':three['hold_last_COM']},'*')]:
 xy=np.array(r['final_COM_m'][:2]);ax.scatter([xy[0]],[xy[1]],marker=marker,s=100,label=name)
 ax.annotate(name,xy+np.array([.006,-.015]),fontsize=8)
ax.set_xlabel('world X (m)');ax.set_ylabel('world Y (m)')
ax.set_title('Four support feet and simulated COM poses (top view)')
ax.set_aspect('equal');ax.grid(alpha=.2);ax.legend(fontsize=8,loc='lower right');fig.tight_layout()
fig.savefig(OUT/'optimized_four_leg_support_geometry_2026-11-04.png',dpi=150);plt.close(fig)
print('Saved summary, figures:',summary['kinematics'],summary['physics_screen']['valid_4_contact_count'])
print('Baseline COM',base['final_COM_m'],'margin',round(m_score(base),1),'optimized',safe['final_COM_m'],round(m_score(safe),1),'tau',safe['peak_tau_nm'])
print('Middle lift',three['hold_last_middle_mesh_min_z'],three['hold_last_com_margin_m'],'sat',three['saturation_fraction'])
