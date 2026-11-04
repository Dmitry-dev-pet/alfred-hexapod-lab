import json,math,time
from optimize_four_leg_pose_dynamics import run,ROOTDIR
configs=[(0,-.02,0,0),(0,-.04,0,0),(0,-.06,0,0),(0,-.08,0,0),(0,-.04,.02,0),(0,-.06,.02,0),(0,-.08,.02,0),(0,-.04,.02,6),(0,-.06,.02,6),(0,-.06,.04,0),(0,-.06,.04,6),(0,-.04,.04,0),(0,-.04,.04,6),(0,-.04,.04,-6),(0,-.06,.02,-6),(0,-.08,.02,-6),(0,-.06,.04,-6),(0,-.08,.04,0)]
rows=[]
for config in configs:
 t0=time.time();r=run(*config,seconds=3.5);r.pop('trace',None);r['run_wall_s']=round(time.time()-t0,2)
 r['mean_middle_margin_m']=.5*(r['final_leave_one_foot_margins_m']['ml']+r['final_leave_one_foot_margins_m']['mr'])
 rows.append(r)
 print('cfg',config,'actual_COM',r['final_COM_m'][:2],'MARGIN_mm',round(1000*r['mean_middle_margin_m'],2),'tilt',r['final_tilt_deg'],'tau',r['peak_tau_nm'],'sat',r['saturation_step_fraction'],'valid',r['valid_four_contact_unsaturated'],'front_clear',min(r['front_min_clearance_last_m'].values()),flush=True)
rows.sort(key=lambda r:r['mean_middle_margin_m'],reverse=True)
open(ROOTDIR/'results/body_pose_dynamics_nominal_grid_2026-11-04.json','w').write(json.dumps({'description':'Nominal CAD30 MuJoCo physics, 18 preselected body-pose setpoints, 3.5 s from high-front-guard snapshot; no torque-speed realism','n':len(rows),'best_valid':next((r for r in rows if r['valid_four_contact_unsaturated']),None),'runs':rows},indent=2)+'\n')
