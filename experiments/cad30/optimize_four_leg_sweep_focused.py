import json
from optimize_four_leg_pose_dynamics import run,ROOTDIR
configs=[(0,-.085,.02,0),(0,-.09,.02,0),(0,-.095,.02,0),(0,-.09,.025,0),(0,-.10,.03,0),(0,-.09,.03,-3),(0,-.085,.025,-3),(0,-.085,.03,0),(0,-.075,.02,-3),(0,-.095,.03,3)]
res=[]
for x in configs:
 r=run(*x,seconds=4);r.pop('trace',None);r['mean_middle_margin_m']=.5*(r['final_leave_one_foot_margins_m']['ml']+r['final_leave_one_foot_margins_m']['mr']);res.append(r)
 print(x,'actual y',r['final_COM_m'][1],'margin',round(1000*r['mean_middle_margin_m'],2),'tilt',r['final_tilt_deg'],'torque',r['peak_tau_nm'],'sat',r['saturation_step_fraction'],'valid',r['valid_four_contact_unsaturated'],flush=True)
res.sort(key=lambda x:x['mean_middle_margin_m'],reverse=True)
open(ROOTDIR/'results/body_pose_focused_grid_2026-11-04.json','w').write(json.dumps({'run_count':len(res),'trials':res,'best_valid':next((x for x in res if x['valid_four_contact_unsaturated']),None)},indent=2)+'\n')
