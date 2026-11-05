import json,sys,time
from pathlib import Path
import numpy as np
ROOT=Path(__file__).parent
sys.path.insert(0,str(ROOT))
from check_period_ik import Check,WALK
r=json.load(open(ROOT/'search_2d_results.json'));rows=r['all_candidates']
distinct={}
for a in rows:
 key=tuple(a['adjustments_mm'].values())
 if key not in distinct:distinct[key]=a
check=Check();t=time.time();records=[]
for k, a in distinct.items():
 try:
  chk=check.m;chk.set_pose(check.original_q)
  target={k:np.array(a['start_foot_xy_m'][k]+[0.]) for k in WALK}
  root0=check.original_root+(np.asarray(a['COM0_xy_m'])-check.original_com)
  q,err=chk.ik(root0,target,check.original_q,anchored=WALK)
  chk.set_pose(q)
  truecom=chk.com()[:2]
  margin=chk.margin(chk.toes(),WALK)
  w=chk.wrench(WALK)
  record={'adjustments_mm':a['adjustments_mm'],'max_toe_error_mm':max(err.values()),'max_abs_joint':float(np.max(abs(q[7:25]))),'initial_true_com_xy':truecom.tolist(),'expected_com_xy':a['COM0_xy_m'],'initial_margin_mm':margin,'lp_pass':w['pass'],'tau':w.get('min_peak_tau_Nm',None),'2d_score':a['rank_score_m'],'best_order':a['order']}
  record['start_pass']=record['max_toe_error_mm']<=1 and record['max_abs_joint']<=1.7 and margin>=10 and w['pass'] and record['tau']<=.7
 except Exception as e:record={'adjustments_mm':a['adjustments_mm'],'start_pass':False,'exception':repr(e)}
 records.append(record)
 print('checked',len(records), 'startpass',sum(x['start_pass'] for x in records),flush=True) if len(records)%18==0 else None
records.sort(key=lambda x:(not x['start_pass'], x.get('max_toe_error_mm',float('inf'))))
out={'total':len(records),'start_pass':sum(x['start_pass'] for x in records),'seconds':time.time()-t,'results':records}
(ROOT/'search_stance_results.json').write_text(json.dumps(out,indent=2,ensure_ascii=False))
print(json.dumps({'total':out['total'],'pass':out['start_pass'],'best':records[:10],'seconds':out['seconds']},indent=2)[:8000])