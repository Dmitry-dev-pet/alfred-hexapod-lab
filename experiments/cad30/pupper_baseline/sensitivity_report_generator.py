"""Recheck math point-contact LP for altered friction/torque at stored virtual poses."""
import json,sys,collections
from pathlib import Path
import numpy as np
from math_first_four_crawl_v1 import ModelStudy
here=Path(__file__).resolve().parent
trace=json.loads((here/'math_first_four_crawl_trace.json').read_text())['nodes']
study=ModelStudy(here/'source/mjcf/scene_local30_nominal.xml',trace[0]['qpos'])
cases=[]
for mu,cap in ((1.1,.7),(1.1,.63),(1.1,.5),(1.1,.45),(.8,.7),(.6,.7),(.3,.7),(.8,.5),(.3,.63)):
 passing=0;fail=[];max_tau=0.
 for i,n in enumerate(trace):
  study.set_pose(np.array(n['qpos']))
  w=study.wrench(tuple(n['grounded']),mu=mu,tau_cap=cap)
  if not w['pass']:fail.append({'index':i,'phase':n['phase']})
  else:passing+=1;max_tau=max(max_tau,w['min_peak_tau_Nm'])
 cases.append({'mu':mu,'torque_cap_Nm':cap,'passing_nodes':passing,'failing_nodes':len(fail),'max_required_tau_among_passing_Nm':max_tau,'first_failures':fail[:10]})
nonfloor=[];unwanted_floor=[]
for i,n in enumerate(trace):
 study.set_pose(np.array(n['qpos']))
 for j in range(study.data.ncon):
  c=study.data.contact[j];gn=[study.model.geom(int(x)).name for x in (c.geom1,c.geom2)]
  if 'floor' not in gn:nonfloor.append({'index':i,'names':gn,'dist':float(c.dist)})
  elif not any('tibia'+k in ''.join(gn) for k in ('ml','mr','bl','br','fl','fr')):unwanted_floor.append({'index':i,'names':gn,'dist':float(c.dist)})
report={'schema':'cad30-math-only-sensitivity-v1','cases':cases,'unexpected_nonfloor_contact_count':len(nonfloor),'unexpected_nontibia_floor_contact_count':len(unwanted_floor),'limitations':'Collision checks are mj_forward at sampled virtual qpos, not mj_step dynamics. Model contact at toes can differ from real contact.'}
(here/'math_first_four_crawl_sensitivity.json').write_text(json.dumps(report,indent=2,ensure_ascii=False)+'\n')
print(json.dumps(report,indent=2,ensure_ascii=False))
