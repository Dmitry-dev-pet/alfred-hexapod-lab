"""Feedback correction of unintended unloaded rear after first prep swing.

Models only physically measured qpos/qvel; applies bounded joint increments to
PREVIOUS motor target, not a target reset. Tests multiple push distances from
exactly the same physical abort state. Nominal simulation, not hardware.
"""
from mink_contact_preflight import *
from pathlib import Path
import numpy as np,mink,mujoco as mj,json
from scipy.spatial import ConvexHull

def run_case(ctx,plan,base,push_mm,seconds=.8):
 m=ctx.model
 m.opt.timestep=.002
 m.geom_friction[m.geom('floor').id,0]=1.1
 for k in (*STANCE,'fl','fr'):m.geom_friction[m.geom(f'tibia{k}_collision_0').id,0]=1.1
 q=np.array(base['last_qpos']);v=np.array(base['last_qvel'])
 d=mj.MjData(m);d.qpos[:]=q;d.qvel[:]=v;mj.mj_forward(m,d)
 goalQ=np.asarray(plan['prep_waypoints'][23]['qpos'])
 jids=np.asarray(m.actuator_trnid[:,0]);qa=m.jnt_qposadr[jids].astype(int);da=m.jnt_dofadr[jids].astype(int)
 cap=m.actuator_ctrlrange[:,1]; original_goal=goalQ[qa].copy();final_goal=original_goal.copy()
 unloaded='br';site=PREFIX+unloaded
 if push_mm<0 or push_mm>3.0:raise ValueError('Exploratory push must be 0..3 mm')
 if push_mm>0:
  config=mink.Configuration(m);config.update(q)
  tasks=[]; targets={leg:config.get_transform_frame_to_world(PREFIX+leg,'site').as_matrix() for leg in STANCE}
  for leg in STANCE:
   T=targets[leg].copy()
   if leg==unloaded:T[2,3]-=push_mm/1000.
   task=mink.FrameTask(PREFIX+leg,'site',position_cost=10.,orientation_cost=0.)
   task.set_target(mink.SE3.from_matrix(T));tasks.append(task)
  pose=mink.PostureTask(m,cost=1e-5);pose.set_target_from_configuration(config)
  for i in range(70):
   dv=mink.solve_ik(config,tasks+[pose],dt=1.,solver='daqp',damping=1e-8,limits=ctx.limits,constraints=ctx.freeze)
   config.update(config.integrate(dv,dt=1.))
   err=max(np.linalg.norm(config.get_transform_frame_to_world(PREFIX+leg,'site').translation() - (targets[leg][:3,3] - np.array([0.,0.,push_mm/1000.])*(leg==unloaded))) for leg in STANCE)
   if err<.0001:break
  if err>.0002:return {'push_mm':push_mm,'status':'IK_FAILED','err_mm':err*1000}
  for part in ('coxa','femur','tibia'):
   j=int(m.body(part+unloaded).jntadr[0]);qidx=int(m.jnt_qposadr[j]);idx=int(np.flatnonzero(qa==qidx)[0])
   final_goal[idx]+=config.q[qidx]-q[qidx]
 n=round(seconds/.002);samples=[];minimum=4;min_margin=1e9;max_tau=0;maxq=0;sat=0
 floor=int(m.geom('floor').id);geom={int(m.geom(f'tibia{k}_collision_0').id):k for k in STANCE}
 for i in range(n):
  a=min(1.,(i+1)/125);a=a*a*(3-2*a);target=original_goal+(final_goal-original_goal)*a
  u=7.*(target-d.qpos[qa])-.32*d.qvel[da]
  sat+=int(np.any(np.abs(u)>cap+1e-9));d.ctrl[:]=np.clip(u,-cap,cap);mj.mj_step(m,d)
  fz={k:0. for k in STANCE};pts={k:[] for k in STANCE}
  for ci in range(d.ncon):
   c=d.contact[ci];g=int(c.geom2) if int(c.geom1)==floor else int(c.geom1) if int(c.geom2)==floor else -1
   leg=geom.get(g)
   if leg is not None:
    w=np.zeros(6);mj.mj_contactForce(m,d,ci,w);ff=c.frame.reshape(3,3).T@w[:3]
    fz[leg]+=float(ff[2]);pts[leg].append((c.pos.copy(),max(float(ff[2]),1e-8)))
  loaded=[k for k in STANCE if fz[k]>.08]
  minimum=min(minimum,len(loaded));maxq=max(maxq,float(np.max(np.abs(d.qpos[ctx.joint_qpos_addrs]))));max_tau=max(max_tau,float(max(abs(d.ctrl))))
  if len(loaded)>=3:
   pp=np.array([np.average([p for p,f in pts[k]],axis=0,weights=[f for p,f in pts[k]])[:2] for k in loaded]);h=ConvexHull(pp).equations
   mar=float(np.min(-(h[:,:2]@com_xyz(m,d)[:2]+h[:,2]))*1e3)
  else:mar=float('-inf')
  min_margin=min(min_margin,mar)
  samples.append({'loaded':loaded,'fz_N':fz})
 tail=samples[-100:];sustained=all(len(s['loaded'])==4 for s in tail)
 final=samples[-1]
 return {'push_mm':push_mm,'status':'DYNAMIC_RECOVERY_PASS' if sustained and minimum>=3 and min_margin>0 and maxq<=1.700001 and sat==0 and min(final['fz_N'].values())>=.2 else 'DYNAMIC_RECOVERY_VETO',
         'loaded_end':final['loaded'],'end_fz_N':final['fz_N'],'min_loaded_all':minimum,'min_margin_mm':min_margin,'max_joint_rad':maxq,'max_tau_Nm':max_tau,'saturations':sat,'tail_0p2_s_all_four':sustained,
         'qpos_end':d.qpos.copy().tolist(),'qvel_end':d.qvel.copy().tolist(),'actuator_goal_end':final_goal.tolist()}

def run_batch(ctx, plan, physical_result, pushes=(0,.5,1.,1.5,2.,3.)):
    if physical_result['status']!='DYNAMIC_CHECK_ABORTED':
        raise ValueError('Recovery must start from a measured stopped physics state')
    if 'bl_touchdown_4' not in physical_result['stopped_reason']:
        raise ValueError('Only this physically measured failed BL prep step is analyzed')
    results={str(p):run_case(ctx,plan,physical_result,p) for p in pushes}
    return {'schema':'cad30_prep_contact_recovery_v1','status':'FEEDBACK_LOCAL_SUCCESS_ONLY',
            'model':'nominal MuJoCo 3.14, mu=1.1, research joint cap 1.7 rad',
            'cases':results,'autonomous_followup_authorized':False,
            'full_cycle_dynamic_validated':False,
            'warning':'Recovered contact at one static step; full future plan must be regenerated from measured qpos/qvel.'}

def main():
    import argparse
    parser=argparse.ArgumentParser()
    parser.add_argument('--model',type=Path,required=True)
    parser.add_argument('--state',type=Path,required=True)
    parser.add_argument('--plan',type=Path,required=True)
    parser.add_argument('--stopped-dynamic',type=Path,required=True)
    parser.add_argument('--out',type=Path)
    args=parser.parse_args()
    ctx=load_context(args.model,args.state)
    result=run_batch(ctx,json.loads(args.plan.read_text()),json.loads(args.stopped_dynamic.read_text()))
    if args.out:args.out.parent.mkdir(parents=True,exist_ok=True);args.out.write_text(json.dumps(result,indent=2)+'\n')
    compact={k:{kk:vv for kk,vv in x.items() if kk not in ('qpos_end','qvel_end','actuator_goal_end')}
             for k,x in result['cases'].items()}
    print(json.dumps({'schema':result['schema'],'cases':compact,'full_cycle_dynamic_validated':False},indent=2))
if __name__=='__main__':main()