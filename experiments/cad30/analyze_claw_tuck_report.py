"""Additional metrics on recorded qpos: no re-simulation or fabricated motion."""
import json,sys
from pathlib import Path
import numpy as np,mujoco
ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT),str(ROOT/'experiments/cad30')]
from hexlab.stand_geometry_v04 import load_legs,tip_world,geom_mesh_world_vertices
from optimized_claw_gait import FRONT
SRC=ROOT/'results/claw_tuck_compare_2026-11-04.json'
M=mujoco.MjModel.from_xml_path(str(ROOT/'source/mjcf/scene_local30_nominal.xml'))
D=mujoco.MjData(M);legs=load_legs(M,D,mujoco)
by={l['suffix']:l for l in legs}
records=json.loads(SRC.read_text())
summary={}
for name,record in records.items():
    collision=[];footsep=[];toe_distance=[];frs=[]
    for s in record['qpos_samples_10hz']:
        if s['t']<6.8:continue
        D.qpos[:]=s['qpos'];D.qvel[:]=0;mujoco.mj_forward(M,D)
        dist=[]
        for c in FRONT:
            dist.append(float(mujoco.mj_geomDistance(M,D,M.geom('torso_simple_collision').id,M.geom('tibia'+c+'_collision_0').id,1.0,np.zeros(6))))
        collision.append(min(dist))
        footsep.append(float(mujoco.mj_geomDistance(M,D,M.geom('tibiafl_collision_0').id,M.geom('tibiafr_collision_0').id,1.0,np.zeros(6))))
        if s['t']>=9.29:
            for c in FRONT:
                leg=by[c];hip=D.xanchor[leg['jids'][0]];tip=tip_world(M,D,leg)
                toe_distance.append(float(np.linalg.norm((tip-hip)[:2])))
                frs.append({'code':c,'q':np.round(D.qpos[leg['qadrs']],6).tolist(),
                            'tip_xyz_m':np.round(tip,6).tolist(),'hip_to_toe_horizontal_m':float(np.linalg.norm((tip-hip)[:2]))})
    r={'pose':name,'final_COM_y_mm':round(record['after_tuck']['COM_xyz_m'][1]*1000,3),
       'final_COM_margin_mm':record['after_tuck']['COM_margin_four_mm'],
       'min_four_contacts_after_front_unload':record['min_support_four_after_unload'],
       'min_margin_during_fold_mm':record['min_support_margin_during_tuck_mm'],
       'max_tilt_during_fold_deg':record['max_tilt_during_tuck_deg'],
       'motor_saturation_fraction':record['saturated_steps_fraction'],
       'peak_unclipped_motor_torque_nm':record['max_control_torque_unclipped_nm'],
       'final_joint_margin_to_1p7_deg':record['after_tuck']['joint_limit_reserve_front_deg'],
       'min_torso_tibia_proxy_distance_mm':round(min(collision)*1000,3),
       'min_inter_tibia_proxy_distance_mm':round(min(footsep)*1000,3),
       'final_front_tibia_min_height_mm':record['after_tuck']['front_min_mesh_z_mm'],
       'final_front_toe_horizontal_dist_mm':round(float(np.mean(toe_distance))*1000,3),
       'final_front_legs':frs,'passed_strict_proxy_checks':record['strict_pass']}
    summary[name]=r
# compare to original 6.8s high claw pose.
a=summary['extended']
for k,r in summary.items():
 r['delta_vs_extended_final_margin_mm']=round(r['final_COM_margin_mm']-a['final_COM_margin_mm'],3)
 r['delta_vs_extended_COM_y_mm']=round(r['final_COM_y_mm']-a['final_COM_y_mm'],3)
 r['delta_vs_extended_toe_horizontal_mm']=round(r['final_front_toe_horizontal_dist_mm']-a['final_front_toe_horizontal_dist_mm'],3)
meta={'schema':'claw-tuck-cad30-comparison-v1', 'purpose':'Compare 3 preplanned raised front-leg postures after six-grounded->four-support transition; no gait',
 'physics':'MuJoCo 3.14.0, CAD30 nominal estimated mass 1.804868 kg; friction μ=1.1; PD gain 7 / 0.32; assumed servo cap ±0.7 Nm',
 'started_from_six_grounded':True, 'calculated_trajectory_seconds_after_settle':9.3, 'pre_stabilization_seconds':1.0,
 'joint_bound_for_IK_radians':1.7,'comparison':summary,'recommended':'semi_tucked',
 'reason':'Semi-tucked captures most observed extra stability margin while keeping ~3.4 deg to the conservative IK angular bound, versus <1 deg for compact; no measured hardware motor limits.',
 'limitations':['Model geometry inertias/servo torque-speed/friction are estimated, not physically calibrated',
   'Torso-to-tibia mesh proxy and tibia-tibia distances measured, but all visual STL mesh self-collisions are not physically enabled; no complete collision guarantee',
   'The three postures are candidates, not a continuous pose optimization or proof of global optimum',
   'The study does not test full four-legged walking and cannot predict the real robot balance',
   'No real-world energy/power or temperature model available']}
out=ROOT/'results/claw_tuck_comparison_compact_2026-11-04.json'
out.write_text(json.dumps(meta,indent=2,ensure_ascii=False)+'\n',encoding='utf-8')
for name,m in summary.items():print(name, json.dumps({k:v for k,v in m.items() if k!='final_front_legs'},ensure_ascii=False))
print('OUTPUT',out)
