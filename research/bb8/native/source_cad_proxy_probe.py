"""MuJoCo 3.14: source-CAD visual meshes, explicit *uncalibrated* drive/collision proxy.
Original HWeigo parts and placements are preserved for display. This is NOT
hardware-accurate inner-wheel contact nor a magnetic head simulation.
"""
from pathlib import Path
from xml.etree import ElementTree as ET
from collections import Counter
from math import pi
import argparse,hashlib,json,os,numpy as np
import mujoco

BASE=Path(os.environ.get('BB8_WORKSPACE','/mnt/data'))/'bb8-hweigo-reconstructed'
DIR=BASE/'meshes'; SOURCE=json.loads((DIR/'scene.json').read_text())
CENTER=np.array([-41.24816673006777,-27.2571967211034,99.26489749059236])*0.001
ALIGN=np.array([[1,0,0],[0,0,1],[0,-1,0]],dtype=float)

def quat_to_matrix(q):
 x,y,z,w=map(float,q)
 return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])

def fmt(v):return ' '.join(f'{x:.9g}' for x in v)

def make_model(include_visuals=True):
 doc=ET.Element('mujoco',{'model':'HWeigo_BB8_SOURCE_CAD_VISUAL_uncalibrated_proxy'})
 ET.SubElement(doc,'compiler',{'angle':'radian','inertiafromgeom':'false','meshdir':str(DIR)})
 ET.SubElement(doc,'option',{'timestep':'0.002','gravity':'0 0 -9.81','integrator':'implicitfast'})
 visual=ET.SubElement(doc,'visual'); ET.SubElement(visual,'global',{'offwidth':'768','offheight':'576'})
 mesh_assets=ET.SubElement(doc,'asset')
 shell_key=next(k for k,v in SOURCE['meshes'].items() if v['source_file']=='球壳1.SLDPRT')
 visual_parts=[x for x in SOURCE['instances'] if SOURCE['meshes'][x['part_key']]['source_file'] not in ('球壳小.SLDPRT','头部底面.SLDPRT','SG90.SLDPRT','牛眼轮NL-8H.SLDPRT','舵机转板新.SLDPRT')]
 visuals=[]
 for item in visual_parts:
  src=SOURCE['meshes'][item['part_key']]['source_file']
  if src=='球壳1.SLDPRT' and item['id']!='c22':continue # remove duplicate near-identical sphere
  visuals.append(item)
 if include_visuals:
  for key in sorted(set(o['part_key'] for o in visuals)):
   mesh=SOURCE['meshes'][key]
   ET.SubElement(mesh_assets,'mesh',{'name':mesh['file'].split('.')[0], 'file':mesh['file'],'scale':'0.001 0.001 0.001'})
 wb=ET.SubElement(doc,'worldbody')
 ET.SubElement(wb,'geom',{'name':'floor','type':'plane','size':'3 3 0.1','friction':'0.9 0.01 0.0001'})
 shell=ET.SubElement(wb,'body',{'name':'shell','pos':'0 0 0.1001'})
 ET.SubElement(shell,'freejoint',{'name':'shell_free'})
 ET.SubElement(shell,'inertial',{'pos':'0 0 0','mass':'0.20','diaginertia':'0.001333 0.001333 0.001333'})
 ET.SubElement(shell,'geom',{'name':'source_scale_shell_collision_proxy','type':'sphere','size':'0.1','friction':'0.9 0.01 0.0001','mass':'0'})
 inner=ET.SubElement(shell,'body',{'name':'inner_chassis','pos':'0 0 0'})
 ET.SubElement(inner,'joint',{'name':'drive_reaction_hinge_proxy','type':'hinge','axis':'0 1 0','damping':'0.02'})
 ET.SubElement(inner,'inertial',{'pos':'0 0 -0.045','mass':'0.38','diaginertia':'0.0004 0.0004 0.0005'})
 ET.SubElement(inner,'geom',{'name':'pendulum_mass_visual_proxy','type':'box','size':'0.028 0.033 0.022','pos':'0 0 -0.045','contype':'0','conaffinity':'0','mass':'0','rgba':'0.3 0.4 0.6 0.7'})
 if include_visuals:
  for i,entry in enumerate(visuals):
   src=SOURCE['meshes'][entry['part_key']]['source_file']
   target=shell if src=='球壳1.SLDPRT' else inner
   rot=quat_to_matrix(entry['rotation_xyzw'])
   newrot=ALIGN@rot
   pos=ALIGN@(np.array(entry['position_mm'])*0.001-CENTER)
   # MuJoCo only accepts positive inertial meshes; use source meshes for visual only.
   quat=np.zeros(4);mujoco.mju_mat2Quat(quat,newrot.reshape(-1))  # MuJoCo wxyz
   rgba='0.85 0.86 0.87 0.35' if src=='球壳1.SLDPRT' else '0.45 0.52 0.6 1'
   ET.SubElement(target,'geom',{'name':'cad_visual_'+str(i),'type':'mesh','mesh':SOURCE['meshes'][entry['part_key']]['file'].split('.')[0], 'pos':fmt(pos),'quat':fmt(quat),'mass':'0','contype':'0','conaffinity':'0','rgba':rgba})
 act=ET.SubElement(doc,'actuator');ET.SubElement(act,'motor',{'name':'net_drive_torque_ASSUMED','joint':'drive_reaction_hinge_proxy','gear':'1','ctrllimited':'true','ctrlrange':'-0.08 0.08'})
 xml=ET.tostring(doc,encoding='unicode')
 return xml

def simulate(torque,duration=6.,with_visuals=True):
 xml=make_model(include_visuals=with_visuals)
 out=BASE/'mujoco-proxy';out.mkdir(exist_ok=True)
 xmlpath=out/('bb8_cad_visual_proxy.xml' if with_visuals else 'bb8_minimal_proxy.xml')
 xmlpath.write_text(xml,encoding='utf-8')
 model=mujoco.MjModel.from_xml_path(str(xmlpath))
 d=mujoco.MjData(model)
 nsteps=int(round(duration/model.opt.timestep))
 frames=[];max_abs_hinge=0.;nfloor=0
 for step in range(nsteps+1):
  t=step*model.opt.timestep
  ctrl=torque if t<1.5 else (-torque if t<3 else 0.)
  d.ctrl[0]=ctrl
  if step%10==0:
   qpos=d.qpos.copy().tolist()
   frames.append({'t':round(t,4),'qpos':qpos,'qvel':d.qvel.copy().tolist(),'ctrl':[ctrl],'contact_count':int(d.ncon)})
   max_abs_hinge=max(max_abs_hinge,abs(qpos[7]))
   nfloor+=int(d.ncon>0)
  if step<nsteps:mujoco.mj_step(model,d)
 report={'schema':'alfred-bb8-hweigo-source-cad-proxy-physics/v1','status':'MEASURED_MUJOCO_WITH_ORIGINAL_CAD_VISUALS_BUT_ASSUMED_COLLISION_AND_MASS','engine':'MuJoCo','version':mujoco.__version__,'source_git_commit':'7318411cfbfbffdbd68f75e3734a50c3933969e5','source_assembly_blob_sha1':'bcfead6e92329575aa474fb4be05c24191ac5294','source_sphere_radius_m':0.1,'calibrated':False,'source_cad_visual_meshes':with_visuals,'limitations':['No verified shell cavity contacts','One surrogate reaction-torque drive hinge rather than real twin wheel contacts','Mass and inertia assumed','Head omitted from dynamics; magnetic retention not simulated','Source CAD shells represent filled geometries, not shell wall mass'], 'model_xml_sha256':hashlib.sha256(xml.encode()).hexdigest(),'timestep_s':model.opt.timestep,'duration_s':duration,'command_torque_Nm_assumed':torque,'summary':{'final_displacement_x_m':round(float(d.qpos[0]),7), 'min_x_m':round(min(f['qpos'][0] for f in frames),7), 'max_x_m':round(max(f['qpos'][0] for f in frames),7), 'max_abs_x_m':round(max(abs(f['qpos'][0]) for f in frames),7), 'integrated_abs_sampled_dx_m':round(sum(abs(frames[i]['qpos'][0]-frames[i-1]['qpos'][0]) for i in range(1,len(frames))),7),'final_displacement_y_m':round(float(d.qpos[1]),7),'final_height_m':round(float(d.qpos[2]),7),'max_abs_hinge_deg':round(float(max_abs_hinge*180/pi),4),'contact_samples':nfloor,'qpos_count':model.nq,'qvel_count':model.nv,'geom_count':model.ngeom,'mesh_count':model.nmesh,'frames':len(frames)},'frames':frames}
 return report

if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--torque',type=float,default=.02);p.add_argument('--no-cad',action='store_true');args=p.parse_args()
 result=simulate(args.torque,with_visuals=not args.no_cad)
 out=BASE/'mujoco-proxy'/f'bb8_cad_source_proxy_torque_{int(args.torque*1000):03d}mNm.json'
 out.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
 print('RESULT',json.dumps(result['summary']),flush=True)
