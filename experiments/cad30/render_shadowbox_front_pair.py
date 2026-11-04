"""Render MuJoCo qpos replays as paired video views; no invented motion."""
from __future__ import annotations
import json,argparse,subprocess,sys,time
from pathlib import Path
import mujoco,numpy as np
from PIL import Image,ImageDraw,ImageFont
import imageio.v2 as imageio
HERE=Path(__file__).resolve().parent
ROOT=HERE if (HERE/'source/mjcf').is_dir() else HERE.parents[1]

def render(source:Path,output_dir:Path):
 j=json.loads(source.read_text())
 if not j['qpos_frames'] or not j['high_guard_and_air_jabs_valid']:raise ValueError('No validated dynamics frames')
 m=mujoco.MjModel.from_xml_path(str(ROOT/'source/mjcf'/f"scene_local30_{j['mass_scenario']}.xml"));d=mujoco.MjData(m)
 W,H=400,300
 cams=[]
 for az in (315,270):
  c=mujoco.MjvCamera();c.type=mujoco.mjtCamera.mjCAMERA_FREE;c.azimuth=az;c.elevation=-18;c.distance=.73;c.lookat[:]=[0.,-.013,.125];cams.append(c)
 try:font=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',13)
 except OSError:font=ImageFont.load_default()
 snaps=j['qpos_frames'];samples=[snaps[i] for i in np.linspace(0,len(snaps)-1,60,dtype=int)]
 output_dir.mkdir(exist_ok=True,parents=True)
 mp4=output_dir/'front_pair_high_air_jab.mp4';gif=output_dir/'front_pair_high_air_jab.gif';key=output_dir/'front_pair_high_air_jab_keyframe.png'
 def phase(t):
  if t<2:return 'SHIFT WEIGHT'
  if t<3.1:return 'RAISE FRONT LEGS'
  if t<3.55:return 'HIGH GUARD'
  if t<3.87:return 'LEFT AIR JAB'
  if t<4.2:return 'RETRACT'
  if t<4.45:return 'GUARD'
  if t<4.77:return 'RIGHT AIR JAB'
  if t<5.1:return 'RETRACT'
  if t<5.55:return 'HIGH GUARD'
  if t<6.5:return 'LOWER LEGS'
  return 'RECOVER STANCE'
 with imageio.get_writer(mp4,mode='I',fps=6,codec='libx264',quality=6,macro_block_size=16) as writer:
  with mujoco.Renderer(m,height=H,width=W) as renderer:
   for i,snap in enumerate(samples):
    d.qpos[:]=snap['qpos'];d.qvel[:]=0;mujoco.mj_forward(m,d)
    canvas=Image.new('RGB',(2*W,H+54),(14,20,27));dr=ImageDraw.Draw(canvas)
    for n,c in enumerate(cams):
     renderer.update_scene(d,camera=c)
     canvas.paste(Image.fromarray(renderer.render().copy()),(n*W,54))
    dr.text((12,8),f"FNK0052 CAD30  |  {snap['t']:.2f}s  |  {phase(snap['t'])}",font=font,fill=(247,249,251))
    dr.text((12,30),'SIMULATED AIR JAB | NO IMPACT | uncalibrated servos',font=font,fill=(182,201,215))
    writer.append_data(np.asarray(canvas))
    if abs(snap['t']-4.77)<.10:canvas.save(key)
    if i%12==0:print('rendered',i,'of',len(samples),flush=True)
 if not key.exists():raise RuntimeError('missing keyframe')
 print('mp4 done',mp4,flush=True)
 subprocess.run(['ffmpeg','-y','-hide_banner','-loglevel','error','-i',str(mp4),
                 '-vf','fps=5,scale=640:-1:flags=bilinear',str(gif)],check=True,timeout=40)
 print('gif done',gif,flush=True)
 return [str(mp4),str(gif),str(key)]

if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('source',type=Path);p.add_argument('out',type=Path)
 a=p.parse_args();print(*render(a.source,a.out),sep='\n')


def render_closeup(source:Path, output_dir:Path):
 """Render the two actual 0.32-s air jabs at half-speed for inspection.

 The speed change applies to video playback only. Every image is from a saved
 MuJoCo state, no inferred simulation timestep or made-up joint motion.
 """
 j=json.loads(source.read_text())
 if not j['high_guard_and_air_jabs_valid']:raise ValueError('Simulation did not pass')
 snaps=[s for s in j['qpos_frames'] if 3.25<=s['t']<=5.5]
 m=mujoco.MjModel.from_xml_path(str(ROOT/'source/mjcf'/f"scene_local30_{j['mass_scenario']}.xml"));d=mujoco.MjData(m)
 cams=[]
 for az in (315,270):
  c=mujoco.MjvCamera();c.type=mujoco.mjtCamera.mjCAMERA_FREE;c.azimuth=az;c.elevation=-18;c.distance=.73;c.lookat[:]=[0.,-.013,.125];cams.append(c)
 try:font=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',13)
 except OSError:font=ImageFont.load_default()
 out=output_dir/'front_pair_air_jabs_slow_motion.mp4';output_dir.mkdir(exist_ok=True,parents=True)
 with imageio.get_writer(out,mode='I',fps=6,codec='libx264',quality=7,macro_block_size=16) as writer:
  with mujoco.Renderer(m,height=300,width=400) as renderer:
   for snap in snaps:
    d.qpos[:]=snap['qpos'];d.qvel[:]=0;mujoco.mj_forward(m,d)
    canvas=Image.new('RGB',(800,368),(13,20,28))
    for i,c in enumerate(cams):
     renderer.update_scene(d,camera=c)
     canvas.paste(Image.fromarray(renderer.render().copy()),(400*i,64))
    dr=ImageDraw.Draw(canvas)
    label='FIRST JAB' if 3.55<=snap['t']<4.2 else 'SECOND JAB' if 4.45<=snap['t']<5.1 else 'HIGH GUARD'
    dr.text((12,12),f"FNK0052 | t_sim = {snap['t']:.2f}s | {label} | 0.5x PLAYBACK",font=font,fill=(245,248,250))
    dr.text((12,35),'Recorded MuJoCo qpos; no impact or real servo validation',font=font,fill=(182,201,215))
    writer.append_data(np.asarray(canvas))
 return str(out)
