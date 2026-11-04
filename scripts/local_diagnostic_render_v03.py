"""Render actual MuJoCo v0.3 surrogate snapshots; no kinematic body motion."""
from __future__ import annotations
import argparse, json, os, subprocess, sys
from pathlib import Path
import numpy as np
from PIL import Image, ImageDraw, ImageFont
import mujoco
ROOT=Path(__file__).resolve().parents[1]

def main():
 p=argparse.ArgumentParser();p.add_argument('report',type=Path);p.add_argument('--mp4',type=Path);p.add_argument('--gif',type=Path);p.add_argument('--fps',type=int,default=7)
 a=p.parse_args()
 doc=json.loads(a.report.read_text());s=doc['metrics'];g=s['mode'];snap=doc['snapshots'];samples=doc['samples']
 model=mujoco.MjModel.from_xml_path(str(ROOT/'models/freenove_fnk0052_approx.xml'))
 data=mujoco.MjData(model)
 mass=s['total_mass_estimated_kg'];torso=model.body('torso').id;old=float(model.body_mass[torso]);diff=mass-float(sum(model.body_mass));model.body_mass[torso]+=diff;model.body_inertia[torso]*=model.body_mass[torso]/old
 w,h=640,400
 target_dir=a.report.parent;a.mp4=a.mp4 or target_dir/f'{g}_local_surrogate.mp4';a.gif=a.gif or target_dir/f'{g}_local_surrogate.gif'
 try:font=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',19)
 except:font=ImageFont.load_default()
 cam=mujoco.MjvCamera();cam.type=mujoco.mjtCamera.mjCAMERA_FREE;cam.distance=.80;cam.azimuth=138;cam.elevation=-16
 os.makedirs(a.mp4.parent,exist_ok=True)
 args=['ffmpeg','-hide_banner','-loglevel','error','-y','-f','rawvideo','-pix_fmt','rgb24','-s:v',f'{w}x{h}','-r',str(a.fps),'-i','-','-c:v','libx264','-preset','veryfast','-crf','24','-pix_fmt','yuv420p',str(a.mp4)]
 proc=subprocess.Popen(args,stdin=subprocess.PIPE,stderr=subprocess.PIPE)
 snapshots=np.array([row[0] for row in snap]);timeline=np.arange(0,s['seconds_executed'],1/a.fps)
 with mujoco.Renderer(model,height=h,width=w) as r:
  for i,t in enumerate(timeline):
   idx=np.argmin(abs(snapshots-t));timestamp,qs=snap[idx]
   data.qpos[:]=qs;data.qvel[:]=0;data.time=timestamp;mujoco.mj_forward(model,data)
   # follow body forward motion, framing whole robot, no special camera tricks
   cam.lookat[:]=[float(qs[0]),0,.115]
   r.update_scene(data,camera=cam)
   pic=Image.fromarray(r.render());draw=ImageDraw.Draw(pic)
   info=min(samples,key=lambda x:abs(x['t']-timestamp));dx=float(qs[0])*100
   draw.rounded_rectangle((8,8,w-8,42),radius=6,fill=(17,23,32))
   draw.text((16,14),f'{g.upper()}  |  {timestamp:4.1f}s   |   body +{dx:4.1f} cm',fill='white',font=font)
   draw.rectangle((8,h-35,w-8,h-5),fill=(17,23,32))
   draw.text((16,h-29),'SURROGATE v0.3  /  estimated 1.8 kg  /  MuJoCo 3.14',fill=(225,230,236),font=font)
   proc.stdin.write(pic.tobytes())
 proc.stdin.close();stderr=proc.stderr.read().decode();rc=proc.wait();assert rc==0,(rc,stderr[-3000:])
 subprocess.run(['ffmpeg','-hide_banner','-loglevel','error','-y','-i',str(a.mp4),'-vf',f'fps={a.fps},scale=480:-1:flags=lanczos','-loop','0',str(a.gif)],check=True)
 print(json.dumps({'mp4':str(a.mp4),'gif':str(a.gif),'frames':len(timeline),'fps':a.fps,'size_mp4':a.mp4.stat().st_size,'size_gif':a.gif.stat().st_size},indent=2))

if __name__=='__main__':main()