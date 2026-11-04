"""Render side-by-side actual saved CAD30 MuJoCo qpos states. No staged motion."""
import json, subprocess, sys
from pathlib import Path
import numpy as np
import mujoco
from PIL import Image, ImageDraw, ImageFont
ROOT=next((p for p in Path(__file__).resolve().parents if (p/'hexlab').is_dir()),None)
if ROOT is None: raise RuntimeError('Project hexlab/ missing: restore the main project sources')
DAT=ROOT/'results'
A=json.loads((DAT/'failure_leg_intact_neighbor_inhibit_seed23_2026-11-04.json').read_text())
B=json.loads((DAT/'failure_leg_fl_neighbor_inhibit_seed23_2026-11-04.json').read_text())
xml=str(ROOT/'source/mjcf/scene_local30_nominal.xml')
model=mujoco.MjModel.from_xml_path(xml)
data=mujoco.MjData(model)
cam=mujoco.MjvCamera();cam.type=mujoco.mjtCamera.mjCAMERA_FREE
cam.azimuth=140;cam.elevation=-16;cam.distance=.83
cam.lookat[:]=np.array([0,0,.12])
fps=8;w,h=600,410
output=DAT/'one_leg_failure_seed23_comparison_2026-11-04.mp4'
args=['ffmpeg','-y','-hide_banner','-loglevel','error','-f','rawvideo','-pix_fmt','rgb24',
      '-s:v',f'{w*2}x{h}','-r',str(fps),'-i','-','-c:v','libx264','-preset','veryfast','-crf','25',
      '-pix_fmt','yuv420p','-movflags','+faststart',str(output)]
proc=subprocess.Popen(args,stdin=subprocess.PIPE,stderr=subprocess.PIPE)
try:font=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',16)
except Exception:font=ImageFont.load_default()
with mujoco.Renderer(model,height=h,width=w) as renderer:
 for k in range(8*fps):
  t=(k+.5)/fps
  images=[]
  for rec,lab in ((A,'NO DAMAGE'),(B,'FL SUPPORT+MOTORS OFF AT 3 s')):
   samples=rec['qpos_frames']; snap=min(samples,key=lambda x:abs(x['t']-t))
   data.qpos[:]=snap['qpos'];data.qvel[:]=0.;mujoco.mj_forward(model,data)
   renderer.update_scene(data,camera=cam)
   im=Image.fromarray(renderer.render())
   tr=min(rec['trace_50hz'],key=lambda x:abs(x['t']-t))
   d=ImageDraw.Draw(im)
   d.rounded_rectangle((4,5,w-4,42),radius=4,fill=(15,20,29))
   d.text((12,9),lab,font=font,fill=(250,170,170) if rec is B and t>=3 else 'white')
   d.text((12,26),f't={t:.1f}s  forward={tr["advance_m"]*100:.1f} cm   ground={tr["n_contacts"]}   tilt={tr["tilt_deg"]:.1f} deg',font=font,fill='white')
   d.rectangle((4,h-22,w-4,h-4),fill=(15,20,29))
   d.text((12,h-21),'CAD30 MuJoCo 3.14  |  1.805 kg  |  simulated only',font=font,fill='white')
   if rec is B and t>=3:
    d.rectangle((w-16,51,w-4,h-28),fill=(200,65,65))
   images.append(im)
  canvas=Image.new('RGB',(2*w,h))
  for i,im in enumerate(images):canvas.paste(im,(i*w,0))
  proc.stdin.write(canvas.tobytes())
proc.stdin.close();err=proc.stderr.read().decode(); code=proc.wait()
if code:raise RuntimeError(err)
gif=output.with_suffix('.gif')
subprocess.run(['ffmpeg','-y','-hide_banner','-loglevel','error','-i',str(output),'-vf','fps=7,scale=960:-1:flags=lanczos','-loop','0',str(gif)],check=True)
print({'mp4':str(output),'mp4_bytes':output.stat().st_size,'gif':str(gif),'gif_bytes':gif.stat().st_size})