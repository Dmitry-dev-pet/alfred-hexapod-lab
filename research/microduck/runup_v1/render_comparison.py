#!/usr/bin/env python3
"""Actual original-STL MuJoCo frames of matched moving control and 5 cm step."""
import os,sys
os.environ.setdefault('MUJOCO_GL','egl')
from pathlib import Path
HERE=Path(__file__).resolve().parent;sys.path.insert(0,str(HERE.parent/'microduck-contact'))
import contextlib,io,json
import mujoco as mj
from PIL import Image,ImageDraw,ImageFont
from collision_model import build,terrain

def main():
 ip=terrain.bench.upstream_inference(terrain.RL);images=[];times=[4.8,5.2,6.0]
 canvas=Image.new('RGB',(1800,1040),'#102537');draw=ImageDraw.Draw(canvas);font=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',21);small=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',18)
 for row,name in enumerate(['flat-moving','1-long']):
  tr=json.loads((HERE/'traces'/f'runup-{name}.json').read_text());terrain.EDGE=.25
  with contextlib.redirect_stdout(io.StringIO()):m,d,*_=build(ip,tr['terrain']['height_m'])
  m.mat_reflectance[:]=0;m.vis.global_.offwidth=600;m.vis.global_.offheight=440
  step=mj.mj_name2id(m,mj.mjtObj.mjOBJ_GEOM,'step_top')
  if step>=0:m.geom_rgba[step]=[.55,.69,.43,1.]
  opt=mj.MjvOption();opt.geomgroup[:]=0;opt.geomgroup[0]=1;opt.geomgroup[2]=1
  r=mj.Renderer(m,height=440,width=600)
  for col,t in enumerate(times):
   f=min(tr['frames'],key=lambda f:abs(f['t']-t));d.qpos[:]=f['qpos'];mj.mj_forward(m,d)
   cam=mj.MjvCamera();cam.lookat[:]=[d.qpos[0],d.qpos[1],.115];cam.distance=.49;cam.azimuth=150;cam.elevation=-20
   r.update_scene(d,cam,scene_option=opt);im=Image.fromarray(r.render());canvas.paste(im,(600*col,500*row+48))
   label=('Ровный пол' if row==0 else 'Ступень 5 см')+f" · +{t-4.6:.1f} с · {f['raw_forward_deg']:.0f}°"
   draw.text((600*col+12,500*row+12),label,font=font,fill='#f0f7ff')
  r.close()
 draw.text((18,1000),'Одинаковый разбег и запуск t = 4,60 с. Реальные кадры MuJoCo; камера следует за роботом.',font=small,fill='#d1e1e8');canvas.save(HERE/'runup-flat-vs-step.png');print('Rendered matched run-up comparison')
if __name__=='__main__':main()
