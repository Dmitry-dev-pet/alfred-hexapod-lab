#!/usr/bin/env python3
"""Actual MuJoCo original-STL frames; obstacle translucency is diagnostic only."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import contextlib,io,json
from pathlib import Path
import numpy as np,mujoco as mj
from PIL import Image,ImageDraw,ImageFont
from collision_model import old,setup,terrain
HERE=Path(__file__).resolve().parent

def main():
    ip=terrain.bench.upstream_inference(terrain.RL);frames=[]
    for corrected in [False,True]:
        path=HERE/'traces/close-h50.json' if corrected else old.WORK/'microduck-wall/traces/close-h50.json';tr=json.loads(path.read_text());f=next(f for f in tr['frames'] if f['t']==7.92)
        with contextlib.redirect_stdout(io.StringIO()):m,d,*_=(setup if corrected else old.setup)(ip,.05,tr['terrain']['edge_x_m'],'back')
        d.qpos[:]=f['qpos'];mj.mj_forward(m,d);step=mj.mj_name2id(m,mj.mjtObj.mjOBJ_GEOM,'step_top');m.geom_rgba[step]=[.55,.69,.43,.45];m.mat_reflectance[:]=0
        m.vis.global_.offwidth=800;m.vis.global_.offheight=600
        r=mj.Renderer(m,height=600,width=800);cam=mj.MjvCamera();cam.lookat[:]=[.045,-.025,.115];cam.distance=.50;cam.azimuth=155;cam.elevation=-22
        opt=mj.MjvOption();opt.geomgroup[:]=0;opt.geomgroup[0]=1;opt.geomgroup[2]=1;r.update_scene(d,cam,scene_option=opt);frames.append(Image.fromarray(r.render()));r.close()
    canvas=Image.new('RGB',(1600,720),'#102537');draw=ImageDraw.Draw(canvas);font=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',23);small=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',18)
    for i,(im,label) in enumerate(zip(frames,['Было: корпус проходит в ступеньку','Исправлено: корпус останавливается на опоре'])):
        canvas.paste(im,(i*800,64));draw.text((i*800+20,20),label,font=font,fill='#f0f7ff')
    draw.text((20,678),'Реальные кадры MuJoCo, t = 7,92 с. Ступенька 5 см показана полупрозрачно для проверки пересечений.',font=small,fill='#d1e1e8')
    canvas.save(HERE/'body-step-before-after.png');print('Rendered exact original-STL comparison')
if __name__=='__main__':main()
