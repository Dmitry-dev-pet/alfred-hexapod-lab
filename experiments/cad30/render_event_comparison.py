"""Replay *recorded* CAD30 MuJoCo qpos: contact selection vs force tracking.

This is numerical evidence, not a staged or model-generated animation.
Requires the private CAD30 source meshes restored locally; does not redistribute them.
"""
from __future__ import annotations
import argparse
import json
import subprocess
from pathlib import Path
import mujoco
import numpy as np
from PIL import Image,ImageDraw
ROOT=Path(__file__).resolve().parents[2]

def create(a:Path,b:Path,out:Path,fps:int=8)->dict:
    x=json.loads(a.read_text());y=json.loads(b.read_text())
    aa=x['qpos_frames'];bb=y['qpos_frames']
    if not aa or not bb:raise ValueError('Save --save-frames before replay')
    model=mujoco.MjModel.from_xml_path(str(ROOT/'source/mjcf/scene_local30_nominal.xml'))
    data=mujoco.MjData(model)
    cam=mujoco.MjvCamera();cam.type=mujoco.mjtCamera.mjCAMERA_FREE
    cam.lookat=np.array([0.,0.02,.075]);cam.distance=.75;cam.azimuth=141.;cam.elevation=-24.
    w,h=492,336
    renderer=mujoco.Renderer(model,height=h,width=w)
    out.mkdir(parents=True,exist_ok=True)
    mp4=out/'event_contact_math_vs_force.mp4';gif=out/'event_contact_math_vs_force.gif'
    proc=subprocess.Popen(['ffmpeg','-y','-loglevel','error','-f','rawvideo','-pixel_format','rgb24',
       '-video_size',f'{w*2}x{h+44}','-framerate',str(fps),'-i','-',
       '-c:v','libx264','-pix_fmt','yuv420p','-crf','24',str(mp4)],stdin=subprocess.PIPE)
    frames=[]
    # Every replay frame corresponds to a time-indexed MuJoCo state.
    for k in range(min(len(aa),len(bb))):
        canvas=Image.new('RGB',(2*w,h+44),'white')
        d=ImageDraw.Draw(canvas)
        for side,state in enumerate((aa[k],bb[k])):
            data.qpos[:]=state['qpos'];data.qvel[:]=0.;mujoco.mj_forward(model,data)
            renderer.update_scene(data,camera=cam)
            canvas.paste(Image.fromarray(renderer.render().copy()),(side*w,44))
        d.text((12,12),f'CONTACT MATH ONLY  t={aa[k]["t"]:.1f}s',fill='black')
        d.text((w+12,12),f'CONTACT MATH + FORCE  t={bb[k]["t"]:.1f}s',fill='black')
        proc.stdin.write(canvas.tobytes())
        if k%4==0:frames.append(canvas.resize((787,304)))
    proc.stdin.close();status=proc.wait();renderer.close()
    if status:raise RuntimeError(f'ffmpeg failed {status}')
    frames[0].save(gif,save_all=True,append_images=frames[1:],duration=500,loop=0,optimize=True)
    return {'mp4':str(mp4),'gif':str(gif),'source_frames':min(len(aa),len(bb))}

if __name__=='__main__':
    p=argparse.ArgumentParser()
    p.add_argument('--base',type=Path,required=True)
    p.add_argument('--force',type=Path,required=True)
    p.add_argument('--out',type=Path,required=True)
    a=p.parse_args();print(json.dumps(create(a.base,a.force,a.out),indent=2))