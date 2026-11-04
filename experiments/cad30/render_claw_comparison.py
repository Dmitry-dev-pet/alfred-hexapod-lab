"""Render one-claw-step vs two-claw-hold from saved real MuJoCo qpos frames.

No synthetic/root-pinned motion; source CAD STL must be restored separately.
"""
from __future__ import annotations
import argparse,json,subprocess
from pathlib import Path
import mujoco
import numpy as np
from PIL import Image,ImageDraw,ImageFont

ROOT=Path(__file__).resolve().parents[2]


def frames(src):
    d=json.loads(Path(src).read_text())
    return d,d['qpos_frames']


def create(left:Path,right:Path,out:Path,fps=8):
    a,fa=frames(left);b,fb=frames(right)
    model=mujoco.MjModel.from_xml_path(str(ROOT/'source/mjcf/scene_local30_nominal.xml'))
    data=mujoco.MjData(model)
    cam=mujoco.MjvCamera();cam.type=mujoco.mjtCamera.mjCAMERA_FREE
    cam.lookat=np.array([0.,-.01,.075]);cam.distance=.80;cam.azimuth=142.;cam.elevation=-26.
    w,h=510,336
    renderer=mujoco.Renderer(model,height=h,width=w)
    out.mkdir(parents=True,exist_ok=True)
    ffmpeg=subprocess.Popen(['ffmpeg','-y','-loglevel','error','-f','rawvideo','-pixel_format','rgb24',
                              '-video_size',f'{2*w}x{h+40}','-framerate',str(fps),'-i','-',
                              '-c:v','libx264','-pix_fmt','yuv420p','-crf','22',str(out/'optimized_claw_modes.mp4')],stdin=subprocess.PIPE)
    gifs=[]
    n=min(len(fa),len(fb))
    # A short evidence montage, not full-frame-rate real-time playback.
    # Every shown body pose is an actual recorded physics state.
    keyframes=np.unique(np.linspace(0,n-1,16,dtype=int))
    for k in keyframes:
        imgs=[]
        for r,cap in ((fa[k],'1 claw + rear step'),(fb[k],'2 claws raised')):
            data.qpos[:]=r['qpos'];data.qvel[:]=0.;mujoco.mj_forward(model,data)
            renderer.update_scene(data,camera=cam);imgs.append(Image.fromarray(renderer.render().copy()))
        canvas=Image.new('RGB',(2*w,h+40),'white')
        d=ImageDraw.Draw(canvas)
        for i,img in enumerate(imgs):canvas.paste(img,(i*w,40))
        d.text((14,12),f'ONE CLAW + REAR STEP   {fa[k]["t"]:.1f}s',fill='black')
        d.text((w+14,12),f'TWO FRONT CLAWS   {fb[k]["t"]:.1f}s',fill='black')
        gifs.append(canvas.resize((int(w*1.45),int((h+40)*.725))))
        ffmpeg.stdin.write(canvas.tobytes())
    ffmpeg.stdin.close();rc=ffmpeg.wait();renderer.close()
    if rc:raise RuntimeError('ffmpeg failed to encode the physical replay')
    if gifs:gifs[0].save(out/'optimized_claw_modes.gif',save_all=True,append_images=gifs[1:],duration=550,loop=0,optimize=True)
    return {'mp4':str(out/'optimized_claw_modes.mp4'),'gif':str(out/'optimized_claw_modes.gif')}

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--one',type=Path,required=True);p.add_argument('--two',type=Path,required=True);p.add_argument('--out',type=Path,required=True)
    a=p.parse_args();print(create(a.one,a.two,a.out))