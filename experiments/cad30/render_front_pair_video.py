"""Replay MuJoCo qpos samples from a completed front-pair balance experiment.
The viewer is not a kinematic animation controller; each frame comes from
physics-simulated state saved by front_pair_trick.py.
"""
from __future__ import annotations
import argparse, json, subprocess
from pathlib import Path
import mujoco
import numpy as np
from PIL import Image,ImageDraw,ImageFont

def render(src:Path, out:Path):
    j=json.loads(src.read_text())
    assert j['qpos_frames'] and j['front_pair_clear_four_leg_balance_hold']
    model=mujoco.MjModel.from_xml_path(str((Path(__file__).resolve().parent if (Path(__file__).resolve().parent/'source/mjcf').is_dir() else Path(__file__).resolve().parents[2])/'source/mjcf'/f"scene_local30_{j['scenario']}.xml"))
    data=mujoco.MjData(model)
    W,H=640,480
    cameras=[]
    for az in (300,210):
        cam=mujoco.MjvCamera();cam.type=mujoco.mjtCamera.mjCAMERA_FREE
        cam.azimuth=az;cam.elevation=-20;cam.distance=.82
        cam.lookat[:]=np.array([0.,-.013,.105])
        cameras.append(cam)
    out.mkdir(parents=True,exist_ok=True)
    frames=[]
    font_path='/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'
    try:font=ImageFont.truetype(font_path,16)
    except OSError:font=ImageFont.load_default()
    with mujoco.Renderer(model,height=H,width=W) as renderer:
        for snap in j['qpos_frames']:
            data.qpos[:]=snap['qpos'];data.qvel[:]=0;mujoco.mj_forward(model,data)
            canvas=Image.new('RGB',(2*W,H+48),(18,24,31))
            for i,cam in enumerate(cameras):
                renderer.update_scene(data,camera=cam)
                im=Image.fromarray(renderer.render().copy())
                canvas.paste(im,(i*W,48))
            draw=ImageDraw.Draw(canvas)
            t=snap['t']
            phase='prepare support' if t<2 else 'lift front legs' if t<3 else 'four-leg hold' if t<4 else 'lower front legs' if t<5 else 'return to stance'
            draw.text((18,15),f'FNK0052 CAD30 | front pair balance | t={t:.1f}s | {phase}',fill=(239,244,248),font=font)
            frames.append(np.asarray(canvas))
    import imageio.v2 as imageio
    mp4=out/'front_pair_balance_cad30.mp4'
    gif=out/'front_pair_balance_cad30.gif'
    imageio.mimsave(mp4,frames,fps=10,codec='libx264',quality=7,macro_block_size=16)
    subprocess.run(['ffmpeg','-y','-hide_banner','-loglevel','error','-i',str(mp4),'-vf','fps=8,scale=768:-1:flags=bicubic',str(gif)],check=True)
    Image.fromarray(frames[36]).save(out/'front_pair_hold_keyframe.png')
    return [mp4,gif,out/'front_pair_hold_keyframe.png']

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('report',type=Path);p.add_argument('out',type=Path)
    a=p.parse_args();print(*(str(x) for x in render(a.report,a.out)),sep='\n')
