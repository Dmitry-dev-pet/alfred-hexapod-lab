"""Two-camera evidence video from recorded MuJoCo qpos, no synthetic movement."""
from __future__ import annotations
import argparse
import json
import subprocess
from pathlib import Path
import numpy as np
from PIL import Image,ImageDraw,ImageFont
import imageio.v2 as imageio
import mujoco

ROOT=Path(__file__).resolve().parents[2]

def render(trace_file:Path, outdir:Path, fps:int=10)->dict:
    result=json.loads(trace_file.read_text(encoding='utf8'))
    frames=result['qpos_10hz']
    trace=result['trace_50hz']
    if not frames:raise ValueError('No simulation qpos; simulation is required before rendering.')
    model=mujoco.MjModel.from_xml_path(str(ROOT/'source/mjcf/scene_local30_nominal.xml'))
    data=mujoco.MjData(model)
    outdir.mkdir(exist_ok=True,parents=True)
    w,h=390,275
    cams=[]
    for az,el,dist in ((245,-22,.89),(145,-22,.89)):
        cam=mujoco.MjvCamera()
        cam.type=mujoco.mjtCamera.mjCAMERA_FREE
        cam.azimuth=az;cam.elevation=el;cam.distance=dist
        cam.lookat[:]=[0,-.035,.12]
        cams.append(cam)
    font_path='/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'
    f16=ImageFont.truetype(font_path,16);f13=ImageFont.truetype(font_path,13)
    video=outdir/'claw_two_rear_lifts_physical.mp4'
    key_png=outdir/'claw_two_rear_lifts_keyframe.png'
    wrote=0;keysaved=False
    with imageio.get_writer(video, fps=fps,codec='libx264',quality=7,macro_block_size=16,ffmpeg_log_level='error') as wr:
        with mujoco.Renderer(model,height=h,width=w) as renderer:
            for frame in frames:
                data.qpos[:]=frame['qpos'];data.qvel[:]=0.
                mujoco.mj_forward(model,data)
                t=frame['t']
                status=min(trace,key=lambda row:abs(row['t']-t))
                pic=Image.new('RGB',(2*w,h+89),(17,22,29))
                for idx,cam in enumerate(cams):
                    renderer.update_scene(data,camera=cam)
                    pic.paste(Image.fromarray(renderer.render().copy()),(idx*w,54))
                dr=ImageDraw.Draw(pic)
                dr.text((12,7),f'FREENOVE FNK0052  /  physical MuJoCo replay   t={t:.1f}s',font=f16,fill=(244,248,250))
                dr.text((12,31),'Both front claws raised  |  rear legs: event-selected',font=f13,fill=(158,205,249))
                if status['phase'] in ('step','touchdown'):
                    status_txt='DIAGNOSTIC SWING (not a complete 4-leg gait)'
                elif status['phase']=='recover':
                    status_txt='Recovery: front claws return to the floor'
                else:
                    status_txt='Support planning / preparatory balance'
                dr.text((12,h+62),status_txt,font=f13,fill=(245,198,143))
                dr.text((495,h+62),f"contacts: {status['support_count']}  margin: {status['support_margin_mm']:+.1f} mm",font=f13,fill=(199,225,209))
                wr.append_data(np.asarray(pic))
                wrote+=1
                if not keysaved and abs(t-6.1)<.06:
                    pic.save(key_png);keysaved=True
    if not keysaved:pic.save(key_png)
    gif=outdir/'claw_two_rear_lifts_physical.gif'
    subprocess.run(['ffmpeg','-y','-hide_banner','-loglevel','error',
                    '-i',str(video),'-vf','fps=6,scale=720:-1:flags=lanczos',
                    '-loop','0',str(gif)],check=True)
    return {'source':str(trace_file),'frames':wrote,'fps':fps,
            'video':str(video),'gif':str(gif),'keyframe':str(key_png),
            'generated_from': 'saved physical qpos (no scripted graphics)',
            'strict_four_leg_walk':bool(result['strict_four_leg_walk'])}

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--trace',type=Path,default=ROOT/'results/event_claw_crawl_mu1p1.json')
    p.add_argument('--out',type=Path,default=ROOT/'results/event_claw_demo')
    args=p.parse_args();print(json.dumps(render(args.trace,args.out),ensure_ascii=False,indent=2))