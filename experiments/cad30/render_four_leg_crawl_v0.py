"""Evidence-only rendering of the INCOMPLETE four-leg gait attempt.
Frames come exclusively from saved physical MuJoCo qpos, never a hand-posed walk.
"""
from pathlib import Path
import json, subprocess
import numpy as np, mujoco
from PIL import Image,ImageDraw,ImageFont
import imageio.v2 as imageio
ROOT=Path(__file__).resolve().parent

def render(json_path:Path,out:Path):
 d=json.loads(json_path.read_text());states=[s for s in d['frames'] if 2.8<=s['t']<=12.]
 if not states:raise ValueError('No physical qpos frames recorded')
 out.mkdir(parents=True,exist_ok=True)
 m=mujoco.MjModel.from_xml_path(str(ROOT/'source/mjcf/scene_local30_nominal.xml'))
 data=mujoco.MjData(m);w,h=300,210
 cams=[]
 for az,el in [(310,-18),(220,-28)]:
  c=mujoco.MjvCamera();c.type=mujoco.mjtCamera.mjCAMERA_FREE;c.azimuth=az;c.elevation=el;c.distance=.85;c.lookat[:]=[0,-.02,.12];cams.append(c)
 try:font=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',11)
 except:font=ImageFont.load_default()
 jpg=out/'four_leg_cycle_failed_keyframe.png';mp4=out/'four_leg_cycle_failed_diagnostic.mp4'
 i0=0
 with imageio.get_writer(mp4,fps=7,codec='libx264',quality=6,macro_block_size=16) as wr:
  with mujoco.Renderer(m,height=h,width=w) as ren:
   for s in states[::2]:
    data.qpos[:]=s['qpos'];data.qvel[:]=0;mujoco.mj_forward(m,data)
    pic=Image.new('RGB',(2*w,h+58),(15,20,26))
    for j,cam in enumerate(cams):
     ren.update_scene(data,camera=cam);pic.paste(Image.fromarray(ren.render().copy()),(j*w,58))
    dr=ImageDraw.Draw(pic)
    dr.text((10,8),f"Freenove FNK0052 CAD30 | t={s['t']:.1f}s | exploratory FOUR-LEG cycle",fill=(245,247,249),font=font)
    dr.text((10,30),'FAILED GAIT: middle feet do not lift; no meaningful net forward travel',fill=(244,195,120),font=font)
    wr.append_data(np.asarray(pic))
    if abs(s['t']-9.0)<.10 and not jpg.exists():pic.save(jpg)
    i0+=1
 if not jpg.exists():pic.save(jpg)
 gif=out/'four_leg_cycle_failed_diagnostic.gif'
 subprocess.run(['ffmpeg','-y','-hide_banner','-loglevel','error','-i',str(mp4),'-vf','fps=4,scale=600:-1:flags=bicubic',str(gif)],check=True)
 return {'frames':i0,'mp4':str(mp4),'gif':str(gif),'png':str(jpg)}
if __name__=='__main__':
 import sys;print(render(Path(sys.argv[1]),Path(sys.argv[2])))
