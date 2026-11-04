"""Two-view video of one rear-leg swing with the two front legs high.

Every frame is a recorded MuJoCo qpos state. This is not a full four-legged gait.
"""
from pathlib import Path
import json,subprocess
import numpy as np
import mujoco
from PIL import Image,ImageDraw,ImageFont
import imageio.v2 as imageio
HERE=Path(__file__).resolve().parent
ROOT=HERE if (HERE/'source/mjcf').is_dir() else HERE.parents[1]

def render(receipt:Path,out_dir:Path):
 d=json.loads(receipt.read_text())
 if not d['qpos_frames'] or d.get('fail'):
  raise ValueError('Validated recorded experiment needed')
 if d['under_3_contacts'] or not d['all_front_clear'] or not d['complete']:
  raise ValueError('Refuse to render a failed rear-leg swing as a success')
 m=mujoco.MjModel.from_xml_path(str(ROOT/'source/mjcf'/f"scene_local30_{d['mass']}.xml"));state=mujoco.MjData(m)
 out_dir.mkdir(parents=True,exist_ok=True)
 c1=mujoco.MjvCamera();c2=mujoco.MjvCamera()
 for c,az,el in [(c1,310,-18),(c2,220,-30)]:
  c.type=mujoco.mjtCamera.mjCAMERA_FREE;c.azimuth=az;c.elevation=el;c.distance=.78;c.lookat[:]=[0.,-.018,.120]
 try:font=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',12)
 except OSError:font=ImageFont.load_default()
 snaps=[s for s in d['qpos_frames'] if 2.60<=s['t']<=6.3]
 out=out_dir/'four_support_rear_single_step.mp4';still=out_dir/'four_support_rear_step.png';w,h=360,260
 with imageio.get_writer(out,mode='I',fps=10,codec='libx264',quality=6,macro_block_size=16) as writer:
  with mujoco.Renderer(m,height=h,width=w) as renderer:
   for s in snaps:
    state.qpos[:]=s['qpos'];state.qvel[:]=0;mujoco.mj_forward(m,state)
    pic=Image.new('RGB',(2*w,h+58),(14,19,25))
    for i,c in enumerate((c1,c2)):
     renderer.update_scene(state,camera=c)
     pic.paste(Image.fromarray(renderer.render().copy()),(i*w,58))
    dr=ImageDraw.Draw(pic)
    phase='REAR LEG SWING' if 3.5<=s['t']<5.5 else 'FOUR-LEG SUPPORT'
    dr.text((10,8),f"FNK0052 CAD30 | t={s['t']:.2f}s | {phase}",font=font,fill=(248,248,249))
    dr.text((10,32),'TWO FRONT LEGS RAISED  |  SINGLE REAR FOOT STEP  |  NOT YET WALKING',font=font,fill=(185,198,207))
    writer.append_data(np.asarray(pic))
    if abs(s['t']-4.70)<.055:pic.save(still)
 if not still.exists():raise RuntimeError('Expected frame not found')
 gif=out_dir/'four_support_rear_single_step.gif'
 subprocess.run(['ffmpeg','-y','-hide_banner','-loglevel','error','-i',str(out),'-vf','fps=6,scale=640:-1:flags=bilinear',str(gif)],check=True)
 return str(out),str(gif),str(still)

if __name__=='__main__':
 import argparse
 p=argparse.ArgumentParser();p.add_argument('receipt',type=Path);p.add_argument('dest',type=Path)
 a=p.parse_args();print(render(a.receipt,a.dest))
