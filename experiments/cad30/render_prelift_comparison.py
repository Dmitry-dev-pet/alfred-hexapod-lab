"""Only render recorded physical MuJoCo qpos; do not animate prescribed gait."""
from __future__ import annotations
import argparse,json,subprocess,sys
from pathlib import Path
import numpy as np
import mujoco
from PIL import Image,ImageDraw,ImageFont
ROOT=next(p for p in Path(__file__).resolve().parents if (p/'hexlab').is_dir())

def main():
 p=argparse.ArgumentParser()
 p.add_argument('--mu',type=float,default=.3)
 p.add_argument('--seed',type=int,default=11)
 p.add_argument('--fps',type=int,default=9)
 a=p.parse_args();mu=str(a.mu).replace('.','p')
 paths=[ROOT/'results'/f'active_support_{mode}_seed{a.seed}_nominal_mu{mu}_press4mm.json'
        for mode in ('neighbor_force_gate_supported','neighbor_pre_lift_transfer')]
 records=[json.loads(p.read_text()) for p in paths]
 xml=str(ROOT/'source/mjcf/scene_local30_nominal.xml')
 model=mujoco.MjModel.from_xml_path(xml);data=mujoco.MjData(model)
 cam=mujoco.MjvCamera();cam.type=mujoco.mjtCamera.mjCAMERA_FREE
 cam.azimuth=140;cam.elevation=-17;cam.distance=.80;cam.lookat[:]=np.array([0,0,.13])
 W,H=420,290;fps=a.fps
 out=ROOT/'results'/f'prelift_compare_mu{mu}_seed{a.seed}_2026-11-04.mp4'
 ff=['ffmpeg','-y','-hide_banner','-loglevel','error','-f','rawvideo','-pix_fmt','rgb24','-s:v',f'{W*2}x{H}',
     '-r',str(fps),'-i','-','-c:v','libx264','-preset','veryfast','-crf','25','-pix_fmt','yuv420p',
     '-movflags','+faststart',str(out)]
 proc=subprocess.Popen(ff,stdin=subprocess.PIPE,stderr=subprocess.PIPE)
 try:font=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',16)
 except Exception:font=ImageFont.load_default()
 labels=('BASELINE: CONTACT + LOAD GATE','NEW: PRE-LIFT LOCAL UNLOAD')
 with mujoco.Renderer(model,height=H,width=W) as renderer:
  for k in range(8*fps):
   t=(k+.5)/fps;ims=[]
   for rec,label in zip(records,labels):
    snap=min(rec['qpos_frames'],key=lambda x:abs(x['t']-t))
    data.qpos[:]=snap['qpos'];data.qvel[:]=0.;mujoco.mj_forward(model,data)
    renderer.update_scene(data,camera=cam)
    im=Image.fromarray(renderer.render());d=ImageDraw.Draw(im)
    sample=min(rec['trace_50hz'],key=lambda x:abs(x['t']-t))
    d.rectangle((4,4,W-4,48),fill=(15,20,29))
    d.text((12,7),label,font=font,fill='white')
    d.text((12,25),f't={t:.1f}s forward={sample["advance_m"]*100:.2f}cm   nfeet={sample["n_contacts"]}   tilt={sample["tilt_deg"]:.1f}deg',font=font,fill='white')
    d.rectangle((4,H-23,W-4,H-4),fill=(15,20,29))
    d.text((12,H-23),f'CAD30 MuJoCo | mu={a.mu}, seed={a.seed} | SIMULATION ONLY',font=font,fill='white')
    ims.append(im)
   canvas=Image.new('RGB',(2*W,H));canvas.paste(ims[0],(0,0));canvas.paste(ims[1],(W,0))
   proc.stdin.write(canvas.tobytes())
 proc.stdin.close();err=proc.stderr.read().decode();rc=proc.wait()
 if rc:raise RuntimeError(err)
 gif=out.with_suffix('.gif')
 subprocess.run(['ffmpeg','-y','-hide_banner','-loglevel','error','-i',str(out),'-vf','fps=7,scale=960:-1:flags=lanczos',
                 '-loop','0',str(gif)],check=True)
 print('video',out, out.stat().st_size,'GIF',gif,gif.stat().st_size)
if __name__=='__main__':main()