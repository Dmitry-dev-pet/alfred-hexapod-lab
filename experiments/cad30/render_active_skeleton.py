"""Two physical MuJoCo qpos replays rendered as accurately projected skeletal joints.
Not a CAD mesh movie: xyz are computed by mj_forward of actual recorded qpos.
"""
import json, subprocess, math, bisect
from pathlib import Path
import numpy as np
import mujoco
from PIL import Image, ImageDraw, ImageFont
from hexlab.stand_geometry_v04 import load_legs, tip_world
HERE=Path(__file__).resolve().parent
P=HERE if (HERE/"source"/"mjcf").exists() else HERE.parents[1]
MODES=('neighbor_force_gate_supported','neighbor_active_support')
R=[json.loads((P/'results'/f'active_support_{mode}_seed11_nominal_mu0p6_press4mm.json').read_text()) for mode in MODES]
model=mujoco.MjModel.from_xml_path(str(P/'source/mjcf/scene_local30_nominal.xml'))
data=mujoco.MjData(model)
legs=load_legs(model,data,mujoco)
rootid=model.body('base_link_2').id
W,H,FPS,SEC=680,550,9,8
try:
 f1=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',20)
 f2=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',16)
 f3=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',13)
except: f1=f2=f3=ImageFont.load_default()
# 3D-to-2D orthographic oblique scientific sketch, in absolute world axes (x,y,z).
def projected(pt,xoffset):
 x,y,z=pt
 s=690
 return (int(xoffset+W/2+s*(0.79*x+0.52*y)),int(353+s*(0.21*x-0.42*y-1.02*z)))
WHITE=(239,247,255); BG=(12,23,36); MUTED=(137,160,183)
PANEL_TITLE=('ТОЛЬКО ПРОВЕРКА НАГРУЗКИ','АКТИВНАЯ ПОДДЕРЖКА СОСЕДЕЙ')
OUT=Path('/mnt/data/hexapod_active_support_skeleton_2026-11-03.mp4')
proc=subprocess.Popen(['ffmpeg','-hide_banner','-loglevel','error','-y','-f','rawvideo','-pix_fmt','rgb24','-s:v',f'{2*W}x{H}','-r',str(FPS),'-i','-','-c:v','libx264','-preset','veryfast','-crf','23','-pix_fmt','yuv420p','-movflags','+faststart',str(OUT)],stdin=subprocess.PIPE,stderr=subprocess.PIPE)
for k in range(FPS*SEC):
 t=min((k+0.5)/FPS,SEC)
 im=Image.new('RGB',(2*W,H),BG);d=ImageDraw.Draw(im)
 for pane,r in enumerate(R):
  left=pane*W
  d.rectangle((left+2,4,left+W-2,H-3),outline=(46,67,89),width=2)
  d.text((left+25,19),PANEL_TITLE[pane],font=f1,fill=WHITE)
  d.text((left+25,48),'Схема суставов из рассчитанной траектории MuJoCo',font=f3,fill=MUTED)
  # floor grid in physical y/x coordinate system, not decorative CAD render
  for v in np.arange(-.35,.42,.05):
   p1=projected((-.33,v,0),left);p2=projected((.33,v,0),left)
   d.line((p1,p2),fill=(31,51,70),width=1)
  for v in np.arange(-.30,.36,.05):
   p1=projected((v,-.35,0),left);p2=projected((v,.4,0),left)
   d.line((p1,p2),fill=(31,51,70),width=1)
  idx=min(len(r['qpos_frames'])-1, int(round(t*9)))
  pos=r['qpos_frames'][idx]
  trace_times=[x['t'] for x in r['trace_50hz']]
  rec=r['trace_50hz'][max(0,bisect.bisect_left(trace_times,t)-1)]
  data.qpos[:]=pos['qpos'];data.qvel[:]=0;mujoco.mj_forward(model,data)
  root=data.xpos[rootid].copy()
  hips={leg['suffix']:np.asarray(data.xanchor[leg['jids'][0]]).copy() for leg in legs}
  # chassis polygon follows hips (physical model anchors)
  hull=['fl','fr','mr','br','bl','ml','fl']
  d.polygon([projected(hips[c],left) for c in hull],fill=(55,76,98),outline=(185,202,223),width=2)
  # coordinate forward arrow along +y
  start=projected((.29,-.20,.003),left);end=projected((.29,-.1,.003),left)
  d.line((start,end),fill=(229,223,110),width=3)
  d.text((end[0]+4,end[1]-6),'вперёд +Y',font=f3,fill=(229,223,110))
  for leg in legs:
   c=leg['suffix'];j0,j1,j2=leg['jids']
   points=[data.xanchor[j0].copy(),data.xanchor[j1].copy(),data.xanchor[j2].copy(),tip_world(model,data,leg)]
   contact=c in rec['contacts'];swing=c in rec['swinging']
   color=(66,223,161) if contact else ((252,159,79) if swing else (160,171,193))
   pts=[projected(p,left) for p in points]
   d.line(pts,fill=color,width=7,joint='curve')
   for ix,(px,py) in enumerate(pts):d.ellipse((px-4,py-4,px+4,py+4),fill=WHITE if ix<3 else color)
   px,py=pts[-1];d.text((px+7,py-10),c,font=f3,fill=color)
  rp=projected(root,left);d.ellipse((rp[0]-8,rp[1]-8,rp[0]+8,rp[1]+8),fill=(248,219,123),outline=WHITE,width=2)
  d.rounded_rectangle((left+23,403,left+W-23,521),radius=9,fill=(22,38,54))
  d.text((left+43,413),f'{t:.1f} с   ·   продвижение {100*rec["advance_m"]:.2f} см   ·   опор {rec["n_contacts"]}/6',font=f2,fill=WHITE)
  d.text((left+43,446),f'Наклон {rec["tilt_deg"]:.2f}°   ·   max момент {rec["max_torque_nm"]:.2f} Н·м',font=f2,fill=(187,209,226))
  d.ellipse((left+43,482,left+59,498),fill=(66,223,161));d.text((left+65,480),'контакт',font=f3,fill=WHITE)
  d.ellipse((left+168,482,left+184,498),fill=(252,159,79));d.text((left+190,480),'перенос',font=f3,fill=WHITE)
  d.text((left+325,480),'μ=0.6  ·  масса 1.805 кг',font=f3,fill=MUTED)
 if k==36:im.save('/mnt/data/hexapod_active_support_keyframe_2026-11-03.png')
 proc.stdin.write(im.tobytes())
proc.stdin.close();error=proc.stderr.read().decode();ret=proc.wait();assert ret==0,error
GIF=OUT.with_suffix('.gif')
subprocess.run(['ffmpeg','-hide_banner','-loglevel','error','-y','-i',str(OUT),'-vf','fps=6,scale=1000:-1:flags=lanczos','-loop','0',str(GIF)],check=True)
print('SUCCESS',[(str(p),p.stat().st_size) for p in (OUT,GIF,Path('/mnt/data/hexapod_active_support_keyframe_2026-11-03.png'))])
