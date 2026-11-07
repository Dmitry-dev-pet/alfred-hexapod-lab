#!/usr/bin/env python3
"""Schematic 3D playback of recorded MuJoCo qpos; NOT original wheelbot CAD."""
from pathlib import Path
import json, math
import numpy as np
from scipy.spatial.transform import Rotation
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, FFMpegWriter
from mpl_toolkits.mplot3d.art3d import Line3DCollection

ROOT=Path(__file__).resolve().parents[2]/'results'/'wheelbot-offline-proxy'
raw=json.loads((ROOT/'v1.json').read_text())
frames=raw['frames']
fig=plt.figure(figsize=(8.5,5.2),dpi=120)
ax=fig.add_subplot(111,projection='3d')
fig.patch.set_facecolor('#0c1521');ax.set_facecolor('#101b2b')
ax.set_xlim(-.18,.18);ax.set_ylim(-.18,.18);ax.set_zlim(-.03,.32)
ax.set_box_aspect((1,1,1));ax.view_init(elev=20,azim=-75)
ax.set_xlabel('X, м');ax.set_ylabel('Y, м');ax.set_zlabel('Z, м')
ax.tick_params(colors='#b1c6d5',labelsize=7)
for axis in (ax.xaxis,ax.yaxis,ax.zaxis):
 axis.label.set_color('#b1c6d5')
 axis._axinfo['grid']['color']=(.28,.38,.47,.22)
 ax.xaxis.pane.fill=False;ax.yaxis.pane.fill=False;ax.zaxis.pane.fill=False

hx,hy,hz=.043,.0515,.043
verts=np.array([[sx*hx,sy*hy,sz*hz] for sx in [-1,1] for sy in [-1,1] for sz in [-1,1]])
edges=[(i,j) for i in range(8) for j in range(i+1,8) if np.count_nonzero(verts[i]!=verts[j])==1]
bodylines=[ax.plot([0,0],[0,0],[0,0],lw=2,color='#76c8ea')[0] for _ in edges]
phi=np.linspace(0,2*math.pi,90)
wheel_lines=[ax.plot([0,0],[0,0],[0,0],lw=3,color=color)[0] for color in ('#ffbf69','#f97e65')]
shaft_lines=[ax.plot([0,0],[0,0],[0,0],lw=1.8,color='#dae8f3')[0] for _ in range(2)]
ax.plot([-2,2],[0,0],[0,0],lw=.3,alpha=.3)
text=fig.text(.035,.92,'Mini Wheelbot V2 · физический прогон',color='#eaf4ff',size=16,weight='bold')
meta=fig.text(.035,.86,'Схематическая геометрия · физические координаты из MuJoCo · исходные CAD/STL не включены',color='#b7cce0',size=8)
status=fig.text(.035,.055,'',color='#ffffff',size=11)
result=fig.text(.71,.055,'',color='#ffbf69',size=11)

def move_world(R, pos, xyz):return R@xyz+pos

def update(idx):
 f=frames[idx]
 t=f[0];pos=np.array(f[1:4],float)
 q=np.array(f[4:8]); R=Rotation.from_quat([q[1],q[2],q[3],q[0]]).as_matrix()
 corners=(R@verts.T).T+pos
 for line,(i,j) in zip(bodylines,edges):
  p=corners[[i,j]]
  line.set_data(p[:,0],p[:,1]);line.set_3d_properties(p[:,2])
 # Source STEP motor axes represented in model: lower X, upper -Y.
 for wheelidx,(zoff,normal,ang) in enumerate([(-.0325,np.array([1,0,0.]),f[8]),(.0325,np.array([0,-1.,0.]),f[9])]):
  center=pos+R@np.array([0,0,zoff])
  # Choose perpendicular basis for a wheel disk in plane normal to axis.
  if wheelidx==0: u0=np.array([0.,1,0]);v0=np.array([0.,0,1]);
  else: u0=np.array([1.,0,0]);v0=np.array([0.,0,1]);
  # Angles originate from recorded physical joint qpos, not synthesized.
  u=np.cos(ang)*u0+np.sin(ang)*v0
  v=-np.sin(ang)*u0+np.cos(ang)*v0
  disk=center[:,None]+R@(.032*(u[:,None]*np.cos(phi)+v[:,None]*np.sin(phi)))
  wheel_lines[wheelidx].set_data(disk[0],disk[1]);wheel_lines[wheelidx].set_3d_properties(disk[2])
  p=np.stack([center-.022*R@normal,center+.022*R@normal])
  shaft_lines[wheelidx].set_data(p[:,0],p[:,1]);shaft_lines[wheelidx].set_3d_properties(p[:,2])
 tilt=f[10];omega=f[11];torque=f[12]
 status.set_text(f't={t:5.2f} с   Наклон {tilt:6.1f}°   Тяга {torque:+.2f} Н·м')
 result.set_text(f'ω={omega:6.0f} рад/с')
 return bodylines+wheel_lines+shaft_lines+[status,result]

anim=FuncAnimation(fig,update,frames=len(frames),interval=50,blit=False)
print('render',len(frames),'frames',flush=True)
mp4=ROOT/'Wheelbot_Proxy_Physical_MuJoCo_20261107.mp4'
anim.save(mp4,writer=FFMpegWriter(fps=20,codec='libx264',extra_args=['-pix_fmt','yuv420p','-crf','24']),dpi=120)
update(80);png=ROOT/'Wheelbot_Proxy_Keyframe_20261107.png';fig.savefig(png,facecolor=fig.get_facecolor(),bbox_inches='tight')
plt.close(fig)
print(mp4,mp4.stat().st_size,flush=True)
