"""Draw predicted static support polygons from recorded CAD30 MuJoCo contact locations."""
import argparse,json
from pathlib import Path
import numpy as np
from scipy.spatial import ConvexHull
import matplotlib.pyplot as plt

p=argparse.ArgumentParser();p.add_argument('--input',type=Path,required=True);p.add_argument('--out',type=Path,required=True);args=p.parse_args()
r=json.loads(args.input.read_text()); loc=r['contact_point_world_xyz_m']; codes=list(loc);xy=np.array([loc[c][:2] for c in codes]);cm=np.array(r['center_of_mass_xyz_m'][:2]);
fig,ax=plt.subplots(figsize=(8.2,6.2),dpi=160)
for ids,sty,label in [(codes,'-', 'All six supports'),([c for c in codes if c not in ('fl','fr')],'--','Both front supports removed')]:
    pts=np.array([loc[c][:2] for c in ids]);h=ConvexHull(pts);order=np.r_[h.vertices,h.vertices[0]]
    ax.plot(pts[order,0],pts[order,1],sty,linewidth=2,label=label)
ax.scatter(xy[:,0],xy[:,1],marker='o',s=48,zorder=3)
for c,pos in zip(codes,xy):
    ax.annotate(c.upper(),(pos[0],pos[1]),xytext=(6,6),textcoords='offset points',fontsize=10)
ax.scatter([cm[0]],[cm[1]],marker='x',s=110,linewidths=2,zorder=4,label='Measured model COM projection')
ax.axhline(0,linewidth=.7,alpha=.2);ax.axvline(0,linewidth=.7,alpha=.2)
ax.set_xlabel('World x, m');ax.set_ylabel('World y (front positive), m')
ax.set_title('FNK0052 CAD30: static support polygons, nominal stand')
ax.set_aspect('equal',adjustable='box');ax.legend(loc='lower left',fontsize=9);ax.grid(alpha=.22)
fig.tight_layout();args.out.parent.mkdir(parents=True,exist_ok=True);fig.savefig(args.out)
print(args.out)