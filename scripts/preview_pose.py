"""Kinematic drawing only: not a MuJoCo simulation."""
import math
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from hexlab.mujoco_model import LEGS, LINKS


def draw(out=Path('results/robot_pose_kinematic.png')):
    fig=plt.figure(figsize=(9,6.5))
    ax=fig.add_subplot(111,projection='3d')
    h=0.165
    for i, (name, base, azimuth) in enumerate(LEGS):
        a=math.radians(azimuth)
        ux,uy=math.cos(a),math.sin(a)
        points=[(base[0],base[1],h)]
        r,z=LINKS[0],0
        points.append((base[0]+ux*r,base[1]+uy*r,h+z))
        r+=LINKS[1]*math.cos(.55); z-=LINKS[1]*math.sin(.55)
        points.append((base[0]+ux*r,base[1]+uy*r,h+z))
        r+=LINKS[2]*math.cos(.55+.70); z-=LINKS[2]*math.sin(.55+.70)
        points.append((base[0]+ux*r,base[1]+uy*r,h+z))
        ax.plot([p[0] for p in points],[p[1] for p in points],[p[2] for p in points],lw=3,marker='o',markersize=3,label=name.replace('_',' ') if i==0 else None)
        ax.text(points[-1][0],points[-1][1],0,name.replace('_',' '),fontsize=6,ha='center')
    ax.plot([-0.125,0.125,0.125,-0.125,-0.125],[-0.078,-0.078,0.078,0.078,-0.078],[h]*5,lw=3)
    ax.scatter([0],[0],[h],s=60,marker='x')
    X=[-.3,-.3,.3,.3];Y=[-.3,.3,.3,-.3]
    ax.plot(X+[X[0]],Y+[Y[0]],[0]*5,ls='--',lw=1,alpha=.5)
    ax.set(xlabel='X, м (вперёд)',ylabel='Y, м',zlabel='Z, м',zlim=(0,.26))
    ax.set_box_aspect((1.3,1,0.5))
    ax.view_init(elev=26,azim=-60)
    fig.suptitle('Freenove FNK0052: приближённая геометрия 18 DoF')
    fig.text(.5,.04,'Схема начальной позы по принятой геометрии, не результат физической симуляции',ha='center',fontsize=9)
    plt.tight_layout(rect=[0,0.05,1,0.96]); out.parent.mkdir(parents=True,exist_ok=True); fig.savefig(out,dpi=170);print(out)

if __name__=='__main__':draw()