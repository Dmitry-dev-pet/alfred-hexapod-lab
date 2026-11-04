"""Recorded MuJoCo replay and measured support-polygon plot for rear relocation.

The two front legs remain airborne. This is NOT four-legged walking or a
learned local neural reflex. Do not fabricate poses or root movement.
"""
import argparse,json,subprocess
from pathlib import Path
import mujoco
import numpy as np
from PIL import Image,ImageDraw,ImageFont
import imageio.v2 as imageio
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from front_pair_trick import SUPPORT,contact_points,margin
ROOT=Path(__file__).resolve().parent
PROJECT=ROOT if (ROOT/'source/mjcf').is_dir() else ROOT.parents[1]

def make_visuals(receipt:Path,dest:Path):
    result=json.loads(receipt.read_text())
    if not result['safe_rear_reposition'] or not result['qpos_frames']:
        raise ValueError('Requires a fully validated recorded rear relocation')
    dest.mkdir(parents=True,exist_ok=True)
    m=mujoco.MjModel.from_xml_path(str(PROJECT/'source/mjcf'/f"scene_local30_{result['mass']}.xml"))
    d=mujoco.MjData(m)
    for name in ['floor']:
        m.geom_friction[m.geom(name).id,0]=result['mu']
    for c in ('fr','fl','ml','bl','br','mr'):
        m.geom_friction[m.geom(f'tibia{c}_collision_0').id,0]=result['mu']
    views=[]
    for az,el in [(300,-19),(215,-28)]:
        c=mujoco.MjvCamera();c.type=mujoco.mjtCamera.mjCAMERA_FREE
        c.azimuth=az;c.elevation=el;c.distance=.81;c.lookat[:]=[0.,-.035,.14]
        views.append(c)
    try:font=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',12)
    except OSError:font=ImageFont.load_default()
    w,h=360,260
    path=dest/'rear_foothold_reposition_v1.mp4'
    frames=[v for v in result['qpos_frames'] if 2.9<=v['t']<=8.8]
    picpath=dest/'rear_reposition_keyframe.png'
    with imageio.get_writer(path,fps=10,codec='libx264',quality=6,macro_block_size=16) as wr:
        with mujoco.Renderer(m,width=w,height=h) as renderer:
            for snap in frames:
                d.qpos[:]=snap['qpos'];d.qvel[:]=0;mujoco.mj_forward(m,d)
                pic=Image.new('RGB',(2*w,h+54),(17,21,30))
                for i,cam in enumerate(views):
                    renderer.update_scene(d,camera=cam)
                    pic.paste(Image.fromarray(renderer.render().copy()),(w*i,54))
                dr=ImageDraw.Draw(pic)
                phase='REAR FOOT RELOCATION' if 3.5<=snap['t']<6.1 else 'CHECK FOUR-SUPPORT STANCE'
                dr.text((10,7),f"FNK0052 CAD30   {snap['t']:.1f}s  |  {phase}",font=font,fill=(245,247,249))
                dr.text((10,29),'RIGHT REAR FOOT REPOSITIONED  |  MIDDLE FOOT NOT LIFTED  |  NOT WALKING',font=font,fill=(244,186,118))
                wr.append_data(np.asarray(pic))
                if abs(snap['t']-5.0)<.051:pic.save(picpath)
    gif=dest/'rear_foothold_reposition_v1.gif'
    subprocess.run(['ffmpeg','-y','-hide_banner','-loglevel','error','-i',str(path),'-vf','fps=6,scale=720:-1:flags=bilinear',str(gif)],check=True,timeout=40)
    # One plot: independently measured 3-foot support triangles, before/after.
    fig,ax=plt.subplots(figsize=(9,6.2))
    candidate=result['candidate']
    for wanted,label,linestyle,marker in [(3.3,'Before','--','x'),(7.0,'After','-','o')]:
        snap=min(result['qpos_frames'],key=lambda x:abs(x['t']-wanted))
        d.qpos[:]=snap['qpos'];d.qvel[:]=0;mujoco.mj_forward(m,d)
        cp=contact_points(m,d)
        com=d.subtree_com[m.body('base_link_2').id][:2]
        pts=[]
        for c in SUPPORT:
            if not cp[c]:continue
            point=np.mean(cp[c],axis=0)
            if c!=candidate:pts.append(point)
            if label=='After':
                ax.annotate(c,(point[0]*100,point[1]*100),xytext=(4,5),textcoords='offset points',fontsize=9)
        if len(pts)==3:
            pts=np.array(pts);idx=[0,1,2,0]
            ax.plot(pts[idx,0]*100,pts[idx,1]*100,linestyle=linestyle,linewidth=2.7,
                    label=f'{label}: remaining three contacts')
            mar=margin(com,pts)
            print(label,'mean-foot-point COM margin',round(mar*1000,2),'mm')
        ax.scatter([com[0]*100],[com[1]*100],marker=marker,s=110,linewidth=2.6,
                   label=f'{label}: COM')
    ax.set_title('Right rear foot relocated: the support triangle changes')
    ax.set_xlabel('X position (cm)');ax.set_ylabel('Y position (cm)')
    ax.set_xlim(-27,27);ax.set_ylim(-23,5)
    ax.set_aspect('equal','box');ax.grid(alpha=.2)
    ax.legend(loc='upper center',bbox_to_anchor=(.5,-.16),fontsize=8,frameon=False,ncol=2)
    fig.subplots_adjust(left=.08,right=.97,top=.91,bottom=.28)
    plot=dest/'rear_foothold_support_geometry.png'
    fig.savefig(plot,dpi=160)
    plt.close(fig)
    return [str(path),str(gif),str(picpath),str(plot)]

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('receipt',type=Path);p.add_argument('dest',type=Path);args=p.parse_args()
    print(make_visuals(args.receipt,args.dest))
