"""Honest render of recorded real MuJoCo qpos from the negative reflex experiment.
Original CAD STL must be provided separately (never embedded in GIF/video archive).
"""
import json,subprocess,argparse
from pathlib import Path
import numpy as np,mujoco
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from PIL import Image, ImageDraw, ImageFont
import imageio.v2 as imageio
ROOT=Path(__file__).resolve().parent

def make_plot(baseline:dict,reflex:dict,output_dir:Path):
    a=baseline['trace'];b=reflex['trace'];output_dir.mkdir(parents=True,exist_ok=True)
    fig,ax=plt.subplots(figsize=(9,4.1))
    ax.plot([r['t'] for r in a],[r['load_N']['ml'] for r in a],label='No feedback: middle leg')
    ax.plot([r['t'] for r in b],[r['load_N']['ml'] for r in b],label='Reflex: middle leg')
    ax.axvline(3.6,linestyle='--',alpha=.65,label='Reflex request')
    ax.axhline(.65,linestyle=':',alpha=.6,label='Liftoff load gate (0.65 N)')
    ax.set_xlim(2.6,7.8);ax.set_ylim(bottom=0);ax.set_xlabel('Simulation time, s');ax.set_ylabel('Filtered normal load, N')
    ax.set_title('FNK0052: force-feedback reduces load, but cannot unload middle leg')
    ax.legend(loc='lower left',fontsize=8);ax.grid(alpha=.3)
    fig.tight_layout()
    output=output_dir/'reflex_middle_load_comparison.png';fig.savefig(output,dpi=145);plt.close(fig)
    return output

def render(data:dict,outdir:Path):
    snaps=data['qpos_frames']
    if data['release_at_s'] is not None:raise ValueError('This negative diagnostic expects no leg release')
    if not snaps:raise ValueError('No qpos snapshots')
    outdir.mkdir(exist_ok=True,parents=True)
    m=mujoco.MjModel.from_xml_path(str(ROOT/'source/mjcf/scene_local30_nominal.xml'));d=mujoco.MjData(m)
    w,h=360,260;cams=[]
    for az in (305,220):
        c=mujoco.MjvCamera();c.type=mujoco.mjtCamera.mjCAMERA_FREE;c.azimuth=az;c.elevation=-20;c.distance=.87;c.lookat[:]=[0,-.07,.14];cams.append(c)
    try:font=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',12)
    except OSError:font=ImageFont.load_default()
    mp4=outdir/'reflex_middle_no_liftoff.mp4';picpath=outdir/'reflex_middle_keyframe.png'
    samples=snaps[::2]
    with imageio.get_writer(mp4,fps=8,codec='libx264',quality=6,macro_block_size=16) as writer:
        with mujoco.Renderer(m,width=w,height=h) as renderer:
            for snap in samples:
                d.qpos[:]=snap['qpos'];d.qvel[:]=0;mujoco.mj_forward(m,d)
                pic=Image.new('RGB',(w*2,h+52),(19,23,30));dr=ImageDraw.Draw(pic)
                for i,cam in enumerate(cams):
                    renderer.update_scene(d,camera=cam);pic.paste(Image.fromarray(renderer.render().copy()),(i*w,52))
                phase='PREPARE FOUR-FOOT STANCE' if snap['t']<3.6 else 'SENSOR-DRIVEN SHIFT / UNLOAD REFUSED'
                dr.text((12,7),f"FNK0052 CAD30 | {snap['t']:.1f}s | {phase}",fill=(240,242,245),font=font)
                dr.text((12,27),'DIAGNOSTIC NEGATIVE: no middle-foot lift; not four-legged walking',fill=(245,193,127),font=font)
                writer.append_data(np.asarray(pic))
                if abs(snap['t']-6.0)<.11:pic.save(picpath)
    if not picpath.exists():pic.save(picpath)
    gif=outdir/'reflex_middle_no_liftoff.gif'
    subprocess.run(['ffmpeg','-y','-hide_banner','-loglevel','error','-i',str(mp4),'-vf','fps=5,scale=720:-1:flags=bicubic',str(gif)],check=True,timeout=45)
    return [mp4,gif,picpath]
if __name__=='__main__':
    a=argparse.ArgumentParser();a.add_argument('--reflex',type=Path,default=ROOT/'results/reflex_shift_nominal_mu11.json')
    a.add_argument('--control',type=Path,default=ROOT/'results/no_feedback_nominal_mu11.json')
    a.add_argument('--out',type=Path,default=ROOT.parent/'reflex_media');ns=a.parse_args()
    r=json.loads(ns.reflex.read_text());c=json.loads(ns.control.read_text());print(make_plot(c,r,ns.out));print(render(r,ns.out))
