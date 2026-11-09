"""Render original meshes at measured states; produces a PNG, no synthetic motion."""
import argparse,json
from pathlib import Path
import mujoco
import numpy as np
from PIL import Image,ImageDraw,ImageFont

def main():
    p=argparse.ArgumentParser();p.add_argument("--results",type=Path,required=True);p.add_argument("--out",type=Path,required=True);a=p.parse_args()
    m=mujoco.MjModel.from_xml_path(str((a.results/"scene.xml").resolve()));d=mujoco.MjData(m)
    names=["front","back","left","right","upside-down"]
    font=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',18)
    sheet=Image.new('RGB',(1600,5*280+65),'#142333');draw=ImageDraw.Draw(sheet)
    draw.text((14,12),'OpenBeni | measured recovery | contact time constant 5 ms',font=font,fill='white')
    with mujoco.Renderer(m,240,400) as r:
        opt=mujoco.MjvOption();opt.geomgroup[:]=0;opt.geomgroup[1]=1;opt.geomgroup[0]=1
        for row,name in enumerate(names):
            trace=json.loads((a.results/(name+'.json')).read_text());samples=trace['samples'];onset=trace['result']['recovery_onset_s']
            times=[.5,.75,.5+onset+1.,samples[-1]['t']]
            for col,t in enumerate(times):
                s=min(samples,key=lambda x:abs(x['t']-t));d.qpos[:]=s['qpos'];d.qvel[:]=s['qvel'];mujoco.mj_forward(m,d)
                cam=mujoco.MjvCamera();mujoco.mjv_defaultFreeCamera(m,cam)
                cam.azimuth=130;cam.elevation=-18;cam.distance=.55;cam.lookat[:]=[d.qpos[0],d.qpos[1],.09]
                r.update_scene(d,cam,scene_option=opt)
                sheet.paste(Image.fromarray(r.render()),(col*400,65+row*280))
                draw.text((col*400+8,65+row*280+243),f'{name}: {s["t"]:.2f}s | {s["tilt_deg"]:.1f} deg',font=font,fill='white')
    a.out.parent.mkdir(parents=True,exist_ok=True);sheet.save(a.out);print(a.out)

if __name__=='__main__':main()
