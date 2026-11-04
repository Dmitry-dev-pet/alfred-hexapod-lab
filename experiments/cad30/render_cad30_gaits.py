"""Render real joint states from saved physical local MuJoCo simulation.
No fake translation/animation; moving camera only follows the physical root.
"""
from pathlib import Path
import subprocess, json, sys
import numpy as np
import mujoco
from PIL import Image, ImageDraw, ImageFont
ROOT=Path(__file__).resolve().parents[2]

def run(gait='tripod',case='nominal',fps=7):
    receipt=json.loads((ROOT/'results'/f'cad30_gait_{gait}_{case}.json').read_text())
    samples=receipt['qpos_samples']
    root=ROOT/'source'/'mjcf'/f'scene_local30_{case}.xml'
    model=mujoco.MjModel.from_xml_path(str(root));data=mujoco.MjData(model)
    cam=mujoco.MjvCamera();cam.type=mujoco.mjtCamera.mjCAMERA_FREE
    cam.azimuth=140.;cam.elevation=-16.;cam.distance=.83
    frames=int(receipt['duration_s']*fps)
    width,height=640,480
    out=ROOT/'results'/f'fnk0052_cad30_{gait}_{case}.mp4'
    gif=out.with_suffix('.gif')
    args=['ffmpeg','-hide_banner','-loglevel','error','-y','-f','rawvideo','-pix_fmt','rgb24',
        '-s:v',f'{width}x{height}','-r',str(fps),'-i','-','-c:v','libx264','-preset','veryfast',
        '-crf','23','-pix_fmt','yuv420p','-movflags','+faststart',str(out)]
    try:font=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',19)
    except OSError:font=ImageFont.load_default()
    proc=subprocess.Popen(args,stdin=subprocess.PIPE,stderr=subprocess.PIPE)
    ts=np.array([x['t'] for x in samples]);traces=receipt['trace'];tr_ts=np.array([x['t'] for x in traces]);b=model.body('base_link_2').id
    with mujoco.Renderer(model,height=height,width=width) as renderer:
        for i in range(frames):
            t=(i+.5)/fps
            snap=samples[int(np.argmin(abs(ts-t)))]
            data.qpos[:]=snap['qpos'];data.qvel[:]=0;mujoco.mj_forward(model,data)
            root_xyz=data.xpos[b]
            cam.lookat[:]=(float(root_xyz[0]),float(root_xyz[1]),.12)
            renderer.update_scene(data,camera=cam)
            img=Image.fromarray(renderer.render())
            d=ImageDraw.Draw(img)
            metrics=traces[int(np.argmin(abs(tr_ts-t)))]
            d.rounded_rectangle((8,8,width-8,45),radius=7,fill=(23,29,40))
            d.text((18,17),f'{gait.upper()} | t={t:4.1f}s | body forward={metrics["advance_m"]*100:5.1f} cm | contacts={metrics["n_contacts"]}',font=font,fill='white')
            d.rectangle((8,height-30,width-8,height-6),fill=(23,29,40))
            d.text((18,height-27),f'Freenove 30/32 original STL - MuJoCo 3.14 - {case} mass',fill=(235,240,245))
            proc.stdin.write(img.tobytes())
    proc.stdin.close();err=proc.stderr.read().decode();ret=proc.wait()
    if ret:raise RuntimeError(f'FFmpeg encoding failed: {err[-1000:]}')
    subprocess.run(['ffmpeg','-hide_banner','-loglevel','error','-y','-i',str(out),'-vf','fps=7,scale=480:-1:flags=lanczos', '-loop','0',str(gif)],check=True)
    return {'gait':gait,'frames':frames,'mp4':str(out),'mp4_bytes':out.stat().st_size,'gif':str(gif),'gif_bytes':gif.stat().st_size}

if __name__=='__main__':
    for gait in ('tripod','wave'):
        print(json.dumps(run(gait),ensure_ascii=False))
