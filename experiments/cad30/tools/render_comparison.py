"""Two-panel replay of actually integrated qpos, not fake root movement."""
from pathlib import Path
import subprocess, json
import mujoco
import numpy as np
from PIL import Image,ImageDraw,ImageFont

ROOT=next((p for p in Path(__file__).resolve().parents if (p/'decentralized_ablation.py').exists()), None)
if ROOT is None: raise RuntimeError('Missing decentralized_ablation.py in experiment folder')
MODES=('independent','neighbor_inhibit')
LABELS={'independent':'БЕЗ СООБЩЕНИЙ','neighbor_inhibit':'ЛОКАЛЬНОЕ ТОРМОЖЕНИЕ'}
SEED=23

def render():
    reports={mode:json.loads((ROOT/'results'/f'decentral_{mode}_seed{SEED}_nominal_mu1p1.json').read_text()) for mode in MODES}
    model=mujoco.MjModel.from_xml_path(str(ROOT/'source/mjcf/scene_local30_nominal.xml'))
    data=mujoco.MjData(model)
    cam=mujoco.MjvCamera()
    cam.type=mujoco.mjtCamera.mjCAMERA_FREE
    cam.azimuth=138
    cam.elevation=-16
    cam.distance=.81
    cam.lookat[:]=(0.,.058,.13)
    W,H=512,390
    fps=8
    frames=int(8*fps)
    path=ROOT/'results'/f'decentralized_seed{SEED}_independent_vs_neighbor.mp4'
    gif=path.with_suffix('.gif')
    proc=subprocess.Popen(['ffmpeg','-hide_banner','-loglevel','error','-y','-f','rawvideo','-pix_fmt','rgb24',
       '-s:v',f'{W*2}x{H}','-r',str(fps),'-i','-','-c:v','libx264','-preset','fast','-crf','23',
       '-pix_fmt','yuv420p','-movflags','+faststart',str(path)],stdin=subprocess.PIPE,stderr=subprocess.PIPE)
    try:
        font=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',17)
        fontsm=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',14)
    except OSError:font=ImageFont.load_default();fontsm=font
    references={}
    for mode,r in reports.items():
        references[mode]={
            'state_t':np.array([z['t'] for z in r['qpos_frames']]),
            'trace_t':np.array([z['t'] for z in r['trace_50hz']]),
        }
    leg_objids={mode:tuple(model.geom('tibia'+c+'_visual_0').id for c in ('ml','bl','br','mr','fr','fl')) for mode in MODES}
    with mujoco.Renderer(model,height=H,width=W) as renderer:
        for i in range(frames):
            t=(i+.5)/fps
            canvas=Image.new('RGB',(W*2,H),(22,31,42))
            for j,mode in enumerate(MODES):
                report=reports[mode]
                k=int(np.argmin(abs(references[mode]['state_t']-t)))
                z=int(np.argmin(abs(references[mode]['trace_t']-t)))
                sample=report['qpos_frames'][k]
                met=report['trace_50hz'][z]
                data.qpos[:]=sample['qpos'];data.qvel[:]=0.;mujoco.mj_forward(model,data)
                renderer.update_scene(data,camera=cam)
                # Highlight in orange only original tibia visual meshes whose legs are swinging.
                swing=set(met['swinging'])
                chosen={model.geom(f'tibia{n}_visual_0').id for n in swing}
                for idx in range(renderer.scene.ngeom):
                    geom=renderer.scene.geoms[idx]
                    if geom.objtype==mujoco.mjtObj.mjOBJ_GEOM and geom.objid in chosen:
                        geom.rgba[:]=(1.,.36,.05,1.)
                frame=Image.fromarray(renderer.render())
                draw=ImageDraw.Draw(frame)
                draw.rounded_rectangle((10,7,W-10,60),radius=5,fill=(18,26,34))
                draw.text((18,10),LABELS[mode],font=font,fill=(250,250,250))
                draw.text((18,33),f't={t:.1f} c | пройдено {met["advance_m"]*100:.1f} см | опор: {met["n_contacts"]}',font=fontsm,fill=(243,244,247))
                draw.rectangle((10,H-34,W-10,H-8),fill=(18,26,34))
                swing_text=', '.join(sorted(swing)) if swing else 'нет'
                draw.text((18,H-30),f'Перенос: {swing_text} (оранжевый)',font=fontsm,fill=(240,240,240))
                canvas.paste(frame,(j*W,0))
            if i in (8,33,56):canvas.save(ROOT/'results'/f'decentral_comparison_frame{i}.png')
            proc.stdin.write(canvas.tobytes())
    proc.stdin.close();stderr=proc.stderr.read().decode();ret=proc.wait()
    if ret: raise RuntimeError(stderr[-500:])
    subprocess.run(['ffmpeg','-hide_banner','-loglevel','error','-y','-i',str(path),'-vf','fps=7,scale=840:-1:flags=lanczos','-loop','0',str(gif)],check=True)
    return {'mp4':str(path),'gif':str(gif),'bytes':[path.stat().st_size,gif.stat().st_size],'frames':frames}

if __name__=='__main__':print(json.dumps(render(),ensure_ascii=False))