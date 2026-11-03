"""Two-panel replay from real integrated MuJoCo qpos; never animate body by hand."""
from pathlib import Path
import subprocess,json
import mujoco
import numpy as np
from PIL import Image,ImageDraw,ImageFont

ROOT=Path(__file__).resolve().parent
CASE='nominal'; MU='0p6'; SEED=11
MODES=('neighbor_inhibit','neighbor_force_gate_supported')
LABELS={'neighbor_inhibit':'ТОЛЬКО СИГНАЛЫ СОСЕДЕЙ','neighbor_force_gate_supported':'СОСЕДИ + НАГРУЗКА НА НОГИ'}


def render():
 reports={mode:json.loads((ROOT/'results'/f'load_feedback_{mode}_seed{SEED}_{CASE}_mu{MU}.json').read_text()) for mode in MODES}
 model=mujoco.MjModel.from_xml_path(str(ROOT/'source/mjcf/scene_local30_nominal.xml'))
 data=mujoco.MjData(model)
 cam=mujoco.MjvCamera();cam.type=mujoco.mjtCamera.mjCAMERA_FREE
 cam.azimuth=138;cam.elevation=-16;cam.distance=.81;cam.lookat[:]=(0.,.058,.13)
 W,H=512,384;fps=8
 frames=int(8*fps)
 out=ROOT/'results'/'load_feedback_seed11_mu06_comparison.mp4'
 gif=out.with_suffix('.gif')
 proc=subprocess.Popen(['ffmpeg','-hide_banner','-loglevel','error','-y','-f','rawvideo','-pix_fmt','rgb24',
        '-s:v',f'{W*2}x{H}','-r',str(fps),'-i','-','-c:v','libx264','-preset','fast','-crf','23',
        '-pix_fmt','yuv420p','-movflags','+faststart',str(out)],stdin=subprocess.PIPE,stderr=subprocess.PIPE)
 try:
  font=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',16)
  smaller=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',13)
 except OSError:font=ImageFont.load_default();smaller=font
 lookup={m:{'state_t':np.array([x['t'] for x in r['qpos_frames']]),'trace_t':np.array([x['t'] for x in r['trace_50hz']])}for m,r in reports.items()}
 try:
  with mujoco.Renderer(model,height=H,width=W) as renderer:
   for k in range(frames):
    t=(k+.5)/fps
    canvas=Image.new('RGB',(W*2,H),(16,21,31))
    for j,mode in enumerate(MODES):
     report=reports[mode];lk=lookup[mode]
     sample=report['qpos_frames'][int(np.argmin(abs(lk['state_t']-t)))]
     met=report['trace_50hz'][int(np.argmin(abs(lk['trace_t']-t)))]
     data.qpos[:]=sample['qpos'];data.qvel[:]=0.;mujoco.mj_forward(model,data)
     renderer.update_scene(data,camera=cam)
     swing=set(met['swinging']);chosen={model.geom(f'tibia{x}_visual_0').id for x in swing}
     for a in range(renderer.scene.ngeom):
      g=renderer.scene.geoms[a]
      if g.objtype==mujoco.mjtObj.mjOBJ_GEOM and g.objid in chosen:g.rgba[:]=(1.,.36,.05,1.)
     frame=Image.fromarray(renderer.render());dr=ImageDraw.Draw(frame)
     dr.rounded_rectangle((8,7,W-8,66),radius=5,fill=(20,29,40))
     dr.text((17,11),LABELS[mode],font=font,fill=(240,242,245))
     dr.text((17,37),f't={t:4.1f} с  |  корпус +{met["advance_m"]*100:.1f} см  |  опор: {met["n_contacts"]}',font=smaller,fill=(240,242,245))
     dr.rectangle((8,H-31,W-8,H-8),fill=(20,29,40))
     dr.text((17,H-27),'Перенос: '+(', '.join(sorted(swing)) if swing else 'нет'),font=smaller,fill=(235,235,235))
     canvas.paste(frame,(j*W,0))
    if k in (9,32,53):canvas.save(ROOT/'results'/f'load_feedback_video_frame_{k}.png')
    proc.stdin.write(canvas.tobytes())
  proc.stdin.close();stderr=proc.stderr.read().decode(errors='replace');ret=proc.wait()
  if ret:raise RuntimeError('FFmpeg failed: '+stderr[-1000:])
 except Exception:
  proc.kill();proc.wait();raise
 subprocess.run(['ffmpeg','-hide_banner','-loglevel','error','-y','-i',str(out),
                 '-vf','fps=6,scale=840:-1:flags=lanczos','-loop','0',str(gif)],check=True)
 return {'mp4':str(out),'gif':str(gif),'n_frames':frames,'bytes':[out.stat().st_size,gif.stat().st_size]}

if __name__=='__main__':print(json.dumps(render(),ensure_ascii=False))
