from pathlib import Path
import json,sys
import mujoco
from PIL import Image,ImageDraw
ROOT=Path(__file__).resolve().parents[2]
scene=ROOT/'source/mjcf/scene_local30_nominal.xml'
model=mujoco.MjModel.from_xml_path(str(scene))
data=mujoco.MjData(model)
receipt=json.loads((ROOT/'results/cad30_stand_nominal.json').read_text())
data.qpos[:]=receipt['hold_test']['final_qpos'];mujoco.mj_forward(model,data)
cam=mujoco.MjvCamera();cam.type=mujoco.mjtCamera.mjCAMERA_FREE
cam.lookat[:]=(0,0,.11);cam.distance=.82;cam.azimuth=125;cam.elevation=-21
with mujoco.Renderer(model,height=480,width=640) as r:
    r.update_scene(data,camera=cam); pic=Image.fromarray(r.render())
    d=ImageDraw.Draw(pic);d.rounded_rectangle((10,10,570,60),radius=8,fill=(20,27,37));d.text((24,25),'Freenove CAD — 30/32 original STL | six-leg stable stand',fill='white')
    path=ROOT/'results/fnk0052_cad30_stand.png';pic.save(path)
    print('SAVED',path,'size',path.stat().st_size,'numgeoms',model.ngeom,'mesh',model.nmesh)
