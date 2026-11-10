"""Blender: render original CAD meshes at a saved MuJoCo state; no new motion."""
import json
import math
from pathlib import Path
import sys

import bpy
from mathutils import Vector, Matrix, Quaternion

args=sys.argv[sys.argv.index('--')+1:]
assets,trial,case,output=Path(args[0]),Path(args[1]),args[2],Path(args[3])
manifest=json.loads((trial/'asset-manifest.json').read_text())
replay=json.loads((trial/(case+'-replay.json')).read_text())
frame=replay['frames'][-1]
bpy.ops.object.select_all(action='SELECT');bpy.ops.object.delete(use_global=False)

def material(name,color,metal=.1):
    m=bpy.data.materials.new(name);m.diffuse_color=(*color,1);m.use_nodes=True
    bs=m.node_tree.nodes.get('Principled BSDF');bs.inputs['Base Color'].default_value=(*color,1)
    bs.inputs['Metallic'].default_value=metal;bs.inputs['Roughness'].default_value=.5
    return m

mats={
 'base':material('chassis',(.12,.19,.24)),
 'coxa':material('hip',(.15,.31,.4)),
 'femur':material('upper leg',(.75,.51,.18)),
 'tibia':material('lower leg',(.37,.42,.46))}
for i,part in enumerate(manifest['meshes']):
    bpy.ops.wm.stl_import(filepath=str(assets/part['file']))
    obj=bpy.context.object;obj.name=part['body']
    obj.data.transform(Matrix.Scale(.001,4))
    pos=Vector(frame[8+7*i:11+7*i]);quat=Quaternion(frame[11+7*i:15+7*i])
    obj.matrix_world=Matrix.Translation(pos)@quat.to_matrix().to_4x4()@Matrix.Translation(Vector(part['local_translation_m']))
    key=part['body'].split('_')[-1]
    obj.data.materials.append(mats[key if key in mats else 'base'])

floor_mat=material('floor',(.48,.51,.54),0)
bpy.ops.mesh.primitive_plane_add(size=6,location=(0,0,-.0005));bpy.context.object.data.materials.append(floor_mat)
if 'step' in case:
    height=.01 if '10mm' in case else .02
    bpy.ops.mesh.primitive_cube_add(size=1,location=(0,1.11,height/2))
    bpy.context.object.scale=(1.2,1.78,height)
    bpy.context.object.data.materials.append(material('step',(.30,.33,.37),0))

center=Vector((frame[1],frame[2],frame[3]+.015))
bpy.ops.object.camera_add(location=center+Vector((.56,-.62,.47)))
camera=bpy.context.object;camera.rotation_euler=(center-camera.location).to_track_quat('-Z','Y').to_euler()
camera.data.type='ORTHO';camera.data.ortho_scale=.68;bpy.context.scene.camera=camera
for location,power,size in (((.8,-.4,1.5),120,1.5),((-.6,.8,1.0),75,1.1)):
    bpy.ops.object.light_add(type='AREA',location=center+Vector(location))
    light=bpy.context.object;light.data.energy=power;light.data.shape='DISK';light.data.size=size
    light.rotation_euler=(center-light.location).to_track_quat('-Z','Y').to_euler()
scene=bpy.context.scene;scene.render.engine='CYCLES';scene.cycles.device='CPU';scene.cycles.samples=32
scene.cycles.use_denoising=False
for layer in scene.view_layers:layer.cycles.use_denoising=False
scene.world.color=(.22,.22,.22)
scene.render.resolution_x=960;scene.render.resolution_y=720;scene.render.resolution_percentage=100
scene.render.image_settings.file_format='PNG';scene.render.filepath=str(output)
bpy.ops.render.render(write_still=True)
