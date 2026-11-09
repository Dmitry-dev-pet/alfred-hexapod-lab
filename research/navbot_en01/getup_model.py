"""v5 mechanics with separately declared CAD outer-hull floor contacts.

v5 CAD hulls only contacted one another. Fallen poses additionally need the
original part envelopes to contact the floor. Generic floor boxes/capsules are
disabled: otherwise they can prop the visible CAD above the surface. Inertias, self-contact exclusions,
actuators, operating ranges and timestep are unchanged. Hulls fill concavities;
this remains a conservative contact approximation, not calibrated hardware.
"""
import base64
import hashlib
import xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
import mujoco
from scipy.spatial import ConvexHull
from .contact_model import create as baseline, load_mesh
from .linkage_export import part_groups

V5_SHA = '04f24a31a9c1f72e6f8599c266876ec6bdf17f2819ed79f510d1c145c82663f3'


def create(mesh_path):
    xml = baseline(mesh_path)
    if hashlib.sha256(xml.encode()).hexdigest() != V5_SHA:
        raise ValueError('Frozen v5 baseline changed')
    source = load_mesh(mesh_path)
    root = ET.fromstring(xml)
    root.set('model', 'EN01_v7_CAD_FLOOR_SELF_RIGHTING_SURROGATE')
    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    groups = part_groups()
    fmt = lambda a: ' '.join(f'{float(v):.15g}' for v in a)
    assets = root.find('asset')
    for geom in root.findall('.//geom'):
        if geom.get('name','').endswith('_geom'):
            geom.set('contype','0')
            geom.set('conaffinity','0')
    for row in source['parts']:
        pid = row['id']
        group = groups[pid]
        vertices = np.frombuffer(base64.b64decode(row['vertices_f32le']), dtype='<f4').reshape(-1, 3).astype(float)
        b = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, group)
        vertices -= data.xpos[b] - data.xpos[1]
        vertices = vertices[ConvexHull(vertices).vertices]
        name = f'floorcad_{pid}'
        ET.SubElement(assets, 'mesh', name=name, vertex=fmt(vertices.ravel()))
        body = root.find(f'.//body[@name="{group}"]')
        ET.SubElement(body, 'geom', name=name, type='mesh', mesh=name,
                      mass='0', contype='8', conaffinity='1', margin='0', priority='1',
                      solref='-1000000 -1000', solimp='.999 .999 .001')
    ET.indent(root, space='  ')
    result = ET.tostring(root, encoding='unicode')+'\n'
    changed = mujoco.MjModel.from_xml_string(result)
    for name in ('body_mass', 'body_inertia', 'body_ipos', 'jnt_range', 'actuator_ctrlrange'):
        if not np.array_equal(getattr(model, name), getattr(changed, name)):
            raise ValueError(f'Mechanics changed: {name}')
    return result


def cad_floor_minimum(mesh_path, model, data):
    """Minimum Z of all original triangle vertices at a measured physical pose."""
    source = load_mesh(mesh_path)
    rest = mujoco.MjData(model)
    mujoco.mj_forward(model, rest)
    minimum = float('inf')
    for row in source['parts']:
        b = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, part_groups()[row['id']])
        vertices = np.frombuffer(base64.b64decode(row['vertices_f32le']), dtype='<f4').reshape(-1, 3).astype(float)
        vertices -= rest.xpos[b]-rest.xpos[1]
        xyz = vertices @ data.xmat[b].reshape(3,3).T + data.xpos[b]
        minimum = min(minimum, float(xyz[:, 2].min()))
    return minimum
