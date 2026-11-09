"""CAD outer-envelope self contact and conservative operating guards.

Working bounds are not measured factory stops or a servo-zero calibration.
Original CAD mesh and generated MJCF remain external geometry assets.
"""
from __future__ import annotations
import argparse, base64, hashlib, json
from pathlib import Path
import xml.etree.ElementTree as ET
from .linkage_model import create as old_create
from .linkage_export import part_groups

CAD_SHA256 = '8b034155dc50bb6c4e94a55bfa4ebca7b1fb742f2f38fc5c6c9f158c0cae313b'
PARTS = (0,1,2,9,10,11,18,20,22,23,24,25,26,27,28,29,40,41,42,43,44,52,53,54,56,62,63,72)
OPERATING_RANGE = (0., .65)
TIMESTEP = .00025


def load_mesh(path):
    raw = Path(path).read_bytes()
    if hashlib.sha256(raw).hexdigest() != CAD_SHA256:
        raise ValueError('Pinned CAD mesh mismatch')
    return json.loads(raw)


def create(mesh_path, *, fixed_base=False, gravity=True, timestep=TIMESTEP, floor_contact=True):
    import mujoco
    import numpy as np
    from scipy.spatial import ConvexHull
    source = load_mesh(mesh_path)
    root = ET.fromstring(old_create(fixed_base=fixed_base, gravity=gravity, timestep=timestep))
    root.set('model', 'EN01_v5_CAD_CONTACT_OPERATING_GUARDS')
    root.find('option').set('tolerance', '1e-12')
    if not floor_contact:
        root.find('.//geom[@name="floor"]').attrib.update(contype='0', conaffinity='0')
    model = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding='unicode'))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    fmt = lambda a: ' '.join(f'{float(x):.15g}' for x in a)
    # Preserve prior assumed inertia, independently of the added contact shapes.
    for b in range(1, model.nbody):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, b)
        body = root.find(f'.//body[@name="{name}"]')
        ET.SubElement(body, 'inertial', pos=fmt(model.body_ipos[b]), quat=fmt(model.body_iquat[b]),
                      mass=str(model.body_mass[b]), diaginertia=fmt(model.body_inertia[b]))
    for joint in root.findall('.//joint'):
        if joint.get('name', '').startswith('hip_'):
            joint.attrib.update(range='0 .65', margin='.01', solreflimit='-1000000 -1000',
                                solimplimit='.9999 .9999 .001')
    assets = ET.SubElement(root, 'asset')
    groups = part_groups()
    for pid in PARTS:
        row = source['parts'][pid]
        vertices = np.frombuffer(base64.b64decode(row['vertices_f32le']), dtype='<f4').reshape(-1, 3).astype(float)
        b = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, groups[pid])
        vertices -= data.xpos[b]-data.xpos[1]
        vertices = vertices[ConvexHull(vertices).vertices]
        name = 'cad_'+str(pid)
        ET.SubElement(assets, 'mesh', name=name, vertex=fmt(vertices.flatten()))
        body = root.find(f'.//body[@name="{groups[pid]}"]')
        ET.SubElement(body, 'geom', name=name, type='mesh', mesh=name, mass='0',
                      contype='4', conaffinity='4', solref='-1000000 -1000',
                      solimp='.999 .999 .001', margin='.00025', gap='0')
    contacts = ET.SubElement(root, 'contact')
    for side in ('left', 'right'):
        # Make mating exclusions explicit for fixed-base as well as free-base tests.
        ET.SubElement(contacts, 'exclude', body1='torso', body2=side+'_upper')
        ET.SubElement(contacts, 'exclude', body1='torso', body2=side+'_rod')
        ET.SubElement(contacts, 'exclude', body1=side+'_rod', body2=side+'_calf')
        # Hulls fill link holes. Check the actual rod/upper triangle surfaces
        # independently instead of allowing these false hull contacts.
        ET.SubElement(contacts, 'exclude', body1=side+'_rod', body2=side+'_upper')
    ET.indent(root, space='  ')
    return ET.tostring(root, encoding='unicode')+'\n'


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--cad-mesh', type=Path, required=True)
    p.add_argument('--out', type=Path, required=True)
    a = p.parse_args()
    xml = create(a.cad_mesh)
    a.out.parent.mkdir(parents=True, exist_ok=True)
    a.out.write_text(xml)
    print(hashlib.sha256(xml.encode()).hexdigest())


if __name__ == '__main__':
    main()
