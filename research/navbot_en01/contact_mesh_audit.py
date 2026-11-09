"""Independent original triangle-mesh intersection check for measured body poses."""
from __future__ import annotations
import base64, json
from pathlib import Path
from .contact_model import load_mesh
from .linkage_export import part_groups


def joint_mates(a, b):
    if a == b:
        return True
    if a == 'torso' or b == 'torso':
        return (b if a == 'torso' else a).split('_')[-1] in ('upper', 'rod')
    side_a, kind_a = a.split('_')
    side_b, kind_b = b.split('_')
    return side_a == side_b and frozenset((kind_a, kind_b)) in (
        frozenset(('upper', 'calf')), frozenset(('calf', 'wheel')), frozenset(('rod', 'calf')))


class MeshAudit:
    def __init__(self, mesh_path, model_xml):
        import fcl
        import mujoco
        import numpy as np
        self.fcl, self.np, self.mujoco = fcl, np, mujoco
        source = load_mesh(mesh_path)
        model = mujoco.MjModel.from_xml_string(model_xml)
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        self.model = model
        self.groups = part_groups()
        self.body_names = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, b) for b in range(1, model.nbody)]
        self.bodies = [self.body_names.index(self.groups[i]) for i in range(73)]
        self.objects = []
        for row in source['parts']:
            pid = row['id']
            vertices = np.frombuffer(base64.b64decode(row['vertices_f32le']), dtype='<f4').reshape(-1, 3).astype(float)
            vertices -= data.xpos[self.bodies[pid]+1]-data.xpos[1]
            faces = np.frombuffer(base64.b64decode(row['indices_u32le']), dtype='<u4').reshape(-1, 3).astype('int32')
            mesh = fcl.BVHModel()
            mesh.beginModel(len(faces), len(vertices))
            mesh.addSubModel(vertices, faces)
            mesh.endModel()
            self.objects.append(fcl.CollisionObject(mesh))
        self.pairs = [(i, j) for i in range(73) for j in range(i+1, 73)
                      if not joint_mates(self.groups[i], self.groups[j])]
        self.request = fcl.CollisionRequest(num_max_contacts=1, enable_contact=True)

    def intersections(self, poses):
        np, fcl = self.np, self.fcl
        for i, obj in enumerate(self.objects):
            pose = poses[self.bodies[i]]
            rotation = np.empty(9)
            self.mujoco.mju_quat2Mat(rotation, np.asarray(pose[3:7], dtype=float))
            obj.setTransform(fcl.Transform(rotation.reshape(3, 3), np.asarray(pose[:3])))
        hits = []
        for i, j in self.pairs:
            result = fcl.CollisionResult()
            if fcl.collide(self.objects[i], self.objects[j], self.request, result):
                hits.append({'parts': [i, j], 'groups': [self.groups[i], self.groups[j]],
                             'depth_m': max((max(0., c.penetration_depth) for c in result.contacts), default=0.)})
        return hits


def audit_record(record, mesh_path, model_xml):
    audit = MeshAudit(mesh_path, model_xml)
    if record['body_names'] != audit.body_names:
        raise ValueError('Unexpected body order')
    failures = []
    for index, frame in enumerate(record['frames']):
        hits = audit.intersections(frame['body_poses'])
        if hits:
            failures.append({'frame': index, 't': frame['t'], 'hits': hits})
    return {'schema': 'en01-original-mesh-contact-audit/v1',
            'saved_frames': len(record['frames']), 'unexpected_intersection_frames': len(failures),
            'pairs_per_frame': len(audit.pairs), 'failures': failures,
            'scope': 'Original 73-part tessellation; intended rigid joint mating groups excluded. '
                     'Rod/upper is checked despite its conservative hull-contact exclusion. '
                     'This is a saved-frame surface audit, not exact continuous solid certification.'}
