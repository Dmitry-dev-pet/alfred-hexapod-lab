"""HWeigo-size diagnostic: independent free shell/chassis, torque-driven wheel contact.

The hollow shell is an explicitly ASSUMED 3 mm wall. Its inner boundary is a
refinable set of convex spherical triangular wedges, never a convex solid ball.
Motor, mass, inertia and friction values are hypotheses, not hardware ratings.
"""
from __future__ import annotations
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as E
import numpy as np

PIN = '7318411cfbfbffdbd68f75e3734a50c3933969e5'
ASSEMBLY = 'bcfead6e92329575aa474fb4be05c24191ac5294'
CENTER = np.array([-41.24816673006777, -27.2571967211034, 99.26489749059236]) * .001
ALIGN = np.array([[1.,0,0],[0,0,1],[0,-1,0]])
RADIUS = .1
AXLE = np.array([-.00313716624742838, .00073510250940764, -.03922508767425143])
RINGS = {
    'left': [(-.00333770339745348,-.0665148976315104,-.0388259314293029),
             (-.00343186483176787,-.0615148974905925,-.0391730996818734)],
    'right':[(-.00267690342588543,.0679851025094074,-.0389137175412861),
             (-.00280140852314103,.0629851025094074,-.0388413651455990)]}

def fmt(v): return ' '.join(format(float(x), '.12g') for x in np.ravel(v))

def sphere_faces(level):
    """Octahedron subdivisions; shared vertices are normalized onto unit sphere."""
    v = np.array([[1,0,0],[-1,0,0],[0,1,0],[0,-1,0],[0,0,1],[0,0,-1]],float)
    f = [(0,2,4),(2,1,4),(1,3,4),(3,0,4),(2,0,5),(1,2,5),(3,1,5),(0,3,5)]
    triangles = [v[list(t)] for t in f]
    for _ in range(level):
        result=[]
        for a,b,c in triangles:
            ab=(a+b)/np.linalg.norm(a+b);bc=(b+c)/np.linalg.norm(b+c);ca=(c+a)/np.linalg.norm(c+a)
            result.extend([np.array(x) for x in [(a,ab,ca),(ab,b,bc),(ca,bc,c),(ab,bc,ca)]])
        triangles=result
    return triangles

def make_model(level=3, segments=48, friction=.8, wall=.003, timestep=.001,
               wheel_contact=True, fixed_chassis=False, parts=None, fit_translation=None,allow_assumed_mass=False):
    if not allow_assumed_mass:
        raise ValueError('BB8 component masses are unverified. Explicit allow_assumed_mass is required for diagnostic-only work.')
    root=E.Element('mujoco',model='HWeigo_BB8_CONTACT_DIAGNOSTIC_v2')
    E.SubElement(root,'compiler',angle='radian',inertiafromgeom='false')
    E.SubElement(root,'option',timestep=str(timestep),gravity='0 0 -9.81',integrator='implicitfast',
                 solver='Newton',iterations='60',tolerance='1e-10')
    default=E.SubElement(root,'default')
    E.SubElement(default,'geom',condim='1' if friction==0 else '3',solref='.006 1',solimp='.95 .99 .001',margin='0',gap='0',
                 friction=f'{friction} 0 0',mass='0')
    assets=E.SubElement(root,'asset');world=E.SubElement(root,'worldbody')
    E.SubElement(world,'geom',name='floor',type='plane',size='3 3 .1',contype='1',conaffinity='1',friction='.9 0 0',condim='3')
    shell=E.SubElement(world,'body',name='shell',pos='0 0 .1001')
    E.SubElement(shell,'freejoint',name='shell_free')
    inertia=2/3*.2*.1**2
    E.SubElement(shell,'inertial',pos='0 0 0',mass='.2',diaginertia=fmt([inertia]*3))
    E.SubElement(shell,'geom',name='shell_outer',type='sphere',size='.1',contype='1',conaffinity='1',friction='.9 0 0',condim='3',rgba='.8 .85 .9 .25')
    triangles=sphere_faces(level);inner=RADIUS-wall;max_sag=0.
    # Each wedge is its own convex hull, allowing internal wheel contacts.
    for i,t in enumerate(triangles):
        normal=np.cross(t[1]-t[0],t[2]-t[0]);normal/=np.linalg.norm(normal)
        plane=abs(float(normal@t[0]));max_sag=max(max_sag,inner*(1-plane))
        vertices=np.concatenate([t*inner,t*(RADIUS+.0001)])
        E.SubElement(assets,'mesh',name=f'wedge{i}',vertex=fmt(vertices))
        E.SubElement(shell,'geom',name=f'cavity{i}',type='mesh',mesh=f'wedge{i}',contype='2',conaffinity='12',rgba='.5 .65 .8 .15')
    offset=np.array([0.,0,.1001])+(np.array(fit_translation) if fit_translation is not None else 0)
    chassis=E.SubElement(world,'body',name='chassis',pos=fmt(offset))
    if not fixed_chassis:E.SubElement(chassis,'freejoint',name='chassis_free')
    E.SubElement(chassis,'inertial',pos='-.003 .000735 .008',mass='.38',diaginertia='.0008 .0006 .0009')
    E.SubElement(chassis,'geom',name='chassis_display',type='box',size='.04 .045 .04',pos='-.003 .000735 .008',contype='0',conaffinity='0',rgba='.3 .4 .6 1')
    act=E.SubElement(root,'actuator')
    bodies={'chassis':chassis};axles={}
    for side,rings in RINGS.items():
        axle=AXLE.copy();axle[1]=sum(p[1] for p in rings)/2
        body=E.SubElement(chassis,'body',name=side+'_wheel',pos=fmt(axle))
        bodies[side]=body;axles[side]=axle
        E.SubElement(body,'joint',name=side+'_axle',type='hinge',axis='0 1 0',damping='.00001',armature='.000002')
        E.SubElement(body,'inertial',pos='0 0 0',mass='.025',diaginertia='.00001 .00002 .00001')
        for ring,p in enumerate(rings):
            center=np.array(p)-axle
            for j in range(segments):
                # Capsule chain approximates the exact 25mm major-radius torus.
                a=2*math.pi*j/segments;b=2*math.pi*(j+1)/segments
                p1=center+np.array([.025*math.cos(a),0,.025*math.sin(a)])
                p2=center+np.array([.025*math.cos(b),0,.025*math.sin(b)])
                E.SubElement(body,'geom',name=f'{side}_ring{ring}_{j}',type='capsule',size='.0025',fromto=fmt([*p1,*p2]),
                             contype='4' if wheel_contact else '0',conaffinity='2' if wheel_contact else '0',rgba='.15 .15 .15 1')
        E.SubElement(act,'motor',name=side+'_motor_ASSUMED',joint=side+'_axle',gear='1',ctrllimited='true',ctrlrange='-.02 .02')
    collision_map={}
    if parts is not None:
        from scipy.spatial import ConvexHull
        wheel_groups={'left':{'c07','c09','c11','c28'},'right':{'c08','c10','c16','c29'}}
        for cid,points_mm in parts.items():
            if cid in {'c12','c13','c46','c47'}:continue  # exact-size capsule tori above
            group=next((s for s,ids in wheel_groups.items() if cid in ids),'chassis')
            points=points_mm*.001-(axles[group] if group!='chassis' else 0)
            points=points[ConvexHull(points).vertices]
            E.SubElement(assets,'mesh',name='cad_'+cid,vertex=fmt(points))
            # Whole caster hull has small sliding friction. Ball rotation is not reconstructed.
            mu=min(.02,friction) if cid in {'c35','c36','c37','c38'} else friction
            E.SubElement(bodies[group],'geom',name='cad_'+cid,type='mesh',mesh='cad_'+cid,
                         contype='8' if wheel_contact or group=='chassis' else '0',conaffinity='2',
                         friction=f'{mu} 0 0',priority='1',rgba='.35 .45 .55 1')
            collision_map[cid]={'body':group,'hull_vertices':len(points)}
    meta={'source_commit':PIN,'source_assembly_blob_sha1':ASSEMBLY,'shell_outer_radius_m':RADIUS,
          'wall_thickness_m_ASSUMED':wall,'shell_inner_radius_m_ASSUMED':inner,'sphere_subdivision':level,
          'cavity_wedges':len(triangles),'max_inner_faceting_error_m':max_sag,
          'tire_segments_per_ring':segments,'max_tire_chord_error_m':.025*(1-math.cos(math.pi/segments)),
          'wheel_friction_ASSUMED':friction,'torque_limit_Nm_per_wheel_ASSUMED':.02,
          'motor_no_load_speed_rpm_ASSUMED':200.,
          'mass_ASSUMED_kg':{'shell':.2,'chassis':.38,'each_wheel':.025},
          'head_in_dynamics':False,'wheel_contact':wheel_contact,'fixed_chassis':fixed_chassis,
          'no_shell_chassis_joint':True,'no_shell_actuator':True,'cad_body_collisions_checked':parts is not None,
          'source_collision_hulls':collision_map,'initial_rigid_translation_m':list(fit_translation) if fit_translation is not None else [0,0,0],
          'limitations':['Source-part convex hulls collide only against hollow shell; no internal gear/bearing collision',
                         'Upper caster hulls slide with assumed friction .02; no separate freely rolling ball or preload',
                         'Source gear ratio, mass, COM, inertia, motor curve and wall thickness not calibrated']}
    xml=E.tostring(root,encoding='unicode')
    meta['xml_sha256']=hashlib.sha256(xml.encode()).hexdigest()
    return xml,meta

if __name__=='__main__':
    import argparse
    p=argparse.ArgumentParser();p.add_argument('--out',type=Path,required=True);p.add_argument('--level',type=int,default=3)
    p.add_argument('--allow-assumed-mass',action='store_true')
    a=p.parse_args();a.out.mkdir(parents=True,exist_ok=True);xml,meta=make_model(level=a.level,allow_assumed_mass=a.allow_assumed_mass)
    (a.out/'model.xml').write_text(xml);(a.out/'model.json').write_text(json.dumps(meta,indent=2)+'\n')
