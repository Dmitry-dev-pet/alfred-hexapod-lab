"""Fit source-placed internal CAD in a spherical cavity without modifying parts.

The enclosing-ball calculation is a necessary geometric fit test, not a motion,
mass, motor, gear-mate or head-retention validation. Distances are in mm here.
"""
import argparse,hashlib,json,math,struct
from pathlib import Path
import numpy as np
from scipy.optimize import minimize
from scipy.spatial import ConvexHull

HEAD={'c24','c25','c30','c31','c32','c33'}
SHELL={'c22','c23'}
TIRES={'c12','c13','c46','c47'}
CENTER=np.array([-41.24816673006777,-27.2571967211034,99.26489749059236])
ALIGN=np.array([[1.,0,0],[0,0,1],[0,-1,0]])

def rotation(q):
    x,y,z,w=q
    if abs(np.linalg.norm(q)-1)>1e-8:raise ValueError('Non-unit source quaternion')
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])

def vertices(path,expected):
    raw=path.read_bytes()
    if hashlib.sha256(raw).hexdigest()!=expected:raise ValueError('Mesh SHA256 mismatch: '+str(path))
    n=struct.unpack_from('<I',raw,80)[0]
    if len(raw)!=84+50*n:raise ValueError('Not exact binary STL')
    v=np.frombuffer(raw,dtype=np.dtype([('n','<f4',(3,)),('v','<f4',(3,3)),('a','<u2')]),offset=84)['v'].reshape(-1,3)
    if not np.isfinite(v).all():raise ValueError('Nonfinite STL')
    return np.unique(v.astype(float),axis=0)

def load_parts(source,tire):
    scene=json.loads((source/'visual/scene.json').read_text())
    if scene['source_commit']!='7318411cfbfbffdbd68f75e3734a50c3933969e5':raise ValueError('Source revision mismatch')
    original=(source/'original/Model/初装.SLDASM').read_bytes()
    blob=hashlib.sha1(b'blob '+str(len(original)).encode()+b'\0'+original).hexdigest()
    if blob!='bcfead6e92329575aa474fb4be05c24191ac5294':raise ValueError('Assembly SHA mismatch')
    parts={};rows=[]
    for instance in scene['instances']:
        cid=instance['id'];m=scene['meshes'][instance['part_key']]
        if cid in SHELL|HEAD:continue
        file=source/'meshes'/m['file'];expected=m['sha256']
        if cid in TIRES:
            file=tire;expected='9d1790d179a52d8b26da8945cd273a1f0a64ea25b8cf0a8e3037cf10cb1717c7'
        local=vertices(file,expected)
        pts=(local@rotation(instance['rotation_xyzw']).T+np.array(instance['position_mm'])-CENTER)@ALIGN.T
        parts[cid]=pts
        rows.append({'id':cid,'source_file':m['source_file'],'mesh_sha256':expected,
                     'max_radius_at_source_pose_mm':float(np.linalg.norm(pts,axis=1).max()),
                     'source_position_mm':instance['position_mm'],'source_rotation_xyzw':instance['rotation_xyzw']})
    return parts,rows

def enclosing_ball(parts):
    p=np.concatenate(list(parts.values()));h=p[ConvexHull(p).vertices]
    # Center c of minimal enclosing ball; the physical assembly translation is -c.
    def con(x):return x[3]-np.linalg.norm(h-x[:3],axis=1)
    def jac(x):
        delta=h-x[:3];norm=np.linalg.norm(delta,axis=1)
        return np.c_[delta/norm[:,None],np.ones(len(h))]
    start=np.r_[np.zeros(3),np.linalg.norm(h,axis=1).max()]
    fit=minimize(lambda x:x[3],start,jac=lambda x:np.array([0.,0,0,1]),method='SLSQP',
                 constraints=[{'type':'ineq','fun':con,'jac':jac}],options={'ftol':1e-11,'maxiter':500})
    if not fit.success or min(con(fit.x))< -1e-7:raise RuntimeError('Enclosing ball optimization failed: '+fit.message)
    # Supporting-point convex combination is an independently checkable lower bound.
    dist=np.linalg.norm(h-fit.x[:3],axis=1);active=h[dist>fit.x[3]-1e-6]
    from scipy.optimize import linprog
    lp=linprog(np.zeros(len(active)),A_eq=np.r_[active.T,np.ones((1,len(active)))],
               b_eq=np.r_[fit.x[:3],1.],bounds=[(0,None)]*len(active),method='highs')
    if not lp.success:raise RuntimeError('No optimal supporting-point certificate')
    w=lp.x;lower=math.sqrt(float(np.sum(w*np.sum(active**2,axis=1))-np.dot(fit.x[:3],fit.x[:3])))
    return {'minimum_enclosing_radius_mm':float(fit.x[3]),'rigid_translation_from_source_pose_mm':(-fit.x[:3]).tolist(),
            'support_points_mm':active.tolist(),'support_weights':w.tolist(),'radius_lower_bound_mm':lower,
            'certificate_gap_mm':float(fit.x[3]-lower),'input_vertices':len(p),'convex_hull_vertices':len(h)}

def audit(source,tire):
    parts,rows=load_parts(source,tire);fit=enclosing_ball(parts)
    trans=np.array(fit['rigid_translation_from_source_pose_mm'])
    for row in rows:
        row['max_radius_after_best_rigid_translation_mm']=float(np.linalg.norm(parts[row['id']]+trans,axis=1).max())
    radius=fit['minimum_enclosing_radius_mm']
    return {'schema':'alfred-bb8-source-cavity-fit/v2','source_repo':'HWeigo/BB8_Robot',
            'source_commit':'7318411cfbfbffdbd68f75e3734a50c3933969e5',
            'assembly_blob_sha1':'bcfead6e92329575aa474fb4be05c24191ac5294',
            'excluded_shell_instances':sorted(SHELL),'excluded_external_head_instances':sorted(HEAD),
            'internal_instances':len(rows),'fitted_geometry':'verified source STL vertices with corrected complete source torus; source articulation frozen',
            'fit':fit,'max_uniform_wall_that_fits_source_mesh_mm':100-radius,
            'wall_scenarios':[{'wall_mm':wall,'inner_radius_mm':100-wall,'clearance_at_best_translation_mm':100-wall-radius,
                               'fits_this_rigid_mesh_envelope':radius<=100-wall} for wall in [1.,1.5,2.,2.5,3.]],
            'parts':rows,'limitations':['STL tessellation error in source CAD not bounded; exact BRep not certified',
                                       'Necessary static fit only; wheel rotation, servo articulation and tolerances not covered',
                                       'Wall thickness absent from imported sphere; scenarios are assumptions',
                                       'No mesh/shell modification or hardware inference from this test']}

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--source',type=Path,required=True);p.add_argument('--tire',type=Path,required=True);p.add_argument('--out',type=Path,required=True)
    a=p.parse_args();r=audit(a.source,a.tire);a.out.parent.mkdir(parents=True,exist_ok=True);a.out.write_text(json.dumps(r,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps({k:v for k,v in r.items() if k not in ['parts']},indent=2))
