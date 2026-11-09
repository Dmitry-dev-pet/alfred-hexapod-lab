"""Component-level *estimated* mass profiles. No hardware mass is inferred from CAD labels.

The source photograph supports a polymer-plate hypothesis, not a measured material.
Pololu 4752 is a 37D-class reference, not an identified HWeigo component.
"""
from pathlib import Path
import hashlib,json,math,sys
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'contact_v2'))
from audit_geometry import rotation,CENTER,ALIGN
from model import RINGS,AXLE

WHEELS={'left':{'c07','c09','c11','c28','c12','c46'},'right':{'c08','c10','c16','c29','c13','c47'}}
EXCLUDED={'c22','c23','c24','c25','c30','c31','c32','c33'}
PARAMS={
 'light':dict(polymer=1040.,motor=.15,battery=.06,pcb=.025,servo=.009,bracket=2700.,extras=.04),
 'nominal':dict(polymer=1180.,motor=.20,battery=.09,pcb=.035,servo=.009,bracket=7800.,extras=.07),
 'heavy':dict(polymer=1240.,motor=.23,battery=.14,pcb=.055,servo=.014,bracket=7800.,extras=.12)}

def solid_moments(path,sha):
    b=path.read_bytes()
    if hashlib.sha256(b).hexdigest()!=sha:raise ValueError('STL hash mismatch')
    tri=np.frombuffer(b,offset=84,dtype=np.dtype([('n','<f4',(3,)),('v','<f4',(3,3)),('a','<u2')]))['v'].astype(float)*.001
    a,c,d=tri[:,0],tri[:,1],tri[:,2]
    w=np.einsum('ij,ij->i',a,np.cross(c,d))/6
    volume=w.sum()
    if abs(volume)<1e-12:raise ValueError('Degenerate mass envelope')
    center=np.sum(w[:,None]*(a+c+d)/4,axis=0)/volume
    s=a+c+d
    second=(np.einsum('i,ij,ik->jk',w,s,s)+sum(np.einsum('i,ij,ik->jk',w,p,p) for p in [a,c,d]))/20/volume
    covariance=second-np.outer(center,center)
    inertia=np.trace(covariance)*np.eye(3)-covariance
    if np.linalg.eigvalsh(inertia).min()<=0:raise ValueError('Non-positive mass-envelope inertia')
    return abs(float(volume)),center,inertia

def group_for(cid):return next((k for k,v in WHEELS.items() if cid in v),'chassis')

def axle(side):
    a=AXLE.copy();a[1]=np.mean([x[1] for x in RINGS[side]]);return a

def aggregate(components):
    out={}
    for group in ['chassis','left','right']:
        cs=[c for c in components if c['group']==group]
        mass=sum(c['mass_kg'] for c in cs)
        center=sum(c['mass_kg']*np.array(c['center_m']) for c in cs)/mass
        inertia=np.zeros((3,3))
        for c in cs:
            p=np.array(c['center_m'])-center
            inertia+=np.array(c['inertia_COM_kg_m2'])+c['mass_kg']*((p@p)*np.eye(3)-np.outer(p,p))
        if np.linalg.eigvalsh(inertia).min()<=0:raise ValueError('Non-positive aggregate inertia')
        out[group]={'mass_kg':mass,'center_m':center.tolist(),'inertia_COM_kg_m2':inertia.tolist(),'component_count':len(cs)}
    return out

def build(source,tire,profile='nominal',wall=.002):
    p=PARAMS[profile];scene=json.loads((source/'visual/scene.json').read_text());components=[];cache={}
    for item in scene['instances']:
        cid=item['id'];m=scene['meshes'][item['part_key']];name=m['source_file']
        if cid in EXCLUDED:continue
        if item['part_key'] not in cache:
            path=source/'meshes'/m['file'];sha=m['sha256']
            if m['file']=='part-09.stl':path=tire;sha='9d1790d179a52d8b26da8945cd273a1f0a64ea25b8cf0a8e3037cf10cb1717c7'
            cache[item['part_key']]=solid_moments(path,sha)
        volume,lc,li=cache[item['part_key']]
        status='MATERIAL_AND_DENSITY_ESTIMATE';evidence='Geometric volume and assumed actual material; not CAD cached mass'
        if cid in {'c02','c04'}:
            mass=p['motor'];status='UNIDENTIFIED_MOTOR_CLASS_PROXY'
            evidence='https://www.pololu.com/product/4752/specs : 200 g; class reference only, not matched HWeigo motor'
        elif cid=='c54':mass=p['battery'];status='UNIDENTIFIED_BATTERY_ESTIMATE';evidence='60/90/140 g design scenarios; actual battery unknown; original schematic says 12 V'
        elif cid=='c48':mass=p['pcb'];status='PCB_ENVELOPE_ESTIMATE';evidence='25/35/55 g design scenarios; actual board model/mass unconfirmed'
        elif cid=='c39':mass=p['servo'];status='SERVO_SPEC_REFERENCE';evidence='https://towerpro.com.tw/product/sg90-analog/ : 9 g; actual servo/clone not confirmed'
        else:
            if '铜' in name and '座板' not in name and '连接' not in name:density=8500.
            elif '轮轴' in name or '联轴器' in name:density=7800.
            elif '电机支架' in name:density=p['bracket']
            elif name.startswith('M1-'):density=1410.
            elif '轮胎' in name:density=1100.
            elif '轮子' in name:density=1200.
            elif '牛眼轮' in name:density=7800.
            else:density=p['polymer']
            mass=volume*density
        R=ALIGN@rotation(item['rotation_xyzw']);center=R@lc+ALIGN@(np.array(item['position_mm'])*.001-CENTER*.001)
        # audit_geometry.CENTER is in mm; convert it exactly once.
        inertia=mass*(R@li@R.T);group=group_for(cid)
        if group!='chassis':center-=axle(group)
        components.append({'id':cid,'source_file':name,'group':group,'mass_kg':float(mass),'center_m':center.tolist(),
                           'inertia_COM_kg_m2':inertia.tolist(),'status':status,'evidence':evidence,
                           'inertia_assumption':'uniform source envelope rescaled to selected mass; internal material distribution unmeasured'})
    board=next(c for c in components if c['id']=='c48')['center_m']
    for name,fraction,z in [('driver_power_receiver',.55,.02),('wiring_fasteners_magnets',.45,.005)]:
        mass=p['extras']*fraction
        components.append({'id':'extra_'+name,'source_file':None,'group':'chassis','mass_kg':mass,
                           'center_m':[board[0],board[1],z],'inertia_COM_kg_m2':(np.eye(3)*mass*.015**2*.4).tolist(),
                           'status':'MISSING_FROM_CAD_ESTIMATE','evidence':'Mass and mounting location unresolved; explicit sensitivity profile',
                           'inertia_assumption':'15mm effective-radius isotropic distribution'})
    groups=aggregate(components)
    R=.1;r=R-wall;shell_mass=4*math.pi/3*(R**3-r**3)*p['polymer']
    shell_I=.4*shell_mass*(R**5-r**5)/(R**3-r**3)
    groups['shell']={'mass_kg':shell_mass,'center_m':[0.,0.,0.],'inertia_COM_kg_m2':(np.eye(3)*shell_I).tolist(),'component_count':1}
    total=sum(g['mass_kg'] for g in groups.values())
    inner=sum(g['mass_kg'] for k,g in groups.items() if k!='shell')
    return {'schema':'alfred-bb8-component-mass-scenarios/v3','profile':profile,'hardware_calibrated':False,
            'source_commit':scene['source_commit'],'assumptions':p,'wall_m':wall,'components':components,'groups':groups,
            'total_mass_without_head_kg':total,'internal_mass_kg':inner,'head_excluded':True,
            'limitations':['No weighed HWeigo components; ranges are hypotheses, not confidence intervals',
                           'Source photograph suggests polymer plates; actual material not confirmed',
                           '37D-class motor proxy is not an identified substitution or purchase selection',
                           'Head and its magnetic/dynamic load absent; full robot not validated']}

if __name__=='__main__':
    import argparse
    ap=argparse.ArgumentParser();ap.add_argument('--source',type=Path,required=True);ap.add_argument('--tire',type=Path,required=True);ap.add_argument('--out',type=Path,required=True)
    a=ap.parse_args();a.out.mkdir(parents=True,exist_ok=True)
    for name in PARAMS:
        d=build(a.source,a.tire,name);(a.out/(name+'.json')).write_text(json.dumps(d,ensure_ascii=False,indent=2)+'\n')
        print(name,'total',d['total_mass_without_head_kg'],'chassis_COM',d['groups']['chassis']['center_m'])
