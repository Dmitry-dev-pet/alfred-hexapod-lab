"""Rebuild mass-bounded FNK0052 from the pinned upstream MJCF in THIS environment.

30/32 Freenove original meshes downloaded and SHA-1 Git-blob-verified.
Missing only topplate and bigservo1: these **visual-only** geometry instances
are replaced with clearly documented box proxies. All 6 real tibia STL
collision hulls and community-derived inertias remain unchanged.

The resulting model is *CAD-based, with 2 visual mesh substitutions*, NOT an
exact 32/32-CAD original and not an experimentally calibrated digital twin.
No GitHub Actions or network needed.
"""
from pathlib import Path
from xml.etree import ElementTree as ET
import json, hashlib, sys

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT))
from hexlab.cad_mujoco_v04 import patch_source_mjcf, git_blob_sha
from hexlab.community_mass_v04 import load, validate, REF_PATH, SCENARIOS_PATH

MISSING={
    'bigservo1.stl': '.020 .012 .020',
    'topplate.stl': '.090 .002 .065',
}


def build(scenario='nominal'):
    src=ROOT/'source'; xml=src/'mjcf'/'base.xml'
    lock=load(ROOT/'data/fnk0052_cad_assets_lock.json')
    ref=load(REF_PATH); masses=load(SCENARIOS_PATH)
    validate(ref,masses)
    for key in ('mjcf/base.xml', 'mjcf/scene.xml'):
        b=(src/key).read_bytes()
        assert len(b)==lock['mjcf'][key]['size_bytes']
        assert git_blob_sha(b)==lock['mjcf'][key]['sha_git_blob']
    verified=0
    for x in lock['meshes']:
        p=src/'meshes'/x['name']
        if p.exists():
            b=p.read_bytes()
            assert len(b)==x['size_bytes'] and git_blob_sha(b)==x['sha_git_blob'],x['name']
            verified+=1
        else:
            assert x['name'] in MISSING, f'Unexpected missing {x["name"]}'
    assert verified==30
    tree=ET.parse(xml); root=tree.getroot()
    diagnostics=patch_source_mjcf(root,masses,scenario)
    mesh_asset=root.find('asset')
    removed=0
    for x in list(mesh_asset.findall('mesh')):
        if x.get('name') in MISSING:
            mesh_asset.remove(x);removed+=1
    assert removed==2
    proxies={name:0 for name in MISSING}
    for body in root.findall('.//worldbody//body'):
        for geom in body.findall('geom'):
            if geom.get('mesh') not in MISSING:
                continue
            assert geom.get('class')=='visual',geom.attrib
            name=geom.attrib.pop('mesh')
            geom.set('type','box')
            geom.set('size',MISSING[name])
            geom.set('rgba','0.55 0.55 0.55 0.9' if name=='bigservo1.stl' else '0.85 0.85 0.85 0.8')
            # Original material can override rgba; remove for visibly distinguished proxies
            geom.attrib.pop('material',None)
            geom.set('mass','0')
            proxies[name]+=1
    assert proxies['bigservo1.stl']>=18,proxies
    assert proxies['topplate.stl']>=1,proxies
    dest=src/'mjcf'/f'base_local30_{scenario}.xml'
    tree.write(dest,encoding='unicode',xml_declaration=True)
    scene=ET.parse(src/'mjcf'/'scene.xml')
    scene.getroot().find('include').set('file',dest.name)
    target=src/'mjcf'/f'scene_local30_{scenario}.xml'
    scene.write(target,encoding='unicode',xml_declaration=True)
    import mujoco
    m=mujoco.MjModel.from_xml_path(str(target))
    assert m.nu==20
    actual=round(float(m.body_mass.sum()),6)
    expected=diagnostics['estimated_assembly_mass_kg']
    assert abs(actual-expected)<=2e-5, (actual,expected)
    assert m.nmesh==30,(m.nmesh,verified)
    tibias=[]
    for suffix in ('ml','bl','br','mr','fr','fl'):
        gid=m.geom(f'tibia{suffix}_collision_0').id
        tibias.append(bool(m.geom_type[gid]==mujoco.mjtGeom.mjGEOM_MESH))
    assert all(tibias),tibias
    result={**diagnostics, 'file':str(target), 'nbody':m.nbody,'nq':m.nq,'nu':m.nu,
        'ngeom':m.ngeom,'nmesh':m.nmesh,'estimated_mass_kg':actual,
        'cad_original_stl_verified':verified,'not_exact_original_visual_CAD':True,
        'visual_only_missing_mesh_proxies':proxies,'real_tibia_contact_meshes':6,
        'local_exec_not_github_actions':True,
        'source_commit':lock['pr_commit']}
    (ROOT/'results'/f'cad30_model_{scenario}.json').write_text(json.dumps(result,indent=2)+'\n')
    return result

if __name__=='__main__':
    for s in ('lower','nominal','upper'):
        print(json.dumps(build(s),indent=2))
