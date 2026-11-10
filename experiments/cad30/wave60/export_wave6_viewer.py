"""Export observed 60s CAD30 six-leg wave into existing Hexapod Lab player formats.

No synthetic positions. Free-body qpos from MuJoCo wave6_60s_experiment.py;
Fz contact mask from nearest observed Fz >= 0.2 N sample.
"""
from pathlib import Path
import json, hashlib, importlib.util, os, sys
import numpy as np
import mujoco as mj
REPO_ROOT=Path(__file__).resolve().parents[3]
BASE=Path(os.environ.get("CAD30_ROOT",str(Path.cwd()))).resolve()
OUT=Path(os.environ.get("HEXAPOD_DOCS_ROOT",str(REPO_ROOT/"docs"))).resolve()/"hexapod"
sys.path.insert(0,str(BASE))
from hexlab.stand_geometry_v04 import load_legs,tip_world
from hexlab.step_one_leg_v04 import forward_from_geometry
CODES=('ml','bl','br','mr','fr','fl')

def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()

def export():
    recpath=BASE/'results'/'cad30_gait_wave_nominal.json'
    fzpath=BASE/'results'/'wave6_60s_fz_analysis.json'
    modelpath=BASE/'source'/'mjcf'/'scene_local30_nominal.xml'
    report=json.loads(recpath.read_text());forces=json.loads(fzpath.read_text())
    assert report['finished_full_duration'] and not report['fall_or_nonfinite'] and report['gait']=='wave'
    assert report['duration_s']==60. and len(report['qpos_samples'])==1501
    m=mj.MjModel.from_xml_path(str(modelpath));d=mj.MjData(m);legs=load_legs(m,d,mj)
    assert tuple(x['suffix'] for x in legs)==CODES
    assert m.nq==27
    samples=report['qpos_samples']; actual=forces['force_samples_50hz']; Ftimes=np.array([p['t'] for p in actual]);
    frames=[];associated_support=[]
    for item in samples:
        q=np.asarray(item['qpos'],dtype=float);t=float(item['t'])
        assert len(q)==m.nq and np.all(np.isfinite(q))
        d.qpos[:]=q;d.qvel[:]=0;mj.mj_forward(m,d)
        base=int(m.body('base_link_2').id)
        row=[*d.xpos[base].tolist(),*d.xquat[base].tolist()]
        for leg in legs:
            for j in leg['jids']:row.extend(d.xanchor[j].tolist())
            row.extend(tip_world(m,d,leg).tolist())
        fs=actual[int(np.argmin(abs(Ftimes-t)))]
        row=[round(float(v),5) for v in row]+[int(fs['mask'])]
        assert len(row)==80
        frames.append(row);associated_support.append(int(fs['n']))
    interval=(float(samples[-1]['t'])-float(samples[0]['t']))/(len(samples)-1)
    payload={
      'format':'fnk0052-physical-points-v1',
      'source':'60 s continuous original CAD30 nominal wave gait, free-base MuJoCo 3.14; no root teleport, no foot pinning',
      'visualization':'procedural linkage proxy — no original CAD mesh redistributed',
      'units':'metres','coordinate_system':'MuJoCo Z-up',
      'duration_s':60.,'sample_interval_s':round(interval,6),'sample_count':len(frames),'stride':80,
      'layout':'base position xyz + quaternion wxyz (7), 6 x [hip/coxa, femur, tibia, tip] xyz (72), measured Fz contact mask (1)',
      'leg_order':list(CODES),
      'forward_xy':[round(float(v),6) for v in forward_from_geometry(legs)],
      'summary':{'advance_m':report['advance_m'],'max_tilt_deg':report['max_tilt_deg'],
        'min_grounded_legs_collision':report['min_grounded_legs'],
        'min_loaded_legs_Fz_0p2N_all_2ms':forces['min_Fz_loaded_all_2ms'],
        'fraction_with_below_five_Fz_0p2N':forces['fraction_below_5_loaded'],
        'max_abs_torque_nm':report['max_abs_torque_nm'], 'saturation_fraction':report['saturation_fraction'],
        'estimated_mass_kg':report['robot_mass_kg'],'fall_or_nonfinite':report['fall_or_nonfinite'],
        'step_mm':20.,'lift_mm':12.,'period_s':3.,'mu_model':1.1,
        'strict_five_support_pass':False,'status':'diagnostic'},
      'source_qpos_sha256':sha(recpath),'source_Fz_sha256':sha(fzpath),
      'limitations':['Model assumptions for mass, actuation and friction are not calibrated',
         'Some 2ms frames have fewer than five truly loaded legs; cannot call the gait strictly five-supported',
         'Viewer contact mask uses measured Fz >=0.2N nearest 50Hz sample; 2ms contact trace audited separately',
         'Foot tip is CAD mesh landmark, not instrumented rubber foot'],
      'frames':frames}
    assert frames[0][0:7]!=frames[-1][0:7], 'Unexpected frozen body'
    OUT.joinpath('data').mkdir(parents=True,exist_ok=True)
    points=OUT/'data'/'wave-60s-points.json'
    points.write_text(json.dumps(payload,separators=(',',':'),ensure_ascii=False)+'\n')
    # Use exactly the same CAD exporter implementation as tripod viewer; it
    # works on the observed qpos array regardless of gait type.
    upstream=REPO_ROOT/'scripts'/'export_cad_rig_replay.py'
    spec=importlib.util.spec_from_file_location('original_viewer_rig_export',upstream)
    mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod)
    rig=mod.export(modelpath,report,sha(recpath))
    rig['source_wave_audit_sha256']=sha(fzpath)
    rigfile=OUT/'data'/'cad-rig-wave-60s.json'
    rigfile.write_text(json.dumps(rig,separators=(',',':'),ensure_ascii=False)+'\n')
    print(json.dumps({'trajectory':str(points),'trajectory_size':points.stat().st_size,'rig_size':rigfile.stat().st_size,'frames':len(frames),'support_range_viewer':[min(associated_support),max(associated_support)],'recorded_fz_min':forces['min_Fz_loaded_all_2ms'],'progress_m':report['advance_m']},ensure_ascii=False,indent=2))
if __name__=='__main__':export()
