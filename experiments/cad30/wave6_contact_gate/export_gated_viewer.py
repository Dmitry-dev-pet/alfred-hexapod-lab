"""Export actual 2ms-audited, recorded 25Hz qpos into pre-existing Hexapod Lab viewer formats.

No faked model poses. Original Freenove STL are referenced from pinned upstream, not copied.
"""
from __future__ import annotations
import json, hashlib, importlib.util, sys, os, shutil
from pathlib import Path
import numpy as np
import mujoco as mj

HERE=Path(__file__).resolve().parent
SOURCE=HERE/'feedback60_final.json'
TRACE=HERE/'feedback60_final.2ms.json.gz'
CAD=Path(os.environ.get('CAD30_WORK',str(HERE.parent))).resolve()
ORIGINAL_VIEWER=Path(os.environ.get('HEXAPOD_REPO_ROOT',str(HERE.parents[2]))).resolve()
OUTPUT=Path(os.environ.get('HEXAPOD_DOCS_ROOT',str((HERE/'viewer_assets'/'docs') if (HERE/'viewer_assets'/'docs').is_dir() else (ORIGINAL_VIEWER/'docs')))).resolve()/'hexapod'
ORDER=('ml','bl','br','mr','fr','fl')
COMMIT='5c9a84c16335b30036e451c691d2c544abf4bc72'

def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()
def compact(path,obj):
    path.parent.mkdir(parents=True,exist_ok=True)
    path.write_text(json.dumps(obj,ensure_ascii=False,separators=(',',':'))+'\n',encoding='utf-8')

def main():
    r=json.loads(SOURCE.read_text())
    assert r['elapsed_seconds']==60.0 and len(r['samples'])==1501
    assert r['progress_m']>.3 and r['unplanned_stance_Fz_under_0p2_ticks']==165
    sys.path.insert(0,str(CAD))
    from hexlab.stand_geometry_v04 import load_legs,tip_world
    from hexlab.step_one_leg_v04 import forward_from_geometry
    modelpath=CAD/'source/mjcf/scene_local30_nominal.xml'
    m=mj.MjModel.from_xml_path(str(modelpath));d=mj.MjData(m)
    legs=load_legs(m,d,mj)
    assert tuple(x['suffix'] for x in legs)==ORDER
    frames=[]
    for sample in r['samples']:
        q=np.asarray(sample['qpos'],float)
        assert q.shape==(27,)
        d.qpos[:]=q;d.qvel[:]=0.;mj.mj_forward(m,d)
        body=int(m.body('base_link_2').id)
        row=[*d.xpos[body].tolist(),*d.xquat[body].tolist()]
        for leg in legs:
            for j in leg['jids']:row.extend(d.xanchor[j].tolist())
            row.extend(tip_world(m,d,leg).tolist())
        mask=sum((1<<i) for i,c in enumerate(ORDER) if float(sample['Fz'][c])>=.2)
        row=[round(float(x),5) for x in row]+[mask]
        assert len(row)==80
        frames.append(row)
    t0=float(r['samples'][0]['t']);t_last=float(r['samples'][-1]['t'])
    interval=(t_last-t0)/(len(frames)-1)
    physical={
      'format':'fnk0052-physical-points-v1',
      'source':'60s true free-base MuJoCo 3.14 CAD30 gated/force-feedback Wave; 30,000 physical 2ms steps; diagnostic (named stance leg dropouts)',
      'visualization':'schematic linkages derived from actual qpos; original Freenove CAD loaded by pinned source in rig replay',
      'units':'metres','coordinate_system':'MuJoCo Z-up',
      'duration_s':60.0,'sample_interval_s':round(interval,6),'sample_count':1501,'stride':80,
      'layout':'base xyz and quaternion wxyz (7), 6 x [hip/coxa/femur/tibia/tip] xyz (72), actual Fz contact mask (1)',
      'leg_order':list(ORDER),
      'forward_xy':list(forward_from_geometry(legs)),
      'summary':{
        'advance_m':r['progress_m'],'max_tilt_deg':r['max_actual_tilt_deg'],
        'min_loaded_legs_Fz_0p2N_all_2ms':r['min_real_Fz_loaded_legs'],
        'fraction_with_below_five_Fz_0p2N':0.0,
        'unplanned_stance_Fz_under_0p2_ticks':r['unplanned_stance_Fz_under_0p2_ticks'],
        'unplanned_stance_loss_modes':r['unplanned_stance_loss_mode_counts'],
        'max_abs_torque_nm':r['max_raw_PD_torque_Nm'],
        'saturation_fraction':r['saturated_ticks']/30000,
        'estimated_mass_kg':r['model_mass_kg'],'fall_or_nonfinite':False,
        'step_mm':20.,'lift_mm':12.,'period_s':3.,'mu_model':1.1,
        'strict_five_support_pass':False,'status':'diagnostic',
        'gate_phase_holds_s':r['hold_s_total'],
        'controller':'causal Fz contact gate and small stance target correction; ramp 10s'},
      'source_qpos_sha256':sha(SOURCE),'source_Fz_sha256':sha(TRACE),
      'limitations':['Five TOTAL actual Fz-loaded feet all physical steps; not always the five named stance feet',
                     '165 × 2ms named-support violations (38 during commanded swing, 127 six-stance gaps)',
                     'Viewer contacts sampled every 40ms; all 30,000 actual Fz frames archived separately',
                     'No hardware torque, friction or mass calibration; this is not a certified robot gait'],
      'frames':frames
    }
    compact(OUTPUT/'data'/'wave-gated-60s-points.json',physical)
    # Same exact CAD exporter as original Wave player
    modulepath=ORIGINAL_VIEWER/'scripts'/'export_cad_rig_replay.py'
    spec=importlib.util.spec_from_file_location('wave_rig_export',modulepath)
    mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod)
    recording={'qpos_samples':[{'t':s['t'],'qpos':s['qpos']} for s in r['samples']]}
    rig=mod.export(modelpath,recording,sha(SOURCE))
    rig['source_Fz_trace_sha256']=sha(TRACE)
    rig['controller_label']='gated force-aware 6-legged Wave diagnostic'
    compact(OUTPUT/'data'/'cad-rig-wave-gated-60s.json',rig)
    event_payload={'schema':'cad30-wave-gated-events/1','duration_s':60.0,
      'named_stance_loss_count':165,'events':r['named_stance_loss_events'],
      'force_cutoff_N':.2,'source_audit_sha256':sha(TRACE)}
    compact(OUTPUT/'data'/'wave-gated-60s-events.json',event_payload)
    page=OUTPUT/'wave-gated-60';page.mkdir(parents=True,exist_ok=True)
    original_page=ORIGINAL_VIEWER/'docs'/'hexapod'/'wave-60'
    html=(original_page/'index.html').read_text()
    html=html.replace('Wave · шесть ног · 60 секунд','Wave · контактная проверка · 60 секунд')
    html=html.replace('WAVE · ДИАГНОСТИКА','WAVE · КОНТАКТНЫЙ КОНТРОЛЬ · ДИАГНОСТИКА')
    html=html.replace('0,379 м вперед · max 0,371 Н·м','0,320 м вперед · max 0,329 Н·м')
    html=html.replace('В одном физическом такте остались две опоры при Fz ≥ 0,2 Н; полноценная строго пятиопорная походка НЕ подтверждена.',
      'Все 30 000 тактов имели ≥5 нагруженных ног, но на 165 тактах терялась одна из НУЖНЫХ опор. Пять поимённых опор на каждом такте НЕ подтверждены.')
    html=html.replace('</footer>', '<div>Диагностика: 38 из 165 нарушений произошли во время переноса другой ноги, 127 — в промежутках со всеми ногами в стойке. 2-мс события могут быть незаметны при частоте проигрывателя 25 Гц.</div>\n    </footer>')
    (page/'index.html').write_text(html,encoding='utf-8')
    js=(original_page/'app.js').read_text()
    js=js.replace('wave-60s-points.json','wave-gated-60s-points.json').replace('cad-rig-wave-60s.json','cad-rig-wave-gated-60s.json')
    (page/'app.js').write_text(js,encoding='utf-8')
    # Compact immutable catalogue entry, ready to append with revision increment.
    paths=['hexapod/wave-gated-60/index.html','hexapod/wave-gated-60/app.js',
          'hexapod/data/wave-gated-60s-points.json','hexapod/data/cad-rig-wave-gated-60s.json',
          'hexapod/data/wave-gated-60s-events.json']
    entry={'id':'wave-60-gated-diagnostic','title':'Wave · контактный контроль',
           'detail':'6 ног · 60 с · +0,320 м · 0 тактов ниже 5 опор, но 165 поимённых нарушений · диагностика',
           'duration_s':60,'href':'/hexapod/wave-gated-60/','status':'diagnostic',
           'files':[{'path':p,'sha256':sha(OUTPUT.parent/p),'size':(OUTPUT.parent/p).stat().st_size} for p in paths]}
    compact(HERE/'viewer_entry.json',entry)
    print(json.dumps({'points_frames':len(frames),'point_bytes':(OUTPUT/'data'/'wave-gated-60s-points.json').stat().st_size,
        'rig_bodies':len(rig['bodies']),'rig_bytes':(OUTPUT/'data'/'cad-rig-wave-gated-60s.json').stat().st_size,
        'metadata_events':len(event_payload['events']),'entry':entry['title']},ensure_ascii=False,indent=2))

if __name__=='__main__':main()
