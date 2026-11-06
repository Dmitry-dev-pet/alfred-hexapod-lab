#!/usr/bin/env python3
"""Create compact, independent aggregate receipts from pinned Lite3 CPU experiments."""
import argparse
import hashlib
import json
import math
import statistics
from pathlib import Path

BASE=Path(__file__).resolve().parent
SEQUENCES={
  'standard': [(0,2,(0,0,0)),(2,4,(.2,0,0)),(4,6,(0,0,.35)),(6,8,(0,0,0))],
  'midstride': [(0,1.3,(0,0,0)),(1.3,3.1,(.25,0,0)),(3.1,4.7,(0,.13,0)),
                (4.7,6.3,(0,0,-.5)),(6.3,7.9,(-.15,0,0)),(7.9,9.5,(0,0,0))]
}


def stats(vals):
    vals=[float(v) for v in vals]
    return {'n':len(vals),'mean':statistics.mean(vals),'std_population':statistics.pstdev(vals),
            'min':min(vals),'max':max(vals)}


def body_frame(seg):
    yaw=math.radians(seg['yaw_start_deg'])
    x=seg['dx_m']*math.cos(yaw)+seg['dy_m']*math.sin(yaw)
    y=-seg['dx_m']*math.sin(yaw)+seg['dy_m']*math.cos(yaw)
    return x,y


def aggregate(folder):
    report=json.loads((folder/'SUMMARY.json').read_text())
    protocol=report.get('protocol', 'standard' if folder.name.startswith('onnx_') else 'midstride')
    policy={}
    for mode in ['onnx','zero']:
        runs=[r for r in report['runs'] if r['mode']==mode]
        assert len(runs)==10
        metrics={
            'total_dx_m':stats([r['net_root_m'][0] for r in runs]),
            'max_tilt_deg':stats([r['max_tilt_deg'] for r in runs]),
            'force_saturation_ticks':stats([r['torque_saturation_ticks'] for r in runs]),
            'first_fall_count':sum(r['first_fall'] is not None for r in runs),
        }
        for idx,desc in enumerate(SEQUENCES[protocol]):
            _,_,cmd=desc
            name=f'segment_{idx+1}'
            segments=[r['segments'][idx] for r in runs]
            xs=[body_frame(s)[0] for s in segments]
            ys=[body_frame(s)[1] for s in segments]
            yaw=[s['yaw_end_deg']-s['yaw_start_deg'] for s in segments]
            motion=[math.hypot(s['dx_m'],s['dy_m']) for s in segments]
            metrics[name]={'command':list(cmd), 'duration_s':desc[1]-desc[0],
                           'body_forward_m':stats(xs),'body_lateral_m':stats(ys),
                           'yaw_deg':stats(yaw),'world_net_translation_m':stats(motion)}
        policy[mode]=metrics
    return {'protocol':protocol,'seeds':[r['seed'] for r in report['runs'] if r['mode']=='onnx'],
            'n_pairwise':10,'physics':'MuJoCo 3.14 CPU, dt .004s','policy':'ONNX Runtime CPU 1.23.2, 50Hz',
            'model_joint_order':report['model_joint_order'],
            'actuator_order':report['model_actuator_order'],
            'nn_to_actuator_index':report['nn_to_actuator_index'],
            'onnx_vs_pt_comparison':report['onc_vs_pt'],
            'modes':policy,'warning':'Original STL/MJCF. Independent play-like adapter, NOT native MJLab.'}


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--standard',type=Path,default=BASE/'onnx_corrected')
    ap.add_argument('--midstride',type=Path,default=BASE/'midstride_corrected')
    ap.add_argument('--out',type=Path,default=BASE/'LITE3_ONNX_CORRECTED_AGGREGATE.json')
    args=ap.parse_args()
    data={'date':'2026-11-06', 'experiments':[aggregate(args.standard),aggregate(args.midstride)],
          'source':{'vendor_commit':'447945fd40fb5585271dce4ea8cc82567916032a',
                    'hf_commit':'1646bc65ed37feff3698a43524a65cb3cd5b6981',
                    'onnx_sha256':'773a1bbd0fee45ec38604d979d5fd6fb339ae15d13bfb5d6e2056a171d2a3422',
                    'checkpoint_sha256':'cca3a020163309790f8dc8ecb7553ff0e98e75d02e78a1773ad3734ca535a67c'}}
    args.out.write_text(json.dumps(data,ensure_ascii=False,indent=2)+'\n')
    for item in data['experiments']:
        print('protocol',item['protocol'],'runs',item['n_pairwise']*2)
        for mode in ['onnx','zero']:
            m=item['modes'][mode]
            print(' ',mode,'dx',round(m['total_dx_m']['mean'],5),'falls',m['first_fall_count'],'tilt',round(m['max_tilt_deg']['max'],3),'saturation',round(m['force_saturation_ticks']['mean'],1))
    print('saved',args.out)
if __name__=='__main__':main()