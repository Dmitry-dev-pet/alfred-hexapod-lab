"""Execute the predeclared 40-case sequential-command study and export every outcome, including idle."""
import argparse,hashlib,json
from concurrent.futures import ProcessPoolExecutor,as_completed
from pathlib import Path
from .getup_model import create
from .tuck_probe import run,PROTOCOL
from .tuck_export import export


def one(args):
    out,mesh,orientation,strategy=args;out=Path(out)
    case=orientation+'-'+strategy
    trace=out/'raw'/(case+'_raw.json');replay=out/'replay'/(case+'.json')
    summary=run(trace,orientation=orientation,strategy=strategy,model_path=out/'model.xml',mesh_path=mesh)
    receipt=export(trace,replay,model_path=out/'model.xml',mesh_path=mesh)
    return {'case':case,'summary':summary,'export':receipt}


def main():
    p=argparse.ArgumentParser();p.add_argument('--out',type=Path,required=True);p.add_argument('--cad-mesh',type=Path,required=True);p.add_argument('--workers',type=int,default=4)
    a=p.parse_args();a.out.mkdir(parents=True,exist_ok=True)
    xml=(Path(__file__).resolve().parents[2]/'results/en01_getup_v7/final/model.xml').read_text();(a.out/'model.xml').write_text(xml)
    protocol=json.loads(PROTOCOL.read_text())
    tasks=[(str(a.out.resolve()),str(a.cad_mesh.resolve()),o,s) for o in protocol['orientations'] for s in protocol['strategies'] if s!='balance']
    tasks += [(str(a.out.resolve()),str(a.cad_mesh.resolve()),'upright',s) for s in ('idle','balance')]
    rows=[]
    with ProcessPoolExecutor(max_workers=a.workers) as pool:
        futures={pool.submit(one,t):t for t in tasks}
        for future in as_completed(futures):
            r=future.result();rows.append(r);s=r['summary']
            print(r['case'],'PASS' if r['export']['getup_pass'] else 'FAIL',
                  'final_tilt',round(s['final_tilt_deg'],2),'stable',s['final_stable_s'],
                  'geometry',r['export']['geometry_ok'],flush=True)
    rows.sort(key=lambda r:r['case'])
    (a.out/'RESULTS.json').write_text(json.dumps(rows,indent=2)+'\n')
    manifest={str(p.relative_to(a.out)):{'bytes':p.stat().st_size,'sha256':hashlib.sha256(p.read_bytes()).hexdigest()}
              for p in sorted(a.out.rglob('*')) if p.is_file() and p.name!='MANIFEST.json'}
    (a.out/'MANIFEST.json').write_text(json.dumps(manifest,indent=2)+'\n')


if __name__=='__main__':main()
