"""Prepare immutable source-jump recordings with the existing replay pipeline.

Writes only a disposable review directory. Does not publish or change the source
checkout's catalog. Original CAD/shared player assets are reused separately.
"""
from __future__ import annotations
import argparse,hashlib,json,shutil,sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent))
from replay_pipeline import load_spec,run

ROOT=Path(__file__).resolve().parents[1]

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--dataset',type=Path,required=True);p.add_argument('--catalog',type=Path,required=True);p.add_argument('--out',type=Path,required=True);a=p.parse_args()
    a.dataset=a.dataset.resolve();a.catalog=a.catalog.resolve();a.out=a.out.resolve()
    initial=json.loads(a.catalog.read_text());catalog=a.out/'docs/hexapod/library.json';catalog.parent.mkdir(parents=True,exist_ok=True)
    if catalog.exists() and json.loads(catalog.read_text())!=initial:raise ValueError('Use a fresh disposable review directory')
    catalog.write_text(json.dumps(initial,ensure_ascii=False,indent=2)+'\n')
    manifest=json.loads((a.dataset/'MANIFEST.json').read_text())
    for name,meta in manifest.items():
        path=a.dataset/name
        if path.resolve().is_relative_to(a.dataset.resolve()) is False:raise ValueError('Unsafe dataset path')
        b=path.read_bytes()
        if len(b)!=meta['bytes'] or hashlib.sha256(b).hexdigest()!=meta['sha256']:raise ValueError('Dataset manifest mismatch')
    source=ROOT/'docs/hexapod/en01-source-jump-template';shutil.copytree(source,a.out/'docs/hexapod/en01-source-jump-template',dirs_exist_ok=True)
    receipts=[]
    for path in sorted((ROOT/'experiments/replay-pipeline').glob('en01-source-jump-v4-*.json')):
        job=load_spec(path);trace=a.dataset/'raw'/Path(job['trace']['path']).name;replay=a.dataset/'replay'/Path(job['replay']['path']).name
        for source,dest in [(trace,a.out/job['trace']['path']),(replay,a.out/'docs'/job['replay']['path'])]:
            dest.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(source,dest)
        first=run(a.out,job,skip_execution=True);again=run(a.out,job,skip_execution=True)
        if first['added']!=1 or again['added']!=0:raise ValueError('Registration is not idempotent')
        try:run(a.out,job,skip_execution=True,do_publish=True)
        except ValueError as e:
            if 'review_only' not in str(e):raise
        else:raise ValueError('Publication should have been blocked')
        receipts.append(first)
    now=json.loads(catalog.read_text())
    if any(e not in now['entries'] for e in initial['entries']):raise ValueError('Existing entry changed')
    report={'schema':'en01-source-jump-review-package/v1','recordings':len(receipts),'initial_entries':len(initial['entries']),'review_entries':len(now['entries']),'all_previous_entries_preserved':True,'idempotent':True,'publish_refused':True,'published':False,'recordings_receipts':receipts}
    (a.out/'pipeline_receipt.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k!='recordings_receipts'},indent=2))

if __name__=='__main__':main()
