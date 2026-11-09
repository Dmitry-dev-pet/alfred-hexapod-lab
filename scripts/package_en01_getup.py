"""Register all final self-righting records through the established replay pipeline."""
import argparse,hashlib,json,shutil,sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent))
from replay_pipeline import run

ROOT=Path(__file__).resolve().parents[1]


def main():
    p=argparse.ArgumentParser();p.add_argument('--dataset',type=Path,required=True);p.add_argument('--catalog',type=Path,required=True);p.add_argument('--out',type=Path,required=True);a=p.parse_args()
    dataset=a.dataset.resolve();out=a.out.resolve();initial=json.loads(a.catalog.read_text())
    catalog=out/'docs/hexapod/library.json';catalog.parent.mkdir(parents=True,exist_ok=True)
    catalog.write_text(json.dumps(initial,ensure_ascii=False,indent=2)+'\n')
    shutil.copytree(ROOT/'docs/hexapod/en01-getup-template',out/'docs/hexapod/en01-getup-template',dirs_exist_ok=True)
    for name,meta in json.loads((dataset/'MANIFEST.json').read_text()).items():
        path=dataset/name
        if not path.resolve().is_relative_to(dataset):raise ValueError('Unsafe dataset path')
        b=path.read_bytes()
        if len(b)!=meta['bytes'] or hashlib.sha256(b).hexdigest()!=meta['sha256']:raise ValueError('Manifest mismatch')
    receipts=[]
    for row in json.loads((dataset/'RESULTS.json').read_text()):
        case=row['case'];sid='en01-getup-v7-'+case+'-20261109';raw=dataset/'raw'/(case+'_raw.json');replay=dataset/'replay'/(case+'.json')
        relative_raw='results/en01_getup_v7/final/raw/'+raw.name
        job={'schema':'hexapod-replay-job/v1','id':sid,'title':'EN01 v7 · '+case,'detail':'Попытка вставания · устойчивая стойка не достигнута · динамика не откалибрована',
             'robot':'NavBot EN01 self-righting surrogate','engine':'MuJoCo','status':row['export']['status'],'publish_policy':'review_only','duration_s':6,
             'commands':{'simulate':['python','-m','research.navbot_en01.getup_probe','--orientation',row['summary']['orientation'],'--strategy',row['summary']['strategy'],'--model','results/en01_getup_v7/final/model.xml','--cad-mesh','docs/hexapod/en01-original-cad-v1/original-meshes.json','--out',relative_raw],
                         'export':['python','-m','research.navbot_en01.getup_export','--trace',relative_raw,'--out','docs/hexapod/data/'+sid+'.json','--model','results/en01_getup_v7/final/model.xml','--cad-mesh','docs/hexapod/en01-original-cad-v1/original-meshes.json']},
             'trace':{'path':relative_raw,'samples_field':'frames','time_field':'t','qpos_field':'qpos'},
             'replay':{'path':'hexapod/data/'+sid+'.json','format':'navbot-en01-fourbar-physical-replay-v2','trace_sha256_field':'source.trace_sha256'},
             'player':{'template':'en01-getup-template'}}
        path=ROOT/'experiments/replay-pipeline'/(sid+'.json');path.parent.mkdir(parents=True,exist_ok=True);path.write_text(json.dumps(job,ensure_ascii=False,indent=2)+'\n')
        for src,dst in [(raw,out/relative_raw),(replay,out/'docs'/job['replay']['path'])]:
            dst.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(src,dst)
        first=run(out,job,skip_execution=True);again=run(out,job,skip_execution=True)
        if first['added']!=1 or again['added']!=0:raise ValueError('Non-idempotent registration')
        try:run(out,job,skip_execution=True,do_publish=True)
        except ValueError as e:
            if 'review_only' not in str(e):raise
        else:raise ValueError('Unexpected research publication')
        receipts.append(first)
    after=json.loads(catalog.read_text())
    if any(e not in after['entries'] for e in initial['entries']):raise ValueError('Existing entry changed')
    report={'recordings':len(receipts),'prior_entries':len(initial['entries']),'review_entries':len(after['entries']),
            'previous_entries_preserved':True,'idempotent':True,'research_publication_refused':True,'published':False}
    (out/'pipeline_receipt.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report))


if __name__=='__main__':main()
