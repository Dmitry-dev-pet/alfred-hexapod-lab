#!/usr/bin/env python3
"""One command: EN01 physical runs -> shared Android 3D replay pipeline. NO MP4."""
from __future__ import annotations
import argparse,json
from replay_pipeline import ROOT,load_spec,run

def main():
    p=argparse.ArgumentParser()
    p.add_argument("--skip-execution",action="store_true",help="Use already recorded physical traces")
    args=p.parse_args()
    receipts=[]
    for mode in ("stand","crouch","jump","jump-fast"):
        job=load_spec(ROOT/"experiments/replay-pipeline"/("navbot-en01-"+mode+"-20261108.json"))
        receipts.append(run(ROOT,job,skip_execution=args.skip_execution,do_publish=False))
    print(json.dumps({"created":len(receipts),"published":False,"mp4_created":False,
       "entries":[{"id":r["id"],"added":r["added"],"revision":r["revision"]}
                  for r in receipts]},ensure_ascii=False,indent=2))

if __name__=="__main__":
    main()
