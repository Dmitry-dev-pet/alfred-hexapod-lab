"""Bind source-command metadata to the existing independently checked FK exporter."""
from __future__ import annotations
import argparse, hashlib, json, tempfile
from pathlib import Path
from .feedback_export import export as base_export
from .source_jump import FIRMWARE_BLOB

def export(trace,out):
    raw=Path(trace).read_bytes();record=json.loads(raw);s=record['summary']
    if s.get('experiment')!='source_jump_v4' or s.get('source_firmware_blob')!=FIRMWARE_BLOB:
        raise ValueError('Not a source-jump trace')
    with tempfile.TemporaryDirectory() as td:
        temp=Path(td)/'replay.json';receipt=base_export(trace,temp);doc=json.loads(temp.read_bytes())
    doc['source_command_experiment']={'source_firmware_blob':FIRMWARE_BLOB,
        'loop_period_ms_ASSUMED':s['source_loop_period_ms_ASSUMED'],
        'cad_anchor_q_rad_ASSUMED':s['cad_anchor_q_at_height38_rad_ASSUMED'],
        'events':s['command_events'],'scope':s['source_jump_exactness']}
    doc['limitations'].append('Only source jump command sequencing is reproduced; loop clock and encoder/CAD zero are sensitivity hypotheses')
    if s['source_jump_enabled']:doc['status']='research' if s['jump_pass'] else 'failed'
    payload=json.dumps(doc,allow_nan=False,separators=(',',':'))+'\n';out=Path(out);out.parent.mkdir(parents=True,exist_ok=True)
    if out.exists() and out.read_text()!=payload:raise ValueError('Immutable replay conflict')
    out.write_text(payload)
    return {**receipt,'replay_sha256':hashlib.sha256(payload.encode()).hexdigest(),'status':doc['status']}

def main():
    p=argparse.ArgumentParser();p.add_argument('--trace',type=Path,required=True);p.add_argument('--out',type=Path,required=True)
    a=p.parse_args();print(json.dumps(export(a.trace,a.out),indent=2))

if __name__=='__main__':main()
