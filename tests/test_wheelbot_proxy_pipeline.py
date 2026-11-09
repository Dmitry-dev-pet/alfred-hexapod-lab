"""No-network checks for honest Wheelbot collision-proxy replay publication."""
import hashlib
import importlib.util
import json
import math
import sys
import tempfile
import unittest
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
import replay_pipeline as pipeline
SPEC=ROOT/'experiments/replay-pipeline/wheelbot-offline-proxy-20261107.json'
MODULE=ROOT/'research/wheelbot_offline_proxy/export.py'
spec=importlib.util.spec_from_file_location("wheelbot_proxy_export",MODULE)
exp=importlib.util.module_from_spec(spec);spec.loader.exec_module(exp)


def fixture():
    rows=[];samples=[]
    for i in range(161):
        t=round(i*.05,3)
        q=[0.,0.,.07,1.,0.,0.,0.,t*.1,t*.2]
        row=[t,*q,90.,100.*t,.1,1]
        rows.append(row)
        samples.append({'t':t,'qpos':q,'qvel':[0.]*8,'contacts':1})
    return {'schema':'wheelbot-step-collision-only-raw/v1','frames':rows,'samples':samples,
            'info':{'status':'DIAGNOSTIC_COLLISION_PROXY_NO_VISUAL_CAD',
                    'mode':'upstream','meets_provisional_up_criterion':False,
                    'numerical_warning':None,'physics_steps':8000,
                    'simulated_duration_s':8.,'source_upstream':exp.FMT,
                    'model_xml_sha256':'0'*64,'final_tilt_deg':90.,
                    'min_tilt_deg':.25,'peak_reaction_speed_rad_s':888.,
                    'longest_upright_s':.02}}


class WheelbotProxy(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.base=Path(self.tmp.name)
        self.trace=self.base/'source.json'
        self.output=self.base/'exported.json'
        self.x=fixture()
        self.write()

    def write(self):
        self.trace.write_text(json.dumps(self.x))

    def test_declarative_job_and_renderer_replacements(self):
        job=pipeline.load_spec(SPEC)
        self.assertEqual(job['publish_policy'],'review_only')
        self.assertEqual(job['status'],'failed')
        self.assertEqual(job['replay']['trace_sha256_field'],'verification.source_trace_sha256')
        pages=pipeline.make_pages(ROOT,job)
        self.assertIn('Не удержал вертикаль',pages['hexapod/'+job['id']+'/app.js'].decode())
        self.assertIn('8 С · FAIL',pages['hexapod/'+job['id']+'/index.html'].decode())

    def test_publish_blocked_before_any_physics(self):
        job=pipeline.load_spec(SPEC)
        with self.assertRaisesRegex(ValueError,'review_only'):
            pipeline.run(self.base,job,do_publish=True)

    def test_replay_from_recorded_qpos_only(self):
        receipt=exp.export(self.trace,self.output)
        data=json.loads(self.output.read_text())
        self.assertEqual(len(data['frames']),161)
        self.assertFalse(data['result']['meets_provisional_up_criterion'])
        self.assertEqual(data['verification']['source_trace_sha256'],hashlib.sha256(self.trace.read_bytes()).hexdigest())
        for source,display in zip(self.x['frames'],data['frames']):
            self.assertEqual(display,source[:10]+[source[11],source[12]])
        self.assertEqual(exp.export(self.trace,self.output),receipt)

    def test_invalid_original_qpos_is_blocked(self):
        self.x['samples'][3]['qpos'][0]=1.
        self.write()
        with self.assertRaisesRegex(ValueError,'pose mismatch'):
            exp.export(self.trace,self.output)
        self.assertFalse(self.output.exists())

    def test_simulation_warning_blocks_export(self):
        self.x['info']['numerical_warning']={'warning_index':1}
        self.write()
        with self.assertRaisesRegex(ValueError,'valid complete numerical rollout'):
            exp.export(self.trace,self.output)

    def test_negative_status_is_mandatory(self):
        self.x['info']['meets_provisional_up_criterion']=True
        self.write()
        with self.assertRaisesRegex(ValueError,'negative control'):
            exp.export(self.trace,self.output)

    def test_immutable_replay_refuses_modified_file(self):
        exp.export(self.trace,self.output)
        self.output.write_text('{}')
        with self.assertRaisesRegex(ValueError,'immutable data conflict'):
            exp.export(self.trace,self.output)


if __name__=='__main__':unittest.main()
