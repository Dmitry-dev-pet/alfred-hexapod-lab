"""Manifest registration regression: no CSS/JS changes or fake content may slip in."""
import hashlib
import tempfile
import unittest
from pathlib import Path
from register_viewer_replays import register

class RegistrationTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.docs=Path(self.temp.name)
        self.base={'schema':'hexapod-library/v1','revision':2,'entries':[
            {'id':'tripod-60','href':'/hexapod/','title':'Tripod','detail':'old','duration_s':60,'files':[]}
        ]}
        root=self.docs/'hexapod'/'contact-one'
        root.mkdir(parents=True)
        payload=b'<p>real replay</p>\n'
        (root/'index.html').write_bytes(payload)
        self.entry={'id':'contact-one','href':'/hexapod/contact-one/','title':'Recorded','detail':'negative',
                    'duration_s':2.4,'files':[{'path':'hexapod/contact-one/index.html',
                    'sha256':hashlib.sha256(payload).hexdigest(),'size':len(payload)}]}
    def test_add_and_idempotent(self):
        r=register(self.base,[self.entry],self.docs)
        self.assertEqual((r['revision'],len(r['entries'])),(3,2))
        self.assertEqual(register(r,[self.entry],self.docs),r)
    def test_reject_mutated_file(self):
        self.entry['files'][0]['sha256']='0'*64
        with self.assertRaises(ValueError):register(self.base,[self.entry],self.docs)
    def test_reject_id_reuse(self):
        a=register(self.base,[self.entry],self.docs)
        modified={**self.entry,'detail':'pretend success'}
        with self.assertRaises(ValueError):register(a,[modified],self.docs)
    def test_reject_traversal(self):
        self.entry['files'][0]['path']='hexapod/../secret.js'
        with self.assertRaises(ValueError):register(self.base,[self.entry],self.docs)

if __name__=='__main__':unittest.main()
