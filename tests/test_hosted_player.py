import importlib.util
import json
from pathlib import Path
import tempfile
import unittest

ROOT=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('hosted',ROOT/'scripts/build_hosted_player.py')
hosted=importlib.util.module_from_spec(spec);spec.loader.exec_module(hosted)


class HostedPlayerTests(unittest.TestCase):
    def test_all_published_catalog_files_match_source(self):
        self.assertGreaterEqual(len(hosted.validate_catalog(ROOT/'docs')['entries']),51)

    def test_corrupt_recording_refuses_export(self):
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp);(p/'hexapod').mkdir();(p/'hexapod/index.html').write_text('x')
            (p/'hexapod/data.json').write_text('corrupt')
            catalog={'schema':'hexapod-library/v1','entries':[{'id':'a','href':'/hexapod/','files':[{'path':'hexapod/data.json','sha256':'0'*64,'size':7}]}]}
            (p/'hexapod/library.json').write_text(json.dumps(catalog))
            with self.assertRaisesRegex(ValueError,'integrity'):
                hosted.validate_catalog(p)

    def test_hosted_refresh_has_no_local_server_dependency(self):
        original=(ROOT/'docs/hexapod/player-shell.js').read_text()
        result=hosted.adapt_shell(original,'a'*40,(ROOT/'hosting/player-refresh.js').read_text())
        self.assertNotIn('/api/',result)
        self.assertNotIn('Termux',result)
        self.assertIn("videos.href = '/videos/'",result)
        self.assertIn('clearTimeout(timer)',result)


if __name__=='__main__':unittest.main()
