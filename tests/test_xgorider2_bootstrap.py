"""Static/safe launcher checks; no internet, MuJoCo, GitHub Actions or hardware."""
from pathlib import Path
import shutil
import subprocess
import unittest

ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts" / "research" / "xgorider2_mac.sh"
PIN = "f534986353ce3fe451caf0da72ba06b980a59114"


@unittest.skipUnless(shutil.which("bash"), "bash is needed for the launcher")
class Rider2BootstrapTests(unittest.TestCase):
    def test_bash_syntax(self):
        subprocess.run(["bash", "-n", str(SCRIPT)], check=True, capture_output=True, text=True)

    def test_help_is_offline(self):
        proc = subprocess.run(["bash", str(SCRIPT), "--help"], check=True, capture_output=True, text=True, timeout=5)
        self.assertIn("train-smoke", proc.stdout)
        self.assertIn("no robot", proc.stdout.lower())

    def test_dry_run_does_not_touch_network_or_local_state(self):
        proc = subprocess.run(["bash", str(SCRIPT), "dry-run"], check=True, capture_output=True, text=True, timeout=5)
        self.assertIn("NO EXECUTION", proc.stdout)
        self.assertIn(PIN, proc.stdout)
        self.assertIn("stand_push", proc.stdout)

    def test_pinned_and_no_flash(self):
        src = SCRIPT.read_text()
        self.assertIn(PIN, src)
        self.assertNotIn("flash_app.py", src)
        self.assertNotIn("esptool.py", src)
        self.assertIn("cp -R artifacts/balance-v6", src)


if __name__ == "__main__":
    unittest.main()
